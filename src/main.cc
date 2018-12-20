/**
 * main.cc
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: December 19, 2018
 * Authors: Toki Migimatsu
 */

#include <array>       // std::array
#include <atomic>      // std::atomic
#include <exception>   // std::runtime_error
#include <functional>  // std::function
#include <future>      // std::future
#include <iostream>    // std::cout
#include <memory>      // std::shared_ptr
#include <mutex>       // std::mutex
#include <string>      // std::string
#include <signal.h>    // signal, sig_atomic_t, SIGINT
#include <thread>      // std::thread

#include <cpp_redis/cpp_redis>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "args.h"
#include "controllers.h"
#include "shared_memory.h"
#include "string_utils.h"
#include "timer.h"
#include "redis_client.h"

namespace Eigen {

using Vector7d = Matrix<double,7,1>;

}  // namespace Eigen

namespace {

volatile sig_atomic_t g_runloop = true;
void stop(int signal) { g_runloop = false; }

}  // namespace

namespace FrankaDriver {

void RedisThread(const Args& args, std::shared_ptr<SharedMemory> globals) {
  const std::string KEY_Q            = args.key_prefix + args.key_q;
  const std::string KEY_DQ           = args.key_prefix + args.key_dq;
  const std::string KEY_TAU          = args.key_prefix + args.key_tau;
  const std::string KEY_DTAU         = args.key_prefix + args.key_dtau;
  const std::string KEY_MASS_MATRIX  = args.key_prefix + args.key_mass_matrix;
  const std::string KEY_CORIOLIS     = args.key_prefix + args.key_coriolis;
  const std::string KEY_GRAVITY      = args.key_prefix + args.key_gravity;
  const std::string KEY_TAU_COMMAND  = args.key_prefix + args.key_tau_command;
  const std::string KEY_POSE_COMMAND = args.key_prefix + args.key_pose_command;
  const std::string KEY_CONTROL_MODE = args.key_prefix + args.key_control_mode;

  // Connect to Redis
  SpatialDyn::RedisClient redis_client;
  redis_client.connect();

  // Set default Redis keys
  redis_client.set(KEY_CONTROL_MODE, ControlModeToString(globals->control_mode));
  redis_client.set(KEY_TAU_COMMAND, ArrayToString(globals->tau_command.load(), args.use_json));
  redis_client.sync_commit();

  // Set loop timer to 1kHz (to match Franka Panda's control frequency)
  SpatialDyn::Timer timer(1000);

  while (g_runloop) {
    try {

      timer.Sleep();

      // Get control mode
      ControlMode control_mode = StringToControlMode(redis_client.sync_get<std::string>(KEY_CONTROL_MODE));

      // Get command futures
      std::string key_command;
      switch (control_mode) {
        case ControlMode::CARTESIAN_POSE:
          key_command = KEY_POSE_COMMAND;
          break;
        default:
          key_command = KEY_TAU_COMMAND;
          break;
      };
      std::future<std::string> future_command = redis_client.get<std::string>(key_command);

      // Set sensor values
      redis_client.set(KEY_Q,    ArrayToString(globals->q.load(),    args.use_json));
      redis_client.set(KEY_DQ,   ArrayToString(globals->dq.load(),   args.use_json));
      redis_client.set(KEY_TAU,  ArrayToString(globals->tau.load(),  args.use_json));
      redis_client.set(KEY_DTAU, ArrayToString(globals->dtau.load(), args.use_json));
      if (args.publish_dynamics) {
        redis_client.set(KEY_MASS_MATRIX, ArrayToString(globals->mass_matrix.load(), args.use_json));
        redis_client.set(KEY_CORIOLIS,    ArrayToString(globals->coriolis.load(), args.use_json));
        redis_client.set(KEY_GRAVITY,     ArrayToString(globals->gravity.load(), args.use_json));
      }

      // Commit Redis commands
      redis_client.commit();

      // Wait for command futures
      switch (control_mode) {
        case ControlMode::CARTESIAN_POSE:
          globals->pose_command = StringToTransform(future_command.get(), args.use_json);
          break;
        default:
          globals->tau_command = StringToArray<7>(future_command.get(), args.use_json);
          break;
      }

      // Switch control mode
      globals->control_mode = control_mode;

    } catch (const std::exception& e) {
      std::cerr << e.what() << std::endl;
    }
  }
}

void RunFrankaController(const Args& args, std::shared_ptr<FrankaDriver::SharedMemory>& globals,
                         franka::Robot& robot, const franka::Model& model) {
  FrankaDriver::ControlMode control_mode = globals->control_mode;
  while (g_runloop) {
    try {
      switch (control_mode) {
        case FrankaDriver::ControlMode::TORQUE:
          robot.control(FrankaDriver::CreateTorqueController(args, globals, model),
                        args.limit_rate, args.lowpass_freq_cutoff);  // Blocking
          break;
        case FrankaDriver::ControlMode::CARTESIAN_POSE:
          robot.control(FrankaDriver::CreateCartesianPoseController(args, globals, model),
                        franka::ControllerMode::kCartesianImpedance,
                        args.limit_rate, args.lowpass_freq_cutoff);  // Blocking
          break;
        default:
          throw std::runtime_error("Controller mode " + ControlModeToString(control_mode) + " not supported.");
      }
    } catch (const FrankaDriver::SwitchControllerException& e) {
      // Switch controllers
      control_mode = globals->control_mode;
      std::cout << "Switching controllers from \"" << e.what() << "\" to \""
                << ControlModeToString(control_mode) << "\"" << std::endl;
    }
  }
}

}  // namespace FrankaDriver

int main(int argc, char* argv[]) {
  std::thread redis_thread;

  try {
    // Parse args
    FrankaDriver::Args args = FrankaDriver::ParseArgs(argc, argv);
    std::cout << args << std::endl;

    // Connect to robot
    franka::Robot robot(args.ip_robot);
    franka::Model model = robot.loadModel();

    // Set robot parameters
    robot.setCollisionBehavior(args.tau_contact_thresholds_acc, args.tau_collision_thresholds_acc,
                               args.tau_contact_thresholds,     args.tau_collision_thresholds,
                               args.f_contact_thresholds_acc,   args.f_collision_thresholds_acc,
                               args.f_contact_thresholds,       args.f_collision_thresholds);
    robot.setJointImpedance(args.K_joint);
    robot.setCartesianImpedance(args.K_cart);
    robot.setEE(args.T_ee_to_flange);
    robot.setK(args.T_op_point_to_ee);
    robot.setLoad(args.load_mass, args.load_com, args.load_inertia);

    // Set ctrl-c handler
    signal(SIGINT, stop);

    // Create communication interface between Redis and robot threads
    std::shared_ptr<FrankaDriver::SharedMemory> globals;

    // Run Redis thread
    redis_thread = std::thread(FrankaDriver::RedisThread, args, globals);

    // Run controller thread
    RunFrankaController(args, globals, robot, model);

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    g_runloop = false;
  }

  if (redis_thread.joinable()) {
    redis_thread.join();
  }

  return 0;
}
