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
#include <tuple>       // std::tie

#include <Eigen/Eigen>
#include <cpp_redis/cpp_redis>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <yaml-cpp/yaml.h>

#include "args.h"
#include "controllers.h"
#include "shared_memory.h"
#include "string_utils.h"

namespace {

volatile sig_atomic_t g_runloop = true;
void stop(int signal) { g_runloop = false; }

}  // namespace

namespace franka_driver {

enum class Status { RUNNING, OFF };

std::stringstream& operator<<(std::stringstream& ss, Status status) {
  static const std::map<Status, std::string> kStatusToString = {
    {Status::RUNNING, "running"},
    {Status::OFF, "off"}
  };
  ss << kStatusToString.at(status);
  return ss;
}

enum class GripperMode { IDLE, MOVE, GRASP };

std::stringstream& operator<<(std::stringstream& ss, GripperMode mode) {
  static const std::map<GripperMode, std::string> kGripperModeToString = {
    {GripperMode::IDLE,  "idle"},
    {GripperMode::MOVE,  "move"},
    {GripperMode::GRASP, "grasp"}
  };
  ss << kGripperModeToString.at(mode);
  return ss;
}

std::stringstream& operator>>(std::stringstream& ss, GripperMode& mode) {
  static const std::map<std::string, GripperMode> kStringToGripperMode = {
    {"idle",  GripperMode::IDLE},
    {"move",  GripperMode::MOVE},
    {"grasp", GripperMode::GRASP},
  };
  if (kStringToGripperMode.find(ss.str()) == kStringToGripperMode.end()) {
    std::cerr << "StringToGripperMode(): Unable to parse GripperMode from " << ss.str()
              << ". Must be one of: {\"idle\", \"move\", \"grasp\"}."
              << "Defaulting to \"idle\"." << std::endl;
    mode = GripperMode::IDLE;
  } else {
    mode = kStringToGripperMode.at(ss.str());
  }
  return ss;
}

enum class GripperStatus { OPEN, GRASPED, OFF };

std::stringstream& operator<<(std::stringstream& ss, GripperStatus status) {
  static const std::map<GripperStatus, std::string> kGripperStatusToString = {
    {GripperStatus::OPEN,    "open"},
    {GripperStatus::GRASPED, "grasped"},
    {GripperStatus::OFF,     "off"}
  };
  ss << kGripperStatusToString.at(status);
  return ss;
}

struct GraspTolerance {
  double inner = 0.005;
  double outer = 0.005;
};

std::stringstream& operator<<(std::stringstream& ss, const GraspTolerance& grasp_tol) {
  ss << grasp_tol.inner << " " << grasp_tol.outer << std::endl;
  return ss;
}

std::stringstream& operator>>(std::stringstream& ss, GraspTolerance& grasp_tol) {
  ss >> grasp_tol.inner;
  ss >> grasp_tol.outer;
  return ss;
}

void RedisThread(const Args& args, std::shared_ptr<SharedMemory> globals) {
  const std::string KEY_Q             = args.key_prefix + args.key_q;
  const std::string KEY_DQ            = args.key_prefix + args.key_dq;
  const std::string KEY_TAU           = args.key_prefix + args.key_tau;
  const std::string KEY_DTAU          = args.key_prefix + args.key_dtau;
  const std::string KEY_INERTIA_EE    = args.key_prefix + args.key_inertia_ee;
  const std::string KEY_TAU_COMMAND   = args.key_prefix + args.key_tau_des;
  const std::string KEY_POSE_COMMAND  = args.key_prefix + args.key_pose_des;
  const std::string KEY_CONTROL_MODE  = args.key_prefix + args.key_control_mode;
  const std::string KEY_DRIVER_STATUS = args.key_prefix + args.key_driver_status;

  // Connect to Redis
  ctrl_utils::RedisClient redis_client;
  redis_client.connect(args.ip_redis, args.port_redis);

  // Set default Redis keys
  redis_client.set(KEY_CONTROL_MODE, globals->control_mode);
  redis_client.set(KEY_TAU_COMMAND, ArrayToString(globals->tau_command.load(), args.use_json));
  // redis_client.set(KEY_M_LOAD,     std::to_string(globals->m_load.load()));
  // redis_client.set(KEY_COM_LOAD,   ArrayToString(globals->com_load.load(),   args.use_json));
  // redis_client.set(KEY_I_COM_LOAD, ArrayToString(globals->I_com_load.load(), args.use_json));
  redis_client.sync_commit();

  // Set driver to running
  redis_client.sync_set(KEY_DRIVER_STATUS, Status::RUNNING);

  // Set loop timer to 1kHz (to match Franka Panda's control frequency)
  ctrl_utils::Timer timer(1000);

  while (*globals->runloop) {
    try {

      timer.Sleep();

      // Get control mode
      ControlMode control_mode = redis_client.sync_get<ControlMode>(KEY_CONTROL_MODE);

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

      YAML::Node yaml_ee;
      yaml_ee["m"] = globals->m_ee.load();
      yaml_ee["com"] = globals->com_ee.load();
      std::array<double, 9> I_com_ee = globals->I_com_ee;
      yaml_ee["I_com"] = std::array<double, 6>{I_com_ee[0], I_com_ee[4], I_com_ee[8],
                                               I_com_ee[1], I_com_ee[2], I_com_ee[5]};
      YAML::Emitter json_ee;
      json_ee << YAML::DoubleQuoted << YAML::Flow << yaml_ee;
      redis_client.set(KEY_INERTIA_EE, json_ee.c_str());

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

  // Clear Redis keys
  redis_client.set(KEY_CONTROL_MODE, ControlMode::FLOATING);
  redis_client.set(KEY_DRIVER_STATUS, Status::OFF);
  redis_client.sync_commit();
}

void GripperThread(const Args& args, std::shared_ptr<SharedMemory> globals) {
  const std::string KEY_GRIPPER_MODE      = args.key_prefix + args.key_gripper_mode;
  const std::string KEY_GRIPPER_WIDTH_DES = args.key_prefix + args.key_gripper_width_des;
  const std::string KEY_GRIPPER_SPEED_DES = args.key_prefix + args.key_gripper_speed_des;
  const std::string KEY_GRIPPER_FORCE_DES = args.key_prefix + args.key_gripper_force_des;
  const std::string KEY_GRIPPER_GRASP_TOL = args.key_prefix + args.key_gripper_grasp_tol;
  const std::string KEY_GRIPPER_WIDTH     = args.key_prefix + args.key_gripper_width;
  const std::string KEY_GRIPPER_MAX_WIDTH = args.key_prefix + args.key_gripper_max_width;
  const std::string KEY_GRIPPER_STATUS    = args.key_prefix + args.key_gripper_status;

  // Connect to Redis
  ctrl_utils::RedisClient redis_client;
  redis_client.connect(args.ip_redis, args.port_redis);

  try {
    // Connect to gripper
    franka::Gripper gripper(args.ip_robot);
    franka::GripperState state = gripper.readOnce();

    // Initialize gripper status
    GripperMode mode = GripperMode::IDLE;
    double width = state.width;
    double speed = 0.;
    double force = 0.;
    GraspTolerance grasp_tol = { 0.005, 0.005 };
    bool grasped = false;
    GripperStatus status = GripperStatus::OPEN;

    // Set default Redis keys
    redis_client.mset(std::make_pair(KEY_GRIPPER_MODE, mode),
                      std::make_pair(KEY_GRIPPER_WIDTH_DES, width),
                      std::make_pair(KEY_GRIPPER_SPEED_DES, speed),
                      std::make_pair(KEY_GRIPPER_FORCE_DES, force),
                      std::make_pair(KEY_GRIPPER_GRASP_TOL, grasp_tol));
    redis_client.set(KEY_GRIPPER_WIDTH, state.width);
    redis_client.set(KEY_GRIPPER_MAX_WIDTH, state.max_width);
    redis_client.sync_commit();

    // Set driver to running
    redis_client.sync_set(KEY_GRIPPER_STATUS, status);

    // Set loop timer to 1kHz (to match Franka Panda's control frequency)
    ctrl_utils::Timer timer(1000);

    while (*globals->runloop) {
      timer.Sleep();

      // Get gripper command with MGET
      std::tie(mode, width, speed, force, grasp_tol) =
          redis_client.sync_mget<GripperMode, double, double, double, GraspTolerance>(
              {KEY_GRIPPER_MODE, KEY_GRIPPER_WIDTH_DES, KEY_GRIPPER_SPEED_DES,
               KEY_GRIPPER_FORCE_DES, KEY_GRIPPER_GRASP_TOL});

      switch (mode) {
        case GripperMode::GRASP:
          grasped = gripper.grasp(width, speed, force, grasp_tol.inner, grasp_tol.outer);
          status = grasped ? GripperStatus::GRASPED : GripperStatus::OPEN;
          redis_client.mset(std::make_pair(KEY_GRIPPER_MODE, GripperMode::IDLE),
                            std::make_pair(KEY_GRIPPER_STATUS, status));
          break;
        case GripperMode::MOVE:
          grasped = gripper.move(width, speed);
          status = grasped ? GripperStatus::GRASPED : GripperStatus::OPEN;
          redis_client.mset(std::make_pair(KEY_GRIPPER_MODE, GripperMode::IDLE),
                            std::make_pair(KEY_GRIPPER_STATUS, status));
          break;
        default:
          break;
      }

      // Get the gripper state
      state = gripper.readOnce();
      redis_client.set(KEY_GRIPPER_WIDTH, state.width);
      redis_client.sync_commit();
    }
  } catch (const franka::NetworkException& e) {
    std::cerr << "GripperThread(): Unable to connect to gripper at " << args.ip_robot << "." << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "GripperThread(): " << e.what() << std::endl;
  }

  redis_client.set(KEY_GRIPPER_MODE, GripperMode::IDLE);
  redis_client.set(KEY_GRIPPER_STATUS, GripperStatus::OFF);
  redis_client.sync_commit();
}

void RunFrankaController(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                         franka::Robot& robot, const franka::Model& model) {
  ControlMode control_mode = globals->control_mode;
  while (*globals->runloop) {
    try {
      switch (control_mode) {
        case ControlMode::FLOATING:
        case ControlMode::TORQUE:
          robot.control(CreateTorqueController(args, globals, model),
                        args.limit_rate, args.lowpass_freq_cutoff);  // Blocking
          break;
        case ControlMode::CARTESIAN_POSE:
          robot.control(CreateCartesianPoseController(args, globals, model),
                        franka::ControllerMode::kCartesianImpedance,
                        args.limit_rate, args.lowpass_freq_cutoff);  // Blocking
          break;
        default:
          throw std::runtime_error("Controller mode " + ControlModeToString(control_mode) + " not supported.");
      }
    } catch (const SwitchControllerException& e) {
      // Switch controllers
      control_mode = globals->control_mode;
      std::cout << "Switching controllers from \"" << e.what() << "\" to \""
                << ControlModeToString(control_mode) << "\"" << std::endl;
    }
  }
}

}  // namespace franka_driver

int main(int argc, char* argv[]) {
  std::thread redis_thread, gripper_thread;

  try {
    // Parse args
    franka_driver::Args args = franka_driver::ParseArgs(argc, argv);
    // std::cout << args << std::endl;

    // Connect to robot
    franka::Robot robot(args.ip_robot);
    franka::Model model = robot.loadModel();

    // Set robot parameters
    robot.setCollisionBehavior(args.tau_contact_thresholds_acc, args.tau_collision_thresholds_acc,
                               args.tau_contact_thresholds,     args.tau_collision_thresholds,
                               args.f_contact_thresholds_acc,   args.f_collision_thresholds_acc,
                               args.f_contact_thresholds,       args.f_collision_thresholds);
    try {
      robot.setJointImpedance(args.K_joint);
    } catch (franka::CommandException& e) {
      std::cerr << e.what() << std::endl;
      std::cerr << std::endl << "Make sure the robot isn't close to any joint limits and the e-stop is released." << std::endl;
      g_runloop = false;
      return 0;
    }
    robot.setCartesianImpedance(args.K_cart);
    robot.setEE(args.T_ee_to_flange);
    robot.setK(args.T_op_point_to_ee);
    robot.setLoad(args.load_mass, args.load_com, args.load_inertia);

    // Set ctrl-c handler
    signal(SIGINT, stop);

    // Create communication interface between Redis and robot threads
    auto globals = std::make_shared<franka_driver::SharedMemory>();
    globals->runloop = &g_runloop;

    // Run Redis thread
    redis_thread = std::thread(franka_driver::RedisThread, args, globals);

    // Run gripper thread
    gripper_thread = std::thread(franka_driver::GripperThread, args, globals);

    // Run controller thread
    RunFrankaController(args, globals, robot, model);

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    g_runloop = false;
  }

  if (redis_thread.joinable()) {
    redis_thread.join();
  }
  if (gripper_thread.joinable()) {
    gripper_thread.join();
  }

  return 0;
}
