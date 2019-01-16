/**
 * main.cc
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: December 19, 2018
 * Authors: Toki Migimatsu
 */

#include <exception>   // std::runtime_error
#include <iostream>    // std::cout, std::cerr
#include <memory>      // std::shared_ptr
#include <signal.h>    // signal, sig_atomic_t, SIGINT
#include <thread>      // std::thread

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "args.h"
#include "control_thread.h"
#include "gripper_thread.h"
#include "redis_thread.h"
#include "shared_memory.h"

namespace {

volatile sig_atomic_t g_runloop = true;
void stop(int signal) { g_runloop = false; }

}  // namespace

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
    RunControlLoop(args, globals, robot, model);

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
