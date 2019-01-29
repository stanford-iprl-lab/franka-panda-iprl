/**
 * gripper_thread.cc
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 16, 2019
 * Authors: Toki Migimatsu
 */

#include "gripper_thread.h"

#include <exception>  // std::exception
#include <iostream>   // std::cerr
#include <map>        // std::map
#include <string>     // std::string
#include <tuple>      // std::tie
#include <utility>    // std::make_pair

#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>
#include <franka/exception.h>
#include <franka/gripper.h>

namespace franka_driver {

void GripperThread(std::shared_ptr<const Args> p_args, std::shared_ptr<SharedMemory> globals) {
  const Args& args = *p_args;
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
  redis_client.sync_set(KEY_GRIPPER_STATUS, GripperStatus::OFF);

  try {
    // Connect to gripper
    std::cout << "GripperThread(): Connecting to gripper at " << args.ip_robot << " (timeout 60s)..." << std::endl;
    franka::Gripper gripper(args.ip_robot);
    std::cout << "GripperThread(): Gripper connected." << std::endl;

    franka::GripperState state = gripper.readOnce();

    // Initialize gripper status
    GripperMode mode = GripperMode::IDLE;
    double width = state.width;
    double speed = 0.07;
    double force = 0.;
    GraspTolerance grasp_tol = { 0.005, 0.005 };
    bool grasped = false;
    GripperStatus status = GripperStatus::NOT_GRASPED;

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
          redis_client.sync_set(KEY_GRIPPER_STATUS, GripperStatus::GRASPING);
          grasped = gripper.grasp(width, speed, force, grasp_tol.inner, grasp_tol.outer);
          status = grasped ? GripperStatus::GRASPED : GripperStatus::NOT_GRASPED;
          redis_client.mset(std::make_pair(KEY_GRIPPER_MODE, GripperMode::IDLE),
                            std::make_pair(KEY_GRIPPER_STATUS, status));
          break;
        case GripperMode::MOVE:
          redis_client.sync_set(KEY_GRIPPER_STATUS, GripperStatus::GRASPING);
          grasped = gripper.move(width, speed);
          status = grasped ? GripperStatus::GRASPED : GripperStatus::NOT_GRASPED;
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
    std::cerr << "Unable to connect to gripper at " << args.ip_robot << "." << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "GripperThread(): " << e.what() << std::endl;
  }

  redis_client.set(KEY_GRIPPER_MODE, GripperMode::IDLE);
  redis_client.set(KEY_GRIPPER_STATUS, GripperStatus::OFF);
  redis_client.sync_commit();
  std::cout << "GripperThread(): Exiting..." << std::endl;
}

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

std::stringstream& operator<<(std::stringstream& ss, GripperStatus status) {
  static const std::map<GripperStatus, std::string> kGripperStatusToString = {
    {GripperStatus::OFF,         "off"},
    {GripperStatus::GRASPING,    "grasping"},
    {GripperStatus::NOT_GRASPED, "not_grasped"},
    {GripperStatus::GRASPED,     "grasped"}
  };
  ss << kGripperStatusToString.at(status);
  return ss;
}

std::stringstream& operator<<(std::stringstream& ss, const GraspTolerance& grasp_tol) {
  ss << grasp_tol.inner << " " << grasp_tol.outer << std::endl;
  return ss;
}

std::stringstream& operator>>(std::stringstream& ss, GraspTolerance& grasp_tol) {
  ss >> grasp_tol.inner;
  ss >> grasp_tol.outer;
  return ss;
}

}  // namespace franka_driver
