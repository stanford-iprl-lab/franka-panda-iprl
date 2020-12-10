/**
 * control_thread.cc
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: December 20, 2018
 * Authors: Toki Migimatsu
 */

#include "control_thread.h"

#include <chrono>    // std::chrono
#include <iostream>  // std::cerr, std::cout
#include <map>       // std::map
#include <thread>    // std::this_thread

#include "shared_memory.h"

namespace franka_driver {

void RunControlLoop(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                    franka::Robot& robot, const franka::Model& model) {
  while (*globals->runloop) {
    try {
      ControlMode control_mode = globals->send_idle_mode ? ControlMode::IDLE
                                                         : globals->control_mode.load();
      if (control_mode == ControlMode::IDLE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      std::cout << "Executing " << ControlModeToString(control_mode) << " controller." << std::endl;
      globals->control_status = ControlStatus::RUNNING;
      switch (control_mode) {
        case ControlMode::FLOATING:
        case ControlMode::TORQUE:
          robot.control(CreateTorqueController(args, globals, model),
                        args.limit_rate, args.lowpass_freq_cutoff);  // Blocking
          break;
        case ControlMode::CARTESIAN_POSE:
        case ControlMode::DELTA_CARTESIAN_POSE:
          robot.control(CreateCartesianPoseController(args, globals, robot, model),
                        franka::ControllerMode::kCartesianImpedance,
                        args.limit_rate, args.lowpass_freq_cutoff);  // Blocking
          break;
        default:
          throw std::runtime_error("Controller mode " + ControlModeToString(control_mode) + " not supported.");
      }
      std::cout << "Finished " << ControlModeToString(control_mode) << " controller. Switching to "
                << ControlModeToString(ControlMode::IDLE) << "." << std::endl;

      globals->send_idle_mode = true;
      globals->control_status = ControlStatus::FINISHED;

    } catch (const SwitchControllerException& e) {
      // Switch controllers
      std::cout << "Interrupted " << e.what() << " controller. Switching..." << std::endl;
    }
  }
}

std::stringstream& operator>>(std::stringstream& ss, ControlMode& mode) {
  static const std::map<std::string, ControlMode> kStringToControlMode = {
    {"idle", ControlMode::IDLE},
    {"floating", ControlMode::FLOATING},
    {"torque", ControlMode::TORQUE},
    {"joint_position", ControlMode::JOINT_POSITION},
    {"joint_velocity", ControlMode::JOINT_VELOCITY},
    {"cartesian_pose", ControlMode::CARTESIAN_POSE},
    {"delta_cartesian_pose", ControlMode::DELTA_CARTESIAN_POSE},
    {"cartesian_velocity", ControlMode::CARTESIAN_VELOCITY}
  };
  if (kStringToControlMode.find(ss.str()) == kStringToControlMode.end()) {
    std::cerr << "StringToControlMode(): Unable to parse ControlMode from " << ss.str()
              << ". Must be one of: {\"idle\", \"floating\", \"torque\", \"joint_position\","
              << "\"joint_velocity\", \"cartesian_pose\", \"delta_cartesian_pose\","
              << "\"cartesian_velocity\"}. Defaulting to \"idle\"." << std::endl;
    mode = ControlMode::IDLE;
  } else {
    mode = kStringToControlMode.at(ss.str());
  }
  return ss;
}

std::stringstream& operator<<(std::stringstream& ss, ControlMode mode) {
  ss << ControlModeToString(mode);
  return ss;
}

std::string ControlModeToString(ControlMode mode) {
  static const std::map<ControlMode, std::string> kControlModeToString = {
    {ControlMode::IDLE, "idle"},
    {ControlMode::FLOATING, "floating"},
    {ControlMode::TORQUE, "torque"},
    {ControlMode::JOINT_POSITION, "joint_position"},
    {ControlMode::JOINT_VELOCITY, "joint_velocity"},
    {ControlMode::CARTESIAN_POSE, "cartesian_pose"},
    {ControlMode::DELTA_CARTESIAN_POSE, "delta_cartesian_pose"},
    {ControlMode::CARTESIAN_VELOCITY, "cartesian_velocity"}
  };
  return kControlModeToString.at(mode);
}

std::stringstream& operator<<(std::stringstream& ss, ControlStatus status) {
  static const std::map<ControlStatus, std::string> kControlStatusToString = {
    {ControlStatus::FINISHED, "finished"},
    {ControlStatus::RUNNING, "running"},
    {ControlStatus::ERROR, "error"}
  };
  ss << kControlStatusToString.at(status);
  return ss;
}

}  // namespace franka_driver
