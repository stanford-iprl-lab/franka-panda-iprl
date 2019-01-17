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

#include <iostream>  // std::cerr
#include <map>       // std::map

#include "shared_memory.h"

namespace franka_driver {

void RunControlLoop(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                    franka::Robot& robot, const franka::Model& model) {
  ControlMode control_mode = globals->control_mode;
  while (*globals->runloop) {
    try {
      globals->control_status = ControlStatus::RUNNING;
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
      globals->control_status = ControlStatus::FINISHED;
      control_mode = globals->control_mode;
    } catch (const SwitchControllerException& e) {
      // Switch controllers
      control_mode = globals->control_mode;
      std::cout << "Switching controllers from \"" << e.what() << "\" to \""
                << ControlModeToString(control_mode) << "\"" << std::endl;
    }
  }
}

std::stringstream& operator>>(std::stringstream& ss, ControlMode& mode) {
  static const std::map<std::string, ControlMode> kStringToControlMode = {
    {"floating", ControlMode::FLOATING},
    {"torque", ControlMode::TORQUE},
    {"joint_position", ControlMode::JOINT_POSITION},
    {"joint_velocity", ControlMode::JOINT_VELOCITY},
    {"cartesian_pose", ControlMode::CARTESIAN_POSE},
    {"cartesian_velocity", ControlMode::CARTESIAN_VELOCITY}
  };
  if (kStringToControlMode.find(ss.str()) == kStringToControlMode.end()) {
    std::cerr << "StringToControlMode(): Unable to parse ControlMode from " << ss.str()
              << ". Must be one of: {\"floating\", \"torque\", \"joint_position\","
              << "\"joint_velocity\", \"cartesian_position\", \"cartesian_velocity\"}. "
              << "Defaulting to \"floating\"." << std::endl;
    mode = ControlMode::FLOATING;
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
  switch (mode) {
    case ControlMode::TORQUE: return "torque";
    case ControlMode::JOINT_POSITION: return "joint_position";
    case ControlMode::JOINT_VELOCITY: return "joint_velocity";
    case ControlMode::CARTESIAN_POSE: return "cartesian_pose";
    case ControlMode::CARTESIAN_VELOCITY: return "cartesian_velocity";
    default: return "floating";
  }
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

}  // namespace FrankaDriver
