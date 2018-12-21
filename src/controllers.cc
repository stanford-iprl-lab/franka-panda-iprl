/**
 * controllers.cc
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: December 20, 2018
 * Authors: Toki Migimatsu
 */

#include "controllers.h"

#include <array>     // std::array
#include <iostream>  // std::cerr
#include <map>       // std::map

#include "shared_memory.h"

namespace FrankaDriver {

static const std::map<std::string, ControlMode> kControlModeToString = {
  {"floating", ControlMode::FLOATING},
  {"torque", ControlMode::TORQUE},
  {"joint_position", ControlMode::JOINT_POSITION},
  {"joint_velocity", ControlMode::JOINT_VELOCITY},
  {"cartesian_pose", ControlMode::CARTESIAN_POSE},
  {"cartesian_velocity", ControlMode::CARTESIAN_VELOCITY}
};
ControlMode StringToControlMode(const std::string& mode) {
  if (kControlModeToString.find(mode) == kControlModeToString.end()) {
    std::cerr << "StringToControlMode(): Unable to parse ControlMode from " << mode
              << ". Must be one of: {\"floating\", \"torque\", \"joint_position\","
              << "\"joint_velocity\", \"cartesian_position\", \"cartesian_velocity\"}. "
              << "Defaulting to \"floating\"." << std::endl;
    return ControlMode::FLOATING;
  }
  return kControlModeToString.at(mode);
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

void RedisSetSensorValues(const std::shared_ptr<SharedMemory>& globals,
                          const franka::Model& model, const franka::RobotState& state,
                          bool publish_dynamics) {
  globals->q    = state.q;
  globals->dq   = state.dq;
  globals->tau  = state.tau_J;
  globals->dtau = state.dtau_J;
  if (publish_dynamics) {
    globals->mass_matrix = model.mass(state);
    globals->coriolis    = model.coriolis(state);
    globals->gravity     = model.gravity(state);
  }
}

std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
CreateTorqueController(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                       const franka::Model& model) {
  return [&args, globals, &model](const franka::RobotState& state, franka::Duration dt) -> franka::Torques {
    if (!*globals->runloop) {
      throw std::runtime_error("TorqueController(): SIGINT.");
    }

    // Set sensor values
    RedisSetSensorValues(globals, model, state, args.publish_dynamics);

    // Get control mode
    ControlMode control_mode = globals->control_mode;
    if (control_mode == ControlMode::TORQUE) {

      // Get command torques
      std::array<double, 7> tau_command = globals->tau_command;

      // Cancel gravity from Franka Panda driver torques
      if (!args.compensate_gravity) {
        std::array<double, 7> gravity = args.publish_dynamics ? globals->gravity.load()
                                                              : model.gravity(state);
        for (size_t i = 0; i < tau_command.size(); i++) {
          tau_command[i] -= gravity[i];
        }
      }

      return tau_command;

    } else if (control_mode == ControlMode::FLOATING) {
      // Floating mode
      return std::array<double, 7>{0.};
    }

    throw SwitchControllerException("torque");
  };
}

std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
CreateCartesianPoseController(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                              const franka::Model& model) {
  return [&args, globals, &model](const franka::RobotState& state, franka::Duration dt) -> franka::CartesianPose {
    if (!*globals->runloop) {
      throw std::runtime_error("TorqueController(): SIGINT.");
    }

    // Set sensor values
    RedisSetSensorValues(globals, model, state, args.publish_dynamics);

    // Get command torques
    if (globals->control_mode != ControlMode::CARTESIAN_POSE) {
      throw SwitchControllerException("cartesian_pose");
    }

    // Floating mode
    return globals->pose_command.load();
  };
}

}  // namespace FrankaDriver
