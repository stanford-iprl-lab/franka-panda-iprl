/**
 * torque_control.cc
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 16, 2019
 * Authors: Toki Migimatsu
 */

#include "control_thread.h"

#include <array>  // std::array

#include "shared_memory.h"

namespace franka_driver {

std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
CreateTorqueController(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                       const franka::Model& model) {
  return [&args, globals, &model](const franka::RobotState& state, franka::Duration dt) -> franka::Torques {
    if (!*globals->runloop) {
      throw std::runtime_error("TorqueController(): SIGINT.");
    }

    // Set sensor values
    globals->q        = state.q;
    globals->dq       = state.dq;
    globals->tau      = state.tau_J;
    globals->dtau     = state.dtau_J;

    // Get control mode
    ControlMode control_mode = globals->control_mode;

    if (control_mode == ControlMode::TORQUE) {

      // Get command torques
      std::array<double, 7> tau_command = globals->tau_command;

      // Cancel gravity from Franka Panda driver torques
      if (!args.compensate_gravity) {
        std::array<double, 7> gravity = model.gravity(state);
        for (size_t i = 0; i < tau_command.size(); i++) {
          tau_command[i] -= gravity[i];
        }
      }

      return tau_command;

    } else if (control_mode == ControlMode::FLOATING) {
      // Floating mode
      return std::array<double, 7>{{0., 0., 0., 0., 0., 0., 0.}};
    }

    throw SwitchControllerException("torque");
  };
}

}  // namespace franka_driver
