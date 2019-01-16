/**
 * cartesian_pose_control.cc
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 16, 2019
 * Authors: Toki Migimatsu
 */

#include "control_thread.h"

#include "shared_memory.h"

namespace franka_driver {

std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
CreateCartesianPoseController(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                              const franka::Model& model) {
  return [&args, globals, &model](const franka::RobotState& state, franka::Duration dt) -> franka::CartesianPose {
    if (!*globals->runloop) {
      throw std::runtime_error("TorqueController(): SIGINT.");
    }

    // Set sensor values
    globals->q        = state.q;
    globals->dq       = state.dq;
    globals->tau      = state.tau_J;
    globals->dtau     = state.dtau_J;
    globals->m_ee     = state.m_ee;
    globals->com_ee   = state.F_x_Cee;
    globals->I_com_ee = state.I_ee;

    // Get command torques
    if (globals->control_mode != ControlMode::CARTESIAN_POSE) {
      throw SwitchControllerException("cartesian_pose");
    }

    // Floating mode
    return globals->pose_command.load();
  };
}

}  // namespace franka_driver
