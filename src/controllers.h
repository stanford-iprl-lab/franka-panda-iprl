/**
 * controllers.h
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: December 20, 2018
 * Authors: Toki Migimatsu
 */

#ifndef FRANKA_DRIVER_CONTROLLERS_H_
#define FRANKA_DRIVER_CONTROLLERS_H_

#include <exception>   // std::runtime_error
#include <functional>  // std::function
#include <memory>      // std::shared_ptr
#include <string>      // std::string

#include <franka/model.h>
#include <franka/robot.h>

#include "args.h"

namespace franka_driver {

struct SharedMemory;

enum class ControlMode {
  FLOATING, TORQUE, JOINT_POSITION, JOINT_VELOCITY, CARTESIAN_POSE, CARTESIAN_VELOCITY
};

ControlMode StringToControlMode(const std::string& mode);

std::string ControlModeToString(ControlMode mode);

class SwitchControllerException : public std::runtime_error {
 public:
  SwitchControllerException(const std::string msg) : std::runtime_error(msg) {}
  ~SwitchControllerException() {}
};

std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
CreateTorqueController(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                       const franka::Model& model);

std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
CreateCartesianPoseController(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                              const franka::Model& model);

}  // namespace franka_driver

#endif  // FRANKA_DRIVER_CONTROLLERS_H_
