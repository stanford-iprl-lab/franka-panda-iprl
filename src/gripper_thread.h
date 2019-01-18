/**
 * gripper_thread.h
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 16, 2019
 * Authors: Toki Migimatsu
 */

#ifndef FRANKA_DRIVER_GRIPPER_THREAD_H_
#define FRANKA_DRIVER_GRIPPER_THREAD_H_

#include <memory>   // std::shared_ptr
#include <sstream>  // std::stringstream

#include "args.h"
#include "shared_memory.h"

namespace franka_driver {

enum class GripperMode { IDLE, MOVE, GRASP };

std::stringstream& operator<<(std::stringstream& ss, GripperMode mode);

enum class GripperStatus { OFF, GRASPING, NOT_GRASPED, GRASPED };

std::stringstream& operator>>(std::stringstream& ss, GripperMode& mode);
std::stringstream& operator<<(std::stringstream& ss, GripperStatus status);

struct GraspTolerance {
  double inner = 0.005;
  double outer = 0.005;
};

std::stringstream& operator<<(std::stringstream& ss, const GraspTolerance& grasp_tol);
std::stringstream& operator>>(std::stringstream& ss, GraspTolerance& grasp_tol);

void GripperThread(std::shared_ptr<const Args> args, std::shared_ptr<SharedMemory> globals);

}  // namespace franka_driver

#endif  // FRANKA_DRIVER_GRIPPER_THREAD_H_
