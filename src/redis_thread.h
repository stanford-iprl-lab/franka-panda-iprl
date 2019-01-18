/**
 * redis_thread.h
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 16, 2019
 * Authors: Toki Migimatsu
 */

#ifndef FRANKA_DRIVER_REDIS_THREAD_H_
#define FRANKA_DRIVER_REDIS_THREAD_H_

#include <memory>   // std::shared_ptr
#include <sstream>  // std::stringstream

#include <franka/robot.h>
#include <franka/model.h>

#include "args.h"
#include "shared_memory.h"

namespace franka_driver {

enum class Status { RUNNING, OFF };

std::stringstream& operator<<(std::stringstream& ss, Status status);

void RedisThread(std::shared_ptr<const Args> args, std::shared_ptr<SharedMemory> globals,
                 std::shared_ptr<const franka::Model> model, franka::RobotState state);

}  // namespace franka_driver

#endif  // FRANKA_DRIVER_REDIS_THREAD_H_
