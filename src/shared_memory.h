/**
 * shared_memory.h
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: December 20, 2018
 * Authors: Toki Migimatsu
 */

#ifndef FRANKA_DRIVER_SHARED_MEMORY_H_
#define FRANKA_DRIVER_SHARED_MEMORY_H_

#include <atomic>    // std::atomic
#include <array>     // std::array
#include <signal.h>  // sig_atomic_t

#include "controllers.h"

namespace FrankaDriver {

enum class ControlMode;

struct SharedMemory {
  std::atomic<std::array<double, 7>> tau_command   = {{{0.}}};
  std::atomic<std::array<double, 16>> pose_command = {{{1.,0.,0.,0., 0.,1.,0.,0., 0.,0.,1.,0., 0.,0.,0.,1.}}};

  std::atomic<std::array<double, 7>> q    = {{{0.}}};
  std::atomic<std::array<double, 7>> dq   = {{{0.}}};
  std::atomic<std::array<double, 7>> tau  = {{{0.}}};
  std::atomic<std::array<double, 7>> dtau = {{{0.}}};

  std::atomic<std::array<double, 49>> mass_matrix = {{{0.}}};
  std::atomic<std::array<double, 7>> coriolis     = {{{0.}}};
  std::atomic<std::array<double, 7>> gravity      = {{{0.}}};

  std::atomic<ControlMode> control_mode = {ControlMode::FLOATING};

  volatile sig_atomic_t* runloop = nullptr;
};

}  // namespace FrankaDriver

#endif  // FRANKA_DRIVER_SHARED_MEMORY_H_
