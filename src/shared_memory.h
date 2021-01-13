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
#include <csignal>   // std::sig_atomic_t

#include "control_thread.h"

namespace franka_driver {

struct SharedMemory {
  std::atomic<std::array<double, 7>> tau_command   = {{{0.}}};
  std::atomic<std::array<double, 16>> pose_command = {{{1.,0.,0.,0., 0.,1.,0.,0., 0.,0.,1.,0., 0.,0.,0.,1.}}};

  std::atomic<std::array<double, 7>> q    = {{{0.}}};
  std::atomic<std::array<double, 7>> dq   = {{{0.}}};
  std::atomic<std::array<double, 7>> tau  = {{{0.}}};
  std::atomic<std::array<double, 7>> dtau = {{{0.}}};

  std::atomic<double> time = {0.};

  // std::atomic<double> m_load                    = {0.};
  // std::atomic<std::array<double, 3>> com_load   = {{{0.}}};
  // std::atomic<std::array<double, 9>> I_com_load = {{{0.}}};

  std::atomic<bool> send_idle_mode          = {false};
  std::atomic<ControlMode> control_mode     = {ControlMode::FLOATING};
  std::atomic<ControlStatus> control_status = {ControlStatus::FINISHED};

  volatile std::sig_atomic_t* runloop = nullptr;
};

}  // namespace franka_driver

#endif  // FRANKA_DRIVER_SHARED_MEMORY_H_
