/**
 * redis_thread.cc
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 16, 2019
 * Authors: Toki Migimatsu
 */

#include "redis_thread.h"

#include <exception>  // std::exception
#include <future>     // std::future
#include <iostream>   // std::cerr
#include <memory>     // std::shared_ptr
#include <string>     // std::string

#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>
#include <nlohmann/json.hpp>

#include "string_utils.h"

namespace franka_driver {

void RedisThread(std::shared_ptr<const Args> p_args, std::shared_ptr<SharedMemory> globals,
                 std::shared_ptr<const franka::Model> model, franka::RobotState state) {
  const Args& args = *p_args;
  const std::string KEY_Q              = args.key_prefix + args.key_q;
  const std::string KEY_DQ             = args.key_prefix + args.key_dq;
  const std::string KEY_TAU            = args.key_prefix + args.key_tau;
  const std::string KEY_DTAU           = args.key_prefix + args.key_dtau;
  const std::string KEY_POSE           = args.key_prefix + args.key_pose;
  const std::string KEY_INERTIA_EE     = args.key_prefix + args.key_inertia_ee;
  const std::string KEY_TAU_COMMAND    = args.key_prefix + args.key_tau_des;
  const std::string KEY_POSE_COMMAND   = args.key_prefix + args.key_pose_des;
  const std::string KEY_CONTROL_MODE   = args.key_prefix + args.key_control_mode;
  const std::string KEY_CONTROL_STATUS = args.key_prefix + args.key_control_status;
  const std::string KEY_DRIVER_STATUS  = args.key_prefix + args.key_driver_status;
  const std::string KEY_ROBOT_TIMER    = args.key_prefix + args.key_robot_timer;

  // Connect to Redis
  ctrl_utils::RedisClient redis_client;
  redis_client.connect(args.ip_redis, args.port_redis);

  // Set default Redis keys
  redis_client.set(KEY_CONTROL_MODE, globals->control_mode.load());
  redis_client.set(KEY_TAU_COMMAND, ArrayToString(globals->tau_command.load(), args.use_json));
  redis_client.set(KEY_POSE_COMMAND, ArrayToString(globals->pose_command.load(), args.use_json));
  // redis_client.set(KEY_M_LOAD,     std::to_string(globals->m_load.load()));
  // redis_client.set(KEY_COM_LOAD,   ArrayToString(globals->com_load.load(),   args.use_json));
  // redis_client.set(KEY_I_COM_LOAD, ArrayToString(globals->I_com_load.load(), args.use_json));
  redis_client.set(KEY_ROBOT_TIMER, globals->time.load());

  // Set initial state
  redis_client.set(KEY_Q,    ArrayToString(state.q,    args.use_json));
  redis_client.set(KEY_DQ,   ArrayToString(state.dq,   args.use_json));
  redis_client.set(KEY_TAU,  ArrayToString(state.tau_J,  args.use_json));
  redis_client.set(KEY_DTAU, ArrayToString(state.dtau_J, args.use_json));
  redis_client.set(KEY_POSE, ArrayToString(model->pose(franka::Frame::kEndEffector, state), args.use_json));
  nlohmann::json json_ee;
  json_ee["mass"] = state.m_ee;
  json_ee["com"] = state.F_x_Cee;
  json_ee["I_com_flat"] = std::array<double, 6>{state.I_ee[0], state.I_ee[4], state.I_ee[8],
                                                state.I_ee[1], state.I_ee[2], state.I_ee[5]};
  redis_client.set(KEY_INERTIA_EE, json_ee.dump());

  redis_client.sync_commit();

  // Set driver to running
  redis_client.sync_set(KEY_DRIVER_STATUS, Status::RUNNING);

  // Set loop timer to 1kHz (to match Franka Panda's control frequency)
  ctrl_utils::Timer timer(1000);

  while (*globals->runloop) {
    try {

      timer.Sleep();

      // Get control mode
      ControlMode control_mode = redis_client.sync_get<ControlMode>(KEY_CONTROL_MODE);

      // Get command futures
      std::string key_command;
      switch (control_mode) {
        case ControlMode::CARTESIAN_POSE:
        case ControlMode::DELTA_CARTESIAN_POSE:
          key_command = KEY_POSE_COMMAND;
          break;
        case ControlMode::FLOATING:
        case ControlMode::TORQUE:
          key_command = KEY_TAU_COMMAND;
          break;
        default:
          break;
      };
      std::future<std::string> future_command = redis_client.get<std::string>(key_command);

      // Set Redis values
      state.q = globals->q;
      redis_client.set(KEY_Q,    ArrayToString(state.q,              args.use_json));
      redis_client.set(KEY_DQ,   ArrayToString(globals->dq.load(),   args.use_json));
      redis_client.set(KEY_TAU,  ArrayToString(globals->tau.load(),  args.use_json));
      redis_client.set(KEY_DTAU, ArrayToString(globals->dtau.load(), args.use_json));
      redis_client.set(KEY_POSE, ArrayToString(model->pose(franka::Frame::kEndEffector, state)));
      redis_client.set(KEY_ROBOT_TIMER, globals->time.load());
      redis_client.set(KEY_CONTROL_STATUS, globals->control_status.load());
      redis_client.commit();

      // Wait for command futures
      switch (control_mode) {
        case ControlMode::CARTESIAN_POSE:
        case ControlMode::DELTA_CARTESIAN_POSE:
          globals->pose_command = StringToTransform(future_command.get(), args.use_json);
          break;
        case ControlMode::FLOATING:
        case ControlMode::TORQUE:
          globals->tau_command = StringToArray<7>(future_command.get(), args.use_json);
          break;
        default:
          break;
      }

      // Switch control mode
      if (globals->send_idle_mode) {
        globals->control_mode = ControlMode::IDLE;
        redis_client.sync_set(KEY_CONTROL_MODE, globals->control_mode.load());
        globals->send_idle_mode = false;
      } else {
        globals->control_mode = control_mode;
      }

    } catch (const std::exception& e) {
      std::cerr << e.what() << std::endl;
    }
  }

  // Clear Redis keys
  redis_client.set(KEY_CONTROL_MODE, ControlMode::FLOATING);
  redis_client.set(KEY_DRIVER_STATUS, Status::OFF);
  redis_client.sync_commit();
  std::cout << "RedisThread(): Exiting..." << std::endl;
}

std::stringstream& operator<<(std::stringstream& ss, Status status) {
  static const std::map<Status, std::string> kStatusToString = {
    {Status::RUNNING, "running"},
    {Status::OFF, "off"}
  };
  ss << kStatusToString.at(status);
  return ss;
}

}  // namespace franka_driver
