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
#include <yaml-cpp/yaml.h>

#include "string_utils.h"

namespace franka_driver {

void RedisThread(const Args& args, std::shared_ptr<SharedMemory> globals) {
  const std::string KEY_Q             = args.key_prefix + args.key_q;
  const std::string KEY_DQ            = args.key_prefix + args.key_dq;
  const std::string KEY_TAU           = args.key_prefix + args.key_tau;
  const std::string KEY_DTAU          = args.key_prefix + args.key_dtau;
  const std::string KEY_INERTIA_EE    = args.key_prefix + args.key_inertia_ee;
  const std::string KEY_TAU_COMMAND   = args.key_prefix + args.key_tau_des;
  const std::string KEY_POSE_COMMAND  = args.key_prefix + args.key_pose_des;
  const std::string KEY_CONTROL_MODE  = args.key_prefix + args.key_control_mode;
  const std::string KEY_DRIVER_STATUS = args.key_prefix + args.key_driver_status;

  // Connect to Redis
  ctrl_utils::RedisClient redis_client;
  redis_client.connect(args.ip_redis, args.port_redis);

  // Set default Redis keys
  redis_client.set(KEY_CONTROL_MODE, globals->control_mode);
  redis_client.set(KEY_TAU_COMMAND, ArrayToString(globals->tau_command.load(), args.use_json));
  // redis_client.set(KEY_M_LOAD,     std::to_string(globals->m_load.load()));
  // redis_client.set(KEY_COM_LOAD,   ArrayToString(globals->com_load.load(),   args.use_json));
  // redis_client.set(KEY_I_COM_LOAD, ArrayToString(globals->I_com_load.load(), args.use_json));
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
          key_command = KEY_POSE_COMMAND;
          break;
        default:
          key_command = KEY_TAU_COMMAND;
          break;
      };
      std::future<std::string> future_command = redis_client.get<std::string>(key_command);

      // Set sensor values
      redis_client.set(KEY_Q,    ArrayToString(globals->q.load(),    args.use_json));
      redis_client.set(KEY_DQ,   ArrayToString(globals->dq.load(),   args.use_json));
      redis_client.set(KEY_TAU,  ArrayToString(globals->tau.load(),  args.use_json));
      redis_client.set(KEY_DTAU, ArrayToString(globals->dtau.load(), args.use_json));

      YAML::Node yaml_ee;
      yaml_ee["m"] = globals->m_ee.load();
      yaml_ee["com"] = globals->com_ee.load();
      std::array<double, 9> I_com_ee = globals->I_com_ee;
      yaml_ee["I_com"] = std::array<double, 6>{I_com_ee[0], I_com_ee[4], I_com_ee[8],
                                               I_com_ee[1], I_com_ee[2], I_com_ee[5]};
      YAML::Emitter json_ee;
      json_ee << YAML::DoubleQuoted << YAML::Flow << yaml_ee;
      redis_client.set(KEY_INERTIA_EE, json_ee.c_str());

      // Commit Redis commands
      redis_client.commit();

      // Wait for command futures
      switch (control_mode) {
        case ControlMode::CARTESIAN_POSE:
          globals->pose_command = StringToTransform(future_command.get(), args.use_json);
          break;
        default:
          globals->tau_command = StringToArray<7>(future_command.get(), args.use_json);
          break;
      }

      // Switch control mode
      globals->control_mode = control_mode;

    } catch (const std::exception& e) {
      std::cerr << e.what() << std::endl;
    }
  }

  // Clear Redis keys
  redis_client.set(KEY_CONTROL_MODE, ControlMode::FLOATING);
  redis_client.set(KEY_DRIVER_STATUS, Status::OFF);
  redis_client.sync_commit();
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
