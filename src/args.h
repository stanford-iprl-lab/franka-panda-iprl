/**
 * args.h
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: December 20, 2018
 * Authors: Toki Migimatsu
 */

#ifndef FRANKA_DRIVER_ARGS_H_
#define FRANKA_DRIVER_ARGS_H_

#include <array>    // std::array
#include <string>   // std::string
#include <ostream>  // std::ostream

#include <franka/robot.h>

namespace FrankaDriver {

struct Args {
  std::string ip_robot = "172.16.0.2";
  std::string ip_redis = "127.0.0.1";
  size_t port_redis = 6379;

  // Redis keys
  std::string key_prefix = "franka_panda::";

  // Incoming keys
  std::string key_tau_command  = "control::tau";
  std::string key_pose_command = "control::pose";
  std::string key_dx_command   = "control::dx";
  std::string key_q_command    = "control::q";
  std::string key_dq_command   = "control::dq";
  std::string key_control_mode = "control::mode";

  // Outgoing keys
  std::string key_q    = "sensor::q";
  std::string key_dq   = "sensor::dq";
  std::string key_tau  = "sensor::tau";
  std::string key_dtau = "sensor::dtau";
  std::string key_mass_matrix = "model::mass_matrix";
  std::string key_coriolis    = "model::coriolis";
  std::string key_gravity     = "model::gravity";

  bool use_json = false;
  bool publish_dynamics = true;

  // Load parameters
  double load_mass = 0.;
  std::array<double, 3> load_com = {{0., 0., 0.}};
  std::array<double, 9> load_inertia = {{0.,0.,0., 0.,0.,0., 0.,0.,0.}};

  // General control parameters
  bool limit_rate = true;
  double lowpass_freq_cutoff = franka::kDefaultCutoffFrequency;

  // Torque control parameters
  bool compensate_gravity = true;

  // Joint space control parameters
  std::array<double, 7> K_joint = {{3000., 3000., 3000., 2500., 2500., 2000., 2000.}};

  // Cartesian space control parameters
  std::array<double, 6> K_cart  = {{3000., 3000., 3000., 300., 300., 300.}};
  std::array<double, 16> T_ee_to_flange = {{1.,0.,0.,0., 0.,1.,0.,0., 0.,0.,1.,0., 0.,0.,0.,1.}};
  std::array<double, 16> T_op_point_to_ee = {{1.,0.,0.,0., 0.,1.,0.,0., 0.,0.,1.,0., 0.,0.,0.,1.}};

  // Contact/collision parameters
  std::array<double, 7> tau_contact_thresholds_acc   = {{20., 20., 20., 20., 20., 20., 20.}};
  std::array<double, 7> tau_collision_thresholds_acc = {{20., 20., 20., 20., 20., 20., 20.}};
  std::array<double, 7> tau_contact_thresholds       = {{10., 10., 10., 10., 10., 10., 10.,}};
  std::array<double, 7> tau_collision_thresholds     = {{10., 10., 10., 10., 10., 10., 10.,}};
  std::array<double, 6> f_contact_thresholds_acc     = {{20., 20., 20., 20., 20., 20.}};
  std::array<double, 6> f_collision_thresholds_acc   = {{20., 20., 20., 20., 20., 20.}};
  std::array<double, 6> f_contact_thresholds         = {{10., 10., 10., 10., 10., 10.}};
  std::array<double, 6> f_collision_thresholds       = {{10., 10., 10., 10., 10., 10.}};

};

Args ParseArgs(int argc, char* argv[]);

std::ostream& operator<<(std::ostream& os, const Args& args);

}  // namespace FrankaDriver

#endif  // FRANKA_DRIVER_ARGS_H_
