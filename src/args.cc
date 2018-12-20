/**
 * args.cc
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: December 20, 2018
 * Authors: Toki Migimatsu
 */

#include "args.h"

#include <iostream>  // std::cout

#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

namespace FrankaDriver {

std::array<double, 16> ParseTransform(const YAML::Node& yaml) {
  std::array<double, 16> arr;
  Eigen::Map<Eigen::Matrix4d> map(arr.data());

  // Get position
  for (size_t i = 0; i < 3; i++) {
    map(i,3) = yaml["pos"][i].as<double>();
  }

  // Get rotation
  if (yaml["ori"]["w"]) {
    Eigen::Quaterniond quat;
    quat.w() = yaml["ori"]["w"].as<double>();
    quat.x() = yaml["ori"]["x"].as<double>();
    quat.y() = yaml["ori"]["y"].as<double>();
    quat.z() = yaml["ori"]["z"].as<double>();
    map.topLeftCorner<3,3>() = quat.matrix();
  } else {
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        map(i, j) = yaml["ori"][i][j].as<double>();
      }
    }
    Eigen::Affine3d T(map);
    map.topLeftCorner<3,3>() = T.rotation();
  }

  map.row(map.rows()-1) << 0., 0., 0., 1.;
  return arr;
}

template<unsigned long Dim>
std::array<double, Dim> ParseArray(const YAML::Node& yaml) {
  std::array<double, Dim> arr;
  if (yaml.IsSequence()) {
    arr = yaml.as<std::array<double, Dim>>();
  } else {
    double d = yaml.as<double>();
    for (size_t i = 0; i < arr.size(); i++) {
      arr[i] = d;
    }
  }
  return arr;
}

Args ParseYaml(const char* filename) {
  Args args;
  YAML::Node yaml = YAML::LoadFile(filename);
  std::cout << "Loading config: " << filename << std::endl << std::endl
            << yaml << std::endl << std::endl;

  args.ip_robot   = yaml["robot"]["ip"].as<std::string>();

  // Redis parameters
  args.ip_redis   = yaml["redis"]["ip"].as<std::string>();
  args.port_redis = yaml["redis"]["port"].as<size_t>();

  args.key_prefix       = yaml["redis"]["keys"]["prefix"].as<std::string>();
  args.key_tau_command  = yaml["redis"]["keys"]["tau_command"].as<std::string>();
  args.key_pose_command = yaml["redis"]["keys"]["pose_command"].as<std::string>();
  args.key_dx_command   = yaml["redis"]["keys"]["dx_command"].as<std::string>();
  args.key_q_command    = yaml["redis"]["keys"]["q_command"].as<std::string>();
  args.key_dq_command   = yaml["redis"]["keys"]["dq_command"].as<std::string>();
  args.key_control_mode = yaml["redis"]["keys"]["control_mode"].as<std::string>();
  args.key_q            = yaml["redis"]["keys"]["q"].as<std::string>();
  args.key_dq           = yaml["redis"]["keys"]["dq"].as<std::string>();
  args.key_tau          = yaml["redis"]["keys"]["tau"].as<std::string>();
  args.key_dtau         = yaml["redis"]["keys"]["dtau"].as<std::string>();
  args.key_mass_matrix  = yaml["redis"]["keys"]["mass_matrix"].as<std::string>();
  args.key_coriolis     = yaml["redis"]["keys"]["coriolis"].as<std::string>();
  args.key_gravity      = yaml["redis"]["keys"]["gravity"].as<std::string>();

  args.use_json         = yaml["redis"]["use_json"].as<bool>();
  args.publish_dynamics = yaml["redis"]["publish_dynamics"].as<bool>();

  // Load parameters
  args.load_mass = yaml["load"]["mass"].as<double>();
  args.load_com = yaml["load"]["com"].as<std::array<double, 3>>();

  args.load_inertia[0] = yaml["load"]["inertia"]["xx"].as<double>();
  args.load_inertia[1] = yaml["load"]["inertia"]["xy"].as<double>();
  args.load_inertia[2] = yaml["load"]["inertia"]["xz"].as<double>();

  args.load_inertia[3] = yaml["load"]["inertia"]["xy"].as<double>();
  args.load_inertia[4] = yaml["load"]["inertia"]["yy"].as<double>();
  args.load_inertia[5] = yaml["load"]["inertia"]["yz"].as<double>();

  args.load_inertia[6] = yaml["load"]["inertia"]["xz"].as<double>();
  args.load_inertia[7] = yaml["load"]["inertia"]["yz"].as<double>();
  args.load_inertia[8] = yaml["load"]["inertia"]["zz"].as<double>();

  // Control parameters
  args.limit_rate = yaml["control"]["limit_rate"].as<bool>();
  args.lowpass_freq_cutoff = yaml["control"]["lowpass_freq_cutoff"].as<double>();

  args.compensate_gravity = yaml["control"]["torque_controller"]["compensate_gravity"].as<bool>();

  args.K_joint = yaml["control"]["joint_space_controller"]["K_joint"].as<std::array<double, 7>>();

  args.K_cart = yaml["control"]["cartesian_space_controller"]["K_cart"].as<std::array<double, 6>>();
  args.T_ee_to_flange = ParseTransform(yaml["control"]["cartesian_space_controller"]["T_ee_to_flange"]);
  args.T_op_point_to_ee = ParseTransform(yaml["control"]["cartesian_space_controller"]["T_op_point_to_ee"]);

  args.tau_contact_thresholds_acc   = ParseArray<7>(yaml["control"]["collision_thresholds"]["tau_contact_acc"]);
  args.tau_collision_thresholds_acc = ParseArray<7>(yaml["control"]["collision_thresholds"]["tau_collision_acc"]);
  args.tau_contact_thresholds       = ParseArray<7>(yaml["control"]["collision_thresholds"]["tau_contact"]);
  args.tau_collision_thresholds     = ParseArray<7>(yaml["control"]["collision_thresholds"]["tau_collision"]);
  args.f_contact_thresholds_acc     = ParseArray<6>(yaml["control"]["collision_thresholds"]["f_contact_acc"]);
  args.f_collision_thresholds_acc   = ParseArray<6>(yaml["control"]["collision_thresholds"]["f_collision_acc"]);
  args.f_contact_thresholds         = ParseArray<6>(yaml["control"]["collision_thresholds"]["f_contact"]);
  args.f_collision_thresholds       = ParseArray<6>(yaml["control"]["collision_thresholds"]["f_collision"]);

  return args;
}

Args ParseArgs(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Usage:" << std::endl
              << "\t./franka_panda_driver ../resources/default.yaml" << std::endl;
    exit(0);
  }

  int i = 1;
  Args args = ParseYaml(argv[i]);

  // Parse remaining command line arguments
  std::string arg;
  for ( ; i < argc; i++) {
    arg = argv[i];
  }

  if (i != argc) throw std::invalid_argument("ParseArgs(): Invalid '" + arg + "' argument.");
  return args;
}

template<unsigned long Dim>
std::ostream& operator<<(std::ostream& os, const std::array<double, Dim>& arr) {
  constexpr unsigned long dim_sqrt = round(sqrt(Dim));
  constexpr bool is_square = dim_sqrt * dim_sqrt == Dim;

  // Print square matrix
  if (is_square) {
    std::string sep_outer;
    os << "[";
    for (size_t i = 0; i < dim_sqrt; i++) {
      os << sep_outer;
      std::string sep_inner;
      for (size_t j = 0; j < dim_sqrt; j++) {
        os << sep_inner << arr[dim_sqrt*j + i];
        if (sep_inner.empty()) sep_inner = ", ";
      }
      if (sep_outer.empty()) sep_outer = "; ";
    }
    os << "]";
    return os;
  }

  // Print flat vector
  std::string sep;
  os << "[";
  for (size_t i = 0; i < arr.size(); i++) {
    os << sep << arr[i];
    if (sep.empty()) sep = ", ";
  }
  os << "]";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Args& args) {
  os << "Args {" << std::endl
     << "  ip_robot: " << args.ip_robot << std::endl
     << "  ip_redis: " << args.ip_redis << std::endl
     << "  port_redis: " << args.port_redis << std::endl
     << "  key_prefix: " << args.key_prefix << std::endl
     << "  key_tau_command: " << args.key_tau_command << std::endl
     << "  key_pose_command: " << args.key_pose_command << std::endl
     << "  key_dx_command: " << args.key_dx_command << std::endl
     << "  key_q_command: " << args.key_q_command << std::endl
     << "  key_dq_command: " << args.key_dq_command << std::endl
     << "  key_control_mode: " << args.key_control_mode << std::endl
     << "  key_q: " << args.key_q << std::endl
     << "  key_dq: " << args.key_dq << std::endl
     << "  key_tau: " << args.key_tau << std::endl
     << "  key_dtau: " << args.key_dtau << std::endl
     << "  key_mass_matrix: " << args.key_mass_matrix << std::endl
     << "  key_coriolis: " << args.key_coriolis << std::endl
     << "  key_gravity: " << args.key_gravity << std::endl
     << "  use_json: " << args.use_json << std::endl
     << "  publish_dynamics: " << args.publish_dynamics << std::endl
     << "  load_mass: " << args.load_mass << std::endl
     << "  load_com: " << args.load_com << std::endl
     << "  load_inertia: " << args.load_inertia << std::endl
     << "  limit_rate: " << args.limit_rate << std::endl
     << "  lowpass_freq_cutoff: " << args.lowpass_freq_cutoff << std::endl
     << "  compensate_gravity: " << args.compensate_gravity << std::endl
     << "  K_joint: " << args.K_joint << std::endl
     << "  K_cart: " << args.K_cart << std::endl
     << "  T_ee_to_flange: " << args.T_ee_to_flange << std::endl
     << "  T_op_point_to_ee: " << args.T_op_point_to_ee << std::endl
     << "  tau_contact_thresholds_acc: " << args.tau_contact_thresholds_acc << std::endl
     << "  tau_collision_thresholds_acc: " << args.tau_collision_thresholds_acc << std::endl
     << "  tau_contact_thresholds: " << args.tau_contact_thresholds << std::endl
     << "  tau_collision_thresholds: " << args.tau_collision_thresholds << std::endl
     << "  f_contact_thresholds_acc: " << args.f_contact_thresholds_acc << std::endl
     << "  f_collision_thresholds_acc: " << args.f_collision_thresholds_acc << std::endl
     << "  f_contact_thresholds: " << args.f_contact_thresholds << std::endl
     << "  f_collision_thresholds: " << args.f_collision_thresholds << std::endl
     << "}" << std::endl;
  return os;
}

}  // namespace FrankaDriver