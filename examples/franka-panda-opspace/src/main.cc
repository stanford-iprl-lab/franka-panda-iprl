/**
 * main.cc
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 09, 2019
 * Authors: Toki Migimatsu
 */

#include <cmath>      // std::sin, std::cos
#include <csignal>    // std::signal, std::sig_atomic_t
#include <exception>  // std::exception
#include <future>     // std::future
#include <iostream>   // std::cout
#include <string>     // std::string
#include <stdlib.h>   // realpath
#include <unistd.h>   // getcwd

#include <ctrl_utils/eigen_string.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>
#include <spatial_dyn/parsers/json.h>
#include <spatial_dyn/parsers/urdf.h>
#include <yaml-cpp/yaml.h>

#include "articulated_body.h"

namespace {

volatile std::sig_atomic_t g_runloop = true;
void stop(int) {
  g_runloop = false;
}

// Redis keys
const std::string KEY_PREFIX        = "franka_panda::";

// GET keys
const std::string KEY_SENSOR_Q      = KEY_PREFIX + "sensor::q";
const std::string KEY_SENSOR_DQ     = KEY_PREFIX + "sensor::dq";
const std::string KEY_MODEL_EE      = KEY_PREFIX + "model::inertia_ee";
const std::string KEY_DRIVER_STATUS = KEY_PREFIX + "driver::status";

// SET keys
const std::string KEY_CONTROL_TAU   = KEY_PREFIX + "control::tau";
const std::string KEY_CONTROL_MODE  = KEY_PREFIX + "control::mode";
const std::string KEY_TRAJ_POS      = KEY_PREFIX + "trajectory::pos";
const std::string KEY_TRAJ_ORI      = KEY_PREFIX + "trajectory::ori";
const std::string KEY_TRAJ_POS_ERR  = KEY_PREFIX + "trajectory::pos_err";
const std::string KEY_TRAJ_ORI_ERR  = KEY_PREFIX + "trajectory::ori_err";
const std::string KEY_MODEL         = KEY_PREFIX + "model";
const std::string KEY_WEB_RESOURCES = "webapp::resources";

// Controller gains
const std::string KEY_KP_POS   = KEY_PREFIX + "control::kp_pos";
const std::string KEY_KV_POS   = KEY_PREFIX + "control::kv_pos";
const std::string KEY_KP_ORI   = KEY_PREFIX + "control::kp_ori";
const std::string KEY_KV_ORI   = KEY_PREFIX + "control::kv_ori";
const std::string KEY_KP_JOINT = KEY_PREFIX + "control::kp_joint";
const std::string KEY_KV_JOINT = KEY_PREFIX + "control::kv_joint";

}  // namespace

std::stringstream& operator>>(std::stringstream& ss, YAML::Node& node) {
  node = YAML::Load(ss.str());
  return ss;
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Usage:" << std::endl
              << "\t./franka_panda_opspace <franka_panda.urdf> [--sim]" << std::endl;
    return 0;
  }
  const bool kSim = (argc > 2 && std::string(argv[2]) == "--sim");

  // Create Redis client and timer
  utils::RedisClient redis_client;
  redis_client.connect();
  utils::Timer timer(1000);

  // Load robot
  franka_panda::ArticulatedBody ab = spatial_dyn::urdf::LoadModel(argv[1]);
  Eigen::VectorXd q_home(ab.dof());
  q_home << 0., -M_PI/6., 0., -5.*M_PI/6., 0., 2.*M_PI/3., 0.;
  if (kSim) {
    ab.set_q(q_home);
    ab.set_dq(Eigen::VectorXd::Zero(ab.dof()));
    redis_client.set(KEY_SENSOR_Q, ab.q());
    redis_client.set(KEY_SENSOR_DQ, ab.dq());
    redis_client.sync_commit();
  } else {
    ab.set_q(redis_client.sync_get<Eigen::VectorXd>(KEY_SENSOR_Q));
    ab.set_dq(redis_client.sync_get<Eigen::VectorXd>(KEY_SENSOR_DQ));
  }

  // Send model to visualizer
  redis_client.sync_set(KEY_MODEL, spatial_dyn::json::Serialize(ab).dump());
  std::string path_urdf;
  {
    char* c_path_cwd = get_current_dir_name();
    std::string path_cwd(c_path_cwd);
    free(c_path_cwd);
    path_urdf = path_cwd + "/" + argv[1];
    char* c_path_urdf = realpath(path_urdf.c_str(), NULL);
    path_urdf = c_path_urdf;
    free(c_path_urdf);
    path_urdf = path_urdf.substr(0, path_urdf.find_last_of("/\\") + 1);
  }
  redis_client.hset(KEY_WEB_RESOURCES, "simulator", path_urdf);

  // Initialize parameters
  double kp_pos   = 40.;
  double kv_pos   = 5.;
  double kp_ori   = 40.;
  double kv_ori   = 5.;
  double kp_joint = 5.;
  double kv_joint = 0.;

  const Eigen::Vector3d ee_offset = Eigen::Vector3d(0., 0., 0.107);
  Eigen::VectorXd q_des           = ab.q();
  Eigen::Vector3d x_0             = spatial_dyn::Position(ab, -1, ee_offset);
  Eigen::Quaterniond quat_des     = spatial_dyn::Orientation(ab);

  // ab.set_inertia_compensation(Eigen::Vector3d(0.2, 0.1, 0.1));
  // ab.set_stiction_coefficients(Eigen::Vector3d(0.8, 0.8, 0.6));
  // ab.set_stiction_activations(Eigen::Vector3d(0.1, 0.1, 0.1));

  // Initialize gains in Redis
  redis_client.set(KEY_KP_POS, kp_pos);
  redis_client.set(KEY_KV_POS, kv_pos);
  redis_client.set(KEY_KP_ORI, kp_ori);
  redis_client.set(KEY_KV_ORI, kv_ori);
  redis_client.set(KEY_KP_JOINT, kp_joint);
  redis_client.set(KEY_KV_JOINT, kv_joint);

  // Get end-effector model from driver
  try {
    YAML::Node yaml_ee = redis_client.sync_get<YAML::Node>(KEY_MODEL_EE);
    double m = yaml_ee["m"].as<double>();
    std::vector<double> arr_com = yaml_ee["com"].as<std::vector<double>>();
    std::vector<double> arr_I_com = yaml_ee["I_com"].as<std::vector<double>>();
    Eigen::Map<Eigen::Vector3d> com(arr_com.data());
    Eigen::Map<Eigen::Vector6d> I_com(arr_I_com.data());
    ab.ReplaceLoad(spatial_dyn::SpatialInertiad(m, com, I_com));
  } catch (...) {}

  // Create signal handler
  std::signal(SIGTERM, &stop);
  std::signal(SIGINT, &stop);

  bool is_initialized = false;  // Flat to set control mode on first iteration

  try {
    while (g_runloop) {
      // Wait for next loop
      timer.Sleep();

      std::future<double> fut_kp_pos   = redis_client.get<double>(KEY_KP_POS);
      std::future<double> fut_kv_pos   = redis_client.get<double>(KEY_KV_POS);
      std::future<double> fut_kp_ori   = redis_client.get<double>(KEY_KP_ORI);
      std::future<double> fut_kv_ori   = redis_client.get<double>(KEY_KV_ORI);
      std::future<double> fut_kp_joint = redis_client.get<double>(KEY_KP_JOINT);
      std::future<double> fut_kv_joint = redis_client.get<double>(KEY_KV_JOINT);

      if (!kSim) {
        std::future<std::string> fut_driver_status = redis_client.get<std::string>(KEY_DRIVER_STATUS);
        std::future<Eigen::VectorXd> fut_q         = redis_client.get<Eigen::VectorXd>(KEY_SENSOR_Q);
        std::future<Eigen::VectorXd> fut_dq        = redis_client.get<Eigen::VectorXd>(KEY_SENSOR_DQ);
        redis_client.commit();

        // Break if driver is not running
        if (fut_driver_status.get() != "running") break;

        // Update robot state
        ab.set_q(fut_q.get());
        ab.set_dq(fut_dq.get());
      } else {
        redis_client.commit();
      }

      // Update gains
      kp_pos   = fut_kp_pos.get();
      kv_pos   = fut_kv_pos.get();
      kp_ori   = fut_kp_ori.get();
      kv_ori   = fut_kv_ori.get();
      kp_joint = fut_kp_joint.get();
      kv_joint = fut_kv_joint.get();

      // Position
      Eigen::Vector3d x_des = x_0;
      x_des(0) += 0.2 * (std::cos(timer.time_sim()) - 1.);
      x_des(1) += 0.2 * std::sin(timer.time_sim());
      Eigen::Vector3d x = spatial_dyn::Position(ab, -1, ee_offset);
      Eigen::Vector3d x_err = x - x_des;
      Eigen::Vector3d dx_err = spatial_dyn::LinearJacobian(ab, -1, ee_offset) * ab.dq();
      Eigen::Vector3d ddx = -kp_pos * x_err - kv_pos * dx_err;

      // Orientation
      Eigen::Quaterniond quat = spatial_dyn::Orientation(ab);
      Eigen::Vector3d ori_err = spatial_dyn::opspace::OrientationError(quat, quat_des);
      Eigen::Vector3d w_err = spatial_dyn::AngularJacobian(ab) * ab.dq();
      Eigen::Vector3d dw = -kp_ori * ori_err - kv_ori * w_err;

      // Combine position/orientation
      Eigen::Vector6d ddx_dw;
      ddx_dw << ddx, dw;
      const Eigen::Matrix6Xd& J = spatial_dyn::Jacobian(ab, -1, ee_offset);
      Eigen::MatrixXd N;
      Eigen::VectorXd tau_cmd = spatial_dyn::opspace::InverseDynamics(ab, J, ddx_dw, &N);

      // Nullspace
      static const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ab.dof(), ab.dof());
      Eigen::VectorXd q_err = ab.q() - q_des;
      Eigen::VectorXd dq_err = ab.dq();
      Eigen::VectorXd ddq = -kp_joint * q_err - kv_joint * dq_err;
      tau_cmd += spatial_dyn::opspace::InverseDynamics(ab, I, ddq, &N);

      // Add friction compensation
      tau_cmd += franka_panda::Friction(ab, tau_cmd);

      // Add gravity compensation
      tau_cmd += spatial_dyn::Gravity(ab);

      if (kSim) {
        // Integrate
        spatial_dyn::IntegrationOptions options;
        options.friction = true;
        spatial_dyn::Integrate(ab, tau_cmd, 0.001, {}, options);

        redis_client.set(KEY_CONTROL_TAU, tau_cmd);
        redis_client.set(KEY_SENSOR_Q, ab.q());
        redis_client.set(KEY_SENSOR_DQ, ab.dq());
      } else {
        if (!is_initialized) {
          redis_client.sync_set(KEY_CONTROL_TAU, tau_cmd); // Set control torques before switching
          redis_client.set(KEY_CONTROL_MODE, "torque");
          is_initialized = true;
        } else {
          redis_client.set(KEY_CONTROL_TAU, tau_cmd);
        }
      }

      // Send trajectory info to visualizer
      redis_client.set(KEY_TRAJ_POS, x);
      redis_client.set(KEY_TRAJ_ORI, quat.coeffs());
      redis_client.set(KEY_TRAJ_POS_ERR, x_err);
      redis_client.set(KEY_TRAJ_ORI_ERR, ori_err);
      redis_client.sync_commit();
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  // Clear torques
  redis_client.set(KEY_CONTROL_TAU, Eigen::VectorXd::Zero(ab.dof()));
  if (!kSim) {
    redis_client.set(KEY_CONTROL_MODE, "floating");
    redis_client.sync_commit();
    std::cout << "Cleared torques and set control mode to 'floating'." << std::endl;
  }

  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;

  return 0;
}
