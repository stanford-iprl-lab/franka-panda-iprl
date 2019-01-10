/**
 * main.cc
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 09, 2019
 * Authors: Toki Migimatsu
 */

#include <csignal>   // std::signal, std::sig_atomic_t
#include <future>    // std::future
#include <iostream>  // std::cout
#include <string>    // std::string

#include <SpatialDyn/parsers/json.h>
#include <SpatialDyn/parsers/urdf.h>
#include <SpatialDyn/utils/eigen_string.h>
#include "spatial_dyn_franka_panda.h"

namespace {

volatile std::sig_atomic_t g_runloop = true;
void stop(int) {
  g_runloop = false;
}

const std::string KEY_PREFIX        = "franka_panda::";
const std::string KEY_MODEL         = KEY_PREFIX + "model";
const std::string KEY_SENSOR_Q      = KEY_PREFIX + "sensor::q";
const std::string KEY_SENSOR_DQ     = KEY_PREFIX + "sensor::dq";
const std::string KEY_CONTROL_TAU   = KEY_PREFIX + "control::tau";
const std::string KEY_CONTROL_MODE  = KEY_PREFIX + "control::mode";
const std::string KEY_DRIVER_STATUS = KEY_PREFIX + "driver::status";

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Usage:" << std::endl
              << "\t./franka_panda_opspace <franka_panda.urdf>" << std::endl;
    return 0;
  }

  FrankaPanda::ArticulatedBody ab = SpatialDyn::Urdf::LoadModel(argv[1]);

  SpatialDyn::Timer timer(1000);
  SpatialDyn::RedisClient redis_client;
  redis_client.connect();

  redis_client.sync_set(KEY_MODEL, SpatialDyn::Json::Serialize(ab).dump());

  Eigen::VectorXd q_0 = Eigen::VectorXd::Zero(ab.dof());//redis_client.sync_get<Eigen::VectorXd>(KEY_SENSOR_Q);
  Eigen::VectorXd dq_0 = Eigen::VectorXd::Zero(ab.dof());//redis_client.sync_get<Eigen::VectorXd>(KEY_SENSOR_DQ);
  q_0 << 0., -M_PI/6., 0., -5.*M_PI/6., 0., 2.*M_PI/3., 0.;
  ab.set_q(q_0);
  ab.set_dq(dq_0);

  redis_client.set(KEY_SENSOR_Q, ab.q());
  redis_client.set(KEY_SENSOR_DQ, ab.dq());
  redis_client.sync_commit();

  Eigen::VectorXd q_des       = ab.q();
  Eigen::Vector3d x_0         = SpatialDyn::Position(ab);
  Eigen::Quaterniond quat_des = SpatialDyn::Orientation(ab);

  bool is_initialized = false;

  std::signal(SIGTERM, &stop);
  std::signal(SIGINT, &stop);

  while (g_runloop) {
    timer.Sleep();

    std::future<std::string> fut_driver_status = redis_client.get<std::string>(KEY_DRIVER_STATUS);
    std::future<Eigen::VectorXd> fut_q = redis_client.get<Eigen::VectorXd>(KEY_SENSOR_Q);
    std::future<Eigen::VectorXd> fut_dq = redis_client.get<Eigen::VectorXd>(KEY_SENSOR_DQ);
    redis_client.commit();

    // if (fut_driver_status.get() != "running") break;

    ab.set_q(fut_q.get());
    ab.set_dq(fut_dq.get());

    Eigen::Vector3d x_des = x_0;
    x_des(1) += 0.2 * std::sin(timer.time_sim());
    Eigen::Vector3d x_err = SpatialDyn::Position(ab) - x_des;
    Eigen::Vector3d dx_err = SpatialDyn::LinearJacobian(ab) * ab.dq();
    Eigen::Vector3d ddx = -40 * x_err - 13 * dx_err;

    Eigen::Vector3d ori_err = SpatialDyn::Opspace::OrientationError(SpatialDyn::Orientation(ab), quat_des);
    Eigen::Vector3d w_err = SpatialDyn::AngularJacobian(ab) * ab.dq();
    Eigen::Vector3d dw = -10 * ori_err - 10 * w_err;

    Eigen::Vector6d ddx_dw;
    ddx_dw << ddx, dw;
    const Eigen::Matrix6Xd& J = SpatialDyn::Jacobian(ab);
    Eigen::MatrixXd N;
    Eigen::VectorXd tau = SpatialDyn::Opspace::InverseDynamics(ab, J, ddx_dw, &N);

    static const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ab.dof(), ab.dof());
    Eigen::VectorXd q_err = ab.q() - q_des;
    Eigen::VectorXd dq_err = ab.dq();
    Eigen::VectorXd ddq = -16 * q_err - 8 * dq_err;
    tau += SpatialDyn::Opspace::InverseDynamics(ab, I, ddq, &N);
    tau += SpatialDyn::Gravity(ab);

    if (!is_initialized) {
      redis_client.sync_set(KEY_CONTROL_TAU, tau);
      redis_client.sync_set(KEY_CONTROL_MODE, "torque");
      is_initialized = true;
    } else {
      redis_client.set(KEY_CONTROL_TAU, tau);
      redis_client.commit();
    }

    SpatialDyn::IntegrationOptions options;
    options.friction = true;
    SpatialDyn::Integrate(ab, tau, 0.001, {}, options);
    redis_client.set(KEY_SENSOR_Q, ab.q());
    redis_client.set(KEY_SENSOR_DQ, ab.dq());
    redis_client.sync_commit();
  }
  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;

  redis_client.set(KEY_CONTROL_MODE, "floating");
  redis_client.set(KEY_CONTROL_TAU, Eigen::VectorXd::Zero(ab.dof()));
  redis_client.sync_commit();

  return 0;
}
