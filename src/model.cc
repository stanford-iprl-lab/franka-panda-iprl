/**
 * model.cc
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 07, 2018
 * Authors: Toki Migimatsu
 */

#include "FrankaPanda/model.h"

#include <array>      // std::array
#include <cmath>      // std::abs
#include <exception>  // std::invalid_argument

#include "libfcimodels.h"

namespace FrankaPanda {

void Model::set_q(Eigen::Ref<const Eigen::VectorXd> q) {
  if (q.size() != static_cast<int>(dof_)) {
    throw std::invalid_argument("Model::set_q(): q must be a vector of size 7.");
  }
  q_ = q;
}

void Model::set_dq(Eigen::Ref<const Eigen::VectorXd> dq) {
  if (dq.size() != static_cast<int>(dof_)) {
    throw std::invalid_argument("Model::set_dq(): dq must be a vector of size 7.");
  }
  dq_ = dq;
}

Eigen::Matrix<double,6,1> Model::I_com_load() const {
  Eigen::Matrix<double,6,1> I_result;
  I_result << I_com_load_(0, 0), I_com_load_(1, 1), I_com_load_(2, 2),
              I_com_load_(0, 1), I_com_load_(0, 2), I_com_load_(1, 2);
  return I_result;
}

void Model::set_I_com_load(Eigen::Ref<const Eigen::Matrix<double,6,1>> I_com_flat) {
  I_com_load_ << I_com_flat(0), I_com_flat(3), I_com_flat(4),
                 I_com_flat(3), I_com_flat(1), I_com_flat(5),
                 I_com_flat(4), I_com_flat(5), I_com_flat(2);
}

void Model::set_I_com_load_matrix(Eigen::Ref<const Eigen::Matrix3d> I_com) {
  Eigen::Matrix<double,6,1> I_com_flat;
  I_com_flat << I_com(0, 0), I_com(1, 1), I_com(2, 2), I_com(0, 1), I_com(0, 2), I_com(1, 2);
  set_I_com_load(I_com_flat);
}

void Model::set_inertia_compensation(const Eigen::Vector3d& coeff) {
  if ((coeff.array() < 0.).any()) {
    throw std::invalid_argument("Model::set_inertia_compensation(): Coefficients must be positive.");
  }
  inertia_compensation_ = coeff;
}

void Model::set_stiction_coefficients(const Eigen::Vector3d& coeff) {
  if ((coeff.array() < 0.).any()) {
    throw std::invalid_argument("Model::set_stiction_coefficients(): Coefficients must be positive.");
  }
  stiction_coefficients_ = coeff;
}

void Model::set_stiction_activations(const Eigen::Vector3d& coeff) {
  if ((coeff.array() < 0.).any()) {
    throw std::invalid_argument("Model::set_stiction_activations(): Coefficients must be positive.");
  }
  stiction_activations_ = coeff;
}

Eigen::Isometry3d CartesianPose(const Model& model, int link) {
  if (link < 0) link += model.dof();

  Eigen::Matrix4d T_output;
  switch (link) {
    case 0:
      O_T_J1(model.q().data(), T_output.data());
      break;
    case 1:
      O_T_J2(model.q().data(), T_output.data());
      break;
    case 2:
      O_T_J3(model.q().data(), T_output.data());
      break;
    case 3:
      O_T_J4(model.q().data(), T_output.data());
      break;
    case 4:
      O_T_J5(model.q().data(), T_output.data());
      break;
    case 5:
      O_T_J6(model.q().data(), T_output.data());
      break;
    case 6:
      O_T_J7(model.q().data(), T_output.data());
      break;
  }

  return Eigen::Translation3d(T_output.block<3,1>(0,3)) *
         Eigen::Quaterniond(T_output.topLeftCorner<3,3>());
}

Eigen::Matrix<double,6,-1> Jacobian(const Model& model, int link) {
  if (link < 0) link += model.dof();

  Eigen::Matrix<double,6,-1> J_output(6, model.dof());
  switch (link) {
    case 0:
      O_J_J1(J_output.data());
      break;
    case 1:
      O_J_J2(model.q().data(), J_output.data());
      break;
    case 2:
      O_J_J3(model.q().data(), J_output.data());
      break;
    case 3:
      O_J_J4(model.q().data(), J_output.data());
      break;
    case 4:
      O_J_J5(model.q().data(), J_output.data());
      break;
    case 5:
      O_J_J6(model.q().data(), J_output.data());
      break;
    case 6:
      O_J_J7(model.q().data(), J_output.data());
      break;
  }

  return J_output;
}

Eigen::MatrixXd Inertia(const Model& model) {
  Eigen::MatrixXd A_output(model.dof(), model.dof());
  M_NE(model.q().data(), model.I_com_load_matrix().data(), model.m_load(),
       model.com_load().data(), A_output.data());
  A_output.diagonal().tail<3>() += model.inertia_compensation();
  return A_output;
}

Eigen::VectorXd CentrifugalCoriolis(const Model& model) {
  Eigen::VectorXd V_output(model.dof());
  c_NE(model.q().data(), model.dq().data(), model.I_com_load_matrix().data(),
       model.m_load(), model.com_load().data(), V_output.data());
  return V_output;
}

Eigen::VectorXd Gravity(const Model& model) {
  Eigen::VectorXd G_output(model.dof());
  g_NE(model.q().data(), model.g().data(), model.m_load(),
       model.com_load().data(), G_output.data());
  return G_output;
}

double Sign(double x, double epsilon = 0.01) {
  if (x > epsilon) {
    return 1.;
  } else if (x < -epsilon) {
    return -1.;
  }
  return 0.;
}

Eigen::VectorXd Friction(const Model& model, Eigen::Ref<const Eigen::VectorXd> tau) {
  Eigen::VectorXd F = Eigen::VectorXd::Zero(model.dof());
  for (size_t idx = 0; idx < 3; idx++) {
    const size_t i = idx + model.dof() - 3;
    const double mag_tau = std::abs(tau(i));
    const double threshold = model.stiction_coefficients()(idx);
    if (mag_tau >= threshold) continue;

    const double epsilon = model.stiction_activations()(idx);
    if (mag_tau >= epsilon) {
      F(i) = Sign(tau(i)) * threshold - tau(i);
      continue;
    }

    double dx = tau(i) / epsilon - Sign(tau(i));
    F(i) = -Sign(tau(i)) * threshold * (dx*dx - 1.) - tau(i);
  }
  return F;
}

}  // namespace FrankaPanda
