/**
 * model.h
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 07, 2018
 * Authors: Toki Migimatsu
 */

#ifndef FRANKA_PANDA_MODEL_H_
#define FRANKA_PANDA_MODEL_H_

#include <Eigen/Eigen>

namespace FrankaPanda {

class Model {

 public:

  size_t dof() const { return dof_; }

  const Eigen::VectorXd& q() const { return q_; }
  void set_q(Eigen::Ref<const Eigen::VectorXd> q);

  const Eigen::VectorXd& dq() const { return dq_; }
  void set_dq(Eigen::Ref<const Eigen::VectorXd> dq);

  double m_load() const { return m_load_; }
  void set_m_load(double m) { m_load_ = m; }

  const Eigen::Vector3d& com_load() const { return com_load_; }
  void set_com_load(Eigen::Ref<const Eigen::Vector3d> com) { com_load_ = com; }

  Eigen::Matrix<double,6,1> I_com_load() const;
  void set_I_com_load(Eigen::Ref<const Eigen::Matrix<double,6,1>> I_com_flat);

  const Eigen::Matrix3d& I_com_load_matrix() const { return I_com_load_; }
  void set_I_com_load_matrix(Eigen::Ref<const Eigen::Matrix3d> I_com);

  const Eigen::Vector3d& g() const { return g_; }

 private:

  const size_t dof_ = 7;
  Eigen::VectorXd q_ = Eigen::VectorXd::Zero(dof_);
  Eigen::VectorXd dq_ = Eigen::VectorXd::Zero(dof_);

  double m_load_ = 0.;
  Eigen::Vector3d com_load_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d I_com_load_ = Eigen::Matrix3d::Zero();

  Eigen::Vector3d g_ = Eigen::Vector3d(0., 0., -9.81);

};

Eigen::Isometry3d CartesianPose(const Model& model, int link = -1);

Eigen::Matrix<double,6,-1> Jacobian(const Model& model, int link = -1);

Eigen::MatrixXd Inertia(const Model& model);

Eigen::VectorXd CentrifugalCoriolis(const Model& model);

Eigen::VectorXd Gravity(const Model& model);

}  // namespace FrankaPanda

#endif  // FRANKA_PANDA_MODEL_H_
