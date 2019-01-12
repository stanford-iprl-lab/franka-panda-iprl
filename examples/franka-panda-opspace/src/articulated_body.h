/**
 * articulated_body.h
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 09, 2019
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_FRANKA_PANDA_H_
#define SPATIAL_DYN_FRANKA_PANDA_H_

#include <SpatialDyn/SpatialDyn.h>
#include <FrankaPanda/FrankaPanda.h>
#include <SpatialDyn/structs/articulated_body_cache.h>

namespace FrankaPanda {

class ArticulatedBody : public SpatialDyn::ArticulatedBody {

 public:

  ArticulatedBody() : SpatialDyn::ArticulatedBody() {}

  ArticulatedBody(const std::string& name) : SpatialDyn::ArticulatedBody(name) {}

  ArticulatedBody(const SpatialDyn::ArticulatedBody& ab) : SpatialDyn::ArticulatedBody(ab) {}

  virtual ~ArticulatedBody() {}

  virtual void set_q(Eigen::Ref<const Eigen::VectorXd> q) override {
    SpatialDyn::ArticulatedBody::set_q(q);
    franka_panda_.set_q(q);
    ComputeInertia();
    ComputeCentrifugalCoriolis();
    ComputeGravity();
  }

  virtual void set_dq(Eigen::Ref<const Eigen::VectorXd> dq) override {
    SpatialDyn::ArticulatedBody::set_dq(dq);
    franka_panda_.set_dq(dq);
    ComputeCentrifugalCoriolis();
  }

  const Eigen::Vector3d& inertia_compensation() const { return franka_panda_.inertia_compensation(); }

  void set_inertia_compensation(const Eigen::Vector3d& coeffs) {
    franka_panda_.set_inertia_compensation(coeffs);
  }

  const Eigen::Vector3d& stiction_coefficients() const { return franka_panda_.stiction_coefficients(); }

  void set_stiction_coefficients(const Eigen::Vector3d& coeffs) {
    franka_panda_.set_stiction_coefficients(coeffs);
  }

  const Eigen::Vector3d& stiction_activations() const { return franka_panda_.stiction_activations(); }

  void set_stiction_activations(const Eigen::Vector3d& coeffs) {
    franka_panda_.set_stiction_activations(coeffs);
  }

  const FrankaPanda::Model& franka_panda() const { return franka_panda_; }

  virtual void AddLoad(const SpatialDyn::SpatialInertiad& inertia, int idx_link = -1) override {
    if (idx_link < 0) idx_link += dof();
    SpatialDyn::ArticulatedBody::AddLoad(inertia, idx_link);
    set_inertia_load(inertia_load_.at(idx_link));
    ComputeInertia();
    ComputeCentrifugalCoriolis();
    ComputeGravity();
  }

  virtual void ReplaceLoad(const SpatialDyn::SpatialInertiad& inertia, int idx_link = -1) override {
    if (idx_link < 0) idx_link += dof();
    SpatialDyn::ArticulatedBody::ReplaceLoad(inertia, idx_link);
    set_inertia_load(inertia_load_.at(idx_link));
    ComputeInertia();
    ComputeCentrifugalCoriolis();
    ComputeGravity();
  }

  virtual void ClearLoad(int idx_link = -1) override {
    if (idx_link < 0) idx_link += dof();
    SpatialDyn::ArticulatedBody::ClearLoad(idx_link);
    set_inertia_load(inertia_load_.at(idx_link));
    ComputeInertia();
    ComputeCentrifugalCoriolis();
    ComputeGravity();
  }

 protected:

  void set_inertia_load(const SpatialDyn::SpatialInertiad& I) {
    franka_panda_.set_m_load(I.mass);
    franka_panda_.set_com_load(I.com);
    franka_panda_.set_I_com_load(I.I_com_flat());
  }

  void ComputeInertia() {
    auto& crba = cache_->crba_data_;
    crba.A = Inertia(franka_panda_);
    crba.A.diagonal().tail<3>() += franka_panda_.inertia_compensation();
    crba.is_computed = true;
  }

  void ComputeCentrifugalCoriolis() {
    auto& cc = cache_->cc_data_;
    cc.C = CentrifugalCoriolis(franka_panda_);
    cc.is_computed = true;
  }

  void ComputeGravity() {
    auto& grav = cache_->grav_data_;
    grav.G = Gravity(franka_panda_);
    grav.is_computed = true;
  }

  mutable FrankaPanda::Model franka_panda_;

};

Eigen::VectorXd Friction(const FrankaPanda::ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> tau) {
  return Friction(ab.franka_panda(), tau);
}

}  // namespace FrankaPanda

#endif  // SPATIAL_DYN_FRANKA_PANDA_H_
