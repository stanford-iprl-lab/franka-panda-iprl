/**
 * spatial_dyn_franka_panda.h
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

class ArticulatedBody;

}  // namespace FrankaPanda

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

    auto& crba = cache_->crba_data_;
    crba.A = Inertia(franka_panda_);
    crba.is_computed = true;

    auto& cc = cache_->cc_data_;
    cc.C = CentrifugalCoriolis(franka_panda_);
    cc.is_computed = true;

    auto& grav = cache_->grav_data_;
    grav.G = Gravity(franka_panda_);
    grav.is_computed = true;
  }

  virtual void set_dq(Eigen::Ref<const Eigen::VectorXd> dq) override {
    SpatialDyn::ArticulatedBody::set_dq(dq);
    franka_panda_.set_dq(dq);

    auto& cc = cache_->cc_data_;
    cc.C = CentrifugalCoriolis(franka_panda_);
    cc.is_computed = true;
  }

 protected:

  mutable FrankaPanda::Model franka_panda_;

};

}  // namespace FrankaPanda

#endif  // SPATIAL_DYN_FRANKA_PANDA_H_
