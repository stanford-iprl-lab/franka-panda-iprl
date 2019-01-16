/**
 * string_utils.h
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: December 20, 2018
 * Authors: Toki Migimatsu
 */

#ifndef FRANKA_DRIVER_STRING_UTILS_H_
#define FRANKA_DRIVER_STRING_UTILS_H_

#include <array>    // std::array
#include <string>   // std::string
#include <sstream>  // std::stringstream

#include <ctrl_utils/eigen_string.h>

#include "controllers.h"

namespace franka_driver {

template<unsigned long Dim>
std::string ArrayToString(const std::array<double, Dim>& arr, bool use_json = false) {
  Eigen::Map<const Eigen::VectorXd> map(arr.data(), arr.size());
  if (use_json) {
    ctrl_utils::Eigen::EncodeJson(map);
  }
  return ctrl_utils::Eigen::EncodeMatlab(map);
}

template<unsigned long Dim>
std::array<double, Dim> StringToArray(const std::string str, bool use_json = false) {
  std::array<double, Dim> arr;
  Eigen::Map<Eigen::VectorXd> map(arr.data(), arr.size());
  Eigen::MatrixXd M;
  if (use_json) {
    M = ctrl_utils::Eigen::DecodeJson<Eigen::MatrixXd>(str);
  } else {
    M = ctrl_utils::Eigen::DecodeMatlab<Eigen::MatrixXd>(str);
  }
  Eigen::Map<Eigen::VectorXd> m(&M(0, 0), M.size());
  map = m;
  return arr;
}

std::array<double, 16> StringToTransform(const std::string str, bool use_json) {
  std::array<double, 16> arr;
  Eigen::Map<Eigen::MatrixXd> map(arr.data(), 4, 4);
  Eigen::MatrixXd T;
  if (use_json) {
    // TODO
  } else {
    T = ctrl_utils::Eigen::DecodeMatlab<Eigen::MatrixXd>(str);
  }

  if (T.size() == 16) {
    // Full transformation matrix
    map = T;
  } else if (T.size() == 7) {
    // Position and quaternion
    Eigen::Quaterniond quat;
    quat.coeffs() = T.bottomLeftCorner<4,1>();
    map.topLeftCorner<3,3>() = quat.matrix();
    map.topRightCorner<3,1>() = T.topLeftCorner<3,1>();
    map.row(map.rows()-1) << 0, 0, 0, 1;
  } else if (T.size() == 3) {
    // Position only
    map.setIdentity();
    map.topRightCorner<3,1>() = T;
  }
  return arr;
}

}  // namespace franka_driver

#endif  // FRANKA_DRIVER_STRING_UTILS_H_
