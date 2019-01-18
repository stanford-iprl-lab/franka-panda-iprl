/**
 * cartesian_pose_control.cc
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 16, 2019
 * Authors: Mengyuan Yan
 */

#include "control_thread.h"

#include <algorithm>  // std::all_of
#include <array>      // std::array
#include <cmath>      // std::abs, std::acos, std::cos, std::pow, std::sin, std::sqrt

#include <Eigen/Eigen>
#include <franka/exception.h>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "shared_memory.h"

namespace franka_driver {

/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void setDefaultBehavior(franka::Robot& robot);

/**
 * Generate a joint pose motion to a goal position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
class MotionGenerator {
  public:

  /**
   * Creates a new MotionGenerator instance.
   */
  MotionGenerator();

  /**
   * reset MotionGenerator for a target q.
   *
   * @param[in] speed_factor General speed factor in range [0, 1].
   * @param[in] pose_goal Target end-effector pose.
   */
  void reset(double speed_factor, const std::array<double, 16> pose_goal);

  /**
   * Sends next pose command
   *
   * @param[in] robot_state Current state of the robot.
   * @param[in] period Duration of execution.
   *
   * @return end-effector pose for use inside a control loop.
   */
  franka::CartesianPose operator()(const franka::RobotState& robot_state, franka::Duration period);

  private:
  using Vector3d = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;
  using Vector2d = Eigen::Matrix<double, 2, 1, Eigen::ColMajor>;
  using Matrix4x4d = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>;
  using Matrix3x3d = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;

  bool calculateDesiredValues(double t, Vector2d* delta_pose_d) const;
  void calculateSynchronizedValues();
  void preCalculateRotationMatrices();

  // tranlational and rotation angle residual threshold
  static constexpr std::array<double, 2> kDeltaMotionFinished = {1e-4, 0.01};
  Matrix4x4d pose_goal_;
  Matrix4x4d pose_start_;
  Vector3d unit_T_;
  Vector3d axis_R_;
  Vector2d delta_q_; // delta_trans_, delta_angle_

  Matrix3x3d precompute_pose_eye_;
  Matrix3x3d precompute_pose_dot_;
  Matrix3x3d precompute_pose_cross_;

  Vector2d dq_max_sync_;
  Vector2d t_1_sync_;
  Vector2d t_2_sync_;
  Vector2d t_f_sync_;
  Vector2d q_1_;

  double time_ = 0.0;

  const Vector2d default_dq_max_ = (Vector2d() << 0.1, 0.4).finished(); // 0.2m/s, 1.0rad/s
  const Vector2d default_ddq_max_start_ = (Vector2d() << 0.2, 0.5).finished();
  const Vector2d default_ddq_max_goal_ = (Vector2d() << 0.2, 0.5).finished();
  Vector2d dq_max_;
  Vector2d ddq_max_start_;
  Vector2d ddq_max_goal_;

};

void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

// This is for gcc 5.4, does not need this line with gcc6.4
// Refer to https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
constexpr std::array<double, 2> MotionGenerator::kDeltaMotionFinished;

MotionGenerator::MotionGenerator() {
  dq_max_sync_.setZero();
  pose_start_.setZero();
  pose_goal_.setZero();
  unit_T_.setZero();
  axis_R_.setZero();
  delta_q_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  q_1_.setZero();
}

void MotionGenerator::reset(double speed_factor, const std::array<double, 16> pose_goal)
{
  dq_max_ = default_dq_max_ * speed_factor;
  ddq_max_start_ = default_ddq_max_start_ * speed_factor;
  ddq_max_goal_ = default_ddq_max_goal_ * speed_factor;
  dq_max_sync_.setZero();
  pose_start_.setZero();
  pose_goal_ = Matrix4x4d(pose_goal.data());
  unit_T_.setZero();
  axis_R_.setZero();
  delta_q_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  q_1_.setZero();
  time_ = 0;
}

bool MotionGenerator::calculateDesiredValues(double t, Vector2d* delta_q_d) const {
  // Vector2d t_d = t_2_sync_ - t_1_sync_;
  Vector2d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 2> motion_finished{};

  for (size_t i = 0; i < 2; i++) {
    if (std::abs(delta_q_[i]) < kDeltaMotionFinished[i]) {
      (*delta_q_d)[i] = 0;
      motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_q_d)[i] = -1.0 / (t_1_sync_[i]*t_1_sync_[i]*t_1_sync_[i]) * dq_max_sync_[i] *
                          (0.5 * t - t_1_sync_[i]) * (t*t*t);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
        double tt = t - t_2_sync_[i];
        (*delta_q_d)[i] =
            delta_q_[i] + 0.5 * (1.0 / (delta_t_2_sync[i]*delta_t_2_sync[i]*delta_t_2_sync[i]) *
                                   (tt - 2.0 * delta_t_2_sync[i]) * (tt*tt*tt) +
                                (2.0 * tt - delta_t_2_sync[i])) * dq_max_sync_[i];
      } else {
        (*delta_q_d)[i] = delta_q_[i];
        motion_finished[i] = true;
      }
    }
  }
  return std::all_of(motion_finished.cbegin(), motion_finished.cend(),
                     [](bool x) { return x; });
}

void MotionGenerator::calculateSynchronizedValues() {
  Vector2d dq_max_reach(dq_max_);
  Vector2d t_f = Vector2d::Zero();
  Vector2d delta_t_2 = Vector2d::Zero();
  Vector2d t_1 = Vector2d::Zero();
  Vector2d delta_t_2_sync = Vector2d::Zero();

  unit_T_ = pose_goal_.block<3,1>(0,3) - pose_start_.block<3,1>(0,3);
  delta_q_[0] = unit_T_.norm();
  if (delta_q_[0] > 0) unit_T_ = unit_T_ / delta_q_[0];

  Matrix3x3d delta_R_;
  delta_R_ = pose_goal_.block<3,3>(0,0) * pose_start_.block<3,3>(0,0).transpose();
  // matrix to axis-angle
  double cos_angle = (delta_R_.trace()-1)/2;
  delta_q_[1] = std::acos(cos_angle);
  if (cos_angle < 0) {
    // more stable way of calculating axis when angle close to pi.
    axis_R_(0) = std::sqrt((delta_R_(0,0)-cos_angle)/2);
    axis_R_(1) = std::sqrt((delta_R_(1,1)-cos_angle)/2);
    axis_R_(2) = std::sqrt((delta_R_(2,2)-cos_angle)/2);
    if (delta_R_(0,1)-delta_R_(1,0) > 0) axis_R_(2) = -axis_R_(2);
    if (delta_R_(2,0)-delta_R_(0,2) > 0) axis_R_(1) = -axis_R_(1);
    if (delta_R_(1,2)-delta_R_(2,1) > 0) axis_R_(0) = -axis_R_(0);
    axis_R_.normalize();
  } else {
    // more stable way of calculating axis when angle close to 0.
    axis_R_(0) = delta_R_(2,1) - delta_R_(1,2);
    axis_R_(1) = delta_R_(0,2) - delta_R_(2,0);
    axis_R_(2) = delta_R_(1,0) - delta_R_(0,1);
    if (axis_R_.norm() > 0) axis_R_.normalize();
  }

  for (size_t i = 0; i < 2; i++) {
    if (std::abs(delta_q_[i]) > kDeltaMotionFinished[i]) {
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 2; i++) {
    if (std::abs(delta_q_[i]) > kDeltaMotionFinished[i]) {
      double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
      double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
      double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
      t_2_sync_[i] = t_f_sync_[i] - delta_t_2_sync[i];
      q_1_[i] = dq_max_sync_[i] * 0.5 * t_1_sync_[i];
    }
  }
}

void MotionGenerator::preCalculateRotationMatrices() {
  precompute_pose_eye_ = pose_start_.block<3,3>(0,0);
  precompute_pose_dot_ = axis_R_*axis_R_.transpose()*precompute_pose_eye_;
  precompute_pose_cross_ = Eigen::MatrixXd::Zero(3,3);
  precompute_pose_cross_(0,1) = -axis_R_(2);
  precompute_pose_cross_(1,0) = axis_R_(2);
  precompute_pose_cross_(0,2) = axis_R_(1);
  precompute_pose_cross_(2,0) = -axis_R_(1);
  precompute_pose_cross_(1,2) = -axis_R_(0);
  precompute_pose_cross_(2,1) = axis_R_(0);
  precompute_pose_cross_ = precompute_pose_cross_ * precompute_pose_eye_;
}

franka::CartesianPose MotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    pose_start_ = Matrix4x4d(robot_state.O_T_EE_c.data());
    calculateSynchronizedValues();
    preCalculateRotationMatrices();
  }

  Vector2d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 16> command_pose;
  Eigen::Map<Matrix4x4d> command_pose_e(&command_pose[0]);
  command_pose_e = pose_start_;

  if (delta_q_d[0] > 0) {
    command_pose_e.block<3,1>(0,3) += delta_q_d[0] * unit_T_;
  }
  if (delta_q_d[1] > 0) {
    command_pose_e.block<3,3>(0,0) = std::cos(delta_q_d[1]) * precompute_pose_eye_ +
                                     (1 - std::cos(delta_q_d[1])) * precompute_pose_dot_ +
                                     std::sin(delta_q_d[1]) * precompute_pose_cross_;
  }
//  std::cout << command_pose_e << std::endl;
//  std::cout << command_pose_e.transpose()*command_pose_e << std::endl;
  franka::CartesianPose output(command_pose);
  output.motion_finished = motion_finished;
  return output;
}

std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
CreateCartesianPoseController(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                              franka::Robot& robot, const franka::Model& model) {
  std::shared_ptr<MotionGenerator> motion_generator = std::make_shared<MotionGenerator>();
  
  franka::RobotState state = robot.readOnce();
  std::array<double, 16> pose_command = globals->pose_command;
  if (globals->control_mode == ControlMode::DELTA_CARTESIAN_POSE) {
    Eigen::Map<Eigen::Matrix4d> start_pose_e(state.O_T_EE.data());
    Eigen::Map<Eigen::Matrix4d> command_pose_e(pose_command.data());
    command_pose_e.topRightCorner<3,1>() += start_pose_e.topRightCorner<3,1>();
    command_pose_e.topLeftCorner<3,3>() = command_pose_e.topLeftCorner<3,3>() * start_pose_e.topLeftCorner<3,3>();
  }
  motion_generator->reset(1., pose_command);

  return [globals, motion_generator](const franka::RobotState& state, franka::Duration dt) -> franka::CartesianPose {
    if (!*globals->runloop) {
      throw std::runtime_error("TorqueController(): SIGINT.");
    }

    // Set sensor values
    globals->q    = state.q;
    globals->dq   = state.dq;
    globals->tau  = state.tau_J;
    globals->dtau = state.dtau_J;

    // Get command torques
    if (globals->control_mode != ControlMode::CARTESIAN_POSE) {
      throw SwitchControllerException("cartesian_pose");
    }

    return (*motion_generator)(state, dt);
  };
}

}  // namespace franka_driver
