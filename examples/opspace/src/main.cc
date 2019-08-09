/**
 * main.cc
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 09, 2019
 * Authors: Toki Migimatsu
 */

#include <atomic>     // std::atomic
#include <cmath>      // std::sin, std::cos
#include <csignal>    // std::signal, std::sig_atomic_t
#include <exception>  // std::exception
#include <future>     // std::future
#include <iostream>   // std::cout
#include <mutex>      // std::mutex
#include <string>     // std::string

#include <spatial_dyn/spatial_dyn.h>
#include <ctrl_utils/control.h>
#include <ctrl_utils/euclidian.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/json.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>
#include <franka_panda/articulated_body.h>

// #define USE_WEB_APP

#ifdef USE_WEB_APP
#include <redis_gl/redis_gl.h>
#endif

namespace Eigen {

using Vector7d = Matrix<double,7,1>;
using Matrix32d = Matrix<double,3,2>;

}  // namespace Eigen

namespace {

volatile std::sig_atomic_t g_runloop = true;
void stop(int) {
  g_runloop = false;
}

// Redis keys
const std::string KEY_PREFIX         = "franka_panda::";
const std::string KEY_TRAJ_PREFIX    = KEY_PREFIX + "trajectory::";

// GET keys
const std::string KEY_SENSOR_Q      = KEY_PREFIX + "sensor::q";
const std::string KEY_SENSOR_DQ     = KEY_PREFIX + "sensor::dq";
const std::string KEY_SENSOR_POS    = KEY_PREFIX + "sensor::pos";
const std::string KEY_SENSOR_ORI    = KEY_PREFIX + "sensor::ori";
const std::string KEY_MODEL_EE      = KEY_PREFIX + "model::inertia_ee";
const std::string KEY_DRIVER_STATUS = KEY_PREFIX + "driver::status";

// SET keys
const std::string KEY_CONTROL_TAU     = KEY_PREFIX + "control::tau";
const std::string KEY_CONTROL_MODE    = KEY_PREFIX + "control::mode";
const std::string KEY_CONTROL_POS_DES = KEY_PREFIX + "control::pos_des";
const std::string KEY_CONTROL_ORI_DES = KEY_PREFIX + "control::ori_des";
const std::string KEY_CONTROL_POS     = KEY_PREFIX + "control::pos";
const std::string KEY_CONTROL_ORI     = KEY_PREFIX + "control::ori";
const std::string KEY_CONTROL_POS_ERR = KEY_PREFIX + "control::pos_err";
const std::string KEY_CONTROL_ORI_ERR = KEY_PREFIX + "control::ori_err";
const std::string KEY_TRAJ_POS        = KEY_TRAJ_PREFIX + "pos";

const std::string kNameApp            = "simulator";
const std::string KEY_WEB_INTERACTION = "webapp::" + kNameApp + "::interaction";

// SUB keys
const std::string KEY_PUB_COMMAND = KEY_PREFIX + "control::pub::command";

// PUB keys
const std::string KEY_PUB_STATUS = KEY_PREFIX + "control::pub::status";

// Controller gains
const std::string KEY_KP_KV_POS   = KEY_PREFIX + "control::kp_kv_pos";
const std::string KEY_KP_KV_ORI   = KEY_PREFIX + "control::kp_kv_ori";
const std::string KEY_KP_KV_JOINT = KEY_PREFIX + "control::kp_kv_joint";
const std::string KEY_POS_ERR_MAX = KEY_PREFIX + "control::pos_err_max";
const std::string KEY_ORI_ERR_MAX = KEY_PREFIX + "control::ori_err_max";

// Controller parameters
const Eigen::Vector3d kEeOffset  = Eigen::Vector3d(0., 0., 0.107);  // Without gripper
const Eigen::Vector3d kFrankaGripperOffset  = Eigen::Vector3d(0., 0., 0.1034);
const Eigen::Vector3d kRobotiqGripperOffset = Eigen::Vector3d(0., 0., 0.135);  // Ranges from 0.130 to 0.144
const Eigen::VectorXd kQHome     = (Eigen::Vector7d() <<
                                    0., -M_PI/6., 0., -5.*M_PI/6., 0., 2.*M_PI/3., 0.).finished();
const Eigen::Matrix32d kKpKvPos  = (Eigen::Matrix32d() <<
                                    80., 12.,
                                    80., 12.,
                                    80., 12.).finished();
const Eigen::Vector2d kKpKvOri   = Eigen::Vector2d(80., 10.);
const Eigen::Vector2d kKpKvJoint = Eigen::Vector2d(5., 0.);
const double kTimerFreq          = 1000.;
const double kGainKeyPressPos    = 0.1 / kTimerFreq;
const double kGainKeyPressOri    = 0.3 / kTimerFreq;
const double kGainClickDrag      = 100.;
const double kMaxErrorPos        = kKpKvPos(0, 0) * 0.05;
const double kMaxErrorOri        = kKpKvOri(0, 0) * M_PI / 20;
const double kEpsilonPos         = 0.05;
const double kEpsilonOri         = 0.2;
const double kEpsilonVelPos      = 0.005;
const double kEpsilonVelOri      = 0.005;
const double kMaxForce           = 100.;
const std::chrono::milliseconds kTimePubWait = std::chrono::milliseconds{1000};

const spatial_dyn::opspace::InverseDynamicsOptions kOpspaceOptions = []() {
  spatial_dyn::opspace::InverseDynamicsOptions options;
  options.svd_epsilon = 0.01;
  options.f_acc_max = kMaxForce;
  return options;
}();

const spatial_dyn::IntegrationOptions kIntegrationOptions = []() {
  spatial_dyn::IntegrationOptions options;
  options.friction = true;
  return options;
}();

struct Args {

  Args(int argc, char* argv[]) {
    size_t idx_required = 0;
    for (int i = 1; i < argc; i++) {
      const std::string arg(argv[i]);
      if (arg == "--sim") {
        sim = true;
      } else if (arg == "--no-gripper") {
        gripper = false;
      } else if (arg == "--robotiq") {
        robotiq = true;
        gripper = false;
      } else if (idx_required == 0) {
        path_urdf = arg;
        idx_required++;
      }
    }
  }

  std::string path_urdf = "../../../resources/franka_panda.urdf";
  bool sim = false;
  bool gripper = true;
  bool robotiq = false;

};

}  // namespace

int main(int argc, char* argv[]) {
  std::cout << "Usage:" << std::endl
            << "\t./franka_panda_opspace <franka_panda.urdf> [--sim] [--no-gripper] [--robotiq]" << std::endl
            << std::endl;

  const Args args(argc, argv);

  // Create timer and Redis client
  ctrl_utils::Timer timer(kTimerFreq);
  ctrl_utils::RedisClient redis_client;
  redis_client.connect();
  cpp_redis::subscriber redis_sub;
  redis_sub.connect();

  // Load robot
  std::cout << "Loading urdf: " << args.path_urdf << std::endl;
  franka_panda::ArticulatedBody ab = spatial_dyn::urdf::LoadModel(args.path_urdf);
  if (args.sim) {
    ab.set_q(kQHome);
    ab.set_dq(Eigen::VectorXd::Zero(ab.dof()));
  } else {
    try {
      ab.set_q(redis_client.sync_get<Eigen::VectorXd>(KEY_SENSOR_Q));
      ab.set_dq(redis_client.sync_get<Eigen::VectorXd>(KEY_SENSOR_DQ));
    } catch (...) {
      std::cerr << "Error: franka_panda driver is not running." << std::endl;
      return 0;
    }
  }

  // Set gripper
  const Eigen::Vector3d gripper_offset = args.gripper ? kFrankaGripperOffset
                                                      : args.robotiq ? kRobotiqGripperOffset
                                                                     : Eigen::Vector3d::Zero();
  const Eigen::Vector3d ee_offset = kEeOffset + gripper_offset;
  if (!args.gripper) {
    std::vector<spatial_dyn::Graphics>& graphics = const_cast<std::vector<spatial_dyn::Graphics>&>(ab.rigid_bodies(-1).graphics);
    for (size_t i = 1, num_graphics = graphics.size(); i < num_graphics; i++) {
      graphics.pop_back();
    }
  }
  // ab.set_inertia_compensation(Eigen::Vector3d(0.2, 0.1, 0.1));
  // ab.set_stiction_coefficients(Eigen::Vector3d(0.8, 0.8, 0.6));
  // ab.set_stiction_activations(Eigen::Vector3d(0.1, 0.1, 0.1));

  // Initialize controller parameters
  Eigen::VectorXd q_des       = kQHome;
  Eigen::Vector3d x_des       = spatial_dyn::Position(ab, -1, ee_offset);
  Eigen::Quaterniond quat_des = spatial_dyn::Orientation(ab);
  std::mutex mtx_des;

  // Initialize Redis keys

#ifdef USE_WEB_APP
  const std::filesystem::path path_resources = (std::filesystem::current_path() /
                                                std::filesystem::path(args.path_urdf)).parent_path();
  redis_gl::simulator::RegisterResourcePath(redis_client, path_resources.string());

  redis_gl::simulator::ModelKeys model_keys("franka_panda");
  redis_gl::simulator::RegisterModelKeys(redis_client, model_keys);

  redis_gl::simulator::RegisterRobot(redis_client, model_keys, ab, KEY_SENSOR_Q);

  redis_gl::simulator::RegisterTrajectory(redis_client, model_keys, "x_ee", KEY_CONTROL_POS);

  spatial_dyn::Graphics x_des_marker("x_des_marker");
  x_des_marker.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
  x_des_marker.geometry.radius = 0.01;
  redis_gl::simulator::RegisterObject(redis_client, model_keys, x_des_marker, KEY_CONTROL_POS_DES, KEY_CONTROL_ORI_DES);
#endif  // USE_WEB_APP

  if (args.sim) {
    redis_client.set(KEY_SENSOR_Q, ab.q());
    redis_client.set(KEY_SENSOR_DQ, ab.dq());
  }
  redis_client.set(KEY_SENSOR_POS, spatial_dyn::Position(ab, -1, kEeOffset));
  redis_client.set(KEY_SENSOR_ORI, spatial_dyn::Orientation(ab).coeffs());
  redis_client.set(KEY_KP_KV_POS, kKpKvPos);
  redis_client.set(KEY_KP_KV_ORI, kKpKvOri);
  redis_client.set(KEY_KP_KV_JOINT, kKpKvJoint);
  redis_client.set(KEY_POS_ERR_MAX, kMaxErrorPos);
  redis_client.set(KEY_ORI_ERR_MAX, kMaxErrorOri);
  redis_client.set(KEY_CONTROL_POS_DES, x_des);
  redis_client.set(KEY_CONTROL_ORI_DES, quat_des.coeffs());
  redis_client.sync_commit();

  Eigen::Vector3d x_des_pub       = spatial_dyn::Position(ab, -1, ee_offset);
  Eigen::Quaterniond quat_des_pub = spatial_dyn::Orientation(ab);
  std::atomic<double> epsilon_pos = { kEpsilonPos };
  std::atomic<double> epsilon_ori = { kEpsilonOri };
  bool is_pub_available = false;
  bool is_pub_waiting = false;
  std::mutex mtx_pub;
  redis_sub.subscribe(KEY_PUB_COMMAND,
      [&mtx_des, &x_des, &quat_des, &mtx_pub, &x_des_pub, &quat_des_pub, &epsilon_pos,
       &epsilon_ori, &is_pub_available](const std::string& key, const std::string& val) {
    std::cout << "SUB " << key << ": " << val << std::endl;
    try {
      // Get current des pose
      mtx_des.lock();
      Eigen::Vector3d x = x_des;
      Eigen::Quaterniond quat = quat_des;
      mtx_des.unlock();

      // Parse json command
      nlohmann::json json_cmd = nlohmann::json::parse(val);
      std::string type = json_cmd["type"].get<std::string>();
      if (type == "pose") {
        if (json_cmd.find("pos") != json_cmd.end()) {
          x = json_cmd["pos"].get<Eigen::Vector3d>();
        }
        if (json_cmd.find("quat") != json_cmd.end()) {
          quat.coeffs() = json_cmd["quat"].get<Eigen::Vector4d>();
        } else if (json_cmd.find("rot") != json_cmd.end()) {
          quat = json_cmd["rot"].get<Eigen::Matrix3d>();
        }
      } else if (type == "delta_pose") {
        if (json_cmd.find("pos") != json_cmd.end()) {
          x += json_cmd["pos"].get<Eigen::Vector3d>();
        }
        if (json_cmd.find("quat") != json_cmd.end()) {
          Eigen::Quaterniond quat_delta;
          quat_delta.coeffs() = json_cmd["quat"].get<Eigen::Vector4d>();
          quat = quat * quat_delta;
        } else if (json_cmd.find("rot") != json_cmd.end()) {
          Eigen::Quaterniond quat_delta;
          quat_delta = json_cmd["rot"].get<Eigen::Matrix3d>();
          quat = quat * quat_delta;
        }
      }
      if (json_cmd.find("pos_tolerance") != json_cmd.end()) {
        epsilon_pos = json_cmd["pos_tolerance"].get<double>();
      }
      if (json_cmd.find("ori_tolerance") != json_cmd.end()) {
        epsilon_ori = json_cmd["ori_tolerance"].get<double>();
      }

      // Set pub des pose
      mtx_pub.lock();
      x_des_pub = x;
      quat_des_pub = quat;
      is_pub_available = true;
      mtx_pub.unlock();
    } catch (const std::exception& e) {
      std::cerr << "cpp_redis::subscribe_callback(" << key << ", " << val << "): " << std::endl << e.what() << std::endl;
    }
  });
  redis_sub.commit();

  // Get end-effector model from driver
  try {
    nlohmann::json json_ee = redis_client.sync_get<nlohmann::json>(KEY_MODEL_EE);
    double m_ee = json_ee["m"].get<double>();
    Eigen::Vector3d com_ee = json_ee["com"].get<Eigen::Vector3d>();
    Eigen::Vector6d I_com_ee = json_ee["I_com"].get<Eigen::Vector6d>();
    ab.ReplaceLoad(spatial_dyn::SpatialInertiad(m_ee, com_ee, I_com_ee));
  } catch (...) {}

  // Create signal handler
  std::signal(SIGINT, &stop);

  bool is_initialized = false;  // Flat to set control mode on first iteration
  auto t_pub = std::chrono::steady_clock::now();
  bool is_pose_des_new = true;

  Eigen::Quaterniond quat_0 = Eigen::Quaterniond::Identity();  // Track previous quat for continuity

  try {
    while (g_runloop) {
      // Wait for next loop
      timer.Sleep();

      // Get Redis values
      std::future<Eigen::Matrix32d> fut_kp_kv_pos  = redis_client.get<Eigen::Matrix32d>(KEY_KP_KV_POS);
      std::future<Eigen::Vector2d> fut_kp_kv_ori   = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_ORI);
      std::future<Eigen::Vector2d> fut_kp_kv_joint = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_JOINT);
      std::future<Eigen::Vector3d> fut_x_des       = redis_client.get<Eigen::Vector3d>(KEY_CONTROL_POS_DES);
      std::future<Eigen::Vector4d> fut_quat_des    = redis_client.get<Eigen::Vector4d>(KEY_CONTROL_ORI_DES);
      std::future<double> fut_max_err_pos = redis_client.get<double>(KEY_POS_ERR_MAX);
      std::future<double> fut_max_err_ori = redis_client.get<double>(KEY_ORI_ERR_MAX);
#ifdef USE_WEB_APP
      std::future<redis_gl::simulator::Interaction> fut_interaction =
          redis_client.get<redis_gl::simulator::Interaction>(redis_gl::simulator::KEY_INTERACTION);
#endif  // USE_WEB_APP

      if (!args.sim) {
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

      // Check for PUB commands
      x_des = fut_x_des.get();
      quat_des = Eigen::Quaterniond(fut_quat_des.get());
      mtx_pub.lock();
      if (is_pub_available) {
        mtx_des.lock();
        x_des    = x_des_pub;
        quat_des = quat_des_pub;
        mtx_des.unlock();
        is_pub_available = false;
        is_pub_waiting = true;
        is_pose_des_new = true;
        t_pub = std::chrono::steady_clock::now();
      }
      mtx_pub.unlock();

      // Compute Jacobian
      const Eigen::Matrix6Xd& J = spatial_dyn::Jacobian(ab, -1, ee_offset);

      // Compute position PD control
      Eigen::Vector3d x_err;
      const Eigen::Vector3d x = spatial_dyn::Position(ab, -1, ee_offset);
      const Eigen::Vector3d dx = J.topRows<3>() * ab.dq();
      const Eigen::Vector3d ddx = ctrl_utils::PdControl(x, x_des, dx, fut_kp_kv_pos.get(),
                                                        fut_max_err_pos.get(), &x_err);

      // Compute orientation PD control
      Eigen::Vector3d ori_err;
      const Eigen::Quaterniond quat = ctrl_utils::NearQuaternion(spatial_dyn::Orientation(ab), quat_0);
      quat_0 = quat;
      quat_des = ctrl_utils::NearQuaternion(quat_des, quat);
      const Eigen::Vector3d w = J.bottomRows<3>() * ab.dq();
      const Eigen::Vector3d dw = ctrl_utils::PdControl(quat, quat_des, w, fut_kp_kv_ori.get(),
                                                       fut_max_err_ori.get(), &ori_err);

      // Compute opspace torques
      Eigen::MatrixXd N;
      Eigen::VectorXd tau_cmd;
      if (spatial_dyn::opspace::IsSingular(ab, J, kOpspaceOptions.svd_epsilon)) {
        // If robot is at a singularity, control position only
        tau_cmd = spatial_dyn::opspace::InverseDynamics(ab, J.topRows<3>(), ddx, &N, {}, kOpspaceOptions);
      } else {
        // Control position and orientation
        Eigen::Vector6d ddx_dw;
        ddx_dw << ddx, dw;
        tau_cmd = spatial_dyn::opspace::InverseDynamics(ab, J, ddx_dw, &N, {}, kOpspaceOptions);
      }

      // Add joint task in nullspace
      static const Eigen::MatrixXd J_null = Eigen::MatrixXd::Identity(ab.dof() - 1, ab.dof());
      const Eigen::VectorXd ddq = ctrl_utils::PdControl(ab.q(), q_des, ab.dq(), fut_kp_kv_joint.get());
      tau_cmd += spatial_dyn::opspace::InverseDynamics(ab, J_null, ddq.head(ab.dof() - 1), &N);

      // Add friction compensation
      tau_cmd += franka_panda::Friction(ab, tau_cmd);

      // Add gravity compensation
      tau_cmd += spatial_dyn::Gravity(ab);

      // Send control torques
      redis_client.set(KEY_CONTROL_TAU, tau_cmd);
      redis_client.commit();

      // Parse interaction from web app
#ifdef USE_WEB_APP
      std::map<size_t, spatial_dyn::SpatialForced> f_ext;
      try {
        redis_gl::simulator::Interaction interaction = fut_interaction.get();
        if (!interaction.key_down.empty()) {
          const Eigen::Vector3d x_adjust = redis_gl::simulator::KeypressPositionAdjustment(interaction);
          const Eigen::AngleAxisd aa_adjust = redis_gl::simulator::KeypressOrientationAdjustment(interaction);
          if (x_adjust != Eigen::Vector3d::Zero() || aa_adjust.angle() != 0.) {
            mtx_des.lock();
            x_des += x_adjust;
            quat_des = aa_adjust * quat_des;
            mtx_des.unlock();
            is_pose_des_new = true;
          }
        }

        f_ext = redis_gl::simulator::ComputeExternalForces(model_keys, ab, interaction);
      } catch (...) {}
#endif  // USE_WEB_APP

      if (args.sim) {
        // Integrate
#ifdef USE_WEB_APP
        spatial_dyn::Integrate(ab, tau_cmd, timer.dt(), f_ext, kIntegrationOptions);
#else  // USE_WEB_APP
        spatial_dyn::Integrate(ab, tau_cmd, timer.dt(), {}, kIntegrationOptions);
#endif  // USE_WEB_APP

        redis_client.set(KEY_SENSOR_Q, ab.q());
        redis_client.set(KEY_SENSOR_DQ, ab.dq());
      } else if (!is_initialized) {
        // Send control mode on first iteration (after setting torques)
        redis_client.set(KEY_CONTROL_MODE, "torque");
        is_initialized = true;
      }

      // Send PUB command status
      if (is_pub_waiting &&
          ((x_err.norm() < epsilon_pos.load() && ori_err.norm() < epsilon_ori.load() &&
            dx.norm() < kEpsilonVelPos && w.norm() < kEpsilonVelOri) ||
           (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t_pub) > kTimePubWait &&
            dx.norm() < kEpsilonVelPos && w.norm() < kEpsilonVelOri))) {
        redis_client.publish(KEY_PUB_STATUS, "done");
        is_pub_waiting = false;
        std::cout << "PUB " << KEY_PUB_STATUS << ": done" << std::endl;
      }

      // Send trajectory info to visualizer
      if (is_pose_des_new) {
        redis_client.set(KEY_CONTROL_POS_DES, x_des);
        redis_client.set(KEY_CONTROL_ORI_DES, quat_des.coeffs());
        is_pose_des_new = false;
      }
      redis_client.set(KEY_SENSOR_POS, spatial_dyn::Position(ab, -1, kEeOffset));
      redis_client.set(KEY_SENSOR_ORI, spatial_dyn::Orientation(ab).coeffs());
      redis_client.set(KEY_CONTROL_POS, x);
      redis_client.set(KEY_CONTROL_ORI, quat.coeffs());
      redis_client.set(KEY_CONTROL_POS_ERR, x_err);
      redis_client.set(KEY_CONTROL_ORI_ERR, ori_err);
      redis_client.sync_commit();
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  // Clear torques
  if (!args.sim) {
    redis_client.sync_set(KEY_CONTROL_MODE, "floating");
    std::cout << "Cleared torques and set control mode to 'floating'." << std::endl;
  }
  redis_client.sync_set(KEY_CONTROL_TAU, Eigen::VectorXd::Zero(ab.dof()));

#ifdef USE_WEB_APP
  redis_gl::simulator::UnregisterResourcePath(redis_client, path_resources.string());
  redis_gl::simulator::UnregisterModelKeys(redis_client, model_keys);
  redis_client.sync_commit();
#endif  // USE_WEB_APP

  // Print simulation stats
  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;

  return 0;
}
