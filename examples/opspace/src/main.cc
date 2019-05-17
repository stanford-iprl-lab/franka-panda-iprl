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
#include <ctrl_utils/euclidian.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/json.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>
#include <franka_panda/articulated_body.h>

#define USE_WEB_APP

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
const std::string KEY_MODELS_PREFIX  = KEY_PREFIX + "model::";
const std::string KEY_OBJECTS_PREFIX = KEY_PREFIX + "object::";
const std::string KEY_TRAJ_PREFIX    = KEY_PREFIX + "trajectory::";

// GET keys
const std::string KEY_SENSOR_Q      = KEY_PREFIX + "sensor::q";
const std::string KEY_SENSOR_DQ     = KEY_PREFIX + "sensor::dq";
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
const std::string KEY_WEB_RESOURCES   = "webapp::resources";
const std::string KEY_WEB_ARGS        = "webapp::" + kNameApp + "::args";
const std::string KEY_WEB_INTERACTION = "webapp::" + kNameApp + "::interaction";

// SUB keys
const std::string KEY_PUB_COMMAND = KEY_PREFIX + "control::pub::command";

// PUB keys
const std::string KEY_PUB_STATUS = KEY_PREFIX + "control::pub::status";

// Controller gains
const std::string KEY_KP_KV_POS   = KEY_PREFIX + "control::kp_kv_pos";
const std::string KEY_KP_KV_ORI   = KEY_PREFIX + "control::kp_kv_ori";
const std::string KEY_KP_KV_JOINT = KEY_PREFIX + "control::kp_kv_joint";

// Controller parameters
const Eigen::Vector3d kEeOffset  = Eigen::Vector3d(0., 0., 0.107) + Eigen::Vector3d(0., 0., 0.1034);
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
const double kMaxErrorPos        = 80 * 0.05;
const double kMaxErrorOri        = 80 * M_PI / 20;
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

// General PD control law
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4>
typename Derived1::PlainObject PdControl(const Eigen::MatrixBase<Derived1>& x,
                                         const Eigen::MatrixBase<Derived2>& x_des,
                                         const Eigen::MatrixBase<Derived3>& dx,
                                         const Eigen::MatrixBase<Derived4>& kp_kv,
                                         double x_err_max = 0.,
                                         typename Derived1::PlainObject* p_x_err = nullptr) {
  static_assert(Derived4::ColsAtCompileTime == 2 ||
                (Derived4::ColsAtCompileTime == 1 && Derived4::RowsAtCompileTime == 2),
                "kp_kv must be a vector of size 2 or a matrix of size x.rows() x 2.");
  typename Derived1::PlainObject x_err;
  x_err = x - x_des;
  if (p_x_err != nullptr) {
    *p_x_err = x_err;
  }
  if (Derived4::ColsAtCompileTime == 1) {
    x_err = -kp_kv(0) * x_err;
  } else {
    x_err = -kp_kv.col(0).array() * x_err.array();
  }
  if (x_err_max > 0. && x_err.norm() > x_err_max) {
    x_err = x_err_max * x_err.normalized();
  }
  if (Derived4::ColsAtCompileTime == 1) {
    return x_err - kp_kv(1) * dx;
  } else {
    return x_err.array() - kp_kv.col(1).array() * dx.array();
  }
}

// Special PD control law for orientation
Eigen::Vector3d PdControl(const Eigen::Quaterniond& quat,
                          const Eigen::Quaterniond& quat_des,
                          Eigen::Ref<const Eigen::Vector3d> w,
                          const Eigen::Vector2d& kp_kv,
                          double ori_err_max = 0.,
                          Eigen::Vector3d* p_ori_err = nullptr) {
  Eigen::Vector3d ori_err = ctrl_utils::OrientationError(quat, quat_des);
  if (p_ori_err != nullptr) {
    *p_ori_err = ori_err;
  }
  ori_err = -kp_kv(0) * ori_err;
  if (ori_err_max > 0. && ori_err.norm() > ori_err_max) {
    ori_err = ori_err_max * ori_err.normalized();
  }
  return ori_err - kp_kv(1) * w;
}

#ifdef USE_WEB_APP
void InitializeWebApp(ctrl_utils::RedisClient& redis_client, const spatial_dyn::ArticulatedBody& ab,
                      const std::string& path_urdf);

std::map<size_t, spatial_dyn::SpatialForced> ComputeExternalForces(const spatial_dyn::ArticulatedBody& ab,
                                                                   const nlohmann::json& interaction);

void AdjustPosition(const std::string& key, Eigen::Vector3d* pos);

void AdjustOrientation(const std::string& key, Eigen::Quaterniond* quat);
#endif  // USE_WEB_APP

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Usage:" << std::endl
              << "\t./franka_panda_opspace <franka_panda.urdf> [--sim]" << std::endl;
    return 0;
  }
  const bool kSim = (argc > 2 && std::string(argv[2]) == "--sim");

  // Create timer and Redis client
  ctrl_utils::Timer timer(kTimerFreq);
  ctrl_utils::RedisClient redis_client;
  redis_client.connect();
  cpp_redis::subscriber redis_sub;
  redis_sub.connect();

  // Load robot
  franka_panda::ArticulatedBody ab = spatial_dyn::urdf::LoadModel(argv[1]);
  if (kSim) {
    ab.set_q(kQHome);
    ab.set_dq(Eigen::VectorXd::Zero(ab.dof()));
  } else {
    ab.set_q(redis_client.sync_get<Eigen::VectorXd>(KEY_SENSOR_Q));
    ab.set_dq(redis_client.sync_get<Eigen::VectorXd>(KEY_SENSOR_DQ));
  }
  // ab.set_inertia_compensation(Eigen::Vector3d(0.2, 0.1, 0.1));
  // ab.set_stiction_coefficients(Eigen::Vector3d(0.8, 0.8, 0.6));
  // ab.set_stiction_activations(Eigen::Vector3d(0.1, 0.1, 0.1));

  // Initialize controller parameters
  Eigen::VectorXd q_des       = kQHome;
  Eigen::Vector3d x_des       = spatial_dyn::Position(ab, -1, kEeOffset);
  Eigen::Quaterniond quat_des = spatial_dyn::Orientation(ab);
  std::mutex mtx_des;

  // Initialize Redis keys

#ifdef USE_WEB_APP
  InitializeWebApp(redis_client, ab, std::string(argv[1]));
#endif  // USE_WEB_APP

  if (kSim) {
    redis_client.set(KEY_SENSOR_Q, ab.q());
    redis_client.set(KEY_SENSOR_DQ, ab.dq());
  }
  redis_client.set(KEY_KP_KV_POS, kKpKvPos);
  redis_client.set(KEY_KP_KV_ORI, kKpKvOri);
  redis_client.set(KEY_KP_KV_JOINT, kKpKvJoint);
  redis_client.sync_commit();

  Eigen::Vector3d x_des_pub       = spatial_dyn::Position(ab, -1, kEeOffset);
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

  try {
    while (g_runloop) {
      // Wait for next loop
      timer.Sleep();

      // Get Redis values
      std::future<Eigen::Matrix32d> fut_kp_kv_pos  = redis_client.get<Eigen::Matrix32d>(KEY_KP_KV_POS);
      std::future<Eigen::Vector2d> fut_kp_kv_ori   = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_ORI);
      std::future<Eigen::Vector2d> fut_kp_kv_joint = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_JOINT);
#ifdef USE_WEB_APP
      std::future<nlohmann::json> fut_interaction = redis_client.get<nlohmann::json>(KEY_WEB_INTERACTION);
#endif  // USE_WEB_APP
      
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

      // Check for PUB commands
      mtx_pub.lock();
      if (is_pub_available) {
        mtx_des.lock();
        x_des    = x_des_pub;
        quat_des = quat_des_pub;
        mtx_des.unlock();
        is_pub_available = false;
        is_pub_waiting = true;
        t_pub = std::chrono::steady_clock::now();
      }
      mtx_pub.unlock();

      // Compute Jacobian
      const Eigen::Matrix6Xd& J = spatial_dyn::Jacobian(ab, -1, kEeOffset);

      // Compute position PD control
      Eigen::Vector3d x_err;
      Eigen::Vector3d x   = spatial_dyn::Position(ab, -1, kEeOffset);
      Eigen::Vector3d dx  = J.topRows<3>() * ab.dq();
      Eigen::Vector3d ddx = PdControl(x, x_des, dx, fut_kp_kv_pos.get(), kMaxErrorPos, &x_err);

      // Compute orientation PD control
      Eigen::Vector3d ori_err;
      Eigen::Quaterniond quat = ctrl_utils::NearQuaternion(spatial_dyn::Orientation(ab), quat_des);
      Eigen::Vector3d w       = J.bottomRows<3>() * ab.dq();
      Eigen::Vector3d dw      = PdControl(quat, quat_des, w, fut_kp_kv_ori.get(), kMaxErrorOri, &ori_err);

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
      static const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ab.dof(), ab.dof());
      Eigen::VectorXd ddq = PdControl(ab.q(), q_des, ab.dq(), fut_kp_kv_joint.get());
      tau_cmd += spatial_dyn::opspace::InverseDynamics(ab, I, ddq, &N);

      // Add friction compensation
      tau_cmd += franka_panda::Friction(ab, tau_cmd);

      // Add gravity compensation
      tau_cmd += spatial_dyn::Gravity(ab);

      // Parse interaction from web app
#ifdef USE_WEB_APP
      nlohmann::json interaction = fut_interaction.get();
      std::string key_down = interaction["key_down"].get<std::string>();
      mtx_des.lock();
      AdjustPosition(key_down, &x_des);
      AdjustOrientation(key_down, &quat_des);
      mtx_des.unlock();
#endif  // USE_WEB_APP

      // Send control torques
      redis_client.sync_set(KEY_CONTROL_TAU, tau_cmd);

      if (kSim) {
        // Integrate
#ifdef USE_WEB_APP
        std::map<size_t, spatial_dyn::SpatialForced> f_ext = ComputeExternalForces(ab, interaction);
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
      redis_client.set(KEY_TRAJ_POS, x);
      redis_client.set(KEY_CONTROL_POS_DES, x_des);
      redis_client.set(KEY_CONTROL_ORI_DES, quat_des.coeffs());
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
  if (!kSim) {
    redis_client.sync_set(KEY_CONTROL_MODE, "floating");
    std::cout << "Cleared torques and set control mode to 'floating'." << std::endl;
  }
  redis_client.sync_set(KEY_CONTROL_TAU, Eigen::VectorXd::Zero(ab.dof()));

  // Print simulation stats
  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;

  return 0;
}

namespace {

#ifdef USE_WEB_APP
void InitializeWebApp(ctrl_utils::RedisClient& redis_client, const spatial_dyn::ArticulatedBody& ab,
                      const std::string& path_urdf) {
  // Register the urdf path so the server knows it's safe to fulfill requests for files in that directory.
  std::string abs_path_urdf = ctrl_utils::AbsolutePath(ctrl_utils::CurrentPath() + "/" + path_urdf);
  redis_client.hset(KEY_WEB_RESOURCES, kNameApp, ctrl_utils::ParentPath(abs_path_urdf));

  // Register key prefixes so the web app knows which models and objects to render.
  nlohmann::json web_keys;
  web_keys["key_models_prefix"]       = KEY_MODELS_PREFIX;
  web_keys["key_objects_prefix"]      = KEY_OBJECTS_PREFIX;
  web_keys["key_trajectories_prefix"] = KEY_TRAJ_PREFIX;
  redis_client.set(KEY_WEB_ARGS, web_keys);

  // Register the robot
  nlohmann::json web_model;
  web_model["model"]    = ab;
  web_model["key_q"]    = KEY_SENSOR_Q;
  redis_client.set(KEY_MODELS_PREFIX + ab.name, web_model);

  // Create a sphere marker for x_des
  nlohmann::json web_object;
  spatial_dyn::Graphics x_des_marker("x_des_marker");
  x_des_marker.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
  x_des_marker.geometry.radius = 0.01;
  web_object["graphics"] = std::vector<spatial_dyn::Graphics>{ x_des_marker };
  web_object["key_pos"]  = KEY_CONTROL_POS_DES;
  redis_client.set(KEY_OBJECTS_PREFIX + x_des_marker.name, web_object);
}

std::map<size_t, spatial_dyn::SpatialForced> ComputeExternalForces(const spatial_dyn::ArticulatedBody& ab,
                                                                   const nlohmann::json& interaction) {
  std::map<size_t, spatial_dyn::SpatialForced> f_ext;

  // Check if the clicked object is the robot
  std::string key_object = interaction["key_object"].get<std::string>();
  if (key_object != KEY_MODELS_PREFIX + ab.name) return f_ext;

  // Extract the json fields
  size_t idx_link = interaction["idx_link"].get<size_t>();
  Eigen::Vector3d pos_mouse = interaction["pos_mouse_in_world"].get<Eigen::Vector3d>();
  Eigen::Vector3d pos_click = interaction["pos_click_in_link"].get<Eigen::Vector3d>();

  // Get the click position in world coordinates
  Eigen::Vector3d pos_click_in_world = spatial_dyn::Position(ab, idx_link, pos_click);

  // Set the click force
  Eigen::Vector3d f = kGainClickDrag * (pos_mouse - pos_click_in_world);
  spatial_dyn::SpatialForced f_click(f, Eigen::Vector3d::Zero());

  // Translate the spatial force to the world frame
  f_ext[idx_link] = Eigen::Translation3d(pos_click_in_world) * f_click;

  return f_ext;
}

void AdjustPosition(const std::string& key, Eigen::Vector3d* pos) {
  if (key.empty() || pos == nullptr) return;

  size_t idx = 0;
  int sign = 1;
  switch (key[0]) {
    case 'a': idx = 0; sign = -1; break;
    case 'd': idx = 0; sign = 1; break;
    case 'w': idx = 1; sign = 1; break;
    case 's': idx = 1; sign = -1; break;
    case 'e': idx = 2; sign = 1; break;
    case 'q': idx = 2; sign = -1; break;
    default: return;
  }
  (*pos)(idx) += sign * kGainKeyPressPos;
}

void AdjustOrientation(const std::string& key, Eigen::Quaterniond* quat) {
  if (key.empty() || quat == nullptr) return;

  size_t idx = 0;
  int sign = 1;
  switch (key[0]) {
    case 'j': idx = 0; sign = -1; break;
    case 'l': idx = 0; sign = 1; break;
    case 'i': idx = 1; sign = 1; break;
    case 'k': idx = 1; sign = -1; break;
    case 'o': idx = 2; sign = 1; break;
    case 'u': idx = 2; sign = -1; break;
    default: return;
  }
  Eigen::AngleAxisd aa(sign * kGainKeyPressOri, Eigen::Vector3d::Unit(idx));
  Eigen::Quaterniond quat_prev = *quat;
  *quat = ctrl_utils::NearQuaternion((aa * quat_prev).normalized(), quat_prev);
}
#endif  // USE_WEB_APP

}  // namespace
