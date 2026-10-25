#include "WrenchToActuators.h"
#include <mujoco/mujoco.h>
#include <cstring>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>

namespace MujocoRosUtils
{

// ---------- helpers ----------
static inline double parse_double(const char* s, double fallback) {
  if (!s || *s == '\0') return fallback;
  char* end = nullptr; errno = 0;
  const double v = std::strtod(s, &end);
  if (end == s || errno == ERANGE) return fallback;
  return v;
}

static inline int find_actuator(const mjModel * m, const char * name)
{
  if(!name || std::strlen(name)==0) return -1;
  int id = mj_name2id(m, mjOBJ_ACTUATOR, name);
  if (id >= 0) return id;
  for(int i=0; i<m->nu; ++i)
    if(std::strcmp(name, mj_id2name(m, mjOBJ_ACTUATOR, i)) == 0) return i;
  return -1;
}


// ---------- registration ----------
void WrenchToActuators::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::WrenchToActuators";
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  // Only the essentials. No desired-state attributes here.
  const char * attributes[] = {
    "topic_name",
    "actuator_fz", "actuator_tx", "actuator_ty", "actuator_tz",
    "control_hz",
    "body_name"    // optional; default = 0
  };
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, int){ return 0; };
  plugin.nsensordata = +[](const mjModel *, int, int){ return 0; };
  plugin.needstage = mjSTAGE_VEL;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    auto * plugin_instance = WrenchToActuators::Create(m, d, plugin_id);
    if(!plugin_instance) return -1;
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<WrenchToActuators *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, void * plugin_data, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<WrenchToActuators *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int)
  {
    auto * plugin_instance = reinterpret_cast<WrenchToActuators *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

// ---------------- Factory ----------------
WrenchToActuators * WrenchToActuators::Create(const mjModel * m, mjData * d, int plugin_id)
{
    // 1. Find the actuator index that is using this plugin instance
    int actuator_id = -1;
    for (int i = 0; i < m->nu; ++i) {
        if (m->actuator_plugin[i] == plugin_id) {
            actuator_id = i;
            break;
        }
    }

    // 2. Identify the joint name from the actuator transmission
    std::string joint_name = "unnamed_joint";
    if (actuator_id != -1) {
        // trnid[actuator_id * 2] contains the ID of the joint/object it acts on
        int joint_id = m->actuator_trnid[actuator_id * 2]; 
        const char* j_name = mj_id2name(m, mjOBJ_JOINT, joint_id);
        if (j_name) {
            joint_name = std::string(j_name);
        }
    }

    // 3. Handle topic_name: Use XML config if provided, otherwise auto-generate
    std::string topic_name;
    if (const char * p = mj_getPluginConfig(m, plugin_id, "topic_name"); p && std::strlen(p) > 0) {
        topic_name = p;
    } else {
        // AUTOMATION: joint_name + "/position_cmd"
        topic_name = joint_name + "/position_cmd";
    }

    // [Keep your existing actuator lookup logic for Fz, Tx, Ty, Tz...]
    const char * aFz = mj_getPluginConfig(m, plugin_id, "actuator_fz");
    const char * aTx = mj_getPluginConfig(m, plugin_id, "actuator_tx");
    const char * aTy = mj_getPluginConfig(m, plugin_id, "actuator_ty");
    const char * aTz = mj_getPluginConfig(m, plugin_id, "actuator_tz");

    int id_fz = find_actuator(m, aFz);
    int id_tx = find_actuator(m, aTx);
    int id_ty = find_actuator(m, aTy);
    int id_tz = find_actuator(m, aTz);

    if(id_fz < 0 || id_tx < 0 || id_ty < 0 || id_tz < 0) {
        mju_error("[WrenchToActuators] one or more actuator names not found.");
        return nullptr;
    }

    double control_hz = parse_double(mj_getPluginConfig(m, plugin_id, "control_hz"), 500.0);

    // 4. Body name handling
    int body_id = 0;
    std::string body_name;
    if (const char * bn = mj_getPluginConfig(m, plugin_id, "body_name"); bn && std::strlen(bn) > 0) {
        body_id = mj_name2id(m, mjOBJ_BODY, bn);
        body_name = bn;
    } else {
        // Default to the joint's parent body if not specified
        body_id = m->jnt_bodyid[m->actuator_trnid[actuator_id * 2]];
        body_name = mj_id2name(m, mjOBJ_BODY, body_id);
    }

    return new WrenchToActuators(m, d, id_fz, id_tx, id_ty, id_tz,
                                 topic_name, control_hz, body_id, body_name);
}

// ---------------- Ctor ----------------
WrenchToActuators::WrenchToActuators(const mjModel * m, mjData * d,
                                     int id_fz, int id_tx, int id_ty, int id_tz,
                                     std::string topic_name, double control_hz,
                                     int body_id, std::string body_name)
: id_fz_(id_fz), id_tx_(id_tx), id_ty_(id_ty), id_tz_(id_tz),
  body_id_(body_id), body_name_(std::move(body_name))
{
    ctrl_dt_ = 1.0 / control_hz;
    control_hz_ = control_hz;
    
    // Ensure ROS is initialized
    if(!rclcpp::ok()) { 
        int argc = 0; char ** argv = nullptr;
        rclcpp::init(argc, argv); 
    }

    // Make the node name unique based on the body to avoid global namespace clashes
    rclcpp::NodeOptions node_options;
    auto node_name = std::string("mujoco_wrench_ctrl_") + body_name_;
    nh_ = rclcpp::Node::make_shared(node_name, node_options);

    // Subscribe to the automated topic
    sub_position_cmd_ = nh_->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        topic_name, 10, std::bind(&WrenchToActuators::callback, this, std::placeholders::_1));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(nh_);

    std::cout << "[WrenchToActuators] Initialized Body: " << body_name_ 
              << " Subscribed to: " << topic_name << std::endl;
}
// ---------- reset ----------
void WrenchToActuators::reset(const mjModel *, int)
{
  next_ctrl_time_   = 0.0;
  next_report_time_ = 0.0;
  window_start_t_   = 0.0;
  window_ticks_     = 0;
  max_abs_jitter_   = 0.0;

  // keep refs; clear external cmd
  Fx_ = Fy_ = Fz_ = Tx_ = Ty_ = Tz_ = std::numeric_limits<mjtNum>::quiet_NaN();
}

// ---------- ROS callback ----------
void WrenchToActuators::callback(const quadrotor_msgs::msg::PositionCommand msg)
{
  xd_ << msg.position.x,  msg.position.y, msg.position.z;
  vd_ << msg.velocity.x,  msg.velocity.y, msg.velocity.z;
  ad_ << msg.acceleration.x,  msg.acceleration.y, msg.acceleration.z;
  //Fx_ = static_cast<mjtNum>(msg->force.x);
  //Fy_ = static_cast<mjtNum>(msg->force.y);
  //Fz_ = static_cast<mjtNum>(msg->force.z);
  //Tx_ = static_cast<mjtNum>(msg->torque.x);
  //Ty_ = static_cast<mjtNum>(msg->torque.y);
  //Tz_ = static_cast<mjtNum>(msg->torque.z);
}

// ---------- main step ----------
void WrenchToActuators::compute(const mjModel * m, mjData * d, int)
{
  // Poll ROS non-blocking
  if (executor_) executor_->spin_once(std::chrono::seconds(0));

  // Fixed-rate cadence in sim time
  const double t = d->time;
  if(next_ctrl_time_ == 0.0) {
    // Init Position
    const mjtNum* p = d->xpos + 3*body_id_;
    xd_ << (double)p[0], (double)p[1], (double)p[2];
    next_ctrl_time_   = t;
    next_report_time_ = t + report_period_;
    window_start_t_   = t;

  }

  // Run controller at the scheduled instants (catch up if sim dt jumps)
  while (t + 1e-12 >= next_ctrl_time_)
  {
    // diagnostics
    const double lateness = t - next_ctrl_time_;
    const double absjit   = std::abs(lateness);
    if (absjit > max_abs_jitter_) max_abs_jitter_ = absjit;
    ++window_ticks_;

    const double* xpos  = d->xpos  + 3*body_id_;      // world position
    const double* xquat = d->xquat + 4*body_id_;      // world orientation (wxyz)
    mjtNum vel[6];                                    // [wx wy wz vx vy vz] (world if local=0)
    mj_objectVelocity(m, d, mjOBJ_XBODY, body_id_, vel, 0);

    // Update states
    x_ << xpos[0], xpos[1], xpos[2];
    v_ << static_cast<double>(vel[3]), static_cast<double>(vel[4]), static_cast<double>(vel[5]);
    q_ <<  xquat[0],  xquat[1],  xquat[2],  xquat[3];
    Eigen::Matrix3d R  = quatToRot(q_);

    Eigen::Matrix<double,3,1> wI =
    (Eigen::Matrix<double,3,1>() <<
         static_cast<double>(vel[0]), static_cast<double>(vel[1]), static_cast<double>(vel[2])  
    ).finished();
    w_ = R.transpose()*wI;

    Eigen::Matrix<double,3,1> mu = KP_*(xd_ - x_) + KV_*(vd_ - v_) + ad_ ;
    Eigen::Matrix<double,3,1> force =  mass_ * (mu + g_*ez_);

    // Compute Rotation Matrix
    Eigen::Vector3d Zb   = (R * ez_);

    // Force in body frame
    const double f = Zb.dot(force);

     // compute desired Rotation matrix
    Eigen::Vector3d Zb_d   = force/force.norm();
    Eigen::Vector3d Xc_d(std::cos(psid_), std::sin(psid_), 0.0);

    Eigen::Vector3d cross_z_x = Zb_d.cross(Xc_d);
    double ny = cross_z_x.norm();
    Eigen::Vector3d Yb_d = cross_z_x/ny;

    Eigen::Vector3d Xb_d = Yb_d.cross(Zb_d);

    // Desired Orientation matrix
    Eigen::Matrix3d R_d;
    R_d.col(0) = Xb_d;
    R_d.col(1) = Yb_d;
    R_d.col(2) = Zb_d;

    // Rotation error
    Eigen::Matrix3d S_err = R_d.transpose()*R - R.transpose()*R_d;
    Eigen::Vector3d e_R = 0.5 * vee(S_err);

    // Angular velocity error e_ω = ω − Rᵀ R_d ω_d
    wd_ << 0.0, 0.0, 0.0;
    Eigen::Vector3d w_ref_in_body = R.transpose() * R_d * wd_;
    Eigen::Vector3d e_omega = w_ - w_ref_in_body;

    // Coriolis/gyroscopic term: c = ω × (I ω)
    Eigen::Vector3d Iw = J_ * w_;
    Eigen::Vector3d c = w_.cross(Iw);

    // Moments: M = c − I (K_R e_R) − I (K_ω e_ω)
    Eigen::Vector3d M =
      c - J_ * (KQ_ * e_R) - J_ * (KW_ * e_omega);


    // cache for printing
    last_pos_[0]  = xpos[0]; last_pos_[1]  = xpos[1]; last_pos_[2]  = xpos[2];
    last_quat_[0] = xquat[0]; last_quat_[1] = xquat[1]; last_quat_[2] = xquat[2]; last_quat_[3] = xquat[3];
    last_w_[0]    = static_cast<double>(vel[0]); last_w_[1] = static_cast<double>(vel[1]); last_w_[2] = static_cast<double>(vel[2]);
    last_v_[0]    = static_cast<double>(vel[3]); last_v_[1] = static_cast<double>(vel[4]); last_v_[2] = static_cast<double>(vel[5]);

    d->ctrl[id_fz_] = f;
    d->ctrl[id_tx_] = M(0);
    d->ctrl[id_ty_] = M(1);
    d->ctrl[id_tz_] = M(2);
    next_ctrl_time_ += ctrl_dt_;
  }

    // diagnostics (once per sim-second)
    if (t + 1e-12 >= next_report_time_) {
    //const double dt_win = std::max(1e-12, t - window_start_t_);
    //const double hz_obs = static_cast<double>(window_ticks_) / dt_win;

    //std::printf("[WrenchToActuators] t=%.3f  target=%.3f Hz  observed=%.3f Hz "
    //            "(ticks=%llu over %.3f s)  max|jitter|=%.6f s  (body=%s)\n",
    //            t, control_hz_, hz_obs,
    //            static_cast<unsigned long long>(window_ticks_),
    //            dt_win, max_abs_jitter_, body_name_.c_str());

    // state printout
    //std::printf("  pos = [%.6f %.6f %.6f]\n", last_pos_[0], last_pos_[1], last_pos_[2]);
    //std::printf("  quat(wxyz) = [%.6f %.6f %.6f %.6f]\n",
    //            last_quat_[0], last_quat_[1], last_quat_[2], last_quat_[3]);
    //std::printf("  vel: lin = [%.6f %.6f %.6f]  ang = [%.6f %.6f %.6f]\n",
    //            last_v_[0], last_v_[1], last_v_[2], last_w_[0], last_w_[1], last_w_[2]);

    // reset window
    window_start_t_ = t;
    window_ticks_   = 0;
    max_abs_jitter_ = 0.0;
    next_report_time_ += report_period_;
    }
}
Eigen::Matrix3d WrenchToActuators::quatToRot(const Eigen::Vector4d& qvec)
{
  // q = [w, x, y, z]
  const double q0 = qvec(0);
  const double q1 = qvec(1);
  const double q2 = qvec(2);
  const double q3 = qvec(3);

  const double q0q0 = q0*q0;
  const double q1q1 = q1*q1;
  const double q2q2 = q2*q2;
  const double q3q3 = q3*q3;

  Eigen::Matrix3d R;
  R <<
      q0q0 + q1q1 - q2q2 - q3q3,  2.0*(q1*q2 - q0*q3),  2.0*(q1*q3 + q0*q2),
      2.0*(q1*q2 + q0*q3),        q0q0 + q2q2 - q1q1 - q3q3,  2.0*(q2*q3 - q0*q1),
      2.0*(q1*q3 - q0*q2),        2.0*(q2*q3 + q0*q1),        q0q0 + q3q3 - q1q1 - q2q2;

  return R;
}

Eigen::Vector3d WrenchToActuators::vee(const Eigen::Matrix3d& S)
{
  // S must be skew-symmetric; this picks the independent elements.
  return Eigen::Vector3d(S(2,1), S(0,2), S(1,0));
}
} // namespace MujocoRosUtils
