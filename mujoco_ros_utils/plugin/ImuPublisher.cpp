#include "ImuPublisher.h"
#include <mujoco/mujoco.h>
#include <iostream>
#include <cstring>
#include <algorithm>

namespace MujocoRosUtils
{

// ---------------- Registration ----------------
void ImuPublisher::RegisterPlugin()
{
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);

    plugin.name = "MujocoRosUtils::ImuPublisher";
    plugin.capabilityflags |= mjPLUGIN_SENSOR;
    
    // stage mjSTAGE_ACC is required for accelerometer data
    plugin.needstage = mjSTAGE_ACC; 

    // Mandatory MuJoCo 3.x callbacks to avoid "nstate is null" errors
    plugin.nstate = +[](const mjModel*, int) { return 0; };
    plugin.nsensordata = +[](const mjModel*, int, int) { return 0; };

    // Plugin Configuration Attributes
    static const char * attributes[] = {"accel_sensor", "gyro_sensor", "body_name", "publish_rate"};
    plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
    plugin.attributes = attributes;

    plugin.init = +[](const mjModel * m, mjData * d, int id) {
        auto * inst = ImuPublisher::Create(m, d, id);
        if(!inst) return -1;
        d->plugin_data[id] = reinterpret_cast<uintptr_t>(inst);
        return 0;
    };

    plugin.destroy = +[](mjData * d, int id) {
        delete reinterpret_cast<ImuPublisher *>(d->plugin_data[id]);
        d->plugin_data[id] = 0;
    };

    plugin.reset = +[](const mjModel * m, double *, void * data, int id) {
        reinterpret_cast<ImuPublisher *>(data)->reset(m, id);
    };

    plugin.compute = +[](const mjModel * m, mjData * d, int id, int) {
        reinterpret_cast<ImuPublisher *>(d->plugin_data[id])->compute(m, d, id);
    };

    mjp_registerPlugin(&plugin);
}

// ---------------- Factory ----------------
ImuPublisher* ImuPublisher::Create(const mjModel * m, mjData * d, int plugin_id)
{
    // 1. Resolve Body Name (Used for Topic and Frame ID)
    const char* b_name_cfg = mj_getPluginConfig(m, plugin_id, "body_name");
    if (!b_name_cfg || strlen(b_name_cfg) == 0) {
        mju_error("[ImuPublisher] `body_name` attribute is required for auto-naming.");
        return nullptr;
    }
    int body_id = mj_name2id(m, mjOBJ_BODY, b_name_cfg);
    if (body_id < 0) {
        mju_error("[ImuPublisher] Body '%s' not found.", b_name_cfg);
        return nullptr;
    }

    // 2. Resolve Sensors from XML names
    const char* accel_name = mj_getPluginConfig(m, plugin_id, "accel_sensor");
    const char* gyro_name = mj_getPluginConfig(m, plugin_id, "gyro_sensor");
    
    int aid = mj_name2id(m, mjOBJ_SENSOR, accel_name);
    int gid = mj_name2id(m, mjOBJ_SENSOR, gyro_name);

    if (aid < 0 || gid < 0) {
        mju_error("[ImuPublisher] Could not find sensors: accel='%s', gyro='%s'", 
                  accel_name, gyro_name);
        return nullptr;
    }

    // 3. Resolve Publish Rate
    const char* rat_cfg = mj_getPluginConfig(m, plugin_id, "publish_rate");
    mjtNum rate = (strlen(rat_cfg) > 0) ? strtod(rat_cfg, nullptr) : 100.0;

    return new ImuPublisher(m, d, aid, gid, body_id, b_name_cfg, rate);
}

// ---------------- Constructor ----------------
ImuPublisher::ImuPublisher(const mjModel * m, mjData * d, int aid, int gid, 
                           int bid, const std::string & bname, mjtNum rate)
: accel_id_(aid), gyro_id_(gid), body_id_(bid), body_name_(bname)
{
    // Auto-define ROS 2 parameters
    topic_name_ = body_name_ + "/imu";
    frame_id_   = body_name_; 

    publish_skip_ = std::max(static_cast<int>(1.0 / (rate * m->opt.timestep)), 1);

    if(!rclcpp::ok()) { rclcpp::init(0, nullptr); }
    
    auto node_name = "imu_node_" + body_name_;
    nh_ = rclcpp::Node::make_shared(node_name);
    pub_ = nh_->create_publisher<sensor_msgs::msg::Imu>(topic_name_, 10);
}

// ---------------- Reset ----------------
void ImuPublisher::reset(const mjModel *, int) 
{ 
    sim_cnt_ = 0; 
}

// ---------------- Compute ----------------
void ImuPublisher::compute(const mjModel * m, mjData * d, int)
{
    sim_cnt_++;
    if(sim_cnt_ % publish_skip_ != 0) return;

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = frame_id_;

    // Linear Acceleration (m/s^2)
    int a_adr = m->sensor_adr[accel_id_];
    msg.linear_acceleration.x = d->sensordata[a_adr + 0];
    msg.linear_acceleration.y = d->sensordata[a_adr + 1];
    msg.linear_acceleration.z = d->sensordata[a_adr + 2];

    // Angular Velocity (rad/s)
    int g_adr = m->sensor_adr[gyro_id_];
    msg.angular_velocity.x = d->sensordata[g_adr + 0];
    msg.angular_velocity.y = d->sensordata[g_adr + 1];
    msg.angular_velocity.z = d->sensordata[g_adr + 2];

    // Orientation (Default to identity if no framequat is provided)
    msg.orientation.w = 1.0; 
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;

    pub_->publish(msg);
}

} // namespace MujocoRosUtils
