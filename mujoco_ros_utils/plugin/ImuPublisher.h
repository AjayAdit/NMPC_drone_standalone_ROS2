#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

#include <string>

namespace MujocoRosUtils
{

/** \brief Plugin to publish sensor_msgs::msg::Imu data from MuJoCo accel/gyro sensors. */
class ImuPublisher
{
public:
    /** \brief Register plugin with MuJoCo. */
    static void RegisterPlugin();

    /** \brief Factory method to create a plugin instance. */
    static ImuPublisher * Create(const mjModel * m, mjData * d, int plugin_id);

public:
    ImuPublisher(ImuPublisher &&) = default;
    ImuPublisher(const ImuPublisher &) = delete;
    ImuPublisher & operator=(const ImuPublisher &) = delete;

    /** \brief Reset internal state. */
    void reset(const mjModel * m, int plugin_id);

    /** \brief Compute and publish ROS 2 message. */
    void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
    /** \brief Constructor. */
    ImuPublisher(const mjModel * m,
                 mjData * d,
                 int accel_id,
                 int gyro_id,
                 int body_id,
                 const std::string & body_name,
                 mjtNum publish_rate);

protected:
    // MuJoCo IDs
    int accel_id_ = -1;
    int gyro_id_  = -1;
    int body_id_  = -1;
    std::string body_name_;

    // ROS 2
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    std::string topic_name_;
    std::string frame_id_;

    // Throttling
    int publish_skip_ = 0;
    int sim_cnt_ = 0;
};

} // namespace MujocoRosUtils
