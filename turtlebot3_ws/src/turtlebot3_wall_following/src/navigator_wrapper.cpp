//-----------------------------------------------------------------------------
// navigator_wrapper.cpp
//
// SID: 510516950
// Implements NavigatorWrapper and the main() that runs it as the
// turtlebot3_navigator executable.
//-----------------------------------------------------------------------------

#include "navigator_wrapper.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//---NavigatorWrapper Implementation-------------------------------------------
NavigatorWrapper::NavigatorWrapper()
    : Node("turtlebot3_navigator_node")
{
    last_processed_update_ = this->now();

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/nav_cmd_vel", 10);

    // LidarNode publishes /lidar best effort - a reliable subscription would never connect to it
    rclcpp::QoS lidar_qos(10);
    lidar_qos.best_effort();
    partner_lidar_sub_ = create_subscription<turtlebot3_lidar_processing::msg::Lidar>(
        "/lidar", lidar_qos,
        std::bind(&NavigatorWrapper::ProcessedLidarCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&NavigatorWrapper::odomCallback, this, std::placeholders::_1));

    // 50 ms must equal Navigator's CONTROL_PERIOD. create_timer() runs on the node
    // clock (sim time if use_sim_time=true), so the period stays correct if Gazebo
    // runs slower than real time
    update_timer_ = create_timer(
        std::chrono::milliseconds(50),
        std::bind(&NavigatorWrapper::updateCallback, this));

    RCLCPP_INFO(get_logger(), "Turtlebot3 Navigator Node Started (with Fallback Active)");
}

// LidarNode publishes once per scan - every message here is a fresh measurement
// handed straight to the navigator. The navigator pairs it with the pose from the
// last /odom message received, not the pose at the instant of the scan.
void NavigatorWrapper::ProcessedLidarCallback(const turtlebot3_lidar_processing::msg::Lidar::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    mInput.front_distance = msg->forward_wall_distance;
    mInput.right_distance = msg->right_wall_distance;
    mInput.tilt_angle = msg->tilt;
    navigator_.updateInput(mInput);
    last_processed_update_ = this->now();
}

// Only yaw is kept from the /odom quaternion - the robot is assumed to stay flat.
// The pose is built before the lock is taken so the mutex is held only for the
// hand-over.
void NavigatorWrapper::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    Pose pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose.yaw = yaw;

    std::lock_guard<std::mutex> lock(data_mutex_);
    navigator_.updatePose(pose);
}

// mutex stops a lidar or odom callback changing the Navigator halfway through a step
void NavigatorWrapper::updateCallback() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    CommandVelocity cmd = navigator_.navigate();
    publishCmdVel(cmd.linear, cmd.angular);
}

// wraps the command in a TwistStamped (base_link frame) on /nav_cmd_vel
// the wheel controller clamps it and forwards it to /cmd_vel
void NavigatorWrapper::publishCmdVel(double linear, double angular) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x  = linear;
    msg.twist.angular.z = angular;
    cmd_vel_pub_->publish(msg);
}

//---Main----------------------------------------------------------------------
// Spins the node on a multi-threaded executor. All three callbacks are in the node's
// default callback group, which is mutually exclusive, so they still run one at a time
// (on whichever thread is free). data_mutex_ covers the case where one is later given
// its own callback group.
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigatorWrapper>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
