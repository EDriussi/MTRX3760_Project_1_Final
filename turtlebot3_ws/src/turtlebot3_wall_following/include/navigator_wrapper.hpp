//-----------------------------------------------------------------------------
// navigator_wrapper.hpp
//
// SID: 510516950
// Declares NavigatorWrapper, the ROS 2 node that connects a Navigator to the
// rest of the system.
//-----------------------------------------------------------------------------

#ifndef NAVIGATOR_WRAPPER_HPP
#define NAVIGATOR_WRAPPER_HPP

#include "navigator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <turtlebot3_lidar_processing/msg/lidar.hpp>

#include <mutex>

//---NavigatorWrapper Interface------------------------------------------------
// NavigatorWrapper is the ROS 2 node around a mutex-guarded Navigator.
// It holds no navigation logic - it converts ROS messages into Navigator inputs,
// steps the Navigator on a 20 Hz timer and publishes the command it returns.
// Subscribes - /lidar (processed wall telemetry from LidarNode), /odom (robot pose)
// Publishes - /nav_cmd_vel (TwistStamped) for the wheel controller
class NavigatorWrapper : public rclcpp::Node {
    public:
        NavigatorWrapper();
        ~NavigatorWrapper() override = default;

    private:
        // passes each /lidar message to the navigator as a new reading
        void lidarCallback(const turtlebot3_lidar_processing::msg::Lidar::SharedPtr msg);

        // reduces each /odom message to an x, y, yaw pose for the navigator
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        // 20 Hz control loop on the node clock (sim time in Gazebo - real time on the robot)
        void updateCallback();

        // publishes one command (m/s, rad/s) on /nav_cmd_vel as a TwistStamped
        void publishCmdVel(double linear, double angular);

        // ROS interfaces
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

        rclcpp::Subscription<turtlebot3_lidar_processing::msg::Lidar>::SharedPtr lidar_sub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        rclcpp::TimerBase::SharedPtr update_timer_;

        // logic and state
        Navigator navigator_;

        // held by every callback while it touches navigator_
        std::mutex data_mutex_;
};

#endif
