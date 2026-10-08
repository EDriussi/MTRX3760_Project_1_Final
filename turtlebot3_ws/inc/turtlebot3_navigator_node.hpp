#ifndef TURTLEBOT3_NAVIGATOR_NODE_HPP
#define TURTLEBOT3_NAVIGATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "inc/navigator.hpp"
#include <mutex>

class Turtlebot3NavigatorNode : public rclcpp::Node {
public:
    Turtlebot3NavigatorNode();
    ~Turtlebot3NavigatorNode() override = default;

private:
    // Callbacks for Partner's Processed Data
    void partnerFrontCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void partnerRightCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void partnerTiltCallback(const std_msgs::msg::Float64::SharedPtr msg);

    // Callbacks for Fallback & Odometry
    void rawScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Control Loop
    void updateCallback();
    void publishCmdVel(double linear, double angular);

    // ROS Interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr partner_front_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr partner_right_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr partner_tilt_sub_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr update_timer_;

    // Logic & State
    Navigator navigator_;
    WallFollowerInput mInput;
    rclcpp::Time last_processed_update_;
    
    std::mutex data_mutex_;
};

#endif // TURTLEBOT3_NAVIGATOR_NODE_HPP