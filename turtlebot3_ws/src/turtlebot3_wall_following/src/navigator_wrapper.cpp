//-----------------------------------------------------------------------------
// navigator_wrapper.cpp
//
// Written by SID: 530504205
//
// Edited and cleaned by SID: 510516950
// Implements NavigatorWrapper and the main() that runs it as the
// turtlebot3_navigator executable.
//-----------------------------------------------------------------------------

#include "navigator_wrapper.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

//---NavigatorWrapper Implementation-------------------------------------------
NavigatorWrapper::NavigatorWrapper()
    : Node("turtlebot3_navigator")
{
    mCmdVelPub = create_publisher<geometry_msgs::msg::TwistStamped>("/nav_cmd_vel", 10);

    // LidarNode publishes /lidar best effort - a reliable subscription would never connect to it
    rclcpp::QoS LidarQos(10);
    LidarQos.best_effort();
    mLidarSub = create_subscription<turtlebot3_lidar_processing::msg::Lidar>(
        "/lidar", LidarQos,
        std::bind(&NavigatorWrapper::LidarCallback, this, std::placeholders::_1));

    mOdomSub = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&NavigatorWrapper::OdomCallback, this, std::placeholders::_1));

    // 50 ms must equal Navigator's ControlPeriod. create_timer() runs on the node
    // clock (sim time if use_sim_time=true), so the period stays correct if Gazebo
    // runs slower than real time
    mUpdateTimer = create_timer(
        std::chrono::milliseconds(50),
        std::bind(&NavigatorWrapper::UpdateCallback, this));

    RCLCPP_INFO(get_logger(), "Turtlebot3 Navigator Node Started");
}

// LidarNode publishes once per scan - every message here is a fresh measurement
// handed straight to the navigator. The navigator pairs it with the pose from the
// last /odom message received, not the pose at the instant of the scan.
// As in OdomCallback, the reading is built before the lock is taken.
void NavigatorWrapper::LidarCallback(const turtlebot3_lidar_processing::msg::Lidar::SharedPtr aMsg) {
    WallFollowerInput Input;
    Input.mFrontDistance = aMsg->forward_wall_distance;
    Input.mRightDistance = aMsg->right_wall_distance;
    Input.mTiltAngle = aMsg->tilt;

    std::lock_guard<std::mutex> Lock(mDataMutex);
    mNavigator.UpdateInput(Input);
}

// Only yaw is kept from the /odom quaternion - the robot is assumed to stay flat.
// The pose is built before the lock is taken so the mutex is held only for the
// hand-over.
void NavigatorWrapper::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr aMsg) {
    Pose OdomPose;
    OdomPose.mX = aMsg->pose.pose.position.x;
    OdomPose.mY = aMsg->pose.pose.position.y;

    tf2::Quaternion Q(
        aMsg->pose.pose.orientation.x, aMsg->pose.pose.orientation.y,
        aMsg->pose.pose.orientation.z, aMsg->pose.pose.orientation.w);
    tf2::Matrix3x3 M(Q);
    double Roll  = 0.0;
    double Pitch = 0.0;
    double Yaw   = 0.0;
    M.getRPY(Roll, Pitch, Yaw);
    OdomPose.mYaw = Yaw;

    std::lock_guard<std::mutex> Lock(mDataMutex);
    mNavigator.UpdatePose(OdomPose);
}

// mutex stops a lidar or odom callback changing the Navigator halfway through a step
void NavigatorWrapper::UpdateCallback() {
    std::lock_guard<std::mutex> Lock(mDataMutex);

    const CommandVelocity Cmd = mNavigator.Navigate();
    PublishCmdVel(Cmd.mLinear, Cmd.mAngular);
}

// wraps the command in a TwistStamped (base_link frame) on /nav_cmd_vel
// the wheel controller clamps it and forwards it to /cmd_vel
void NavigatorWrapper::PublishCmdVel(double aLinear, double aAngular) {
    geometry_msgs::msg::TwistStamped Msg;
    Msg.header.stamp = now();
    Msg.header.frame_id = "base_link";
    Msg.twist.linear.x  = aLinear;
    Msg.twist.angular.z = aAngular;
    mCmdVelPub->publish(Msg);
}

//---Main----------------------------------------------------------------------
// Spins the node on a multi-threaded executor. All three callbacks are in the node's
// default callback group, which is mutually exclusive, so they still run one at a time
// (on whichever thread is free). mDataMutex covers the case where one is later given
// its own callback group.
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto Node = std::make_shared<NavigatorWrapper>();

    rclcpp::executors::MultiThreadedExecutor Executor;
    Executor.add_node(Node);
    Executor.spin();

    rclcpp::shutdown();
    return 0;
}
