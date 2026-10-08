//-----------------------------------------------------------------------------
// WheelController.cpp
//
// Written by SID: 530489485
//
// Edited and cleaned by SID: 510516950
// Implements WheelController and the main() that runs it as the
// wheel_controller_node executable.
//-----------------------------------------------------------------------------

#include "WheelController.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>


//---WheelController Implementation--------------------------------------------

WheelController::WheelController()
    : Node("wheel_controller")
{
    // Select the velocity message type expected by the robot bringup.
    // The default is TwistStamped; set the parameter false for plain Twist.
    mUseStampedCommand =
        declare_parameter<bool>(
            "use_stamped_cmd_vel",
            true);

    // Only the publisher for the selected type is created. The other stays
    // null, so SetWheelSpeed must branch on mUseStampedCommand the same way.
    if (mUseStampedCommand)
    {
        mVelocityPublisher =
            create_publisher<geometry_msgs::msg::TwistStamped>(
                "/cmd_vel",
                10);
    }
    else
    {
        mUnstampedVelocityPublisher =
            create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10);
    }

    mCommandSubscription =
        create_subscription<geometry_msgs::msg::TwistStamped>(
            "/nav_cmd_vel",
            10,
            std::bind(
                &WheelController::CommandCallback,
                this,
                std::placeholders::_1));

    // Check every 100 ms using the node clock. With use_sim_time enabled,
    // the watchdog timeout follows simulation time.
    mWatchdogTimer =
        create_timer(
            std::chrono::milliseconds(100),
            std::bind(
                &WheelController::WatchdogCallback,
                this));

    mLastCommandTime = get_clock()->now();

    RCLCPP_INFO(
        get_logger(),
        "Wheel controller started: /nav_cmd_vel -> /cmd_vel (%s)",
        mUseStampedCommand ? "TwistStamped" : "Twist");
}

//---Navigator command to robot------------------------------------------------
// Record command arrival and forward the requested body velocity.
// Any command also clears a previous watchdog timeout.

void WheelController::CommandCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr
        aCommand)
{
    mLastCommandTime = get_clock()->now();
    mHasCommand = true;

    if (mTimedOut)
    {
        RCLCPP_INFO(
            get_logger(),
            "Navigator commands resumed.");

        mTimedOut = false;
    }

    SetWheelSpeed(
        aCommand->twist.linear.x,
        aCommand->twist.angular.z);
}

//---Watchdog------------------------------------------------------------------
// Publish a stop command once when the navigator commands time out.
// A new command clears the timeout state and resumes normal forwarding.
// Nothing is published before the first command arrives. The timer runs every
// 100 ms, so the stop can come up to 100 ms after CommandTimeout has passed.

void WheelController::WatchdogCallback()
{
    if (mHasCommand && !mTimedOut)
    {
        const double TimeSinceLastCommand =
            (get_clock()->now() - mLastCommandTime).seconds();

        if (TimeSinceLastCommand > CommandTimeout)
        {
            RCLCPP_WARN(
                get_logger(),
                "No navigator command for %.1f s - stopping wheels.",
                CommandTimeout);

            SetWheelSpeed(0.0, 0.0);
            mTimedOut = true;
        }
    }
}

//---Wheel speed---------------------------------------------------------------
// Clamp the requested linear and angular velocities then publish them
// using the configured velocity message type. Only linear.x and angular.z
// are set; the other components keep their zero defaults.

void WheelController::SetWheelSpeed(
    double aLinear,
    double aAngular)
{
    const double Linear =
        std::clamp(aLinear, -MaximumLinearSpeed, MaximumLinearSpeed);

    const double Angular =
        std::clamp(aAngular, -MaximumAngularSpeed, MaximumAngularSpeed);

    if (mUseStampedCommand)
    {
        geometry_msgs::msg::TwistStamped Command;

        Command.header.stamp = get_clock()->now();
        Command.header.frame_id = "base_link";
        Command.twist.linear.x = Linear;
        Command.twist.angular.z = Angular;

        mVelocityPublisher->publish(Command);
    }
    else
    {
        geometry_msgs::msg::Twist Command;

        Command.linear.x = Linear;
        Command.angular.z = Angular;

        mUnstampedVelocityPublisher->publish(Command);
    }
}

//---Main----------------------------------------------------------------------

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    const std::shared_ptr<WheelController> Controller =
        std::make_shared<WheelController>();

    rclcpp::spin(Controller);
    rclcpp::shutdown();

    return 0;
}
