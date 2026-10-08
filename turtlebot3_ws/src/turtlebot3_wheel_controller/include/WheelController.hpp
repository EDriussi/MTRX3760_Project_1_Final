//-----------------------------------------------------------------------------
// WheelController.hpp
//
// SID: 510516950
// Declares WheelController, the node that is the final connection between the
// navigation system and the TurtleBot3.
//
// Pipeline:  /scan -> lidar_node -> /lidar -> turtlebot3_navigator
//                  -> /nav_cmd_vel -> wheel_controller -> /cmd_vel -> robot
// (node names as given in the launch files; turtlebot3_navigator also reads /odom)
//-----------------------------------------------------------------------------

#ifndef WHEEL_CONTROLLER_HPP
#define WHEEL_CONTROLLER_HPP

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"


//---WheelController Interface-------------------------------------------------
// The node receives the navigator's velocity commands on /nav_cmd_vel,
// clamps them to the TurtleBot3 Burger's limits and publishes them on
// /cmd_vel. This node is intended to be the sole publisher of /cmd_vel.
// If the commands time out, a watchdog publishes a stop command.
// The use_stamped_cmd_vel parameter selects whether /cmd_vel carries
// TwistStamped (default) or plain Twist, to match the robot bringup.
class WheelController : public rclcpp::Node
{
    public:
        WheelController();

    private:
        // TurtleBot3 Burger speed limits.
        static constexpr double MaximumLinearSpeed = 0.22;  // m/s
        static constexpr double MaximumAngularSpeed = 2.84; // rad/s

        // Stop the wheels if no command has arrived for this long, in seconds.
        static constexpr double CommandTimeout = 0.5;

        // Called whenever the navigator sends a movement command.
        // Records its arrival and forwards its linear.x and angular.z.
        void CommandCallback(
            const geometry_msgs::msg::TwistStamped::SharedPtr
                aCommand);

        // Publishes a stop command when navigator commands time out.
        void WatchdogCallback();

        // Publish the requested velocity (m/s, rad/s), clamped to the
        // robot's limits.
        void SetWheelSpeed(
            double aLinear,
            double aAngular);

        // Publishes on /cmd_vel as TwistStamped. Only created when
        // use_stamped_cmd_vel is true, otherwise left null.
        rclcpp::Publisher<
            geometry_msgs::msg::TwistStamped>::SharedPtr
            mVelocityPublisher;

        // Publishes on /cmd_vel as plain Twist. Only created when
        // use_stamped_cmd_vel is false, otherwise left null.
        rclcpp::Publisher<
            geometry_msgs::msg::Twist>::SharedPtr
            mUnstampedVelocityPublisher;

        // Value of the use_stamped_cmd_vel parameter, read once in the
        // constructor (default true).
        bool mUseStampedCommand = true;

        // Receives movement commands from the navigator on /nav_cmd_vel.
        rclcpp::Subscription<
            geometry_msgs::msg::TwistStamped>::SharedPtr
            mCommandSubscription;

        // Calls WatchdogCallback every 100 ms.
        rclcpp::TimerBase::SharedPtr mWatchdogTimer;

        // Node clock time the last navigator command was received.
        rclcpp::Time mLastCommandTime;

        // True once the first navigator command has been received.
        // The watchdog does nothing before then.
        bool mHasCommand = false;

        // True after the watchdog sends a stop command.
        // Resets when a new navigator command arrives.
        bool mTimedOut = false;
};

#endif
