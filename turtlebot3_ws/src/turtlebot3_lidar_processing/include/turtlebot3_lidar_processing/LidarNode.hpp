//-----------------------------------------------------------------------------
// LidarNode.hpp
//
// Written by SID: 530506519
//
// Edited and cleaned by SID: 510516950
// Declares LidarNode, the ROS 2 node that reduces each raw laser scan to the
// three values the wall follower steers on.
//-----------------------------------------------------------------------------

#ifndef TURTLEBOT3_LIDAR_PROCESSING__LIDAR_NODE_HPP_
#define TURTLEBOT3_LIDAR_PROCESSING__LIDAR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlebot3_lidar_processing/msg/lidar.hpp"

#include <vector>

//---LidarNode Interface-------------------------------------------------------
// The LidarNode subscribes to the robot's lidar scan on /scan.
// For each scan it finds the closest point in a cone of about +-15 degrees
// straight ahead and the closest point in the right-hand sector, and publishes
// those two distances and the robot's tilt relative to the wall as one custom
// Lidar message on /lidar.
// Angles follow the LaserScan convention: 0 is straight ahead and angles increase
// anticlockwise, so the robot's right is -90 degrees.
// Rays that are not finite or are outside [range_min, range_max] are ignored. A
// window with no valid ray reports the scan's range_max.
// The ray indexing wraps around the scan, so a full 360 degree scan is assumed.
class LidarNode : public rclcpp::Node {
    public:

        LidarNode();

    private:

        // Positions in the vector returned by GetRequiredRays
        static constexpr int ForwardRayIndex = 0;
        static constexpr int RightDistanceIndex = 1;
        static constexpr int RightBearingIndex = 2;
        static constexpr int NumDesiredRays = 3;

        // Result of processing one scan - distances in metres, angle in radians.
        // The tilt is 0 when the closest RHS point is directly to the right and
        // negative when the robot is heading toward the wall.
        struct WallTelemetry {
            float mTiltAngle = 0.0f;
            float mDistanceToFrontWall = 0.0f;
            float mDistanceToRightWall = 0.0f;
        };

        // Reduces a full lidar scan to the required values, in this order:
        // { forward distance, RHS closest distance, RHS closest bearing (rad) }
        // The angle and range arguments are the LaserScan fields of the same name
        // (aAngleMin is angle_min, and so on).
        std::vector<float> GetRequiredRays( const std::vector<float>& aRanges,
                                            float aAngleMin, float aAngleIncrement,
                                            float aRangeMin, float aRangeMax) const;

        // Converts the output of GetRequiredRays into the robot's tilt, the distance
        // to the nearest forward wall and the distance to the nearest RHS wall.
        WallTelemetry CalculateTelemetry( const std::vector<float>& aRays ) const;

        // Finds the index of the smallest valid ray within aWindowRays either side of
        // aIndexNum, wrapping around the scan. Returns -1 if no valid ray closer than
        // aRangeMax is found.
        int GetMinimumRayIndex ( const std::vector<float>& aRanges, int aIndexNum,
                                 int aWindowRays, float aRangeMin, float aRangeMax ) const;

        // Returns a copy of the scan in which each valid ray is averaged with its
        // valid neighbours to reduce noise. Invalid rays are returned as infinity.
        std::vector<float> SmoothRays( const std::vector<float>& aRanges,
                                       float aRangeMin, float aRangeMax ) const;

        // Processes and publishes each scan
        void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr aMsg);

        // Publishes the processed telemetry on /lidar
        rclcpp::Publisher<turtlebot3_lidar_processing::msg::Lidar>::SharedPtr mPublisher;

        // Receives the raw scans on /scan
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mSubscription;

};

#endif
