//-----------------------------------------------------------------------------
// LidarNode.hpp
//
// SID: 510516950
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

        // Result of processing one scan - distances in metres, angle in radians.
        // The tilt is 0 when the closest RHS point is directly to the right and
        // negative when the robot is heading toward the wall.
        struct WallTelemetry {
            float mTiltAngle;
            float mDistanceToFrontWall;
            float mDistanceToRightWall;
        };

        LidarNode();

        // Reduces a full lidar scan to the required values, in this order:
        // { forward distance, RHS closest distance, RHS closest bearing (rad) }
        // The angle and range arguments are the fields of the same name in the
        // LaserScan message.
        std::vector<float> GetRequiredRays( const std::vector<float>& ranges,
                                            float angle_min, float angle_increment,
                                            float range_min, float range_max);

        // Converts the output of GetRequiredRays into the robot's tilt, the distance
        // to the nearest forward wall and the distance to the nearest RHS wall. The
        // result is returned and also kept as the node's current telemetry.
        WallTelemetry CalculateTelemetry( const std::vector<float>& Rays );

        // Finds the index of the smallest valid ray within WindowRays either side of
        // IndexNum, wrapping around the scan. Returns -1 if no valid ray closer than
        // range_max is found.
        int GetMinimumRayIndex ( const std::vector<float>& ranges, int IndexNum,
                                 int WindowRays, float range_min, float range_max );

        // Returns a copy of the scan in which each valid ray is averaged with its
        // valid neighbours to reduce noise. Invalid rays are returned as infinity.
        std::vector<float> SmoothRays( const std::vector<float>& ranges,
                                       float range_min, float range_max );

    private:

        // Telemetry calculated from the most recent scan
        WallTelemetry mWallTelemetry;

        // Processes and publishes each scan
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        // Publishes the processed telemetry on /lidar
        rclcpp::Publisher<turtlebot3_lidar_processing::msg::Lidar>::SharedPtr publisher_;

        // Receives the raw scans on /scan
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

};

#endif
