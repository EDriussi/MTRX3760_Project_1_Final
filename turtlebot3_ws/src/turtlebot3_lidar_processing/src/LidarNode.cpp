//-----------------------------------------------------------------------------
// LidarNode.cpp
//
// Written by SID: 530506519
//
// Edited and cleaned by SID: 510516950
// Implements LidarNode and the main() that runs it as the lidar executable.
//-----------------------------------------------------------------------------

#include "turtlebot3_lidar_processing/LidarNode.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <functional>

//---LidarNode Implementation--------------------------------------------------

LidarNode::LidarNode()
    : Node("lidar_node")
{

    // Best effort with a queue of the last 10 messages, used for both topics.
    // A best effort subscriber connects to /scan whether it is published reliably or best
    // effort, so the same settings work in simulation and on the robot.
    // /lidar is therefore published best effort too, and its subscriber must ask for the same.
    rclcpp::QoS Qos = rclcpp::QoS(10);
    Qos.history(rclcpp::HistoryPolicy::KeepLast);
    Qos.best_effort();

    mSubscription = create_subscription<sensor_msgs::msg::LaserScan>("scan",
    Qos, std::bind(&LidarNode::ScanCallback, this, std::placeholders::_1));

    mPublisher = create_publisher<turtlebot3_lidar_processing::msg::Lidar>("/lidar", Qos);

}

// Reduces the scan, converts the result to telemetry and publishes it, so one
// message goes out on /lidar for every scan that comes in.
void LidarNode::ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr aMsg)
{
    RCLCPP_DEBUG(get_logger(), "Received message");

    const std::vector<float> Rays = GetRequiredRays( aMsg->ranges, aMsg->angle_min, aMsg->angle_increment,
                                                      aMsg->range_min, aMsg->range_max );

    const WallTelemetry Telemetry = CalculateTelemetry( Rays );

    auto Message = turtlebot3_lidar_processing::msg::Lidar();

    Message.tilt = Telemetry.mTiltAngle;
    Message.forward_wall_distance = Telemetry.mDistanceToFrontWall;
    Message.right_wall_distance = Telemetry.mDistanceToRightWall;

    RCLCPP_DEBUG(get_logger(), "Publishing: Tilt=%.2f, Right Wall Distance=%.2f, Forward Wall Distance=%.2f", Message.tilt, Message.right_wall_distance, Message.forward_wall_distance);
    mPublisher->publish(Message);
}

// Returns { forward distance, RHS closest distance, RHS closest bearing } out of
// a lidar scan message
//
// Forward: the smallest raw ray within about +-15 degrees of straight ahead.
// RHS:     the smallest smoothed ray in the sector centred on -65 degrees with a half
//          width of 85 degrees, i.e. from about -150 to +20 degrees. The sector reaches
//          past straight ahead, so a wall across the robot's path becomes the closest
//          point as the robot nears an inside corner and the bearing swings round to it.
// Bearing: against a flat wall several neighbouring rays are almost equally close, so
//          the bearing is the average angle of every ray within ClusterTolerance of the
//          closest one. This is steadier than the angle of the single closest ray.
// The half widths are truncated to a whole number of rays.
// If nothing valid is in the RHS sector the distance is aRangeMax and the bearing is
// -90 degrees, which CalculateTelemetry turns into zero tilt.
std::vector<float> LidarNode::GetRequiredRays( const std::vector<float>& aRanges, float aAngleMin, float aAngleIncrement,
                                               float aRangeMin, float aRangeMax ) const
{
    // Angles in radians
    constexpr float ForwardAngle = 0.0f;                 // 0 degrees
    constexpr float ForwardHalfWidth = 0.2617994f;       // +- 15 degrees
    constexpr float RightSectorAngle = -1.1344640f;      // -65 degrees
    constexpr float RightSectorHalfWidth = 1.4835299f;   // +- 85 degrees
    constexpr float ClusterTolerance = 0.015f;           // m - range band that sets the bearing
    constexpr float DefaultRHSBearing = -1.5707963f;     // -90 degrees, reported when the RHS sector is empty

    std::vector<float> Rays(NumDesiredRays);

    // Find the index of the ray nearest the centre of each window. When the scan starts at
    // angle_min = 0 the RHS index is negative - the window loops below wrap it into range.
    int ForwardIndex = static_cast<int>( std::lround(( ForwardAngle - aAngleMin ) / aAngleIncrement ));
    int RightIndex = static_cast<int>( std::lround(( RightSectorAngle - aAngleMin ) / aAngleIncrement ));

    // Get number of rays either side of the centre ray
    int ForwardRaysNum = static_cast<int>( ForwardHalfWidth / aAngleIncrement );
    int RightRaysNum = static_cast<int>( RightSectorHalfWidth / aAngleIncrement );

    // Determine the forward ray: smallest raw ray in the forward window
    int FrontMinIndex = GetMinimumRayIndex( aRanges, ForwardIndex, ForwardRaysNum, aRangeMin, aRangeMax );
    Rays[ForwardRayIndex] = ( FrontMinIndex >= 0 ) ? aRanges[FrontMinIndex] : aRangeMax;

    // Determine the RHS ray: smallest smoothed ray in the RHS sector
    std::vector<float> Smoothed = SmoothRays( aRanges, aRangeMin, aRangeMax );
    int ClosestIndex = GetMinimumRayIndex( Smoothed, RightIndex, RightRaysNum, aRangeMin, aRangeMax );

    // Ensure that if nothing is on the RHS, max range is reported
    if ( ClosestIndex < 0 )
    {
        Rays[RightDistanceIndex] = aRangeMax;
        Rays[RightBearingIndex] = DefaultRHSBearing;
        return Rays;
    }

    float ClosestDistance = Smoothed[ClosestIndex];

    // The bearing is the average angle of every ray in the sector within ClusterTolerance
    // of the closest.
    // Unit vectors are summed and atan2 taken because, when the scan starts at
    // angle_min = 0, the sector straddles the 0 / 360 degree seam of the scan. The
    // result comes back in [-pi, pi].
    int NumRays = aRanges.size();
    float SumSin = 0.0f;
    float SumCos = 0.0f;

    for ( int i = -RightRaysNum; i <= RightRaysNum; i++ )
    {
        // Wrapped once in either direction, which assumes the index is never more
        // than one scan length out of range
        int Index = RightIndex + i;
        if ( Index < 0 )
        {
            Index = Index + NumRays;
        }
        if ( Index >= NumRays )
        {
            Index = Index - NumRays;
        }

        if ( Smoothed[Index] < ClosestDistance + ClusterTolerance )
        {
            float RayAngle = aAngleMin + Index * aAngleIncrement;
            SumSin += std::sin( RayAngle );
            SumCos += std::cos( RayAngle );
        }
    }

    Rays[RightDistanceIndex] = ClosestDistance;
    Rays[RightBearingIndex] = std::atan2( SumSin, SumCos );

    return Rays;
}

// Averages each valid ray with its valid neighbours (wrapping around the scan).
// Invalid rays stay invalid (infinity) so GetMinimumRayIndex skips them.
std::vector<float> LidarNode::SmoothRays( const std::vector<float>& aRanges, float aRangeMin, float aRangeMax ) const
{
    constexpr int SmoothHalfWidth = 2;    // rays either side

    int NumRays = aRanges.size();
    std::vector<float> Smoothed( NumRays, std::numeric_limits<float>::infinity() );

    for ( int i = 0; i < NumRays; i++ )
    {
        if ( !std::isfinite( aRanges[i] ) || aRanges[i] < aRangeMin || aRanges[i] > aRangeMax )
        {
            continue;
        }

        float Sum = 0.0f;
        int Count = 0;

        // Count is at least 1 because ray i itself is valid
        for ( int k = -SmoothHalfWidth; k <= SmoothHalfWidth; k++ )
        {
            int Index = ( i + k + NumRays ) % NumRays;
            float CurrentRay = aRanges[Index];

            if ( std::isfinite( CurrentRay ) && CurrentRay >= aRangeMin && CurrentRay <= aRangeMax )
            {
                Sum += CurrentRay;
                Count++;
            }
        }

        Smoothed[i] = Sum / Count;
    }

    return Smoothed;
}

// Returns the index of the smallest valid lidar ray from a search window of rays.
// Filters out invalid rays ( out of range or not finite ).
int LidarNode::GetMinimumRayIndex ( const std::vector<float>& aRanges, int aIndexNum, int aWindowRays, float aRangeMin, float aRangeMax ) const
{

    // Starting at aRangeMax means a ray must be closer than aRangeMax to be chosen
    float MinimumRay = aRangeMax;
    int MinimumIndex = -1;
    int NumRays = aRanges.size();

    for ( int i = -aWindowRays; i <= aWindowRays; i++ )
    {
        // Wrapped once in either direction, which assumes the index is never more
        // than one scan length out of range
        int Index = aIndexNum + i;
        if ( Index < 0 )
        {
            Index = Index + NumRays;
        }
        if (Index >= NumRays )
        {
            Index = Index - NumRays;
        }

        float CurrentRay = aRanges[Index];

        if ( !std::isfinite( CurrentRay ))
        {
            continue;
        }

        if ( CurrentRay < aRangeMin || CurrentRay > aRangeMax )
        {
            continue;
        }

        if ( CurrentRay < MinimumRay )
        {
            MinimumRay = CurrentRay;
            MinimumIndex = Index;
        }

    }

    return MinimumIndex;
}

// Calculates the distance and orientation of any RHS and front wall from the output
// of GetRequiredRays. The Navigator negates the tilt to get its heading error.
LidarNode::WallTelemetry LidarNode::CalculateTelemetry( const std::vector<float>& aRays ) const
{
    constexpr float NinetyDegrees = 1.5707963f;    // rad

    // Find tilt of robot with respect to the wall: 0 when the closest wall point
    // is directly to the right (-90 deg), negative when heading toward the wall
    const float Tilt = -( aRays[RightBearingIndex] + NinetyDegrees );

    WallTelemetry Telemetry;
    Telemetry.mTiltAngle = std::atan2( std::sin( Tilt ), std::cos( Tilt ));

    // Closest point is already the true perpendicular distance for a flat wall
    Telemetry.mDistanceToRightWall = aRays[RightDistanceIndex];
    Telemetry.mDistanceToFrontWall = aRays[ForwardRayIndex];

    return Telemetry;
}

//---Main----------------------------------------------------------------------

int main( int argc, char * argv[] )
{
    rclcpp::init( argc, argv );
    rclcpp::spin(std::make_shared<LidarNode>());
    rclcpp::shutdown();

    return 0;
}
