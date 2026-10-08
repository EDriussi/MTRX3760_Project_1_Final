//-----------------------------------------------------------------------------
// navigator.hpp
//
// SID: 510516950
// Declares Navigator, the right-hand wall-following controller, and the plain
// data types passed into and out of it. Nothing in this file depends on ROS.
//-----------------------------------------------------------------------------

#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP

// Robot pose in the odometry frame: x, y in metres, yaw in radians anticlockwise from +x
struct Pose {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

// Body velocity: linear in m/s (+ve forward), angular in rad/s (+ve anticlockwise = left turn)
struct CommandVelocity {
    double linear  = 0.0;
    double angular = 0.0;
};

// What the navigator needs from one lidar scan
// tilt_angle is 0 when the closest right-hand point is square to the right and
// -ve when the robot is heading toward the wall
// The defaults sit beyond WALL_LOST_DISTANCE, so an unset input reads as "no wall"
struct WallFollowerInput {
    double front_distance = 10.0;  // m - closest return in the forward cone
    double right_distance = 10.0;  // m - closest return in the right-hand sector
    double tilt_angle     = 0.0;   // rad
};

//---Navigator Interface-------------------------------------------------------
// Navigator is the right-hand wall follower. It contains no ROS code: its owner
// feeds it odometry poses and lidar readings and asks for a velocity command
// once per control period.
// It steers to keep the closest right-hand point TARGET_DISTANCE away and square
// to the robot's right, which also carries it around wall ends and corners.
// Scans arrive at 5-10 Hz, slower than the control loop, so between scans the
// right-hand distance and heading error are carried forward with odometry.
// front_distance is not carried forward - it keeps its last scanned value.
class Navigator {
    public:
        Navigator() = default;

        // Stores the latest odometry pose. Call on every odometry update.
        void updatePose(const Pose& pose);

        // Stores a new lidar reading. Call once per scan, not once per tick: the
        // pose held at the time of the call is recorded as the pose of the scan.
        void updateInput(const WallFollowerInput& input);

        // Runs one control step and returns the acceleration-limited command.
        // Must be called every CONTROL_PERIOD seconds, because the ramp and the
        // steering filter assume that spacing. Returns zero velocity until the
        // first updateInput().
        CommandVelocity navigate();

    private:
        // Controller mode, chosen afresh on every control tick by getState()
        enum TB3State {
            WAIT_FOR_SCAN,  // no lidar data received yet - hold still
            FOLLOW_WALL,    // tracking the closest point on the right-hand side
            SEARCH_WALL     // nothing within WALL_LOST_DISTANCE - arc right until something appears
        };

        // wall following
        static constexpr double TARGET_DISTANCE    = 0.15;  // m - lidar range to the wall to hold
        static constexpr double K_DIST             = 7.0;   // rad of approach angle per m of error
        static constexpr double MAX_APPROACH_ANGLE = 0.6;   // rad - steepest approach or retreat
        static constexpr double K_ANGLE            = 3.0;   // rad/s per rad of heading error
        static constexpr double WALL_LOST_DISTANCE = 0.80;  // m - wall is lost beyond this
        static constexpr double SEARCH_RADIUS      = 0.50;  // m - arc radius while searching

        // front obstacle handling - the robot never fully stops for a wall ahead,
        // because MIN_LINEAR takes over before the front speed scale reaches zero
        static constexpr double FRONT_SLOW  = 0.45;  // m - start slowing below this
        static constexpr double FRONT_STOP  = 0.15;  // m - front speed scale reaches zero here
        static constexpr double FRONT_TURN  = 0.35;  // m - start adding a left-turn bias below this
        static constexpr double K_FRONT     = 5.0;   // rad/s per metre inside FRONT_TURN

        // limits
        static constexpr double MIN_LINEAR  = 0.03;  // m/s - floor on target speed in FOLLOW_WALL
        static constexpr double MAX_LINEAR  = 0.18;  // m/s - top target speed, and the search speed
        static constexpr double MAX_ANGULAR = 1.80;  // rad/s - clamp on the steering command
        static constexpr double TURN_SLOWDOWN = 0.7;       // fraction of speed shed at MAX_ANGULAR
        static constexpr double ANGULAR_FILTER_TAU = 0.15; // s - steering low-pass time constant

        static constexpr double CONTROL_PERIOD    = 0.05;  // s - required navigate() call period
        static constexpr double MAX_LINEAR_ACCEL  = 1.0;   // m/s^2
        static constexpr double MAX_ANGULAR_ACCEL = 8.0;   // rad/s^2

        // lap counting - distances from the odometry origin, where the robot starts
        static constexpr double START_ZONE_RADIUS = 1.0;   // m - go beyond this to arm the lap
        static constexpr double END_ZONE_RADIUS   = 0.3;   // m - back inside this counts the lap

        // Carries the last reading forward to the current pose. Outputs the
        // estimated wall distance (m) and heading error (rad, +ve = heading
        // toward the wall).
        void estimateWall(double& distance, double& heading_error) const;

        // WAIT_FOR_SCAN before the first reading, SEARCH_WALL when wall_distance
        // is beyond WALL_LOST_DISTANCE, otherwise FOLLOW_WALL
        TB3State getState(double wall_distance) const;

        // Counts a lap each time the robot has been beyond START_ZONE_RADIUS and
        // then comes back within END_ZONE_RADIUS of the odometry origin
        void checkLapCondition();

        // Moves the last command toward target by at most one tick's worth of
        // acceleration and returns the result
        CommandVelocity rampCommand(const CommandVelocity& target);

        // Keeps an angle in [-pi, pi] so differences never jump by 2*pi
        static double wrapAngle(double angle);

        // Readable state name for the state change log
        static const char* stateName(TB3State state);

        // inputs
        WallFollowerInput mInput;       // most recent lidar reading
        Pose mCurrentPose;
        Pose mScanPose;                 // pose held when mInput arrived, used as the scan pose
        bool mHasInput = false;         // false until the first updateInput()

        // state
        TB3State mState = TB3State::WAIT_FOR_SCAN;
        CommandVelocity mLastCmd;       // last command returned - start point of the next ramp step
        double mFilteredAngular = 0.0;  // rad/s - low-pass filtered steering command

        // lap counting
        bool mHasLeftStart = false;     // true once beyond START_ZONE_RADIUS on the current lap
        int  mLapCount     = 0;         // laps completed - reported in the lap log only
};

#endif
