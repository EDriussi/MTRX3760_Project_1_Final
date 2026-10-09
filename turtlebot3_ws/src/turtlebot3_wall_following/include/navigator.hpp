//-----------------------------------------------------------------------------
// navigator.hpp
//
// Written by SID: 530478283
//
// Edited and cleaned by SID: 510516950
// Declares Navigator, the right-hand wall-following controller, and the plain
// data types passed into and out of it. Nothing in this file depends on ROS.
//-----------------------------------------------------------------------------

#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP

// Robot pose in the odometry frame: mX, mY in metres, mYaw in radians anticlockwise from +x
struct Pose {
    double mX   = 0.0;
    double mY   = 0.0;
    double mYaw = 0.0;
};

// Body velocity: mLinear in m/s (+ve forward), mAngular in rad/s (+ve anticlockwise = left turn)
struct CommandVelocity {
    double mLinear  = 0.0;
    double mAngular = 0.0;
};

// What the navigator needs from one lidar scan
// mTiltAngle is 0 when the closest right-hand point is square to the right and
// -ve when the robot is heading toward the wall
// The defaults sit beyond WallLostDistance, so an unset input reads as "no wall"
struct WallFollowerInput {
    double mFrontDistance = 10.0;  // m - closest return in the forward cone
    double mRightDistance = 10.0;  // m - closest return in the right-hand sector
    double mTiltAngle     = 0.0;   // rad
};

//---Navigator Interface-------------------------------------------------------
// Navigator is the right-hand wall follower. It contains no ROS code: its owner
// feeds it odometry poses and lidar readings and asks for a velocity command
// once per control period.
// It steers to keep the closest right-hand point TargetDistance away and square
// to the robot's right, which also carries it around wall ends and corners.
// Scans arrive at 5-10 Hz, slower than the control loop, so between scans the
// right-hand distance and heading error are carried forward with odometry.
// mFrontDistance is not carried forward - it keeps its last scanned value.
class Navigator {
    public:
        Navigator() = default;

        // Stores the latest odometry pose. Call on every odometry update.
        void UpdatePose(const Pose& aPose);

        // Stores a new lidar reading. Call once per scan, not once per tick: the
        // pose held at the time of the call is recorded as the pose of the scan.
        void UpdateInput(const WallFollowerInput& aInput);

        // Runs one control step and returns the acceleration-limited command.
        // Must be called every ControlPeriod seconds, because the ramp and the
        // steering filter assume that spacing. Returns zero velocity until the
        // first UpdateInput().
        CommandVelocity Navigate();

    private:
        // Controller mode, chosen afresh on every control tick by GetState()
        enum TB3State {
            WaitForScan,  // no lidar data received yet - hold still
            FollowWall,   // tracking the closest point on the right-hand side
            SearchWall    // nothing within WallLostDistance - arc right until something appears
        };

        // wall following
        static constexpr double TargetDistance   = 0.15;  // m - lidar range to the wall to hold
        static constexpr double KDist            = 7.0;   // rad of approach angle per m of error
        static constexpr double MaxApproachAngle = 0.6;   // rad - steepest approach or retreat
        static constexpr double KAngle           = 3.0;   // rad/s per rad of heading error
        static constexpr double WallLostDistance = 0.80;  // m - wall is lost beyond this
        static constexpr double SearchRadius     = 0.50;  // m - arc radius while searching

        // front obstacle handling - the robot never fully stops for a wall ahead,
        // because MinLinear takes over before the front speed scale reaches zero
        static constexpr double FrontSlow  = 0.45;  // m - start slowing below this
        static constexpr double FrontStop  = 0.15;  // m - front speed scale reaches zero here
        static constexpr double FrontTurn  = 0.35;  // m - start adding a left-turn bias below this
        static constexpr double KFront     = 5.0;   // rad/s per metre inside FrontTurn

        // limits
        static constexpr double MinLinear  = 0.03;  // m/s - floor on target speed in FollowWall
        static constexpr double MaxLinear  = 0.18;  // m/s - top target speed, and the search speed
        static constexpr double MaxAngular = 1.80;  // rad/s - clamp on the steering command
        static constexpr double TurnSlowdown = 0.7;      // fraction of speed shed at MaxAngular
        static constexpr double AngularFilterTau = 0.15; // s - steering low-pass time constant

        static constexpr double ControlPeriod   = 0.05;  // s - required Navigate() call period
        static constexpr double MaxLinearAccel  = 1.0;   // m/s^2
        static constexpr double MaxAngularAccel = 8.0;   // rad/s^2

        // lap counting - distances from the odometry origin, where the robot starts
        static constexpr double StartZoneRadius = 1.0;   // m - go beyond this to arm the lap
        static constexpr double EndZoneRadius   = 0.3;   // m - back inside this counts the lap

        // Carries the last reading forward to the current pose. Outputs the
        // estimated wall distance (m) and heading error (rad, +ve = heading
        // toward the wall).
        void EstimateWall(double& aDistance, double& aHeadingError) const;

        // WaitForScan before the first reading, SearchWall when aWallDistance
        // is beyond WallLostDistance, otherwise FollowWall
        TB3State GetState(double aWallDistance) const;

        // Counts a lap each time the robot has been beyond StartZoneRadius and
        // then comes back within EndZoneRadius of the odometry origin
        void CheckLapCondition();

        // Moves the last command toward aTarget by at most one tick's worth of
        // acceleration and returns the result
        CommandVelocity RampCommand(const CommandVelocity& aTarget);

        // Keeps an angle in [-pi, pi] so differences never jump by 2*pi
        static double WrapAngle(double aAngle);

        // Readable state name for the state change log
        static const char* StateName(TB3State aState);

        // inputs
        WallFollowerInput mInput;       // most recent lidar reading
        Pose mCurrentPose;
        Pose mScanPose;                 // pose held when mInput arrived, used as the scan pose
        bool mHasInput = false;         // false until the first UpdateInput()

        // state
        TB3State mState = TB3State::WaitForScan;
        CommandVelocity mLastCmd;       // last command returned - start point of the next ramp step
        double mFilteredAngular = 0.0;  // rad/s - low-pass filtered steering command

        // lap counting
        bool mHasLeftStart = false;     // true once beyond StartZoneRadius on the current lap
        int  mLapCount     = 0;         // laps completed - reported in the lap log only
};

#endif
