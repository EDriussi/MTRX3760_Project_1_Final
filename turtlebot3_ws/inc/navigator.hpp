#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>

// ──────────────────────────────────────────────
//  State definitions
// ──────────────────────────────────────────────
enum class TB3State : uint8_t {
    DRIVE_FORWARD  = 0,
    AVOID_OBSTACLE = 1,
    SEARCH_WALL    = 2,
    STOP           = 3
};

// ──────────────────────────────────────────────
//  Data types
// ──────────────────────────────────────────────
struct Pose {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

struct CommandVelocity {
    double linear  = 0.0;
    double angular = 0.0;
};

// What the Navigator needs from the LiDAR, regardless of source
struct WallFollowerInput {
    // Defaults are set to large safe values so the robot searches for a wall on startup
    double front_distance = 10.0; 
    double right_distance = 10.0;
    double tilt_angle     = 0.0;
};

// ──────────────────────────────────────────────
//  Navigator class
// ──────────────────────────────────────────────
class Navigator {
public:
    Navigator();
    ~Navigator() = default;

    void updatePose(const Pose& pose);
    void updateInput(const WallFollowerInput& input);

    CommandVelocity navigate();

private:
    TB3State getState() const;
    void checkLapCondition();

    // Inputs
    WallFollowerInput mInput;
    Pose mCurrentPose;
    Pose mPreviousPose;

    // State
    TB3State mState = TB3State::SEARCH_WALL;

    // Lap detection
    bool mHasLeftStart    = false;
    bool mHasCompletedLap = false;

    // Tuning parameters
    static constexpr double TARGET_DISTANCE    = 0.35;
    static constexpr double FRONT_THRESHOLD    = 0.50;
    static constexpr double WALL_LOST_DISTANCE = 1.00;

    static constexpr double MAX_LINEAR         = 0.20;
    static constexpr double MAX_ANGULAR        = 1.50;

    static constexpr double START_ZONE_RADIUS  = 1.0;
    static constexpr double END_ZONE_RADIUS    = 0.4;

    // P-Controller Gains
    static constexpr double KP_DIST = 2.5;
    static constexpr double KP_TILT = 1.5;
};

#endif // NAVIGATOR_HPP