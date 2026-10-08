//-----------------------------------------------------------------------------
// navigator.cpp
//
// Written by SID: 530478283
//
// Edited and cleaned by SID: 510516950
// Implements Navigator: carrying the last lidar reading forward with odometry,
// choosing the controller state, the wall-following control law and the
// acceleration limit on its output.
//-----------------------------------------------------------------------------

#include "navigator.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

//---Navigator Implementation--------------------------------------------------
double Navigator::wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

const char* Navigator::stateName(TB3State state) {
    const char* name = "?";
    switch (state) {
        case TB3State::WAIT_FOR_SCAN: name = "WAIT_FOR_SCAN"; break;
        case TB3State::FOLLOW_WALL:   name = "FOLLOW_WALL";   break;
        case TB3State::SEARCH_WALL:   name = "SEARCH_WALL";   break;
    }
    return name;
}

void Navigator::updatePose(const Pose& pose) {
    mCurrentPose = pose;
}

// The pose held at this moment is recorded as the pose of the scan, which is
// what lets estimateWall() carry the reading forward until the next scan.
// The scan is in fact slightly older than that pose (sweep time plus message
// delay); that offset is not corrected for.
void Navigator::updateInput(const WallFollowerInput& input) {
    mInput    = input;
    mScanPose = mCurrentPose;
    mHasInput = true;
}

// Between scans the last reading is moved forward using odometry, which removes
// most of the lag that would otherwise make the steering overshoot.
// The wall is modelled as a straight line through the closest point, fixed in
// the odometry frame: movement along the line's normal changes the distance and
// rotation since the scan changes the heading error. Near a corner or wall end
// that model is only approximate until the next scan replaces it.
void Navigator::estimateWall(double& distance, double& heading_error) const {
    // heading error at scan time, +ve = heading toward the wall
    const double scan_alpha = -mInput.tilt_angle;

    // direction from robot to the wall point at scan time, in the odometry frame
    const double normal = mScanPose.yaw + scan_alpha - M_PI / 2.0;

    // robot displacement since the scan
    const double dx = mCurrentPose.x - mScanPose.x;
    const double dy = mCurrentPose.y - mScanPose.y;

    // the part of that displacement along the normal is how far the robot has closed on the wall
    distance      = mInput.right_distance - (dx * std::cos(normal) + dy * std::sin(normal));
    heading_error = wrapAngle(scan_alpha - wrapAngle(mCurrentPose.yaw - mScanPose.yaw));
}

// Decided afresh every tick with no hysteresis, so a wall sitting right at
// WALL_LOST_DISTANCE can alternate the state between ticks. rampCommand() keeps
// the output continuous when that happens.
Navigator::TB3State Navigator::getState(double wall_distance) const {
    TB3State state = TB3State::FOLLOW_WALL;
    if (!mHasInput) {
        state = TB3State::WAIT_FOR_SCAN;
    } else if (wall_distance > WALL_LOST_DISTANCE) {
        state = TB3State::SEARCH_WALL;
    }
    return state;
}

// Assumes odometry reads (0, 0) where the robot starts. Odometry drift over a
// lap can carry the true start outside END_ZONE_RADIUS, and the lap is missed.
void Navigator::checkLapCondition() {
    const double dist = std::hypot(mCurrentPose.x, mCurrentPose.y);

    if (!mHasLeftStart) {
        if (dist > START_ZONE_RADIUS) {
            mHasLeftStart = true;
            std::cout << "[Navigator] Left start zone" << std::endl;
        }
    } else if (dist < END_ZONE_RADIUS) {
        mLapCount++;
        mHasLeftStart = false;
        std::cout << "[Navigator] Lap " << mLapCount << " completed" << std::endl;
    }
}

// One control tick:
//   1. carry the last lidar reading forward to the current pose and check for
//      a completed lap (both skipped until the first reading arrives)
//   2. pick the state from the estimated wall distance
//   3. work out the target velocity for that state
//   4. ramp the output toward the target
CommandVelocity Navigator::navigate() {
    CommandVelocity target = {0.0, 0.0};

    double wall_distance = mInput.right_distance;
    double heading_error = 0.0;
    if (mHasInput) {
        estimateWall(wall_distance, heading_error);
        checkLapCondition();
    }

    const TB3State new_state = getState(wall_distance);
    // log state changes (the first one confirms lidar data is arriving)
    if (new_state != mState) {
        std::cout << "[Navigator] " << stateName(mState) << " -> " << stateName(new_state)
                  << " (wall=" << wall_distance << " m, front=" << mInput.front_distance << " m)"
                  << std::endl;
        mState = new_state;
    }

    switch (mState) {
        // no lidar data yet - target stays at zero
        case TB3State::WAIT_FOR_SCAN:
            break;

        case TB3State::SEARCH_WALL:
            // nothing on the right - arc right at SEARCH_RADIUS until a wall shows
            // angular = -speed / radius, -ve = clockwise
            target.linear  = MAX_LINEAR;
            target.angular = -MAX_LINEAR / SEARCH_RADIUS;
            break;

        case TB3State::FOLLOW_WALL:
        default: {
            // distance error sets the approach angle to hold (+ve = toward the wall),
            // clamped so the robot never closes on or leaves the wall too steeply
            const double dist_error = wall_distance - TARGET_DISTANCE;
            const double desired_alpha = std::clamp(K_DIST * dist_error, -MAX_APPROACH_ANGLE, MAX_APPROACH_ANGLE);

            // proportional heading control - pointing more toward the wall than
            // desired turns left (+ve)
            double angular_cmd = K_ANGLE * (heading_error - desired_alpha);

            // start turning left a little before an inside corner. The forward cone
            // also sees the followed wall when the robot is close to it and angled
            // well toward it, so the same bias then helps to turn it back
            angular_cmd += K_FRONT * std::max(0.0, FRONT_TURN - mInput.front_distance);
            angular_cmd = std::clamp(angular_cmd, -MAX_ANGULAR, MAX_ANGULAR);

            // first-order low-pass on the steering command - smooths the step each
            // new 5-10 Hz scan makes in the estimate. Only updated in this state, so
            // after a search it resumes from its last FOLLOW_WALL value
            const double filter_gain = CONTROL_PERIOD / (ANGULAR_FILTER_TAU + CONTROL_PERIOD);
            mFilteredAngular += filter_gain * (angular_cmd - mFilteredAngular);
            angular_cmd = mFilteredAngular;

            // slow down for obstacles and for sharp turns - the turn slowdown uses the
            // angular rate actually commanded last tick (after the ramp), not this tick's target
            const double front_scale = std::clamp((mInput.front_distance - FRONT_STOP) /
                                                  (FRONT_SLOW - FRONT_STOP), 0.0, 1.0);
            const double turn_ratio = std::abs(mLastCmd.angular) / MAX_ANGULAR;

            target.linear  = std::max(MIN_LINEAR,MAX_LINEAR * front_scale * (1.0 - TURN_SLOWDOWN * turn_ratio));
            target.angular = angular_cmd;
            break;
        }
    }

    return rampCommand(target);
}

// The step allowed per tick is acceleration x CONTROL_PERIOD, so the limits only
// hold if navigate() really is called at that period.
CommandVelocity Navigator::rampCommand(const CommandVelocity& target) {
    const double max_linear_step  = MAX_LINEAR_ACCEL  * CONTROL_PERIOD;
    const double max_angular_step = MAX_ANGULAR_ACCEL * CONTROL_PERIOD;

    mLastCmd.linear  += std::clamp(target.linear  - mLastCmd.linear,  -max_linear_step,  max_linear_step);
    mLastCmd.angular += std::clamp(target.angular - mLastCmd.angular, -max_angular_step, max_angular_step);

    return mLastCmd;
}
