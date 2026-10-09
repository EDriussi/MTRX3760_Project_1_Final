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
double Navigator::WrapAngle(double aAngle) {
    return std::atan2(std::sin(aAngle), std::cos(aAngle));
}

const char* Navigator::StateName(TB3State aState) {
    const char* Name = "?";
    switch (aState) {
        case TB3State::WaitForScan: Name = "WAIT_FOR_SCAN"; break;
        case TB3State::FollowWall:  Name = "FOLLOW_WALL";   break;
        case TB3State::SearchWall:  Name = "SEARCH_WALL";   break;
    }
    return Name;
}

void Navigator::UpdatePose(const Pose& aPose) {
    mCurrentPose = aPose;
}

// The pose held at this moment is recorded as the pose of the scan, which is
// what lets EstimateWall() carry the reading forward until the next scan.
// The scan is in fact slightly older than that pose (sweep time plus message
// delay); that offset is not corrected for.
void Navigator::UpdateInput(const WallFollowerInput& aInput) {
    mInput    = aInput;
    mScanPose = mCurrentPose;
    mHasInput = true;
}

// Between scans the last reading is moved forward using odometry, which removes
// most of the lag that would otherwise make the steering overshoot.
// The wall is modelled as a straight line through the closest point, fixed in
// the odometry frame: movement along the line's normal changes the distance and
// rotation since the scan changes the heading error. Near a corner or wall end
// that model is only approximate until the next scan replaces it.
void Navigator::EstimateWall(double& aDistance, double& aHeadingError) const {
    // heading error at scan time, +ve = heading toward the wall
    const double ScanAlpha = -mInput.mTiltAngle;

    // direction from robot to the wall point at scan time, in the odometry frame
    const double Normal = mScanPose.mYaw + ScanAlpha - M_PI / 2.0;

    // robot displacement since the scan
    const double Dx = mCurrentPose.mX - mScanPose.mX;
    const double Dy = mCurrentPose.mY - mScanPose.mY;

    // the part of that displacement along the normal is how far the robot has closed on the wall
    aDistance     = mInput.mRightDistance - (Dx * std::cos(Normal) + Dy * std::sin(Normal));
    aHeadingError = WrapAngle(ScanAlpha - WrapAngle(mCurrentPose.mYaw - mScanPose.mYaw));
}

// Decided afresh every tick with no hysteresis, so a wall sitting right at
// WallLostDistance can alternate the state between ticks. RampCommand() keeps
// the output continuous when that happens.
Navigator::TB3State Navigator::GetState(double aWallDistance) const {
    TB3State State = TB3State::FollowWall;
    if (!mHasInput) {
        State = TB3State::WaitForScan;
    } else if (aWallDistance > WallLostDistance) {
        State = TB3State::SearchWall;
    }
    return State;
}

// Assumes odometry reads (0, 0) where the robot starts. Odometry drift over a
// lap can carry the true start outside EndZoneRadius, and the lap is missed.
void Navigator::CheckLapCondition() {
    const double Dist = std::hypot(mCurrentPose.mX, mCurrentPose.mY);

    if (!mHasLeftStart) {
        if (Dist > StartZoneRadius) {
            mHasLeftStart = true;
            std::cout << "[Navigator] Left start zone" << std::endl;
        }
    } else if (Dist < EndZoneRadius) {
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
CommandVelocity Navigator::Navigate() {
    CommandVelocity Target = {0.0, 0.0};

    double WallDistance = mInput.mRightDistance;
    double HeadingError = 0.0;
    if (mHasInput) {
        EstimateWall(WallDistance, HeadingError);
        CheckLapCondition();
    }

    const TB3State NewState = GetState(WallDistance);
    // log state changes (the first one confirms lidar data is arriving)
    if (NewState != mState) {
        std::cout << "[Navigator] " << StateName(mState) << " -> " << StateName(NewState)
                  << " (wall=" << WallDistance << " m, front=" << mInput.mFrontDistance << " m)"
                  << std::endl;
        mState = NewState;
    }

    switch (mState) {
        // no lidar data yet - target stays at zero
        case TB3State::WaitForScan:
            break;

        case TB3State::SearchWall:
            // nothing on the right - arc right at SearchRadius until a wall shows
            // angular = -speed / radius, -ve = clockwise
            Target.mLinear  = MaxLinear;
            Target.mAngular = -MaxLinear / SearchRadius;
            break;

        case TB3State::FollowWall:
        default: {
            // distance error sets the approach angle to hold (+ve = toward the wall),
            // clamped so the robot never closes on or leaves the wall too steeply
            const double DistError = WallDistance - TargetDistance;
            const double DesiredAlpha = std::clamp(KDist * DistError, -MaxApproachAngle, MaxApproachAngle);

            // proportional heading control - pointing more toward the wall than
            // desired turns left (+ve)
            double AngularCmd = KAngle * (HeadingError - DesiredAlpha);

            // start turning left a little before an inside corner. The forward cone
            // also sees the followed wall when the robot is close to it and angled
            // well toward it, so the same bias then helps to turn it back
            AngularCmd += KFront * std::max(0.0, FrontTurn - mInput.mFrontDistance);
            AngularCmd = std::clamp(AngularCmd, -MaxAngular, MaxAngular);

            // first-order low-pass on the steering command - smooths the step each
            // new 5-10 Hz scan makes in the estimate. Only updated in this state, so
            // after a search it resumes from its last FollowWall value
            const double FilterGain = ControlPeriod / (AngularFilterTau + ControlPeriod);
            mFilteredAngular += FilterGain * (AngularCmd - mFilteredAngular);
            AngularCmd = mFilteredAngular;

            // slow down for obstacles and for sharp turns - the turn slowdown uses the
            // angular rate actually commanded last tick (after the ramp), not this tick's target
            const double FrontScale = std::clamp((mInput.mFrontDistance - FrontStop) /
                                                  (FrontSlow - FrontStop), 0.0, 1.0);
            const double TurnRatio = std::abs(mLastCmd.mAngular) / MaxAngular;

            Target.mLinear  = std::max(MinLinear,MaxLinear * FrontScale * (1.0 - TurnSlowdown * TurnRatio));
            Target.mAngular = AngularCmd;
            break;
        }
    }

    return RampCommand(Target);
}

// The step allowed per tick is acceleration x ControlPeriod, so the limits only
// hold if Navigate() really is called at that period.
CommandVelocity Navigator::RampCommand(const CommandVelocity& aTarget) {
    const double MaxLinearStep  = MaxLinearAccel  * ControlPeriod;
    const double MaxAngularStep = MaxAngularAccel * ControlPeriod;

    mLastCmd.mLinear  += std::clamp(aTarget.mLinear  - mLastCmd.mLinear,  -MaxLinearStep,  MaxLinearStep);
    mLastCmd.mAngular += std::clamp(aTarget.mAngular - mLastCmd.mAngular, -MaxAngularStep, MaxAngularStep);

    return mLastCmd;
}
