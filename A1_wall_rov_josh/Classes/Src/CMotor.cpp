#include "CMotor.h"

#include <iostream>

CDriveMotor::CDriveMotor(const std::string& label, double InitTargetSpeed)
    : CSubsystem(label),
      mTargetSpeed(InitTargetSpeed),
      mCurrentSpeed(0.0)
{
    mTargetSpeed = (mTargetSpeed < MIN_SPEED) ? MIN_SPEED 
                 : (mTargetSpeed > MAX_SPEED) ? MAX_SPEED 
                 : mTargetSpeed;

    std::cout << "[CTor] - CDriveMotor: '" << GetName() << "' created with target speed: " << mTargetSpeed << std::endl;
}

// generic proportional controller - update later
void CDriveMotor::Update() {
    double error = mTargetSpeed - mCurrentSpeed;
    mCurrentSpeed += KP * error;

    if(std::abs(error) < SNAP_THRESHOLD) {
        mCurrentSpeed = mTargetSpeed;
    }
}

// change the current target speed
void CDriveMotor::SetTargetSpeed(double NewTargetSpeed) {
    // clamp speed to valid range
    mTargetSpeed = (NewTargetSpeed < MIN_SPEED) ? MIN_SPEED 
                : (NewTargetSpeed > MAX_SPEED) ? MAX_SPEED 
                : NewTargetSpeed;
}

// return the current speed of motor
double CDriveMotor::GetCurrentSpeed() const {
    return mCurrentSpeed;
}

// method to apply damping on collision
void CDriveMotor::ApplyDamping(double DampingFactor) {
    mCurrentSpeed *= DampingFactor;
}
        
void CDriveMotor::Report() const {
    std::cout << "Motor: '" << GetName() << "' Target Speed: " << mTargetSpeed << ", Current Speed: " << mCurrentSpeed << std::endl;
}
