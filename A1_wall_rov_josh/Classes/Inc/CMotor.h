#ifndef CMOTOR_H_
#define CMOTOR_H_

#include "CSubSystem.h"

#include <string>

// models one of the robot wheel motors 
// each cycle it accelerates one step closer to a target speed
class CDriveMotor : public CSubsystem {
	public:
		// creates a stopped drive motor with the given label and target speed.
		CDriveMotor(const std::string& label, double InitTargetSpeed);

		// update accelerates the motor one step closer to its target speed.
		void Update();

		// report prints the motor's label and current speed.
		void Report() const;

		// called to change what speed this
		// motor should be heading towards. Clamped to the valid range.
		void SetTargetSpeed(double NewTargetSpeed);

		// Lets steering logic / reporting read how fast this wheel is
		// actually turning right now (as opposed to what its aiming for).
		double GetCurrentSpeed() const;

		void ApplyDamping(double DampingFactor);

	private:
		// constants
		static constexpr double SNAP_THRESHOLD = 0.1; // threshold for controller
		static constexpr double KP = 0.767;           // proportion constant
		static constexpr double MIN_SPEED = -1.0;
		static constexpr double MAX_SPEED = 1.0;
		
    	double mTargetSpeed;    // the speed the motor is accelerating towards
    	double mCurrentSpeed;   // the motors current speed -1.0 to 1.0 normalised to the robots max speed
};


#endif
