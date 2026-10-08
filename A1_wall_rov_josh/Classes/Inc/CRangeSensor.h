#ifndef CRANGESENSOR_H_
#define CRANGESENSOR_H_

#include "CSubSystem.h"
#include "CRender.h"       // for Vec2D
#include "CLoopReader.h"   // the wall loop this sensor ray-casts against

#include <string>

// models a a single distance sensor mounted on the robot at a fixed
// angle offset from the robot's own heading
class CRangeSensor : public CSubsystem
{
    public:
        // creates range sensor with the given label, mounted at the given angle offset
        // receives references to the robots position and heading and the loop of walls it casts its ray against
        CRangeSensor(const std::string& label,   float OffsetAngleDeg,
                     const Vec2D& RobotPosition, const float& RobotHeading, const CLoopReader& Walls);

        // casts a ray from RobotPosition, aimed at (robot heading +
        // OffsetAngle) and stores the distance to the nearest wall it hits.
        void Update();

        // reports sensor's label and current reading.
        void Report() const;

        // Distance to the nearest wall along this sensor's ray
        float GetDistance() const;

    private:
        // constants
        static constexpr float MAX_RANGE = 500.0f;   
        static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
        static constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;

        // cross product helper - returns scalar
        float cross(const Vec2D& A, const Vec2D& B);

        // mounting angle
        const float rsOffsetAngle;   // radians, relative to the robot's heading

        // what this sensor reads (referenced, not owned)
        const Vec2D& rsRobotPosition;
        const float& rsRobotHeading;
        const CLoopReader& rsWalls;

        // last reading
        float rsDistance;
};

#endif
