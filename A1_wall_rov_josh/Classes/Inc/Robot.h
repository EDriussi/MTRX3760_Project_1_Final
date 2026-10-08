#ifndef ROBOT_H_
#define ROBOT_H_

#include "CRender.h"
#include "CLoopReader.h"
#include "CMotor.h"
#include "CRangeSensor.h"

#include <stdlib.h>   
#include <iostream>
#include <vector>
#include <cmath>     // for sin() and cos()

// A Robot can move around a map, steer itself based on sensor readings and report the number 
// of collisions and simulation updates. It can also draw itself, trail 
// and its sensors to the simulation window.
class Robot
{
    public:
        // constructs robot that draws itself and the room it goes around
        Robot(CRender& Render, const CLoopReader& Walls);

        // updates the robot and subsystems by simulation timestep
        void Update();
        
        // report the number of collisions and simulation updates
        void Report();

        // draws the robot, trail and sensors to the simulation window
        void Draw() const;

        // simulation reporting
        unsigned int GetUpdateCount() const;
        unsigned int GetCollisionCount() const;

    private:
        // constants
        static constexpr float RADIUS = 15.0f;                  // radius of the robot
        static constexpr float TARGET_WALL_DISTANCE = 40.0f;    // target distance from walls
        static constexpr float DAMPING = 0.97f;                 // energy loss on collision
        static constexpr float RIGHT_SENSOR_ANGLE = 90.0f;      // right sensor angle offset
        static constexpr float FORWARD_SENSOR_ANGLE = 45.0f;    // forward sensor angle offset
        static constexpr float KP_ANGLE = 2.5f;                // proportional constant for angle error (more weight than distance)
        static constexpr float KP_DISTANCE = 0.2f;              // proportional constant for distance error
        static constexpr float BASE_SPEED = 0.2f;              // motor base speeds
        static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
        static constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;
        static constexpr double SPEED_SCALE = 5.0;              // scale factor for motor speed to pixels per update

        // private helper methods for update
        void Steer();
        void Move();
        void CheckCollisions();

        float dot(const Vec2D& A, const Vec2D& B);

        Vec2D rPosition;            
        float rHeading;                 // radians
        std::vector<Vec2D> mPath;       // trail of robot's path

        CLoopReader rWalls;             // the walls of the room

        // susbsystems
        CDriveMotor rLeftMotor;
        CDriveMotor rRightMotor;
        CRangeSensor rSideSensor;      // 90 degrees
        CRangeSensor rForwardSensor;   // 45 degrees
        
        // collection of all subsystems for updating and reporting
        std::vector<CSubsystem*> rSubsystems;

        unsigned int rUpdateCount;
        unsigned int rCollisionCount;
        
        CRender& rRender; 
        Color rRobotColour; 
        Color rTrailColour;
        Color rSensorColour;
};


#endif
