#include "Robot.h"
#include "Map.h"

#include <cmath>
#include <algorithm>

Robot::Robot(CRender& Render, const CLoopReader& Walls)
    : rRender(Render),
      rWalls(Walls),
      rPosition(Walls.GetStartPose().mPosition),
      rHeading(Walls.GetStartPose().mHeading),
      rLeftMotor("Left motor", 0.0),
      rRightMotor("Right motor", 0.0),
      rSideSensor("Side sensor", RIGHT_SENSOR_ANGLE, rPosition, rHeading, rWalls),
      rForwardSensor("Forward sensor", FORWARD_SENSOR_ANGLE, rPosition, rHeading, rWalls ),
      rSubsystems{&rLeftMotor, &rRightMotor, &rSideSensor, &rForwardSensor},
      rUpdateCount(0),
      rCollisionCount(0),
      rRobotColour({(unsigned char)(255), 
                    (unsigned char)(0), 
                    (unsigned char)(0), 
                    (unsigned char)128}),
      rTrailColour({(unsigned char)(255), 
                    (unsigned char)(255), 
                    (unsigned char)(255), 
                    (unsigned char)128}),
      rSensorColour({(unsigned char)(255), 
                     (unsigned char)(255), 
                     (unsigned char)(255), 
                     (unsigned char)128})
{ 
    std::cout << "[CTor] - Robot: created at position (" << rPosition.x << ", " << rPosition.y 
              << ") with heading " << rHeading * RAD_TO_DEG << " degrees" << std::endl;
}

void Robot::Update() {
    mPath.push_back(rPosition);
    
    for(CSubsystem* pSubsystem : rSubsystems) {
        pSubsystem->Update();
    }
    Steer();
    Move();
    CheckCollisions();

    ++rUpdateCount;
}

void Robot::Report()
{
    std::cout << "Number of Collisions with Walls: " << rCollisionCount << std::endl;
    std::cout << "Number of Update Cycles: " << rUpdateCount << std::endl;
}

// steers robot based off sensor readings to maintain heading angle and target distance from wall
void Robot::Steer() {
    float d1 = rSideSensor.GetDistance();       // distance of 90degree sensor to wall
    float d2 = rForwardSensor.GetDistance();    // distance of 45degree sensor to wall

    float phi = std::atan2(d2 * std::sin(FORWARD_SENSOR_ANGLE * DEG_TO_RAD) - d1, 
                           d2 * std::cos(FORWARD_SENSOR_ANGLE * DEG_TO_RAD)); // relative angle between heading and wall in radians    
    float PerpendicularDistance = d1 * std::cos(phi);

    float distanceError = PerpendicularDistance - TARGET_WALL_DISTANCE;
    float angleError = phi; // should be positive if robot is angled towards wall

    float turningAdjustment = KP_DISTANCE * distanceError + KP_ANGLE * angleError;  // proportional control for steering adjustment
    rLeftMotor.SetTargetSpeed(BASE_SPEED + turningAdjustment);
    rRightMotor.SetTargetSpeed(BASE_SPEED - turningAdjustment);
}

// moves robot based on current motor speed and updates heading and position
void Robot::Move() {
    double velocityL_raw = rLeftMotor.GetCurrentSpeed();   // unitless speed
    double velocityR_raw = rRightMotor.GetCurrentSpeed();

    double velocityL = velocityL_raw * SPEED_SCALE;   // pixels per update
    double velocityR = velocityR_raw * SPEED_SCALE;

    double linearVelocity = (velocityL + velocityR) / 2.0;  
    double angularVelocity = (velocityL - velocityR) / (2.0 * (double)RADIUS); // clockwise positive (from vec2d specifics)

    Vec2D moveDirection = {((float)linearVelocity * std::cos(rHeading)), ((float)linearVelocity * std::sin(rHeading))};
    rPosition = {rPosition.x + moveDirection.x, rPosition.y + moveDirection.y};  
    rHeading += (float)angularVelocity;
}

// checks for wall collisions and applies damping to motors if collision
void Robot::CheckCollisions() {
    const std::vector<Vec2D>& vertices = rWalls.GetVertices();
    bool collided = false;

    for(size_t i = 0; i < vertices.size(); ++i) {
        const Vec2D& A = vertices[i];                                // absolute vector of edge
        const Vec2D& B = vertices[(i + 1) % vertices.size()];        // absolute vector of wrapped 

        Vec2D edge_vector = {B.x - A.x, B.y - A.y};                  // direction free vector of edge

        float postionProjectionRatio = dot({rPosition.x - A.x, rPosition.y - A.y}, edge_vector) / 
                                       dot(edge_vector, edge_vector);     // projection of robot position onto edge

        float clampedProjection = std::clamp(postionProjectionRatio, 0.0f, 1.0f); // clamp to edge segment if projected point is outside bounds

        Vec2D AbsoluteEdgeVector = {A.x + (clampedProjection * edge_vector.x), 
                                    A.y + (clampedProjection * edge_vector.y)}; // closest point on edge to robot center

        Vec2D distanceVector = {rPosition.x - AbsoluteEdgeVector.x, rPosition.y - AbsoluteEdgeVector.y}; // vector from closest point to robot center
        
        float distance = std::sqrt(dot(distanceVector, distanceVector)); // distance from robot center
        
        if (distance < RADIUS) {
            if (distance > 1e-6f) {
                Vec2D normalVector = {distanceVector.x / distance, distanceVector.y / distance}; // unit vector from wall to robot
                float penetrationDepth = RADIUS - distance;         // how much the robot is overlapping with the wall
                rPosition = {rPosition.x + normalVector.x * penetrationDepth, 
                             rPosition.y + normalVector.y * penetrationDepth};
            }
            collided = true;
            break; 
        }
    }
    if (collided) {
        rLeftMotor.ApplyDamping(DAMPING);
        rRightMotor.ApplyDamping(DAMPING);
        ++rCollisionCount;
        std::cout << "Robot has collided with a Wall!" << std::endl;
    }
}

// draw robot, trail and sensors to the simulation window
void Robot::Draw() const {
    // Robot
    rRender.DrawCircle(rPosition, RADIUS, rRobotColour);  

    // Trail
    if ( !mPath.empty() )
    {
        for ( int i = 1; i < (int)mPath.size(); ++i )
        {
            rRender.DrawLine( mPath[i], mPath[i - 1], 2.0f, rTrailColour ); 
        }
    }

    // Sensor 
    Vec2D SensorAEnd = { rPosition.x + cosf(rHeading + RIGHT_SENSOR_ANGLE * DEG_TO_RAD) * RADIUS * 2.0f,
                         rPosition.y + sinf(rHeading + RIGHT_SENSOR_ANGLE * DEG_TO_RAD) * RADIUS * 2.0f
                    };  

    Vec2D SensorBEnd = { rPosition.x + cosf(rHeading + FORWARD_SENSOR_ANGLE * DEG_TO_RAD) * RADIUS * 2.0f,
                         rPosition.y + sinf(rHeading + FORWARD_SENSOR_ANGLE * DEG_TO_RAD) * RADIUS * 2.0f
                    };        

    rRender.DrawLine( rPosition, SensorAEnd, 5.0f, rSensorColour ); 
    rRender.DrawLine( rPosition, SensorBEnd, 5.0f, rSensorColour ); 
}

unsigned int Robot::GetUpdateCount() const {
    return rUpdateCount;
}

unsigned int Robot::GetCollisionCount() const {
    return rCollisionCount;
}

// dot product helper
float Robot::dot(const Vec2D& A, const Vec2D& B) {
    return (A.x * B.x) + (A.y * B.y);
}
