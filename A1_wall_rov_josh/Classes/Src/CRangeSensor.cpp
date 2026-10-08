#include "CRangeSensor.h"

#include<iostream>
#include <string>
#include <cmath>   

CRangeSensor::CRangeSensor(const std::string &label, float OffsetAngleDeg, const Vec2D& RobotPosition,
                            const float& RobotHeading, const CLoopReader& Walls)
    : CSubsystem(label),
      rsOffsetAngle(OffsetAngleDeg * DEG_TO_RAD),
      rsRobotPosition(RobotPosition),
      rsRobotHeading(RobotHeading),
      rsWalls(Walls),
      rsDistance(MAX_RANGE)
{
    // should there be a offset angle check?
    std::cout << "[CTor] - CRangeSensor: '" << GetName() << "' created with offset angle: " 
              << rsOffsetAngle * RAD_TO_DEG << " degrees" << std::endl;
}

// casts a ray from RobotPosition aimed at (robot heading + OffsetAngle) and stores the distance to the nearest wall it hits
void CRangeSensor::Update() {
    float theta = rsRobotHeading + rsOffsetAngle;
    Vec2D rayDirection_vector = {std::cos(theta), std::sin(theta)};  // direction vector of the ray

    float closestDistance = MAX_RANGE;  // initialize closest distance to max range
    const std::vector<Vec2D>& vertices = rsWalls.GetVertices();

    for(size_t i = 0; i < vertices.size(); ++i) {
        const Vec2D& A = vertices[i];                                // absolute vector of edge
        const Vec2D& B = vertices[(i + 1) % vertices.size()];        // absolute vector of wrapped 

        Vec2D edge_vector = {B.x - A.x, B.y - A.y};                  // direction free vector of edge

        float D_Cross_S = cross(rayDirection_vector, edge_vector);   // cross product of direction vectors 
    
        if (std::abs(D_Cross_S) < 1e-6f) {       // parallel vectors check
            continue; 
        } 
        
        float RayLength = cross({(A.x - rsRobotPosition.x), (A.y - rsRobotPosition.y)}, edge_vector) / D_Cross_S; // find length of ray
        float EdgeLength = cross({(A.x - rsRobotPosition.x), (A.y - rsRobotPosition.y)}, rayDirection_vector) / D_Cross_S; // find length of edge

        if (RayLength >= 0 && (EdgeLength >= 0 && EdgeLength <= 1) && RayLength < closestDistance) { // intersection check
            closestDistance = RayLength; // update closest distance
        }
        
    }
    rsDistance = closestDistance;
}

// returns last recorded distance to nearest wall along sensor ray or max range if nothing was within range
float CRangeSensor::GetDistance() const {
    return rsDistance;
}

// cross product of 2d vectors
float CRangeSensor::cross(const Vec2D& A, const Vec2D& B) {
    return A.x * B.y - A.y * B.x;
}    

void CRangeSensor::Report() const {
    std::cout << "Range Sensor: '" << GetName() << "' Distance: " << rsDistance << std::endl;
}
