//-----------------------------------------------------------------------------
// CRangeSensor.h
//
// Written by SID: 510516950, SID: 530504205
// Declares a range sensor mounted at a fixed angle on the robot.
//-----------------------------------------------------------------------------

#ifndef CRANGESENSOR_H
#define CRANGESENSOR_H

#include "CGeometry.h"

class CRoom;

// CRangeSensor casts a ray from a supplied robot pose into a supplied room.
class CRangeSensor
{
    public:
        explicit CRangeSensor( float aMountAngleDegrees );

        float Read( const CRoom& arRoom,
                    const CGeometry::CPose& arRobotPose ) const;

    private:
        static const float mMaximumRange;

        const float mMountAngle;
};

#endif
