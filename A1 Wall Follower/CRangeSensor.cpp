//-----------------------------------------------------------------------------
// CRangeSensor.cpp
//
// Written by SID: 510516950, SID: 530504205
// Implements wall-distance sensing without duplicating room geometry.
//-----------------------------------------------------------------------------

#include "CRangeSensor.h"

#include "CRoom.h"

const float CRangeSensor::mMaximumRange = 1200.0f;

CRangeSensor::CRangeSensor( float aMountAngleDegrees )
    :
        mMountAngle( aMountAngleDegrees * CGeometry::mDegreesToRadians )
{
}

float CRangeSensor::Read( const CRoom& arRoom,
                          const CGeometry::CPose& arRobotPose ) const
{
    float Result = mMaximumRange;

    const CGeometry::Vec2D Direction =
            arRobotPose.GetDirectionAt( mMountAngle );

    float Distance = 0.0f;

    if( arRoom.CastRay( arRobotPose.GetPosition(), Direction, Distance ) )
    {
        if( Distance < mMaximumRange )
        {
            Result = Distance;
        }
    }

    return Result;
}
