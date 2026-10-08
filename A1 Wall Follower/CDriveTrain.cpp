//-----------------------------------------------------------------------------
// CDriveTrain.cpp
//
// Written by SID: 510516950, SID: 530504205
// Implements differential-drive motion from the two wheel distances.
//-----------------------------------------------------------------------------

#include "CDriveTrain.h"

const float CDriveTrain::mTrackWidth = 20.0f;

CDriveTrain::CDriveTrain()
    :
        mLeftWheel(),
        mRightWheel()
{
}

void CDriveTrain::SetWheelSpeeds( float aLeftSpeed, float aRightSpeed )
{
    mLeftWheel.SetSpeed( aLeftSpeed );
    mRightWheel.SetSpeed( aRightSpeed );
}

float CDriveTrain::ForwardDistanceInStep( float aTimeStep ) const
{
    const float LeftDistance = mLeftWheel.DistanceInStep( aTimeStep );
    const float RightDistance = mRightWheel.DistanceInStep( aTimeStep );

    return ( LeftDistance + RightDistance ) / 2.0f;
}

float CDriveTrain::HeadingChangeInStep( float aTimeStep ) const
{
    const float LeftDistance = mLeftWheel.DistanceInStep( aTimeStep );
    const float RightDistance = mRightWheel.DistanceInStep( aTimeStep );

    return ( LeftDistance - RightDistance ) / mTrackWidth;
}
