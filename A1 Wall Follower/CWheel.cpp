//-----------------------------------------------------------------------------
// CWheel.cpp
//
// Written by SID: 510516950, SID: 530504205
// Implements speed limiting and per-step wheel travel.
//-----------------------------------------------------------------------------

#include "CWheel.h"

const float CWheel::mMaximumSpeed = 140.0f;

CWheel::CWheel()
    :
        mSpeed( 0.0f )
{
}

void CWheel::SetSpeed( float aSpeed )
{
    if( aSpeed > mMaximumSpeed )
    {
        mSpeed = mMaximumSpeed;
    }
    else if( aSpeed < -mMaximumSpeed )
    {
        mSpeed = -mMaximumSpeed;
    }
    else
    {
        mSpeed = aSpeed;
    }
}

float CWheel::DistanceInStep( float aTimeStep ) const
{
    return mSpeed * aTimeStep;
}
