//-----------------------------------------------------------------------------
// CDriveTrain.h
//
// Written by SID: 510516950, SID: 530504205
// Declares the robot's two-wheel differential drive.
//-----------------------------------------------------------------------------

#ifndef CDRIVETRAIN_H
#define CDRIVETRAIN_H

#include "CWheel.h"

// CDriveTrain owns both wheels and converts their travel into robot motion.
class CDriveTrain
{
    public:
        CDriveTrain();

        void SetWheelSpeeds( float aLeftSpeed, float aRightSpeed );

        float ForwardDistanceInStep( float aTimeStep ) const;

        float HeadingChangeInStep( float aTimeStep ) const;

    private:
        static const float mTrackWidth;

        CWheel mLeftWheel;
        CWheel mRightWheel;
};

#endif
