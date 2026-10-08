//-----------------------------------------------------------------------------
// CWheel.h
//
// Written by SID: 510516950, SID: 530504205
// Declares one speed-limited drive wheel.
//-----------------------------------------------------------------------------

#ifndef CWHEEL_H
#define CWHEEL_H

// CWheel stores a bounded speed and converts it into travel for one step.
class CWheel
{
    public:
        CWheel();

        void SetSpeed( float aSpeed );

        float DistanceInStep( float aTimeStep ) const;

    private:
        static const float mMaximumSpeed;

        float mSpeed;
};

#endif
