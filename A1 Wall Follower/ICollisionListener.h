//-----------------------------------------------------------------------------
// ICollisionListener.h
//
// Written by SID: 510516950, SID: 530504205
// Declares the callback interface used for collision events.
//-----------------------------------------------------------------------------

#ifndef ICOLLISIONLISTENER_H
#define ICOLLISIONLISTENER_H

// Implementations decide how a robot collision is reported.
class ICollisionListener
{
    public:
        virtual ~ICollisionListener() {}

        virtual void OnCollision( int aUpdateNumber ) = 0;
};

#endif
