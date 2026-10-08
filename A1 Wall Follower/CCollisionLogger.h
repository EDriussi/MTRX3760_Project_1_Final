//-----------------------------------------------------------------------------
// CCollisionLogger.h
//
// Written by SID: 510516950, SID: 530504205
// Declares the console implementation of the collision callback.
//-----------------------------------------------------------------------------

#ifndef CCOLLISIONLOGGER_H
#define CCOLLISIONLOGGER_H

#include "ICollisionListener.h"

// CCollisionLogger prints one message whenever a collision begins.
class CCollisionLogger : public ICollisionListener
{
    public:

        void OnCollision( int aUpdateNumber );
};

#endif
