//-----------------------------------------------------------------------------
// CCollisionLogger.cpp
//
// Written by SID: 510516950, SID: 530504205
// Implements collision reporting to the console.
//-----------------------------------------------------------------------------

#include "CCollisionLogger.h"

#include <iostream>

void CCollisionLogger::OnCollision( int aUpdateNumber )
{
    std::cout << "Collision with a wall on update " << aUpdateNumber
              << std::endl;
}
