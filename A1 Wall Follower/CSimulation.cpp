//-----------------------------------------------------------------------------
// CSimulation.cpp
//
// Written by SID: 510516950, SID: 530504205
// Builds, advances, draws and reports the wall-following simulation.
//-----------------------------------------------------------------------------

#include "CSimulation.h"

#include "CLoopReader.h"

#include <iostream>
#include <sstream>

const float CSimulation::mTimeStep = 0.05f;

const int CSimulation::mMaxUpdates = 6000;

const int CSimulation::mUpdatesPerFrame = 4;

const char* const CSimulation::mpDefaultMapFilename = "SimpleWalls.map";

CSimulation::CSimulation( const std::string& arMapFilename )
    :
        mRoom( ReadLoop( arMapFilename ) ),
        mRender(),
        mLogger(),
        mRobot( mRoom, mLogger ),
        mIsFinished( false )
{
}

bool CSimulation::IsValid() const
{
    return mRoom.IsValid();
}

void CSimulation::Run()
{
    while( !mRender.WindowShouldClose() )
    {
        for( int Index = 0; Index < mUpdatesPerFrame; ++Index )
        {
            Step();
        }

        Draw();
    }

}

const char* CSimulation::GetDefaultMapFilename()
{
    return mpDefaultMapFilename;
}

void CSimulation::Step()
{
    if( !mIsFinished )
    {
        mRobot.Update( mTimeStep );

        // The update limit guarantees termination if the controller cannot return.
        mIsFinished = mRobot.HasCompletedCircuit()
                   || ( mRobot.GetUpdateCount() >= mMaxUpdates );

        if( mIsFinished )
        {
            ReportSummary();
        }
    }
}

void CSimulation::Draw()
{
    mRender.BeginDrawing();

    mRoom.Draw( mRender );
    mRobot.Draw( mRender );

    mRender.EndDrawing();
}

void CSimulation::ReportSummary() const
{
    std::stringstream Summary;

    if( mRobot.HasCompletedCircuit() )
    {
        Summary << "Run complete: circuit closed after "
                << mRobot.GetUpdateCount() << " updates";
    }
    else
    {
        Summary << "Run ended: update limit of " << mMaxUpdates
                << " reached without closing the circuit";
    }

    Summary << ", " << mRobot.GetCollisionCount() << " collisions."
            << std::endl;

    std::cout << Summary.str();
}

CLoopReader CSimulation::ReadLoop( const std::string& arFilename )
{
    CLoopReader Loop;

    if( !Loop.ReadFile( arFilename ) )
    {
        std::cout << "CSimulation: could not build a room from '"
                  << arFilename << "'" << std::endl;
    }

    return Loop;
}
