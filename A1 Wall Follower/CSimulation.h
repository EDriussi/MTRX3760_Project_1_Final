//-----------------------------------------------------------------------------
// CSimulation.h
//
// Written by SID: 510516950, SID: 530504205
// Declares the top-level owner and run loop for the simulation.
//-----------------------------------------------------------------------------

#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CCollisionLogger.h"
#include "CRender.h"
#include "CRobot.h"
#include "CRoom.h"

#include <string>

class CLoopReader;

// CSimulation composes the world and advances it with a fixed simulated step.
class CSimulation
{
    public:
        explicit CSimulation( const std::string& arMapFilename );

        bool IsValid() const;

        // Runs fixed updates and keeps drawing the completed trail until closed.
        void Run();

        static const char* GetDefaultMapFilename();

    private:
        void Step();

        void Draw();

        void ReportSummary() const;

        static CLoopReader ReadLoop( const std::string& arFilename );

        static const float mTimeStep;

        static const int mMaxUpdates;

        static const int mUpdatesPerFrame;

        static const char* const mpDefaultMapFilename;

        CRoom mRoom;
        CRender mRender;
        CCollisionLogger mLogger;
        CRobot mRobot;

        bool mIsFinished;
};

#endif
