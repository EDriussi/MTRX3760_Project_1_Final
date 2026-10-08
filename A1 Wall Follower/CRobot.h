//-----------------------------------------------------------------------------
// CRobot.h
//
// Written by SID: 510516950, SID: 530504205
// Declares the two-sensor differential-drive wall-following robot.
//-----------------------------------------------------------------------------

#ifndef CROBOT_H
#define CROBOT_H

#include "CDriveTrain.h"
#include "CGeometry.h"
#include "CRangeSensor.h"

#include <vector>

class CRender;
class CRoom;
class ICollisionListener;

// CRobot owns its sensors, drive train, pose, trail and run statistics.
class CRobot
{
    public:
        CRobot( const CRoom& arRoom, ICollisionListener& arListener );

        // Performs sense, steer, move, collision, trail and lap steps in order.
        void Update( float aTimeStep );

        bool HasCompletedCircuit() const;

        int GetUpdateCount() const;

        int GetCollisionCount() const;

        void Draw( CRender& arRender ) const;

    private:
        enum eSteerMode
        {
            eFollowWall,            // proportional control on both readings
            eTurnAwayFromWall,      // wall close ahead: pivot on the spot
            eTurnTowardWall         // wall lost to the right: arc back to it
        };

        void Sense();

        void Steer();

        void Move( float aTimeStep );

        void CheckCollision();

        void RecordTrail();

        void UpdateLapProgress();

        eSteerMode SelectSteerMode() const;

        void DrawTrail( CRender& arRender ) const;

        void DrawBody( CRender& arRender ) const;

        static const float mRadius;                 // disc radius, units
        static const float mRightSensorAngle;       // degrees, to the right
        static const float mForwardRightSensorAngle;// degrees, forward right
        static const float mHeadingIndicatorLength; // nose line, units
        static const float mTrailThickness;         // nose and trail width

        static const float mTargetStandoff;

        static const float mBaseSpeed;

        static const float mDistanceGain;

        static const float mAlignmentGain;

        static const float mMaximumDifferential;

        static const float mWallAheadThreshold;

        static const float mWallLostThreshold;

        static const float mPivotSpeed;

        static const float mCornerTurnDifferential;

        static const float mDepartureRadius;

        static const float mReturnRadius;

        const CRoom& mrRoom;
        ICollisionListener& mrCollisionListener;

        CRangeSensor mRightSensor;
        CRangeSensor mForwardRightSensor;
        CDriveTrain mDriveTrain;

        CGeometry::CPose mPose;
        std::vector<CGeometry::Vec2D> mTrail;

        int mUpdateCount;
        int mCollisionCount;

        float mRightRange;
        float mForwardRightRange;

        bool mWasColliding; // used to count collision entries, not contact steps

        CGeometry::Vec2D mStartPosition;

        bool mHasDeparted;

        bool mHasCompletedCircuit;
};

#endif
