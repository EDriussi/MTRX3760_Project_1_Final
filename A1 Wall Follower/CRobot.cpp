//-----------------------------------------------------------------------------
// CRobot.cpp
//
// Written by SID: 510516950, SID: 530504205
// Implements wall-following control, movement, collision and trail behaviour.
//-----------------------------------------------------------------------------

#include "CRobot.h"

#include "CRender.h"
#include "CRoom.h"
#include "ICollisionListener.h"

const float CRobot::mRadius = 15.0f;
const float CRobot::mRightSensorAngle = 90.0f;
const float CRobot::mForwardRightSensorAngle = 45.0f;

const float CRobot::mHeadingIndicatorLength = 22.0f;
const float CRobot::mTrailThickness = 2.0f;

const float CRobot::mTargetStandoff = 50.0f;
const float CRobot::mBaseSpeed = 60.0f;
const float CRobot::mDistanceGain = 0.55f;
const float CRobot::mAlignmentGain = 0.75f;
const float CRobot::mMaximumDifferential = 55.0f;

const float CRobot::mWallAheadThreshold = 45.0f;
const float CRobot::mWallLostThreshold = 78.0f;

const float CRobot::mPivotSpeed = 34.0f;
const float CRobot::mCornerTurnDifferential = 12.0f;

const float CRobot::mDepartureRadius = 90.0f;
const float CRobot::mReturnRadius = 25.0f;

CRobot::CRobot( const CRoom& arRoom, ICollisionListener& arListener )
    :
        mrRoom( arRoom ),
        mrCollisionListener( arListener ),
        mRightSensor( mRightSensorAngle ),
        mForwardRightSensor( mForwardRightSensorAngle ),
        mDriveTrain(),
        mPose( arRoom.GetStartPose() ),
        mTrail(),
        mUpdateCount( 0 ),
        mCollisionCount( 0 ),
        mRightRange( 0.0f ),
        mForwardRightRange( 0.0f ),
        mWasColliding( false ),
        mStartPosition( arRoom.GetStartPose().GetPosition() ),
        mHasDeparted( false ),
        mHasCompletedCircuit( false )
{
    mTrail.push_back( mStartPosition );
}

void CRobot::Update( float aTimeStep )
{
    Sense();
    Steer();
    Move( aTimeStep );
    CheckCollision();
    RecordTrail();
    UpdateLapProgress();

    ++mUpdateCount;
}

bool CRobot::HasCompletedCircuit() const
{
    return mHasCompletedCircuit;
}

int CRobot::GetUpdateCount() const
{
    return mUpdateCount;
}

int CRobot::GetCollisionCount() const
{
    return mCollisionCount;
}

void CRobot::Draw( CRender& arRender ) const
{
    DrawTrail( arRender );
    DrawBody( arRender );
}

void CRobot::Sense()
{
    mRightRange = mRightSensor.Read( mrRoom, mPose );
    mForwardRightRange = mForwardRightSensor.Read( mrRoom, mPose );
}

void CRobot::Steer()
{
    // For a parallel wall, the 45-degree range is the right range times sqrt(2).
    const float RootTwo = 1.41421356f;

    switch( SelectSteerMode() )
    {
        case eTurnAwayFromWall:
        {
            mDriveTrain.SetWheelSpeeds( -mPivotSpeed, mPivotSpeed );
            break;
        }

        case eTurnTowardWall:
        {
            mDriveTrain.SetWheelSpeeds( mBaseSpeed + mCornerTurnDifferential,
                                        mBaseSpeed - mCornerTurnDifferential );
            break;
        }

        case eFollowWall:
        default:
        {
            const float DistanceError = mRightRange - mTargetStandoff;
            const float AlignmentError =
                    mForwardRightRange - ( mRightRange * RootTwo );

            float Differential = ( mDistanceGain * DistanceError )
                               + ( mAlignmentGain * AlignmentError );

            if( Differential > mMaximumDifferential )
            {
                Differential = mMaximumDifferential;
            }
            else if( Differential < -mMaximumDifferential )
            {
                Differential = -mMaximumDifferential;
            }

            mDriveTrain.SetWheelSpeeds( mBaseSpeed + Differential,
                                        mBaseSpeed - Differential );
            break;
        }
    }
}

void CRobot::Move( float aTimeStep )
{
    const float Distance = mDriveTrain.ForwardDistanceInStep( aTimeStep );
    const float HeadingChange = mDriveTrain.HeadingChangeInStep( aTimeStep );

    mPose.Advance( Distance, HeadingChange );
}

void CRobot::CheckCollision()
{
    const bool IsColliding =
            mrRoom.IsOverlappingWall( mPose.GetPosition(), mRadius );

    // Count only the transition into overlap so one scrape is one collision.
    if( IsColliding && !mWasColliding )
    {
        ++mCollisionCount;
        mrCollisionListener.OnCollision( mUpdateCount );
    }

    mWasColliding = IsColliding;
}

void CRobot::RecordTrail()
{
    mTrail.push_back( mPose.GetPosition() );
}

void CRobot::UpdateLapProgress()
{
    const float DistanceFromStart =
            CGeometry::DistanceBetween( mPose.GetPosition(), mStartPosition );

    // Departure prevents the starting position from immediately completing a lap.
    if( !mHasDeparted )
    {
        mHasDeparted = ( DistanceFromStart > mDepartureRadius );
    }
    else if( !mHasCompletedCircuit )
    {
        mHasCompletedCircuit = ( DistanceFromStart < mReturnRadius );
    }
}

CRobot::eSteerMode CRobot::SelectSteerMode() const
{
    eSteerMode Result = eFollowWall;

    if( mForwardRightRange < mWallAheadThreshold )
    {
        Result = eTurnAwayFromWall;
    }
    else if( mRightRange > mWallLostThreshold )
    {
        Result = eTurnTowardWall;
    }

    return Result;
}

void CRobot::DrawTrail( CRender& arRender ) const
{
    for( std::size_t Index = 1; Index < mTrail.size(); ++Index )
    {
        arRender.DrawLine( mTrail[ Index - 1 ], mTrail[ Index ],
                           mTrailThickness, CRender::mTrailColour );
    }
}

void CRobot::DrawBody( CRender& arRender ) const
{
    const CGeometry::Vec2D Centre = mPose.GetPosition();
    const CGeometry::Vec2D Forward = mPose.GetDirectionAt( 0.0f );
    const CGeometry::Vec2D Nose = CGeometry::Add(
            Centre, CGeometry::Scale( Forward, mHeadingIndicatorLength ) );

    arRender.DrawCircle( Centre, mRadius, CRender::mRobotColour );
    arRender.DrawLine( Centre, Nose, mTrailThickness, CRender::mHeadingColour );
}
