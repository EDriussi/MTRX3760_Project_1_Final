//-----------------------------------------------------------------------------
// CGeometry.cpp
//
// Written by SID: 510516950, SID: 530504205
// Implements the shared geometry and pose operations.
//-----------------------------------------------------------------------------

#include "CGeometry.h"

#include <cmath>

const float CGeometry::mPi = 3.14159265358979323846f;

const float CGeometry::mDegreesToRadians = CGeometry::mPi / 180.0f;

const float CGeometry::mParallelTolerance = 1.0e-6f;
const float CGeometry::mSquaredLengthTolerance = 1.0e-12f;

CGeometry::CPose::CPose()
    :
        mPosition( { 0.0f, 0.0f } ),
        mHeading( 0.0f )
{
}

CGeometry::CPose::CPose( const Vec2D& arPosition, float aHeadingRadians )
    :
        mPosition( arPosition ),
        mHeading( WrapAngle( aHeadingRadians ) )
{
}

void CGeometry::CPose::Advance( float aDistance, float aHeadingChange )
{
    const Vec2D Forward = DirectionFromAngle( mHeading );

    mPosition = Add( mPosition, Scale( Forward, aDistance ) );
    mHeading = WrapAngle( mHeading + aHeadingChange );
}

const CGeometry::Vec2D& CGeometry::CPose::GetPosition() const
{
    return mPosition;
}

float CGeometry::CPose::GetHeading() const
{
    return mHeading;
}

CGeometry::Vec2D CGeometry::CPose::GetDirectionAt(
        float aRelativeAngleRadians ) const
{
    return DirectionFromAngle( mHeading + aRelativeAngleRadians );
}

CGeometry::Vec2D CGeometry::Add( const Vec2D& arA, const Vec2D& arB )
{
    const Vec2D Result = { arA.x + arB.x, arA.y + arB.y };

    return Result;
}

CGeometry::Vec2D CGeometry::Scale( const Vec2D& arVector, float aScale )
{
    const Vec2D Result = { arVector.x * aScale, arVector.y * aScale };

    return Result;
}

CGeometry::Vec2D CGeometry::DirectionFromAngle( float aRadians )
{
    const Vec2D Result = { std::cos( aRadians ), std::sin( aRadians ) };

    return Result;
}

float CGeometry::Length( const Vec2D& arVector )
{
    return std::sqrt( ( arVector.x * arVector.x )
                    + ( arVector.y * arVector.y ) );
}

float CGeometry::DistanceBetween( const Vec2D& arA, const Vec2D& arB )
{
    const Vec2D Separation = { arB.x - arA.x, arB.y - arA.y };

    return Length( Separation );
}

float CGeometry::DistanceToSegment( const Vec2D& arPoint,
                                    const Vec2D& arStart,
                                    const Vec2D& arEnd )
{
    const Vec2D Edge = { arEnd.x - arStart.x, arEnd.y - arStart.y };
    const Vec2D ToPoint = { arPoint.x - arStart.x, arPoint.y - arStart.y };

    const float EdgeLengthSquared = ( Edge.x * Edge.x ) + ( Edge.y * Edge.y );

    float Projection = 0.0f;

    if( EdgeLengthSquared > mSquaredLengthTolerance )
    {
        Projection = ( ( ToPoint.x * Edge.x ) + ( ToPoint.y * Edge.y ) )
                   / EdgeLengthSquared;

        if( Projection < 0.0f )
        {
            Projection = 0.0f;
        }
        else if( Projection > 1.0f )
        {
            Projection = 1.0f;
        }
    }

    const Vec2D Nearest = Add( arStart, Scale( Edge, Projection ) );

    return DistanceBetween( arPoint, Nearest );
}

bool CGeometry::RayToSegment( const Vec2D& arOrigin,
                              const Vec2D& arDirection,
                              const Vec2D& arSegmentStart,
                              const Vec2D& arSegmentEnd,
                              float& arDistanceOut )
{
    bool Result = false;

    const Vec2D Edge = { arSegmentEnd.x - arSegmentStart.x,
                         arSegmentEnd.y - arSegmentStart.y };

    const float Denominator = ( arDirection.x * Edge.y )
                            - ( arDirection.y * Edge.x );

    if( ( Denominator > mParallelTolerance )
     || ( Denominator < -mParallelTolerance ) )
    {
        const Vec2D ToStart = { arSegmentStart.x - arOrigin.x,
                                arSegmentStart.y - arOrigin.y };

        const float AlongRay = ( ( ToStart.x * Edge.y )
                               - ( ToStart.y * Edge.x ) ) / Denominator;

        const float AlongSegment = ( ( ToStart.x * arDirection.y )
                                   - ( ToStart.y * arDirection.x ) )
                                 / Denominator;

        if( ( AlongRay >= 0.0f )
         && ( AlongSegment >= 0.0f )
         && ( AlongSegment <= 1.0f ) )
        {
            arDistanceOut = AlongRay;
            Result = true;
        }
    }

    return Result;
}

float CGeometry::WrapAngle( float aRadians )
{
    const float FullTurn = 2.0f * mPi;

    float Result = aRadians;

    while( Result > mPi )
    {
        Result -= FullTurn;
    }

    while( Result <= -mPi )
    {
        Result += FullTurn;
    }

    return Result;
}
