//-----------------------------------------------------------------------------
// CRoom.cpp
//
// Written by SID: 510516950, SID: 530504205
// Implements room construction, geometric queries and drawing.
//-----------------------------------------------------------------------------

#include "CRoom.h"

#include "CLoopReader.h"
#include "CRender.h"

#include <iostream>

const float CRoom::mWallThickness = 2.0f;

const std::size_t CRoom::mMinimumVertices = 3;

CRoom::CRoom( const CLoopReader& arLoop )
    :
        mWalls(),
        mStartPose( arLoop.GetStartPose() ),
        mIsValid( arLoop.GetVertices().size() >= mMinimumVertices )
{
    if( mIsValid )
    {
        BuildWalls( arLoop.GetVertices() );
    }
    else
    {
        std::cout << "CRoom: a room needs at least " << mMinimumVertices
                  << " vertices, but the map supplied "
                  << arLoop.GetVertices().size() << std::endl;
    }
}

bool CRoom::IsValid() const
{
    return mIsValid;
}

const CGeometry::CPose& CRoom::GetStartPose() const
{
    return mStartPose;
}

bool CRoom::CastRay( const CGeometry::Vec2D& arOrigin,
                     const CGeometry::Vec2D& arDirection,
                     float& arDistanceOut ) const
{
    bool Result = false;
    float Nearest = 0.0f;

    // Test every wall because the sensor requires the nearest intersection.
    for( const Segment& rWall : mWalls )
    {
        float Distance = 0.0f;

        if( CGeometry::RayToSegment( arOrigin, arDirection,
                                     rWall.mStart, rWall.mEnd, Distance ) )
        {
            if( !Result || ( Distance < Nearest ) )
            {
                Nearest = Distance;
                Result = true;
            }
        }
    }

    if( Result )
    {
        arDistanceOut = Nearest;
    }

    return Result;
}

bool CRoom::IsOverlappingWall( const CGeometry::Vec2D& arCentre,
                               float aRadius ) const
{
    bool Result = false;

    for( const Segment& rWall : mWalls )
    {
        if( !Result )
        {
            const float Distance = CGeometry::DistanceToSegment(
                    arCentre, rWall.mStart, rWall.mEnd );

            Result = ( Distance < aRadius );
        }
    }

    return Result;
}

void CRoom::Draw( CRender& arRender ) const
{
    for( const Segment& rWall : mWalls )
    {
        arRender.DrawLine( rWall.mStart, rWall.mEnd,
                           mWallThickness, CRender::mWallColour );
    }
}

void CRoom::BuildWalls( const std::vector<CGeometry::Vec2D>& arVertices )
{
    const std::size_t Count = arVertices.size();

    mWalls.clear();

    for( std::size_t Index = 0; Index < Count; ++Index )
    {
        // Modulo joins the final vertex back to the first.
        const std::size_t NextIndex = ( Index + 1 ) % Count;

        const Segment Wall = { arVertices[ Index ], arVertices[ NextIndex ] };
        mWalls.push_back( Wall );
    }
}
