//-----------------------------------------------------------------------------
// CRoom.h
//
// Written by SID: 510516950, SID: 530504205
// Declares the closed wall loop used for sensing, collision and drawing.
//-----------------------------------------------------------------------------

#ifndef CROOM_H
#define CROOM_H

#include "CGeometry.h"

#include <cstddef>
#include <vector>

class CLoopReader;
class CRender;

// CRoom owns the wall segments copied from a map and answers geometric queries.
class CRoom
{
    public:
        explicit CRoom( const CLoopReader& arLoop );

        bool IsValid() const;

        const CGeometry::CPose& GetStartPose() const;

        // Returns the nearest ray hit and writes the distance only on success.
        bool CastRay( const CGeometry::Vec2D& arOrigin,
                      const CGeometry::Vec2D& arDirection,
                      float& arDistanceOut ) const;

        // Tests disc overlap against every finite wall segment.
        bool IsOverlappingWall( const CGeometry::Vec2D& arCentre,
                                float aRadius ) const;

        void Draw( CRender& arRender ) const;

    private:
        struct Segment
        {
            CGeometry::Vec2D mStart;
            CGeometry::Vec2D mEnd;
        };

        void BuildWalls( const std::vector<CGeometry::Vec2D>& arVertices );

        static const float mWallThickness;

        static const std::size_t mMinimumVertices;

        std::vector<Segment> mWalls;

        CGeometry::CPose mStartPose;

        bool mIsValid;
};

#endif
