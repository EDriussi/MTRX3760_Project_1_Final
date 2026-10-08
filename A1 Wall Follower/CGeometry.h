//-----------------------------------------------------------------------------
// CGeometry.h
//
// Written by SID: 510516950, SID: 530504205
// Shared two-dimensional geometry and pose operations.
//-----------------------------------------------------------------------------

#ifndef CGEOMETRY_H
#define CGEOMETRY_H

// CGeometry owns the geometric types and stateless operations used by the
// simulator, keeping them out of namespace scope.
class CGeometry
{
    public:
        struct Vec2D
        {
            float x;                // rightward, in renderer units
            float y;                // downward, in renderer units
        };

        class CPose
        {
            public:
                CPose();

                // Constructs a pose with its heading wrapped into (-Pi, Pi].
                CPose( const Vec2D& arPosition, float aHeadingRadians );

                void Advance( float aDistance, float aHeadingChange );

                const Vec2D& GetPosition() const;

                float GetHeading() const;

                Vec2D GetDirectionAt( float aRelativeAngleRadians ) const;

            private:
                Vec2D mPosition;

                float mHeading;
        };

        static const float mPi;

        static const float mDegreesToRadians;

        static Vec2D Add( const Vec2D& arA, const Vec2D& arB );

        static Vec2D Scale( const Vec2D& arVector, float aScale );

        static Vec2D DirectionFromAngle( float aRadians );

        static float Length( const Vec2D& arVector );

        static float DistanceBetween( const Vec2D& arA, const Vec2D& arB );

        static float DistanceToSegment( const Vec2D& arPoint,
                                        const Vec2D& arStart,
                                        const Vec2D& arEnd );

        static bool RayToSegment( const Vec2D& arOrigin,
                                  const Vec2D& arDirection,
                                  const Vec2D& arSegmentStart,
                                  const Vec2D& arSegmentEnd,
                                  float& arDistanceOut );

        static float WrapAngle( float aRadians );

    private:
        static const float mParallelTolerance;
        static const float mSquaredLengthTolerance;
};

#endif
