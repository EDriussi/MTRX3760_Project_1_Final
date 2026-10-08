//-----------------------------------------------------------------------------
// CRender.h
//
// Written by SID: 510516950, SID: 530504205
// Declares the C++ interface that contains all raylib access.
//-----------------------------------------------------------------------------

#ifndef CRENDER_H
#define CRENDER_H

#include "CGeometry.h"

// CRender owns the application window and provides the simulator's drawing
// operations without exposing raylib types.
class CRender
{
    public:
        struct Colour
        {
            unsigned char r;                        // red, 0 to 255
            unsigned char g;                        // green, 0 to 255
            unsigned char b;                        // blue, 0 to 255
            unsigned char a;                        // opacity, 255 is opaque
        };

        static const Colour mBackgroundColour;   // cleared to this every frame
        static const Colour mWallColour;         // the room's segments
        static const Colour mRobotColour;        // the robot's disc
        static const Colour mTrailColour;        // the path travelled, partly
        static const Colour mHeadingColour;      // the nose line

        CRender();

        ~CRender();

        bool WindowShouldClose();

        void BeginDrawing();

        void EndDrawing();

        void DrawCircle( const CGeometry::Vec2D& arCentre,
                         float aRadius,
                         const Colour& arColour );

        void DrawLine( const CGeometry::Vec2D& arStart,
                       const CGeometry::Vec2D& arEnd,
                       float aThickness,
                       const Colour& arColour );

        static int GetScreenWidth();
        static int GetScreenHeight();

    private:
        static const int mScreenWidth = 800;
        static const int mScreenHeight = 600;

        static const int mTargetFramesPerSecond = 60;

        static const char* const mpWindowTitle;

        CRender( const CRender& );
        CRender& operator=( const CRender& );
};

#endif
