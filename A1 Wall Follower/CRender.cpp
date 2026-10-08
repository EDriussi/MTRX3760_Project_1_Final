//-----------------------------------------------------------------------------
// CRender.cpp
//
// Written by SID: 510516950, SID: 530504205
// Implements the raylib wrapper; raylib is not accessed from any other file.
//-----------------------------------------------------------------------------

#include "CRender.h"

#include "raylib.h"

const CRender::Colour CRender::mBackgroundColour = {  10,  10,  16, 255 };
const CRender::Colour CRender::mWallColour       = { 245, 245, 245, 255 };
const CRender::Colour CRender::mRobotColour      = {  90, 190, 255, 255 };
const CRender::Colour CRender::mTrailColour      = { 255, 140,  60, 200 };
const CRender::Colour CRender::mHeadingColour    = { 255, 255, 255, 255 };

const char* const CRender::mpWindowTitle = "MTRX3760 Lab 2 A1 - Wall Follower";

CRender::CRender()
{
    ::InitWindow( mScreenWidth, mScreenHeight, mpWindowTitle );
    ::SetTargetFPS( mTargetFramesPerSecond );
}

bool CRender::WindowShouldClose()
{
    return ::WindowShouldClose();
}

CRender::~CRender()
{
    ::CloseWindow();
}

void CRender::BeginDrawing()
{
    const Color Background = { mBackgroundColour.r,
                               mBackgroundColour.g,
                               mBackgroundColour.b,
                               mBackgroundColour.a };

    ::BeginDrawing();
    ::ClearBackground( Background );
}

void CRender::EndDrawing()
{
    ::EndDrawing();
}

void CRender::DrawCircle( const CGeometry::Vec2D& arCentre,
                          float aRadius,
                          const Colour& arColour )
{
    const Vector2 Centre = { arCentre.x, arCentre.y };
    const Color DrawColour = { arColour.r, arColour.g, arColour.b, arColour.a };

    ::DrawCircleV( Centre, aRadius, DrawColour );
}

void CRender::DrawLine( const CGeometry::Vec2D& arStart,
                        const CGeometry::Vec2D& arEnd,
                        float aThickness,
                        const Colour& arColour )
{
    const Vector2 Start = { arStart.x, arStart.y };
    const Vector2 End = { arEnd.x, arEnd.y };
    const Color DrawColour = { arColour.r, arColour.g, arColour.b, arColour.a };

    ::DrawLineEx( Start, End, aThickness, DrawColour );
}

int CRender::GetScreenWidth()
{
    return mScreenWidth;
}

int CRender::GetScreenHeight()
{
    return mScreenHeight;
}
