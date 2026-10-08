//-----------------------------------------------------------------------------
// CLoopReader.h
//
// Written by SID: 510516950, SID: 530504205. Adapted from the file supplied
// Reads one named loop, start pose and ordered vertex list from a map file.
//-----------------------------------------------------------------------------

#ifndef CLOOPREADER_H
#define CLOOPREADER_H

#include "CGeometry.h"

#include <iosfwd>
#include <string>
#include <vector>

// CLoopReader validates the map while retaining the parsed loop data.
class CLoopReader
{
    public:
        CLoopReader();

        bool ReadFile( const std::string& arFilename );

        const std::string& GetName() const;

        const CGeometry::CPose& GetStartPose() const;

        const std::vector<CGeometry::Vec2D>& GetVertices() const;

    private:

        bool ParseLine( const std::string& arLine, int aLineNumber );

        bool ParseLoop( std::istream& arWords, int aLineNumber );

        bool ParseStartPose( std::istream& arWords, int aLineNumber );

        bool ParseVertex( std::istream& arWords, int aLineNumber );

        static bool HasExtraWords( std::istream& arWords );

        std::string mName;
        CGeometry::CPose mStartPose;

        std::vector<CGeometry::Vec2D> mVertices;

        bool mHaveLoop;
        bool mHaveStartPose;
};

#endif
