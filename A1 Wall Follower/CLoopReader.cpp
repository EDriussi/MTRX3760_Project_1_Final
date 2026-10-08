//-----------------------------------------------------------------------------
// CLoopReader.cpp
//
// Written by SID: 510516950, SID: 530504205. Adapted from the file supplied
// Implements map parsing and validation.
//-----------------------------------------------------------------------------

#include "CLoopReader.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

CLoopReader::CLoopReader()
    :
        mName(),
        mStartPose(),
        mVertices(),
        mHaveLoop( false ),
        mHaveStartPose( false )
{
}

bool CLoopReader::ReadFile( const std::string& arFilename )
{
    bool Okay = true;

    mName.clear();
    mVertices.clear();
    mStartPose = CGeometry::CPose();
    mHaveLoop = false;
    mHaveStartPose = false;

    std::ifstream File( arFilename );
    if( !File )
    {
        std::cout << "CLoopReader: could not open file '"
                  << arFilename << "'" << std::endl;
        Okay = false;
    }

    std::string Line;
    int LineNumber = 0;

    while( Okay && std::getline( File, Line ) )
    {
        ++LineNumber;
        Okay = ParseLine( Line, LineNumber );
    }

    if( Okay && !mHaveLoop )
    {
        std::cout << "CLoopReader: file contains no 'loop' line" << std::endl;
        Okay = false;
    }
    else if( Okay && !mHaveStartPose )
    {
        std::cout << "CLoopReader: file contains no 'startpose' line"
                  << std::endl;
        Okay = false;
    }

    // Never expose a partially parsed loop as usable data.
    if( !Okay )
    {
        mName.clear();
        mVertices.clear();
        mStartPose = CGeometry::CPose();
        mHaveLoop = false;
        mHaveStartPose = false;
    }

    return Okay;
}

const std::string& CLoopReader::GetName() const
{
    return mName;
}

const CGeometry::CPose& CLoopReader::GetStartPose() const
{
    return mStartPose;
}

const std::vector<CGeometry::Vec2D>& CLoopReader::GetVertices() const
{
    return mVertices;
}

bool CLoopReader::ParseLine( const std::string& arLine, int aLineNumber )
{
    bool Okay = true;

    std::string Content = arLine;

    const std::string::size_type Hash = Content.find( '#' );
    if( Hash != std::string::npos )
    {
        Content = Content.substr( 0, Hash );
    }

    std::istringstream Words( Content );
    std::string Keyword;

    if( Words >> Keyword )
    {
        if( Keyword == "loop" )
        {
            Okay = ParseLoop( Words, aLineNumber );
        }
        else if( Keyword == "startpose" )
        {
            Okay = ParseStartPose( Words, aLineNumber );
        }
        else if( Keyword == "vertex" )
        {
            Okay = ParseVertex( Words, aLineNumber );
        }
        else
        {
            std::cout << "CLoopReader: unknown keyword '" << Keyword
                      << "' on line " << aLineNumber << std::endl;
            Okay = false;
        }
    }

    return Okay;
}

bool CLoopReader::ParseLoop( std::istream& arWords, int aLineNumber )
{
    bool Okay = true;
    std::string Name;

    if( mHaveLoop )
    {
        std::cout << "CLoopReader: a second 'loop' on line " << aLineNumber
                  << " (a file describes one loop)" << std::endl;
        Okay = false;
    }
    else if( !( arWords >> Name ) )
    {
        std::cout << "CLoopReader: 'loop' needs a name on line "
                  << aLineNumber << std::endl;
        Okay = false;
    }
    else if( HasExtraWords( arWords ) )
    {
        std::cout << "CLoopReader: unexpected value after 'loop' on line "
                  << aLineNumber << std::endl;
        Okay = false;
    }
    else
    {
        mName = Name;
        mHaveLoop = true;
    }

    return Okay;
}

bool CLoopReader::ParseStartPose( std::istream& arWords, int aLineNumber )
{
    bool Okay = true;

    float X = 0.0f;
    float Y = 0.0f;
    float HeadingDegrees = 0.0f;

    if( !mHaveLoop )
    {
        std::cout << "CLoopReader: 'startpose' before any 'loop' on line "
                  << aLineNumber << std::endl;
        Okay = false;
    }
    else if( mHaveStartPose )
    {
        std::cout << "CLoopReader: a second 'startpose' on line "
                  << aLineNumber << std::endl;
        Okay = false;
    }
    else if( !( arWords >> X >> Y >> HeadingDegrees ) )
    {
        std::cout << "CLoopReader: 'startpose' needs x, y and heading on line "
                  << aLineNumber << std::endl;
        Okay = false;
    }
    else if( !std::isfinite( X ) || !std::isfinite( Y )
          || !std::isfinite( HeadingDegrees ) )
    {
        std::cout << "CLoopReader: non-finite 'startpose' value on line "
                  << aLineNumber << std::endl;
        Okay = false;
    }
    else if( HasExtraWords( arWords ) )
    {
        std::cout << "CLoopReader: unexpected value after 'startpose' on line "
                  << aLineNumber << std::endl;
        Okay = false;
    }
    else
    {
        const CGeometry::Vec2D Position = { X, Y };
        mStartPose = CGeometry::CPose(
                Position, HeadingDegrees * CGeometry::mDegreesToRadians );
        mHaveStartPose = true;
    }

    return Okay;
}

bool CLoopReader::ParseVertex( std::istream& arWords, int aLineNumber )
{
    bool Okay = true;

    float X = 0.0f;
    float Y = 0.0f;

    if( !mHaveLoop )
    {
        std::cout << "CLoopReader: 'vertex' before any 'loop' on line "
                  << aLineNumber << std::endl;
        Okay = false;
    }
    else if( !( arWords >> X >> Y ) )
    {
        std::cout << "CLoopReader: 'vertex' needs x and y on line "
                  << aLineNumber << std::endl;
        Okay = false;
    }
    else if( !std::isfinite( X ) || !std::isfinite( Y ) )
    {
        std::cout << "CLoopReader: non-finite 'vertex' value on line "
                  << aLineNumber << std::endl;
        Okay = false;
    }
    else if( HasExtraWords( arWords ) )
    {
        std::cout << "CLoopReader: unexpected value after 'vertex' on line "
                  << aLineNumber << std::endl;
        Okay = false;
    }
    else
    {
        const CGeometry::Vec2D Vertex = { X, Y };
        mVertices.push_back( Vertex );
    }

    return Okay;
}

bool CLoopReader::HasExtraWords( std::istream& arWords )
{
    std::string Extra;
    return static_cast<bool>( arWords >> Extra );
}
