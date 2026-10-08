#include "Map.h"


Map::Map( CRender& arRender )
    :   mrRender( arRender )
{
}

bool Map::LoadMap( int argc, char* argv[] )
{
    // View the file named on the command line, or a default if none is given.
    std::string Filename = "SimpleWalls.map";
    if( argc > 1 )
    {
        Filename = argv[1];
    }

    bool ReadSuccess = mLoop.ReadFile( Filename );
    std::cout << "Read " << (ReadSuccess ? "successful":"unsuccessful") << std::endl;
    
    return ReadSuccess;
}

void Map::DrawLoop( CRender& aRender, const CLoopReader& aLoop )
{
    const float EdgeThickness = 2.0f;

    const std::vector<Vec2D>& Vertices = aLoop.GetVertices();

    //std::cout << "Vertices size: " << Vertices.size() << std::endl;
    if( !Vertices.empty() )
    {
        // Start from the last vertex so the first edge drawn closes the loop.
        Vec2D Previous = Vertices.back();
        for( const Vec2D& Vertex : Vertices )
        {
            aRender.DrawLine( Previous, Vertex, EdgeThickness, RAYWHITE );
            Previous = Vertex;
        }
    }
}

CLoopReader& Map::GetLoop()
{
    return mLoop;
}