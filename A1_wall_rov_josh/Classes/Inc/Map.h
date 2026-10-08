#ifndef MAP_H_
#define MAP_H_

#include "CRender.h"
#include "CLoopReader.h"

#include <iostream>
#include <vector>
#include <string>

// A Map is a collection of walls that the robot interacts with and can be drawn to the simulation window
class Map
{
    public:
        // creates a map with the given render object   
        Map( CRender& arRender );

        // loads the map from the file name given by the command line or a default if none provided
        bool LoadMap( int argc, char* argv[] );

        // draws the loop to the given render object
        void DrawLoop( CRender& aRender, const CLoopReader& aLoop );

        // returns the loop from the read map file
        CLoopReader& GetLoop();

    private:
        CRender& mrRender; // render object used to draw the map
        CLoopReader mLoop; // loop of walls from the map file

};


#endif