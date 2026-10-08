//-----------------------------------------------------------------------------
// main.cpp
//
// Written by SID: 510516950, SID: 530504205
// Loads an optional map filename and runs the A1 simulation.
//-----------------------------------------------------------------------------

#include "CSimulation.h"

#include <iostream>
#include <string>

int main( int argc, char* argv[] )
{
    int Result = 0;

    std::string Filename = CSimulation::GetDefaultMapFilename();
    if( argc > 1 )
    {
        Filename = argv[ 1 ];
    }

    CSimulation Simulation( Filename );

    if( Simulation.IsValid() )
    {
        Simulation.Run();
    }
    else
    {
        std::cout << "Nothing to run: '" << Filename
                  << "' did not describe a usable room." << std::endl;
        Result = 1;
    }

    return Result;
}
