#include "Robot.h"
#include "Map.h"
#include "CLoopReader.h"
#include "main.h"

#include <iostream>

int main()
{
    CRender Render;
    Map map( Render );

    map.LoadMap( 0, nullptr );

    Robot* robot = new Robot(Render, map.GetLoop() );

    int numCycles = 0;

    //---The main loop---
    while (numCycles++ < MAX_LOOP_CYCLES && !Render.WindowShouldClose())
    {
        Render.BeginDrawing();
        
        map.DrawLoop( Render, map.GetLoop() );
        
        robot->Update();
        robot->Draw();
        
                 
        Render.EndDrawing();
        //std::cout << "Cycle: " << numCycles << std::endl;
    }

    robot->Report();

    //---Cleanup---
    Render.CloseWindow();

    delete robot;

    return 0;
}