#include <iostream>
#include "Agent.h"
#include "Point.h"
#include "XmlLogger.h"
#include "Mission.h"
#include <fstream>
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
    string inp;
    if(argc < 2)
    {
        std::cout<<"Error! Pathfinding task file (XML) is not specified!"<<std::endl;
        return -1;
    }
    else
    {
        inp = string(argv[1]);
    }
    Mission mission(inp);
    mission.StartMission();

    return 0;
}