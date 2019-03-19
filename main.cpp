#include <iostream>
#include "Agent.h"
#include "Point.h"
#include "XmlLogger.h"
#include "Mission.h"
#include <fstream>

int main()
{
    Mission aaa("example.xml");
    aaa.StartMission();

    return 0;
}