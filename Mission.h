#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "Agent.h"
#include "Point.h"
#include "XmlLogger.h"

using namespace std;
using namespace tinyxml2;

#ifndef ORCA_MISSION_H
#define ORCA_MISSION_H


class Mission
{
    public:
        Mission(string filename);
        ~Mission();
        void StartMission();



    private:
        string fileName;
        vector<pair<Agent, Point>> agents;
        int step;
        double timeStep;
        int agentNumber;
        double defaultRadius;
        double defaultMaxSpeed;
        int defaultAgentsMaxNum;
        double defaultTimeBoundary;
        double defaultSightRadius;
        double delta;
        XmlLogger *log;

        bool isFinished();
        bool ReadMissionFromFile();




};


#endif //ORCA_MISSION_H
