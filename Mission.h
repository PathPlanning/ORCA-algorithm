#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>

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
        Mission(string taskFile, string commLogFile, string logFile, int agentsTreshhold);
        ~Mission();
        void StartMission();



    private:
        string fileName;
        string logFile;
        vector<pair<Agent, Point>> agents;
        int step;
        float timeStep;
        int agentNumber;
        float defaultRadius;
        float defaultMaxSpeed;
        int defaultAgentsMaxNum;
        float defaultTimeBoundary;
        float defaultSightRadius;
        float delta;
        XmlLogger *log;
        ofstream commonLog;
        vector<vector<pair<float, float>>> stepsLog;
        vector<pair<bool, int>> results;
        bool isFinished();
        bool ReadMissionFromFile();
        int stepsTreshhold;
        int agNumTreshhold;




};


#endif //ORCA_MISSION_H
