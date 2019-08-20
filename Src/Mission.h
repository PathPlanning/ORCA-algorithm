#ifndef ORCA_MISSION_H
#define ORCA_MISSION_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <unordered_map>
#include <chrono>

#include "Agent.h"
#include "Summary.h"
#include "XMLReader.h"
#include "XMLLogger.h"

using namespace tinyxml2;

class Mission
{
    public:
        Mission() = delete;
        Mission(std::string fileName, unsigned int agentsNum, unsigned int stepsTh);
        Mission (const Mission &obj);
        ~Mission();

        bool ReadTask();
        Summary StartMission();

        Mission & operator = (const Mission &obj);

#ifndef NDEBUG
        bool SaveLog();
#endif

    private:
        void UpdateSate();
        void AssignNeighbours();
        bool IsFinished();

        vector<Agent *> agents;
        Reader *taskReader;
        Map *map;
        EnvironmentOptions *options;
        Summary missionResult;
        std::unordered_map<int, std::pair<bool, int>> resultsLog;

        unsigned int stepsCount;
        unsigned int stepsTreshhold;
        unsigned int agentsNum;
        unsigned int collisionsCount;
        unsigned int collisionsObstCount;

#ifndef NDEBUG
        Logger *taskLogger;
        std::unordered_map<int, std::vector<Point>> stepsLog;
#endif
};


#endif //ORCA_MISSION_H
