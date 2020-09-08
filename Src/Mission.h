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
#include "MAPFInstancesLogger.h"

using namespace tinyxml2;

class Mission
{
    public:
        Mission() = delete;
        Mission(std::string fileName, unsigned int agentsNum, unsigned int stepsTh, bool time, size_t timeTh);
        Mission (const Mission &obj);
        ~Mission();

        bool ReadTask();
        Summary StartMission();

        Mission & operator = (const Mission &obj);

#if FULL_LOG
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
        MAPFInstancesLogger PARLog;

        bool isTimeBounded;
        size_t timeTreshhold;

        unsigned int stepsCount;
        unsigned int stepsTreshhold;
        unsigned int agentsNum;
        unsigned int collisionsCount;
        unsigned int collisionsObstCount;

#if FULL_LOG
        Logger *taskLogger;
        std::unordered_map<int, std::vector<Point>> stepsLog;
        std::unordered_map<int, std::vector<Point>> goalsLog;
#endif
};


#endif //ORCA_MISSION_H
