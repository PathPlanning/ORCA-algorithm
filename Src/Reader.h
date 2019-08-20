#include <vector>

#include "Map.h"
#include "EnvironmentOptions.h"
#include "Agent.h"

#ifndef ORCA_READER_H
#define ORCA_READER_H



class Reader
{
    public:
        virtual ~Reader(){}

        virtual bool ReadData() = 0;
        virtual bool GetMap(Map **map) = 0;
        virtual bool GetEnvironmentOptions(EnvironmentOptions **envOpt) = 0;
        virtual bool GetAgents(std::vector<Agent *> &agents, const int &numThreshold) = 0;
        virtual Reader* Clone() const = 0;
};


#endif //ORCA_READER_H
