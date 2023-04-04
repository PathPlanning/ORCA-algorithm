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

        virtual bool readData() = 0;
        virtual bool getMap(Map **map) = 0;
        virtual bool getEnvironmentOptions(EnvironmentOptions **env_opt) = 0;
        virtual bool getAgents(std::vector<Agent *> &agents, const int &num_threshold) = 0;
        virtual Reader* Clone() const = 0;
};


#endif //ORCA_READER_H
