#ifndef ORCASTAR_MAPFINSTANCESLOGGER_H
#define ORCASTAR_MAPFINSTANCESLOGGER_H


#include <cstdio>
#include <string>
#include <iostream>

#include "MAPF/MAPFActorSet.h"
#include "SubMap.h"

#include "../tinyxml/tinyxml2.h"


using namespace std;
using namespace tinyxml2;


class MAPFInstancesLogger
{
    public:
        MAPFInstancesLogger() = default;
        MAPFInstancesLogger(std::string pathTempl);
        bool SaveInstance(MAPFActorSet &agents, SubMap &map);




    private:
        size_t fileID;
        std::string pathTemplate;


};


#endif //ORCASTAR_MAPFINSTANCESLOGGER_H
