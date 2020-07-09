#ifndef ORCASTAR_PARINSTANCESLOGGER_H
#define ORCASTAR_PARINSTANCESLOGGER_H


#include <cstdio>
#include <string>
#include <iostream>

#include "PARActorSet.h"
#include "SubMap.h"

#include "../tinyxml/tinyxml2.h"


using namespace std;
using namespace tinyxml2;


class PARInstancesLogger
{
    public:
        PARInstancesLogger() = default;
        PARInstancesLogger(std::string pathTempl);
        bool SaveInstance(PARActorSet &agents, SubMap &map);




    private:
        size_t fileID;
        std::string pathTemplate;


};


#endif //ORCASTAR_PARINSTANCESLOGGER_H
