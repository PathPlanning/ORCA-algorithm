#include "MAPFConfig.h"
#include "MAPFActorSet.h"
#include "../SubMap.h"
#include "MAPFSearchResult.h"
#include <vector>

#ifndef ORCASTAR_MAPFSEARCHINTERFACE_H
#define ORCASTAR_MAPFSEARCHINTERFACE_H


class MAPFSearchInterface
{
    public:
        virtual ~MAPFSearchInterface(void) {}
        virtual MAPFSearchResult startSearch(const SubMap &map, const MAPFConfig &config, MAPFActorSet &AgentSet) = 0;
        virtual void clear()
        {
            agentsPaths.clear();
        }

    protected:
        std::vector<std::vector<Node>>  agentsPaths;
};


#endif //ORCASTAR_MAPFSEARCHINTERFACE_H
