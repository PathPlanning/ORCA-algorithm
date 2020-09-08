#include "../Geom.h"


#ifndef ORCASTAR_MAPFSEARCHRESULT_H
#define ORCASTAR_MAPFSEARCHRESULT_H


struct MAPFSearchResult
{
        bool                            pathfound;
        std::vector<ActorMove>*         agentsMoves;
        std::vector<std::vector<Node>>* agentsPaths;
        double                          time;
        double                          AvgLLExpansions = 0;
        double                          AvgLLNodes = 0;
        int                             HLExpansions = 0;
        int                             HLNodes = 0;

        MAPFSearchResult(bool Pathfound = false)
        {
            pathfound = Pathfound;
        }
};


#endif //ORCASTAR_MAPFSEARCHRESULT_H
