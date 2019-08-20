#include "Const.h"

#ifndef ORCA_ENVIRONMENTOPTIONS_H
#define ORCA_ENVIRONMENTOPTIONS_H


class EnvironmentOptions
{
    public:
        EnvironmentOptions() = default;
        EnvironmentOptions(const EnvironmentOptions &obj);
        EnvironmentOptions(int mt, bool bt, bool as, bool cc, float hw, float ts, float del);

        int     metrictype;     //Can be chosen Euclidean, Manhattan, Chebyshev and Diagonal distance
        bool    breakingties;   //Option that defines the priority in OPEN list for nodes with equal f-values.
        bool    allowsqueeze;   //Option that allows to move through "bottleneck"
        bool    cutcorners;     //Option that allows to make diagonal moves, when one adjacent cell is untraversable
        float   hweight;        //Option that defines the weight of the heuristic function
        float   timestep;       //Option that defines time per one step of simulation
        float   delta;          //Option that defines max distance to goal position, which is considered as reaching the goal position

};




#endif //ORCA_ENVIRONMENTOPTIONS_H
