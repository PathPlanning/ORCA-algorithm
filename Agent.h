#include "Point.h"
#include <map>

#ifndef ORCA_AGENT_H
#define ORCA_AGENT_H
#define Velocity Point

using namespace std;

class Agent
{
    public:
        void UpdateVelocity();
        void AddNeighbour(Agent& neighbour);
        Point GetPosition();

    private:
        double radius;
        double maxSpeed;
        double timeBoundary;

        multimap<double, Agent*> agentsNeighbours;
        Point position;
        Velocity prefV;
        Velocity newV;
        Velocity currV;

};


#endif //ORCA_AGENT_H
