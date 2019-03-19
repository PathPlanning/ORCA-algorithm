#include "Point.h"
#include "Line.h"
#include <map>
#include <vector>

#ifndef ORCA_AGENT_H
#define ORCA_AGENT_H
#define Velocity Point
#define eps 1e-5

using namespace std;

class Agent
{
    public:
        Agent();
        Agent(double radius, double maxspeed, int agentsmaxnum, double timeboundary, double sightradius, int id);
        Agent(const Agent &obj);

        void CalculateVelocity();
        void UpdateVelocity();
        void AddNeighbour(Agent& neighbour);
        Point GetPosition();
        Point GetVelocity();
        double GetMaxSpeed();
        void SetPosition(Point pos);
        void SetPrefVelocity(const Velocity &newpref);
        void Stop();
        bool operator == (const Agent &another) const;

    private:
        int id;
        double radius;
        double maxSpeed;
        int agentsMaxNum;
        double timeBoundary;
        double sightRadius;

        multimap <double, Agent*> Neighbours;
        vector <Line> ORCALines;
        Point position;
        Velocity prefV;
        Velocity newV;
        Velocity currV;
        bool finished;
};


#endif //ORCA_AGENT_H
