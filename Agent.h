#include "Point.h"
#include "Line.h"
#include <vector>
#include <cmath>
#include<algorithm>

#ifndef ORCA_AGENT_H
#define ORCA_AGENT_H
#define Velocity Point
#define eps 0.00001f

using namespace std;

class Agent
{
    public:
        Agent();
        Agent(float radius, float maxspeed, int agentsmaxnum, float timeboundary, float timeStep, float sightradius, int id);
        Agent(const Agent &obj);


        void CalculateVelocity(int *colission);
        void UpdateVelocity();
        void AddNeighbour(Agent& neighbour);
        Point GetPosition();
        Point GetVelocity();
        Point GetPrefVelocity();
        float GetMaxSpeed();
        int GetID();
        void SetPosition(Point pos);
        void SetPrefVelocity(const Velocity &newpref);
        bool operator == (const Agent &another) const;
        bool operator != (const Agent &another) const;

    private:
        int id;
        float radius;
        float maxSpeed;
        int agentsMaxNum;
        float timeBoundary;
        float timeStep;
        float sightRadius;
        int count;

        vector <pair<float, Agent*>> Neighbours;
        vector <Line> ORCALines;
        Point position;
        Velocity prefV;
        Velocity newV;
        Velocity currV;
};


#endif //ORCA_AGENT_H
