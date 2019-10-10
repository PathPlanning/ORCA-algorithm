#include "Const.h"
#include "PathPlanner.h"
#include "ThetaStar.h"


#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <type_traits>
#include <cstddef>

#ifndef ORCA_AGENT_H
#define ORCA_AGENT_H

class AgentParam
{
    public:
        AgentParam() : sightRadius(CN_DEFAULT_RADIUS_OF_SIGHT), timeBoundary(CN_DEFAULT_TIME_BOUNDARY), timeBoundaryObst(CN_DEFAULT_OBS_TIME_BOUNDARY),
            radius(CN_DEFAULT_SIZE), maxSpeed(CN_DEFAULT_MAX_SPEED), agentsMaxNum(CN_DEFAULT_AGENTS_MAX_NUM) {}
        AgentParam(float sr, float tb, float tbo, float r, float ms, int amn) : sightRadius(sr), timeBoundary(tb), timeBoundaryObst(tbo), radius(r),
            maxSpeed(ms), agentsMaxNum(amn) {}
        ~AgentParam() = default;

        float sightRadius;
        float timeBoundary;
        float timeBoundaryObst;
        float radius;
        float maxSpeed;
        int agentsMaxNum;
};

class Agent
{
    public:
        Agent();
        Agent(const int &id, const Point &start, const Point &goal, const Map &map, const EnvironmentOptions &options, AgentParam param);
        Agent(const Agent &obj);
        ~Agent();

        bool InitPath();

        void ComputeNewVelocity();
        void ApplyNewVelocity();
        bool UpdatePrefVelocity();

        Point GetPosition() const;
        Point GetVelocity() const;
        float GetRadius() const;
        float GetSightRadius() const;
        std::pair<unsigned int, unsigned int> GetCollision() const;
        int GetID() const;

        void SetPosition(const Point &pos);

        bool isFinished();
        void AddNeighbour(Agent &neighbour, float distSq);
        void UpdateNeighbourObst();

        bool operator == (const Agent &another) const;
        bool operator != (const Agent &another) const;
        Agent & operator = (const Agent& obj);

        template <class Planner> void SetPlanner(const Planner &pl)
        {
            static_assert(std::is_base_of<PathPlanner, Planner>::value, "Planner should be inheritor of PathPlanner");
            this->planner = new Planner(pl);
        }


    private:
        Point start;
        Point goal;
        PathPlanner *planner;
        const EnvironmentOptions *options;
        const Map *map;
        std::vector <std::pair<float, Agent*>> Neighbours;
        std::vector <std::pair<float, ObstacleSegment>> NeighboursObst;
        std::vector <Line> ORCALines;
        int id;
        Point position;
        Velocity prefV;
        Velocity newV;
        Velocity currV;

        AgentParam param;
        float invTimeBoundaryObst;
        float invTimeBoundary;
        float maxSqObstDist;

        unsigned int collisions;
        unsigned int collisionsObst;

};





#endif //ORCA_AGENT_H
