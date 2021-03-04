#include <set>
#include <algorithm>

#include "Agent.h"
#include "SubMap.h"
#include "MAPF/MAPFActorSet.h"
#include "MAPF/MAPFActor.h"
#include "MAPF/ECBS/ConflictBasedSearch.h"
#include "MAPF/AStar.h"
#include "MAPF/FocalSearch.h"
#include "MAPF/SCIPP.h"
#include "MAPFInstancesLogger.h"

#ifndef ORCASTAR_ORCAAGENTWITHECBS_H
#define ORCASTAR_ORCAAGENTWITHECBS_H


class ORCAAgentWithECBS : public Agent
{

    public:
        ORCAAgentWithECBS();
        ORCAAgentWithECBS(const int &id, const Point &start, const Point &goal, const Map &map, const EnvironmentOptions &options,
                         AgentParam param);
        ORCAAgentWithECBS(const ORCAAgentWithECBS &obj);
        ~ORCAAgentWithECBS();


        ORCAAgentWithECBS* Clone() const override ;
        void ComputeNewVelocity() override;
        void ApplyNewVelocity() override;
        bool UpdatePrefVelocity() override;
        void AddNeighbour(Agent &neighbour, float distSq) override;

        bool isMAPFMember() const;


        bool operator == (const ORCAAgentWithECBS &another) const;
        bool operator != (const ORCAAgentWithECBS &another) const;
        ORCAAgentWithECBS &operator = (const ORCAAgentWithECBS &obj);
#if PAR_LOG
        void SetMAPFInstanceLoggerRef(MAPFInstancesLogger *log);
#endif

    private:
        std::vector <std::pair<float, Agent*>>& GetNeighbours();
        std::set<ORCAAgentWithECBS *> GetAgentsForCentralizedPlanning();
        void SetAgentsForCentralizedPlanning(std::set<ORCAAgentWithECBS *> agents);
        void PrepareMAPFExecution(Point common);
        bool ComputeMAPFEnv(Point common, std::vector<std::pair<Point, ORCAAgentWithECBS*>> oldGoals = std::vector<std::pair<Point, ORCAAgentWithECBS*>>());
        Point PullOutIntermediateGoal(Point common);
        bool ComputeMAPF();
        bool UniteMAPF();
        bool UpdateMAPF();


        SubMap MAPFMap;
        MAPFActorSet MAPFSet;
        MAPFConfig conf;
        std::set<ORCAAgentWithECBS *> MAPFAgents;
        float fakeRadius;
        bool inMAPFMode;
        bool moveToMAPFPos;
        bool MAPFVis;
        bool notMAPFVis;
        bool MAPFUnion;
        bool MAPFExec;
        Point MAPFStart;
        Point MAPFGoal;
        FocalSearch<> MAPFsearch;
        ConflictBasedSearch<FocalSearch<>> ECBSSolver;
        MAPFSearchResult MAPFres;
        int currMAPFPos;
        int MAPFActorId;
        Point MAPFcommon;
        std::vector<Point> buffMAPF;
#if PAR_LOG
        MAPFInstancesLogger *MAPFLog;
#endif
};


#endif //ORCASTAR_ORCAAGENTWITHECBS_H
