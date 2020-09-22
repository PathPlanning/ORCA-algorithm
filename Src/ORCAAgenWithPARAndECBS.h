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
#include "MAPF/PAR/PushAndRotate.h"

#ifndef ORCASTAR_ORCAAGENWITHTPARANDECBS_H
#define ORCASTAR_ORCAAGENWITHTPARANDECBS_H

#define ONLY_PAR 1
#define PAR_AND_ECBS 2
#define FAIL 0

class ORCAAgenWithPARAndECBS : public Agent
{
    public:
        ORCAAgenWithPARAndECBS();
        ORCAAgenWithPARAndECBS(const int &id, const Point &start, const Point &goal, const Map &map, const EnvironmentOptions &options,
                               AgentParam param);
        ORCAAgenWithPARAndECBS(const ORCAAgenWithPARAndECBS &obj);
        ~ORCAAgenWithPARAndECBS();


        ORCAAgenWithPARAndECBS* Clone() const override ;
        void ComputeNewVelocity() override;
        void ApplyNewVelocity() override;
        bool UpdatePrefVelocity() override;
        void AddNeighbour(Agent &neighbour, float distSq) override;

        bool isMAPFMember() const;


        bool operator == (const ORCAAgenWithPARAndECBS &another) const;
        bool operator != (const ORCAAgenWithPARAndECBS &another) const;
        ORCAAgenWithPARAndECBS &operator = (const ORCAAgenWithPARAndECBS &obj);
    #if PAR_LOG
        void SetMAPFInstanceLoggerRef(MAPFInstancesLogger *log);
    #endif

    private:
        std::vector <std::pair<float, Agent*>>& GetNeighbours();
        std::set<ORCAAgenWithPARAndECBS *> GetAgentsForCentralizedPlanning();
        void SetAgentsForCentralizedPlanning(std::set<ORCAAgenWithPARAndECBS *> agents);
        void PrepareMAPFExecution(Point common);
        bool ComputeMAPFEnv(Point common, std::vector<std::pair<Point, ORCAAgenWithPARAndECBS*>> oldGoals = std::vector<std::pair<Point, ORCAAgenWithPARAndECBS*>>());
        Point PullOutIntermediateGoal(Point common);
        int ComputeMAPF(int algToUse);
        bool UniteMAPF();
        bool UpdateMAPF();


        SubMap MAPFMap;
        MAPFActorSet MAPFSet;
        MAPFConfig conf;
        std::set<ORCAAgenWithPARAndECBS *> MAPFAgents;
        float fakeRadius;
        bool inMAPFMode;
        bool moveToMAPFPos;
        bool MAPFVis;
        bool notMAPFVis;
        bool MAPFUnion;
        bool MAPFExec;
        Point MAPFStart;
        Point MAPFGoal;
        FocalSearch<> ECBSLLsearch;
        ConflictBasedSearch<FocalSearch<>> ECBSSolver;
        Astar<> PARLLsearch;
        PushAndRotate PARSolver;
        
        MAPFSearchResult fallbackMAPFRes;
        MAPFSearchResult MAPFres;
        int currMAPFPos;
        int MAPFActorId;
        Point MAPFcommon;
        std::vector<Point> buffMAPF;
    #if PAR_LOG
        MAPFInstancesLogger *MAPFLog;
    #endif
};


#endif //ORCASTAR_ORCAAGENWITHTPARANDECBS_H
