#include <set>
#include <algorithm>
#include <cstdlib>

#include "Agent.h"
#include "SubMap.h"
#include "mapf/MAPFActorSet.h"
#include "mapf/MAPFActor.h"
#include "mapf/ecbs/ConflictBasedSearch.h"
#include "mapf/AStar.h"
#include "mapf/FocalSearch.h"
#include "mapf/SCIPP.h"
#include "MAPFInstancesLogger.h"
#include "mapf/push-and-rotate//PushAndRotate.h"

#ifndef ORCASTAR_ORCAAGENWITHTPARANDECBS_H
#define ORCASTAR_ORCAAGENWITHTPARANDECBS_H

#define ONLY_PAR 1
#define PAR_AND_ECBS 2
#define FAIL 0

class ORCAAgentWithPARAndECBS : public Agent
{
    public:
        ORCAAgentWithPARAndECBS();
        ORCAAgentWithPARAndECBS(const int &id, const Point &start, const Point &goal, const Map &map, const EnvironmentOptions &options,
                                AgentParam param);
        ORCAAgentWithPARAndECBS(const ORCAAgentWithPARAndECBS &obj);
        ~ORCAAgentWithPARAndECBS();


        ORCAAgentWithPARAndECBS* clone() const override ;
        void computeNewControl() override;
        void applyNewControl() override;
        bool prepareBeforeStep() override;
        void addNeighbour(Agent &neighbour, float dist_sq) override;

        void PrintMAPFMemberStat() const;
        unordered_map<std::string, float> GetMAPFStatistics() const;


        bool operator == (const ORCAAgentWithPARAndECBS &another) const;
        bool operator != (const ORCAAgentWithPARAndECBS &another) const;
        ORCAAgentWithPARAndECBS &operator = (const ORCAAgentWithPARAndECBS &obj);
    #if PAR_LOG
        void SetMAPFInstanceLoggerRef(MAPFInstancesLogger *log);
    #endif

    private:
        std::vector <std::pair<float, Agent*>>& GetNeighbours();
        std::set<ORCAAgentWithPARAndECBS *> GetAgentsForCentralizedPlanning();
        void SetAgentsForCentralizedPlanning(std::set<ORCAAgentWithPARAndECBS *> agents);
        void PrepareMAPFExecution();
        bool ComputeMAPFEnv();
        Point GetGoalPointForMAPF(SubMap Area);
        int ComputeMAPF(int algToUse);
        bool UniteMAPF();
        bool UpdateMAPF();




        SubMap MAPFMap;
        MAPFActorSet MAPFSet;
        MAPFConfig conf;
        std::set<ORCAAgentWithPARAndECBS *> MAPFAgents;
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
        bool waitForStart;
        bool waitForFinish;

        std::vector<Point> buffMAPF;
        int initCount;
        int updCount;
        int uniCount;
        float timeMAPF;
        int ECBSCount;
        int PARCount;

        int successCount;
        int unsuccessCount;
        int flowtimeCount;

    #if PAR_LOG
        MAPFInstancesLogger *MAPFLog;
    #endif
};


#endif //ORCASTAR_ORCAAGENWITHTPARANDECBS_H
