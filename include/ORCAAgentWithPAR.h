#include <set>
#include <algorithm>
#include <chrono>

#include "Agent.h"
#include "SubMap.h"
#include "MAPF/MAPFActorSet.h"
#include "MAPF/MAPFActor.h"
#include "MAPF/PushAndRotate.h"
#include "MAPF/AStar.h"

// TODO Rewrite MAPF instance logger module from scratch (singleton)
//#include "../simulation/MAPFInstancesLogger.h"

#ifndef ORCA_ORCAAGENTWITHPAR_H
#define ORCA_ORCAAGENTWITHPAR_H


class ORCAAgentWithPAR : public Agent
{

    public:
        ORCAAgentWithPAR();
        ORCAAgentWithPAR(const int &id, const Point &start, const Point &goal, Map &map, const EnvironmentOptions &options,
                  AgentParam param);
        ORCAAgentWithPAR(const ORCAAgentWithPAR &obj);
        ~ORCAAgentWithPAR();


        ORCAAgentWithPAR* Clone() const override ;
        void ComputeNewVelocity() override;
        void ApplyNewVelocity() override;
        bool UpdatePrefVelocity() override;
        void AddNeighbour(Agent &neighbour, float distSq) override;
        bool isPARMember() const;

        std::unordered_map<std::string, float> GetMAPFStatistics() const;


        bool operator == (const ORCAAgentWithPAR &another) const;
        bool operator != (const ORCAAgentWithPAR &another) const;
        ORCAAgentWithPAR &operator = (const ORCAAgentWithPAR &obj);
#if PAR_LOG
        void SetPARInstanceLoggerRef(MAPFInstancesLogger *log);
#endif

    private:
        std::vector <std::pair<float, Agent*>>& GetNeighbours();
        std::set<ORCAAgentWithPAR *> GetAgentsForCentralizedPlanning();
        void SetAgentsForCentralizedPlanning(std::set<ORCAAgentWithPAR *> agents);
        void PreparePARExecution();
        bool ComputePAREnv();
        Point GetGoalPointForMAPF(SubMap Area);
        bool ComputePAR();
        bool UnitePAR();
        bool UpdatePAR();


        SubMap PARMap;
        MAPFActorSet PARSet;
        MAPFConfig conf;
        std::set<ORCAAgentWithPAR *> PARAgents;
        float fakeRadius;
        bool inPARMode;
        bool moveToPARPos;
        bool PARVis;
        bool notPARVis;
        bool PARUnion;
        bool PARExec;
        bool waitForStart;
        bool waitForFinish;
        Point PARStart;
        Point PARGoal;
        Astar<> PARsearch;
        PushAndRotate PARSolver;
        MAPFSearchResult PARres;
        int currPARPos;
        int PARActorId;
        std::vector<Point> buffPar;

        int initCount;
        int updCount;
        int uniCount;
        float timeMAPF;

        int successCount;
        int unsuccessCount;
        int flowtimeCount;

#if PAR_LOG
        MAPFInstancesLogger *PARLog;
#endif


};


#endif //ORCA_ORCAAGENTWITHPAR_H
