#include <list>
#include <vector>
#include <algorithm>
#include <chrono>

#include "ISearch.h"
#include "PARActorSet.h"
#include "Geom.h"
#include "Const.h"


#ifndef ORCA_PUSHANDROTATE_H
#define ORCA_PUSHANDROTATE_H

class PARConfig
{
    public:
        PARConfig() = default;
        PARConfig(const PARConfig& orig) = default;
        ~PARConfig() = default;

    public:
        double*         SearchParams;
        std::string*    LogParams;
        unsigned int    N;
        int             searchType;
        int             minActors;
        int             maxActors;
        int             maxTime;
        int             tasksCount;
        bool            withCAT;
        bool            withPerfectHeuristic;
};


struct PARSearchResult
{
    bool                            pathfound;
    std::vector<ActorMove>*         actorsMoves;
    std::vector<std::vector<Node>>* actorsPaths;
    double                          time;

    PARSearchResult(bool Pathfound = false)
    {
        pathfound = Pathfound;
    }
};


class PushAndRotate
{
    public:
        PushAndRotate();
        PushAndRotate(ISearch* Search);
        ~PushAndRotate(void);
        PARSearchResult startSearch(const SubMap &Map, const PARConfig &config, PARActorSet &actorSet);
        void clear();
    private:
        bool solve(const SubMap &map, const PARConfig &config, PARActorSet &actorSet);
        bool clearNode(const SubMap &map, PARActorSet &actorSet, Node &nodeToClear,
                       const std::unordered_set<Node>& occupiedNodes);
        bool push(const SubMap &map, PARActorSet &actorSet, Node& from, Node& to, std::unordered_set<Node>& occupiedNodes);
        bool multipush(const SubMap &map, PARActorSet &actorSet, Node first, Node second, Node& to, std::list<Node>& path);
        bool clear(const SubMap &map, PARActorSet &actorSet, Node& first, Node& second);
        void exchange(const SubMap &map, PARActorSet &actorSet, Node& first, Node& second);
        void reverse(int begSize, int endSize,
                     int firstId, int secondId, PARActorSet &actorSet);
        bool swap(const SubMap &map, PARActorSet &actorSet, Node& first, Node& second);
        bool rotate(const SubMap &map, PARActorSet &actorSet, std::vector<Node> &qPath, int cycleBeg);
        void getActorPaths(PARActorSet &actorSet);
        void getParallelPaths(PARActorSet &actorSet);
        void getComponent(PARActorSet &actorSet, std::pair<Node, Node> &startEdge,
                          std::vector<std::pair<Node, Node>> &edgeStack,
                          std::vector<std::unordered_set<Node>>& components);
        void combineNodeSubgraphs(PARActorSet &actorSet, std::vector<std::unordered_set<Node>>& components,
                                  Node &subgraphNode, int subgraphNum);
        void getSubgraphs(const SubMap &map, PARActorSet &actorSet);
        int getReachableNodesCount(const SubMap &map, PARActorSet &actorSet, Node &start,
                                   bool (*condition)(const Node&, const Node&, const SubMap&, const PARActorSet&),
                                   const std::unordered_set<Node> &occupiedNodes);
        void assignToSubgraphs(const SubMap &map, PARActorSet &actorSet);
        void getPriorities(const SubMap &map, PARActorSet &actorSet);
        void getPaths(PARActorSet &actorSet);


        ISearch*                        search;
        std::vector<ActorMove>          actorsMoves;
        std::vector<std::vector<Node>>  actorsPaths;
        PARSearchResult             result;
};


#endif //ORCA_PUSHANDROTATE_H
