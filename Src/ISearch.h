#include <list>
#include <unordered_set>
#include <queue>

#include "Geom.h"
#include "SubMap.h"
#include "PARActor.h"
#include "PARActorSet.h"
#include "Constraints.h"
#include "ConflictAvoidanceTable.h"

#ifndef ORCA_ISEARCH_H
#define ORCA_ISEARCH_H




struct SearchResult
{
    bool pathfound;
    float pathlength;
    const std::list<Node>* lppath;
    const std::list<Node>* hppath;
    unsigned int nodescreated;
    unsigned int numberofsteps;
    double time;
    Node lastNode;

    SearchResult()
    {
        pathfound = false;
        pathlength = 0;
        lppath = nullptr;
        hppath = nullptr;
        nodescreated = 0;
        numberofsteps = 0;
        time = 0;
    }

};

class ISearch
{

    public:
        ISearch(bool WithTime = false);
        ~ISearch(void);
        SearchResult startSearch(const SubMap &map, const PARActorSet &actorSet,
                                 int start_i, int start_j, int goal_i = 0, int goal_j = 0,
                                 bool (*isGoal)(const Node&, const Node&, const SubMap&, const PARActorSet&) = nullptr,
                                 bool freshStart = true, bool returnPath = true, int maxDepth = -1,
                                 const std::unordered_set<Node> &occupiedNodes = std::unordered_set<Node>(),
                                 const ConstraintsSet &constraints = ConstraintsSet(),
                                 const ConflictAvoidanceTable &CAT = ConflictAvoidanceTable());

        virtual std::list<Node> findSuccessors(const Node &curNode, const SubMap &map, int goal_i = 0, int goal_j = 0, int actorId = -1,
                                               const std::unordered_set<Node> &occupiedNodes = std::unordered_set<Node>(),
                                               const ConstraintsSet &constraints = ConstraintsSet(),
                                               const ConflictAvoidanceTable &CAT = ConflictAvoidanceTable());

        static int convolution(int i, int j, const SubMap &map, int time = 0, bool withTime = false);
        void getPerfectHeuristic(const SubMap &map, const PARActorSet &aсtorSet);

    protected:
        virtual double computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j) {return 0;}
        virtual void makePrimaryPath(const Node &curNode);
        virtual void makeSecondaryPath(const SubMap &map);

        SearchResult                    sresult;
        std::list<Node>                 lppath, hppath;
        double                          hweight;
        bool                            breakingties;
        std::set<Node>                  open;
        std::unordered_map<int, Node>   sortByIndex;
        std::unordered_map<int, Node>   close;
        bool                            withTime;
        std::unordered_map<std::pair<Node, Node>, int> perfectHeuristic;



};


#endif //ORCA_ISEARCH_H
