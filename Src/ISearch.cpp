#include "ISearch.h"

ISearch::ISearch(bool WithTime)
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
    withTime = WithTime;
}

ISearch::~ISearch(void) {}

int ISearch::convolution(int i, int j, const SubMap &map, int time, bool withTime)
{
    int res = withTime ? map.GetWidth() * map.GetHeight() * time : 0;
    return res + i * map.GetWidth() + j;
}

bool Node::breakingties;

SearchResult ISearch::startSearch(const SubMap &map, const PARActorSet &actorSet,
                                  int start_i, int start_j, int goal_i, int goal_j,
                                  bool (*isGoal)(const Node&, const Node&, const SubMap&, const PARActorSet&),
                                  bool freshStart, bool returnPath, int maxDepth,
                                  const std::unordered_set<Node> &occupiedNodes,
                                  const ConstraintsSet &constraints,
                                  const ConflictAvoidanceTable &CAT)
{
    sresult.pathfound = false;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    Node cur;
    int fin_i, fin_j, fin_depth;
    int agentId = -1;
    if (actorSet.isOccupied(start_i, start_j))
    {
        agentId = actorSet.getActorId(start_i, start_j);
    }

    if (freshStart)
    {
        open.clear();
        close.clear();
        sortByIndex.clear();

        Node::breakingties = breakingties;
        sresult.numberofsteps = 0;
        cur = Node(start_i, start_j, nullptr, 0,
                   computeHFromCellToCell(start_i, start_j, goal_i, goal_j));
        sortByIndex[convolution(cur.i, cur.j, map)] = cur;
        open.insert(cur);
    }

    while(!open.empty())
    {
        ++sresult.numberofsteps;
        auto curIt = open.begin();
        cur = *open.begin();
        close[convolution(cur.i, cur.j, map, cur.depth, withTime)] = cur;
        Node *curPtr = &(close.find(convolution(cur.i, cur.j, map, cur.depth, withTime))->second);
        if ((isGoal != nullptr && isGoal(Node(start_i, start_j), cur, map, actorSet)) ||
            (isGoal == nullptr && cur.i == goal_i && cur.j == goal_j))
        {
            if (!constraints.hasFutureConstraint(cur.i, cur.j, cur.depth, agentId))
            {
                if (!freshStart)
                {
                    freshStart = true;
                }
                else
                {
                    sresult.pathfound = true;
                    fin_i = cur.i;
                    fin_j = cur.j;
                    fin_depth = cur.depth;
                    break;
                }
            }
        }

        if (maxDepth == -1 || cur.depth < maxDepth)
        {
            std::list<Node> successors = findSuccessors(cur, map, goal_i, goal_j, agentId, occupiedNodes, constraints, CAT);
            for (auto neigh : successors)
            {
                if (close.find(convolution(neigh.i, neigh.j, map, neigh.depth, withTime)) == close.end())
                {
                    neigh.parent = curPtr;
                    auto it = sortByIndex.find(convolution(neigh.i, neigh.j, map, neigh.depth, withTime));
                    if (it == sortByIndex.end() || it->second.g > neigh.g)
                    {
                        if (it != sortByIndex.end())
                        {
                            open.erase(open.find(it->second));
                        }
                        sortByIndex[convolution(neigh.i, neigh.j, map, neigh.depth, withTime)] = neigh;
                        open.insert(neigh);
                    }
                }
            }
        }

        sortByIndex.erase(sortByIndex.find(convolution(cur.i, cur.j, map, cur.depth, withTime)));
        open.erase(curIt);
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    sresult.time = static_cast<double>(elapsedMilliseconds) / 1000;
    sresult.nodescreated = open.size() + close.size();
    if (sresult.pathfound)
    {
        auto it = close.find(convolution(fin_i, fin_j, map, fin_depth, withTime));
        sresult.pathlength = it->second.g;
        sresult.lastNode = it->second;
        if (returnPath)
        {
            lppath.clear();
            hppath.clear();
            makePrimaryPath(it->second);
            makeSecondaryPath(map);
            sresult.hppath = &hppath;
            sresult.lppath = &lppath;
        }
    }
    return sresult;
}

std::list<Node> ISearch::findSuccessors(const Node &curNode, const SubMap &map,
                                        int goal_i, int goal_j, int agentId,
                                        const std::unordered_set<Node> &occupiedNodes,
                                        const ConstraintsSet &constraints,
                                        const ConflictAvoidanceTable &CAT)
{
    std::list<Node> successors;
    for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
            int newi = curNode.i + di, newj = curNode.j + dj;
            int depth = curNode.depth;
            if ((di == 0 || dj == 0) && (withTime || di != 0 || dj != 0) && map.CellOnGrid(newi, newj) &&
                map.CellIsTraversable(newi, newj, occupiedNodes) &&
                !constraints.hasNodeConstraint(newi, newj, curNode.depth + 1, agentId) &&
                !constraints.hasEdgeConstraint(newi, newj, curNode.depth + 1, agentId, curNode.i, curNode.j))
            {
                int newh = computeHFromCellToCell(newi, newj, goal_i, goal_j);
                Node neigh(newi, newj, nullptr, curNode.g + 1,
                           newh, curNode.depth + 1, CAT.getActorCount(newi, newj, curNode.depth + 1, map));
                successors.push_back(neigh);
            }
        }
    }
    return successors;
}

void ISearch::makePrimaryPath(const Node &curNode)
{
    lppath.push_front(curNode);
    if (curNode.parent != nullptr)
    {
        makePrimaryPath(*(curNode.parent));
    }
}

void ISearch::makeSecondaryPath(const SubMap &map)
{
    auto it = lppath.begin();
    hppath.push_back(*it);
    ++it;
    for (it; it != lppath.end(); ++it)
    {
        auto prevIt = it;
        --prevIt;
        auto nextIt = it;
        ++nextIt;
        if (nextIt == lppath.end() ||
            (it->i - prevIt->i) * (nextIt->j - it->j) != (it->j - prevIt->j) * (nextIt->i - it->i))
        {
            hppath.push_back(*it);
        }
    }
}

void ISearch::getPerfectHeuristic(const SubMap &map, const PARActorSet &actorSet)
{
    bool oldWithTime = withTime;
    withTime = false;
    perfectHeuristic.clear();
    for (int i = 0; i < actorSet.getActorCount(); ++i)
    {
        close.clear();
        Node goal = actorSet.getActor(i).getGoalPosition();
        std::queue<Node> queue;
        queue.push(goal);

        while (!queue.empty())
        {
            Node cur = queue.front();
            queue.pop();
            if (close.find(convolution(cur.i, cur.j, map)) != close.end())
            {
                continue;
            }
            close[convolution(cur.i, cur.j, map)] = cur;
            perfectHeuristic[std::make_pair(cur, goal)] = cur.g;
            std::list<Node> successors = findSuccessors(cur, map);
            for (auto neigh : successors)
            {
                queue.push(neigh);
            }
        }
    }
    withTime = oldWithTime;
}
