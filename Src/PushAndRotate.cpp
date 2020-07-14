#include "PushAndRotate.h"

PushAndRotate::PushAndRotate()
{
    search = nullptr;
}

PushAndRotate::PushAndRotate(ISearch *Search)
{
    search = Search;
}

PushAndRotate::~PushAndRotate()
{
//    if(search)
//    {
//        delete search;
//    }
}

void PushAndRotate::clear()
{
    actorsPaths.clear();
    actorsMoves.clear();
}

bool PushAndRotate::clearNode(const SubMap &map, PARActorSet &actorSet, Node &nodeToClear,
                              const std::unordered_set<Node> &occupiedNodes)
{
    auto isGoal = [](const Node &start, const Node &cur, const SubMap &map, const PARActorSet &actorSet)
    {
        return !actorSet.isOccupied(cur.i, cur.j);
    };

    ISearch dijkstraSearch;
    SearchResult searchResult = dijkstraSearch.startSearch(map, actorSet, nodeToClear.i, nodeToClear.j, 0, 0, isGoal, true, true, -1, occupiedNodes);
    if(!searchResult.pathfound)
    {
        return false;
    }
    auto path = *searchResult.lppath;
    for(auto it = std::next(path.rbegin()); it != path.rend(); ++it)
    {
        if(actorSet.isOccupied(it->i, it->j))
        {
            Node from = *it;
            Node to = *std::prev(it);
            actorSet.moveActor(from, to, actorsMoves);
        }
    }
    return true;
}

bool PushAndRotate::push(const SubMap &map, PARActorSet &actorSet, Node &from, Node &to,
                         std::unordered_set<Node> &occupiedNodes)
{
    if(occupiedNodes.find(to) != occupiedNodes.end())
    {
        return false;
    }
    if(actorSet.isOccupied(to.i, to.j))
    {
        bool inserted = false;
        if(occupiedNodes.find(from) == occupiedNodes.end())
        {
            occupiedNodes.insert(from);
            inserted = true;
        }
        bool canClear = clearNode(map, actorSet, to, occupiedNodes);
        if(inserted)
        {
            occupiedNodes.erase(from);
        }
        if(!canClear)
        {
            return false;
        }
    }
    actorSet.moveActor(from, to, actorsMoves);
    return true;
}

bool PushAndRotate::multipush(const SubMap &map, PARActorSet &actorSet, Node first, Node second, Node &to,
                              std::list<Node> &path)
{
    if(path.size() > 1 && *std::next(path.begin()) == second)
    {
        std::swap(first, second);
        path.pop_front();
    }
    Node prevNode = second;
    for(auto it = path.begin(); it != std::prev(path.end()); ++it)
    {
        Node curNode = *it;
        Node nextNode = *std::next(it);
        std::unordered_set<Node> occupiedNodes = {prevNode, curNode};
        if(actorSet.isOccupied(nextNode.i, nextNode.j))
        {
            if(!clearNode(map, actorSet, nextNode, occupiedNodes))
            {
                return false;
            }
        }
        actorSet.moveActor(curNode, nextNode, actorsMoves);
        actorSet.moveActor(prevNode, curNode, actorsMoves);
        prevNode = curNode;
    }
    return true;
}

bool PushAndRotate::clear(const SubMap &map, PARActorSet &actorSet, Node &first, Node &second)
{
    std::list<Node> successors = search->findSuccessors(first, map);
    std::set<Node> unoccupied;
    for(auto node : successors)
    {
        if(!actorSet.isOccupied(node.i, node.j))
        {
            unoccupied.insert(node);
        }
    }
    if(unoccupied.size() >= 2)
    {
        return true;
    }

    std::unordered_set<Node> forbidden = {first, second};
    forbidden.insert(unoccupied.begin(), unoccupied.end());
    for(auto node : successors)
    {
        if(unoccupied.find(node) == unoccupied.end() && node != second && clearNode(map, actorSet, node, forbidden))
        {
            if(unoccupied.size() >= 1)
            {
                return true;
            }
            unoccupied.insert(node);
            forbidden.insert(node);
        }
    }
    if(unoccupied.empty())
    {
        return false;
    }

    Node freeNeigh = *unoccupied.begin();
    for(auto node : successors)
    {
        if(node != second && node != freeNeigh)
        {
            size_t curSize = actorsMoves.size();
            PARActorSet newActorSet = actorSet;
            if(clearNode(map, newActorSet, node, {first, second}))
            {
                if(clearNode(map, newActorSet, freeNeigh, {first, second, node}))
                {
                    actorSet = newActorSet;
                    return true;
                }
            }
            actorsMoves.erase(actorsMoves.begin() + curSize, actorsMoves.end());
            break;
        }
    }

    for(auto node : successors)
    {
        if(node != second && node != freeNeigh)
        {
            int curSize = actorsMoves.size();
            PARActorSet newActorSet = actorSet;
            newActorSet.moveActor(first, freeNeigh, actorsMoves);
            newActorSet.moveActor(second, first, actorsMoves);
            if(clearNode(map, newActorSet, node, {first, second}))
            {
                if(clearNode(map, newActorSet, second, {first, second, node}))
                {
                    actorSet = newActorSet;
                    return true;
                }
            }
            actorsMoves.erase(actorsMoves.begin() + curSize, actorsMoves.end());
            break;
        }
    }

    int secondActorId = actorSet.getActorId(second.i, second.j);
    if(!clearNode(map, actorSet, second, {first}))
    {
        return false;
    }
    actorSet.moveActor(first, second, actorsMoves);
    Node secondPosition = actorSet.getActor(secondActorId).getCurPosition();
    if(!clearNode(map, actorSet, freeNeigh, {first, second, secondPosition}))
    {
        return false;
    }
    for(auto node : successors)
    {
        if(node != second && node != freeNeigh)
        {
            actorSet.moveActor(node, first, actorsMoves);
            actorSet.moveActor(first, freeNeigh, actorsMoves);
            actorSet.moveActor(second, first, actorsMoves);
            actorSet.moveActor(secondPosition, second, actorsMoves);
            return clearNode(map, actorSet, freeNeigh, {first, second, node});
        }
    }
    return false;
}

void PushAndRotate::exchange(const SubMap &map, PARActorSet &actorSet, Node &first, Node &second)
{
    std::list<Node> successors = search->findSuccessors(first, map);
    std::vector<Node> freeNeigh;
    for(auto node : successors)
    {
        if(!actorSet.isOccupied(node.i, node.j))
        {
            freeNeigh.push_back(node);
        }
    }
    actorSet.moveActor(first, freeNeigh[0], actorsMoves);
    actorSet.moveActor(second, first, actorsMoves);
    actorSet.moveActor(first, freeNeigh[1], actorsMoves);
    actorSet.moveActor(freeNeigh[0], first, actorsMoves);
    actorSet.moveActor(first, second, actorsMoves);
    actorSet.moveActor(freeNeigh[1], first, actorsMoves);
}

void PushAndRotate::reverse(int begSize, int endSize, int firstActorId, int secondActorId, PARActorSet &actorSet)
{
    for(int i = endSize - 1; i >= begSize; --i)
    {
        ActorMove pos = actorsMoves[i];
        if(pos.id == firstActorId)
        {
            pos.id = secondActorId;
        }
        else if(pos.id == secondActorId)
        {
            pos.id = firstActorId;
        }
        Node from = actorSet.getActor(pos.id).getCurPosition();
        Node to = Node(from.i - pos.di, from.j - pos.dj);
        actorSet.moveActor(from, to, actorsMoves);
    }
}

bool PushAndRotate::swap(const SubMap &map, PARActorSet &actorSet, Node &first, Node &second)
{
    int firstActorId = actorSet.getActorId(first.i, first.j);
    int secondActorId = actorSet.getActorId(second.i, second.j);

    auto isGoal = [](const Node &start, const Node &cur, const SubMap &map, const PARActorSet &actorSet)
    {
        return map.GetCellDegree(cur.i, cur.j) >= 3;
    };

    ISearch dijkstraSearch;
    SearchResult searchResult = dijkstraSearch.startSearch(map, actorSet, first.i, first.j, 0, 0, isGoal);
    while(searchResult.pathfound)
    {
        int begSize = actorsMoves.size();
        PARActorSet newActorSet = actorSet;
        auto path = *searchResult.lppath;
        Node exchangeNode = path.back();
        if(multipush(map, newActorSet, first, second, exchangeNode, path))
        {
            int exchangeActorId = newActorSet.getActorId(exchangeNode.i, exchangeNode.j);
            int neighActorId = (exchangeActorId == firstActorId) ? secondActorId : firstActorId;
            Node neigh = newActorSet.getActor(neighActorId).getCurPosition();
            if(clear(map, newActorSet, exchangeNode, neigh))
            {
                actorSet = newActorSet;
                int endSize = actorsMoves.size();
                exchange(map, actorSet, exchangeNode, neigh);
                reverse(begSize, endSize, firstActorId, secondActorId, actorSet);
                return true;
            }
        }
        searchResult = dijkstraSearch.startSearch(map, actorSet, first.i, first.j, 0, 0, isGoal, false);
    }
    return false;
}

bool PushAndRotate::rotate(const SubMap &map, PARActorSet &actorSet, std::vector<Node> &qPath, int cycleBeg)
{
    int size = qPath.size() - cycleBeg;
    for(int i = cycleBeg; i < qPath.size(); ++i)
    {
        if(!actorSet.isOccupied(qPath[i].i, qPath[i].j))
        {
            for(int j = 0; j < size - 1; ++j)
            {
                int from = cycleBeg + (i - cycleBeg - j - 1 + size) % size;
                int to = cycleBeg + (i - cycleBeg - j + size) % size;
                if(actorSet.isOccupied(qPath[from].i, qPath[from].j))
                {
                    actorSet.moveActor(qPath[from], qPath[to], actorsMoves);
                }
            }
            return true;
        }
    }

    std::unordered_set<Node> cycleNodes(qPath.begin() + cycleBeg, qPath.end());
    for(int i = cycleBeg; i < qPath.size(); ++i)
    {
        cycleNodes.erase(qPath[i]);
        int firstActorId = actorSet.getActorId(qPath[i].i, qPath[i].j);
        int begSize = actorsMoves.size();
        if(clearNode(map, actorSet, qPath[i], cycleNodes))
        {
            int endSize = actorsMoves.size();
            int secondActorIndex = cycleBeg + (i - cycleBeg - 1 + size) % size;
            int secondActorId = actorSet.getActorId(qPath[secondActorIndex].i, qPath[secondActorIndex].j);
            actorSet.moveActor(qPath[secondActorIndex], qPath[i], actorsMoves);
            Node curPosition = actorSet.getActor(firstActorId).getCurPosition();
            swap(map, actorSet, qPath[i], curPosition);
            for(int j = 0; j < size - 1; ++j)
            {
                int from = cycleBeg + (i - cycleBeg - j - 2 + size) % size;
                int to = cycleBeg + (i - cycleBeg - j - 1 + size) % size;
                if(actorSet.isOccupied(qPath[from].i, qPath[from].j))
                {
                    actorSet.moveActor(qPath[from], qPath[to], actorsMoves);
                }
            }
            reverse(begSize, endSize, firstActorId, secondActorId, actorSet);
            return true;
        }
        cycleNodes.insert(qPath[i]);
    }
    return false;
}

void PushAndRotate::getActorPaths(PARActorSet &actorSet)
{
    actorsPaths.resize(actorSet.getActorCount());
    std::vector<Node> actorPositions;
    for(int i = 0; i < actorSet.getActorCount(); ++i)
    {
        Node startPosition = actorSet.getActor(i).getStartPosition();
        actorPositions.push_back(startPosition);
        actorsPaths[i].push_back(startPosition);
    }
    for(int i = 0; i < actorsMoves.size(); ++i)
    {
        actorPositions[actorsMoves[i].id].i += actorsMoves[i].di;
        actorPositions[actorsMoves[i].id].j += actorsMoves[i].dj;
        for(int j = 0; j < actorSet.getActorCount(); ++j)
        {
            actorsPaths[j].push_back(actorPositions[j]);
        }
    }
}

void PushAndRotate::getParallelPaths(PARActorSet &actorSet)
{
   /* int actorCount = actorSet.getActorCount();
    std::vector<std::vector<Node>> actorsPositions(actorCount);
    std::vector<int> actorInd(actorCount, 0);
    std::unordered_map<Node, std::vector<int>> nodesOccupations;
    std::unordered_map<Node, int> nodeInd;

    actorsPaths.resize(actorSet.getActorCount());
    for(int i = 0; i < actorSet.getActorCount(); ++i)
    {
        Node startPosition = actorSet.getActor(i).getStartPosition();
        actorsPositions[i].push_back(startPosition);
        actorsPaths[i].push_back(startPosition);
        if(nodesOccupations.find(startPosition) == nodesOccupations.end())
        {
            nodesOccupations[startPosition] = {};
            nodeInd[startPosition] = 0;
        }
        nodesOccupations[startPosition].push_back(i);
    }

    for(auto move : actorsMoves)
    {
        Node cur = actorsPositions[move.id].back();
        cur.i += move.di;
        cur.j += move.dj;
        if(nodesOccupations.find(cur) == nodesOccupations.end())
        {
            nodesOccupations[cur] = {};
            nodeInd[cur] = 0;
        }
        if(!nodesOccupations[cur].empty() && nodesOccupations[cur].back() == move.id)
        {
            while(actorsPositions[move.id].back() != cur)
            {
                Node curBack = actorsPositions[move.id].back();
                int lastInd;
                for(lastInd = nodesOccupations[curBack].size() - 1; lastInd >= 0 && nodesOccupations[curBack][lastInd] != move.id; --lastInd)
                {
                }
                nodesOccupations[curBack].erase(nodesOccupations[curBack].begin() + lastInd);
                actorsPositions[move.id].pop_back();
            }
        }
        else
        {
            actorsPositions[move.id].push_back(cur);
            nodesOccupations[cur].push_back(move.id);
        }
    }

    std::vector<bool> finished(actorCount, false);
    while(true)
    {
        std::vector<bool> hasMoved(actorCount, false);
        for(int i = 0; i < actorCount; ++i)
        {
            if(hasMoved[i] || finished[i])
            {
                continue;
            }

            std::vector<int> path;
            int curActor = i;
            bool canMove = true;
            while(true)
            {
                path.push_back(curActor);

                if(actorsPositions[curActor].size() == 1)
                {
                    actorsPositions[curActor].push_back(actorsPositions[curActor].back());
                }

                Node nextNode = actorsPositions[curActor][actorInd[curActor] + 1];
                int lastInd = nodeInd[nextNode];
                if(nodesOccupations[nextNode][lastInd] == curActor)
                {
                    break;
                }
                else if(nodesOccupations[nextNode][lastInd + 1] == curActor)
                {
                    int nextActor = nodesOccupations[nextNode][lastInd];
                    if(finished[nextActor] || hasMoved[nextActor] || nextActor < curActor || actorsPositions[nextActor][actorInd[nextActor]] != nextNode)
                    {
                        canMove = false;
                        break;
                    }
                    curActor = nextActor;
                    if(curActor == i)
                    {
                        break;
                    }
                }
                else
                {
                    canMove = false;
                    break;
                }
            }

            if(canMove)
            {
                for(int actorId : path)
                {
                    hasMoved[actorId] = true;
                    ++nodeInd[actorsPositions[actorId][actorInd[actorId]]];
                    ++actorInd[actorId];
                    actorsPaths[actorId].push_back(actorsPositions[actorId][actorInd[actorId]]);
                    if(actorInd[actorId] == actorsPositions[actorId].size() - 1)
                    {
                        finished[actorId] = true;
                    }
                }
            }
            else
            {
                actorsPaths[i].push_back(actorsPositions[i][actorInd[i]]);
            }
        }

        if(!std::any_of(hasMoved.begin(), hasMoved.end(), [](const bool x)
        {
            return x;
        }))
        {
            break;
        }
    }*/

    int actorCount = actorSet.getActorCount();
    std::vector<std::vector<Node>> actorsPositions(actorCount);
    std::vector<int> actorInd(actorCount, 0);
    std::unordered_map<Node, std::vector<int>, Utils::NodeHash> nodesOccupations;
    std::unordered_map<Node, int, Utils::NodeHash> nodeInd;

    actorsPaths.resize(actorSet.getActorCount());
    for (int i = 0; i < actorSet.getActorCount(); ++i) {
        Node startPosition = actorSet.getActor(i).getStartPosition();
        actorsPositions[i].push_back(startPosition);
        actorsPaths[i].push_back(startPosition);
        if (nodesOccupations.find(startPosition) == nodesOccupations.end()) {
            nodesOccupations[startPosition] = {};
            nodeInd[startPosition] = 0;
        }
        nodesOccupations[startPosition].push_back(i);
    }

    for (auto move : actorsMoves) {
        Node cur = actorsPositions[move.id].back();
        cur.i += move.di;
        cur.j += move.dj;
        if (nodesOccupations.find(cur) == nodesOccupations.end()) {
            nodesOccupations[cur] = {};
            nodeInd[cur] = 0;
        }
        if (!nodesOccupations[cur].empty() && nodesOccupations[cur].back() == move.id) {
            while(actorsPositions[move.id].back() != cur) {
                Node curBack = actorsPositions[move.id].back();
                int lastInd;
                for (lastInd = nodesOccupations[curBack].size() - 1;
                     lastInd >= 0 && nodesOccupations[curBack][lastInd] != move.id; --lastInd);
                nodesOccupations[curBack].erase(nodesOccupations[curBack].begin() + lastInd);
                actorsPositions[move.id].pop_back();
            }
        } else {
            actorsPositions[move.id].push_back(cur);
            nodesOccupations[cur].push_back(move.id);
        }
    }

    std::vector<bool> finished(actorCount, false);
    while (true) {
        std::vector<bool> hasMoved(actorCount, false);
        for (int i = 0; i < actorCount; ++i) {
            if (hasMoved[i] || finished[i]) {
                continue;
            }
            if (actorsPositions[i].size() == 1) {
                actorsPaths[i].push_back(actorsPositions[i][0]);
                finished[i] = true;
                continue;
            }

            std::vector<int> path;
            int curActor = i;
            bool canMove = true;
            while (true) {
                path.push_back(curActor);
                Node nextNode = actorsPositions[curActor][actorInd[curActor] + 1];
                int lastInd = nodeInd[nextNode];
                if (nodesOccupations[nextNode][lastInd] == curActor) {
                    break;
                } else if (nodesOccupations[nextNode][lastInd + 1] == curActor) {
                    int nextActor = nodesOccupations[nextNode][lastInd];
                    if (finished[nextActor] || hasMoved[nextActor] || nextActor < curActor ||
                        actorsPositions[nextActor][actorInd[nextActor]] != nextNode) {
                        canMove = false;
                        break;
                    }
                    curActor = nextActor;
                    if (curActor == i) {
                        break;
                    }
                } else {
                    canMove = false;
                    break;
                }
            }

            if (canMove) {
                for (int actorId : path) {
                    hasMoved[actorId] = true;
                    ++nodeInd[actorsPositions[actorId][actorInd[actorId]]];
                    ++actorInd[actorId];
                    actorsPaths[actorId].push_back(actorsPositions[actorId][actorInd[actorId]]);
                    if (actorInd[actorId] == actorsPositions[actorId].size() - 1) {
                        finished[actorId] = true;
                    }
                }
            } else {
                actorsPaths[i].push_back(actorsPositions[i][actorInd[i]]);
            }
        }

        if (!std::any_of(hasMoved.begin(), hasMoved.end(), [](const bool x) {return x;})) {
            break;
        }
    }
}

bool PushAndRotate::solve(const SubMap &map, const PARConfig &config, PARActorSet &actorSet)
{
    auto comparator = [&actorSet](int id1, int id2)
    {
        int subgraph1 = actorSet.getActor(id1).getSubgraph();
        int subgraph2 = actorSet.getActor(id2).getSubgraph();

        if(subgraph1 != subgraph2)
        {
            if(subgraph1 == -1 || actorSet.hasPriority(subgraph2, subgraph1))
            {
                return false;
            }
            else if(subgraph2 == -1 || actorSet.hasPriority(subgraph1, subgraph2))
            {
                return true;
            }
        }
        return id1 < id2;
    };

    std::set<int, decltype(comparator)> notFinished(comparator);
    std::unordered_set<int> finished;
    std::unordered_set<Node> qPathNodes, finishedPositions;
    std::vector<Node> qPath;

    for(int i = 0; i < actorSet.getActorCount(); ++i)
    {
        notFinished.insert(i);
    }

    bool isPolygon = true;
    for(int i = 0; i < map.GetHeight(); ++i)
    {
        for(int j = 0; j < map.GetWidth(); ++j)
        {
            if(!map.CellIsObstacle(i, j) && map.GetCellDegree(i, j) != 2)
            {
                isPolygon = false;
                break;
            }
        }
        if(!isPolygon)
        {
            break;
        }
    }

    int curActorId = -1;
    PARActor curActor;
    while(!notFinished.empty())
    {
        if(curActorId == -1)
        {
            curActor = actorSet.getActor(*notFinished.begin());
        }
        else
        {
            curActor = actorSet.getActor(curActorId);
        }
        notFinished.erase(curActor.getId());

        SearchResult searchResult = search->startSearch(map, actorSet, curActor.getCur_i(), curActor.getCur_j(), curActor.getGoal_i(), curActor.getGoal_j(), nullptr, true, true, -1, isPolygon ? finishedPositions : std::unordered_set<Node>());

        if(!searchResult.pathfound)
        {
            return false;
        }

        auto path = *searchResult.lppath;
        qPath.push_back(*path.begin());
        qPathNodes.insert(*path.begin());
        for(auto it = path.begin(); it != std::prev(path.end()); ++it)
        {
            if(qPathNodes.find(*std::next(it)) != qPathNodes.end())
            {
                int cycleBeg;
                for(cycleBeg = qPath.size() - 1; cycleBeg >= 0 && qPath[cycleBeg] != *std::next(it); --cycleBeg)
                {
                }
                rotate(map, actorSet, qPath, cycleBeg);

                bool toErase = false;
                while(qPath.size() != cycleBeg)
                {
                    Node lastNode = qPath.back();
                    if(actorSet.isOccupied(lastNode.i, lastNode.j) && finished.find(actorSet.getActorId(lastNode.i, lastNode.j)) != finished.end())
                    {
                        if(!toErase)
                        {
                            finishedPositions.insert(lastNode);
                            toErase = true;
                        }
                    }
                    else
                    {
                        if(toErase)
                        {
                            finishedPositions.erase(lastNode);
                            toErase = false;
                        }
                    }
                    qPathNodes.erase(lastNode);
                    qPath.pop_back();
                }
            }
            else if(!push(map, actorSet, *it, *std::next(it), finishedPositions))
            {
                swap(map, actorSet, *it, *std::next(it));
                if(finished.find(actorSet.getActorId(it->i, it->j)) != finished.end())
                {
                    finishedPositions.erase(*std::next(it));
                    finishedPositions.insert(*it);
                }
            }
            qPath.push_back(*std::next(it));
            qPathNodes.insert(*std::next(it));
        }
        finished.insert(curActor.getId());
        finishedPositions.insert(curActor.getGoalPosition());

        curActorId = -1;
        while(!qPath.empty())
        {
            Node lastNode = qPath.back();
            if(actorSet.isOccupied(lastNode.i, lastNode.j))
            {
                PARActor curActor = actorSet.getActor(actorSet.getActorId(lastNode.i, lastNode.j));
                Node goal = Node(curActor.getGoal_i(), curActor.getGoal_j());
                if(notFinished.find(curActor.getId()) == notFinished.end() && lastNode != goal)
                {
                    if(!actorSet.isOccupied(goal.i, goal.j))
                    {
                        actorSet.moveActor(lastNode, goal, actorsMoves);
                        finishedPositions.erase(lastNode);
                        finishedPositions.insert(goal);
                    }
                    else
                    {
                        curActorId = actorSet.getActorId(goal.i, goal.j);
                        break;
                    }
                }
            }
            qPathNodes.erase(lastNode);
            qPath.pop_back();
        }
    }
    return true;
}

void PushAndRotate::getComponent(PARActorSet &actorSet, std::pair<Node, Node> &startEdge,
                                 std::vector<std::pair<Node, Node>> &edgeStack,
                                 std::vector<std::unordered_set<Node>> &components)
{
    std::unordered_set<Node> component;
    std::pair<Node, Node> curEdge;
    do
    {
        curEdge = edgeStack.back();
        component.insert(curEdge.first);
        component.insert(curEdge.second);
        edgeStack.pop_back();
    }
    while(curEdge != startEdge);
    if(component.size() <= 2)
    {
        return;
    }
    for(auto node : component)
    {
        actorSet.setNodeSubgraph(node.i, node.j, components.size());
    }
    components.push_back(component);

}

void PushAndRotate::combineNodeSubgraphs(PARActorSet &actorSet, std::vector<std::unordered_set<Node>> &components,
                                         Node &subgraphNode, int subgraphNum)
{
    std::vector<int> subgraphs = actorSet.getSubgraphs(subgraphNode.i, subgraphNode.j);
    for(int j = 0; j < subgraphs.size(); ++j)
    {
        if(subgraphs[j] != subgraphNum)
        {
            for(auto node : components[subgraphs[j]])
            {
                actorSet.removeSubgraphs(node.i, node.j);
                actorSet.setNodeSubgraph(node.i, node.j, subgraphNum);
            }
            components[subgraphs[j]].clear();
        }
    }
}

void PushAndRotate::getSubgraphs(const SubMap &map, PARActorSet &actorSet)
{
    std::unordered_set<Node> close;
    std::vector<std::unordered_set<Node>> components;
    std::unordered_set<Node> joinNodes;
    int connectedComponentNum = 0;
    for(int i = 0; i < map.GetHeight(); ++i)
    {
        for(int j = 0; j < map.GetWidth(); ++j)
        {
            if(!map.CellIsObstacle(i, j))
            {
                Node curNode = Node(i, j);
                if(close.find(curNode) == close.end())
                {
                    int oldSize = close.size();
                    std::vector<std::pair<Node, Node>> edgeStack;
                    std::unordered_map<Node, int> in, up;
                    std::vector<std::tuple<Node, int, int>> stack = {std::make_tuple(curNode, -1, 0)};

                    while(!stack.empty())
                    {
                        std::tuple<Node, int, int> state = stack.back();
                        Node cur = std::get<0>(state);
                        int lastInd = std::get<1>(state);
                        int depth = std::get<2>(state);

                        std::list<Node> successors = search->findSuccessors(cur, map);
                        std::list<Node>::iterator it = successors.begin();
                        for(int i = 0; i < lastInd; ++i, ++it)
                        {
                        }
                        if(lastInd == -1)
                        {
                            close.insert(cur);
                            actorSet.setConnectedComponent(cur.i, cur.j, connectedComponentNum);
                            in[cur] = depth;
                            up[cur] = depth;
                        }
                        else
                        {
                            if((depth != 0 && up[*it] >= in[cur]) || depth == 0)
                            {
                                std::pair<Node, Node> curEdge = std::make_pair(cur, *it);
                                getComponent(actorSet, curEdge, edgeStack, components);
                                if(depth != 0)
                                {
                                    joinNodes.insert(cur);
                                }
                            }
                            up[cur] = std::min(up[*it], up[cur]);
                            it = std::next(it);
                        }
                        for(it, lastInd = lastInd + 1; it != successors.end(); ++it, ++lastInd)
                        {
                            if(close.find(*it) != close.end())
                            {
                                up[cur] = std::min(in[*it], up[cur]);
                            }
                            else
                            {
                                std::pair<Node, Node> curEdge = std::make_pair(cur, *it);
                                edgeStack.push_back(curEdge);
                                std::get<1>(stack.back()) = lastInd;
                                stack.push_back(std::make_tuple(*it, -1, depth + 1));
                                break;
                            }
                        }
                        if(it == successors.end())
                        {
                            stack.pop_back();
                        }
                    }

                    actorSet.addComponentSize(close.size() - oldSize);
                    ++connectedComponentNum;
                }
            }
        }
    }

    for(int i = 0; i < map.GetHeight(); ++i)
    {
        for(int j = 0; j < map.GetWidth(); ++j)
        {
            if(!map.CellIsObstacle(i, j) && map.GetCellDegree(i, j) >= 3 && actorSet.getSubgraphs(i, j).empty())
            {
                actorSet.setNodeSubgraph(i, j, components.size());
                components.push_back({Node(i, j)});
                joinNodes.insert(Node(i, j));
            }
        }
    }

    int m = map.GetEmptyCellCount() - actorSet.getActorCount();
    auto isGoal = [](const Node &start, const Node &cur, const SubMap &map, const PARActorSet &actorSet)
    {
        std::vector<int> startSubgraphs = actorSet.getSubgraphs(start.i, start.j);
        std::vector<int> curSubgraphs = actorSet.getSubgraphs(cur.i, cur.j);
        return curSubgraphs.size() > 1 || curSubgraphs.size() == 1 && curSubgraphs[0] != startSubgraphs[0];
    };

    std::vector<int> order;
    for(int i = 0; i < components.size(); ++i)
    {
        order.push_back(i);
    }
    std::sort(order.begin(), order.end(), [&components](const int a, const int b)
    {
        return components[a].size() > components[b].size();
    });

    for(int i : order)
    {
        for(auto start : components[i])
        {
            if(joinNodes.find(start) != joinNodes.end())
            {
                combineNodeSubgraphs(actorSet, components, start, i);
                ISearch dijkstraSearch;
                SearchResult searchResult = dijkstraSearch.startSearch(map, actorSet, start.i, start.j, 0, 0, isGoal, true, true, m - 2, components[i]);
                while(searchResult.pathfound)
                {
                    auto path = *searchResult.lppath;
                    for(auto it = std::next(path.begin()); std::next(it) != path.end(); ++it)
                    {
                        if(actorSet.getSubgraphs(it->i, it->j).empty())
                        {
                            actorSet.setNodeSubgraph(it->i, it->j, i);
                        }
                    }
                    combineNodeSubgraphs(actorSet, components, path.back(), i);
                    searchResult = dijkstraSearch.startSearch(map, actorSet, start.i, start.j, 0, 0, isGoal, false, m - 2);
                }
            }
        }
    }
}

int PushAndRotate::getReachableNodesCount(const SubMap &map, PARActorSet &actorSet, Node &start,
                                          bool (*condition)(const Node &, const Node &, const SubMap &,
                                                            const PARActorSet &),
                                          const std::unordered_set<Node> &occupiedNodes)
{
    int res = 0;
    ISearch dijkstraSearch;
    SearchResult searchResult = dijkstraSearch.startSearch(map, actorSet, start.i, start.j, 0, 0, condition, true, false, -1, occupiedNodes);
    while(searchResult.pathfound)
    {
        ++res;
        searchResult = dijkstraSearch.startSearch(map, actorSet, start.i, start.j, 0, 0, condition, false, false, -1, occupiedNodes);
    }
    return res;
}

void PushAndRotate::assignToSubgraphs(const SubMap &map, PARActorSet &actorSet)
{
    auto isUnoccupied = [](const Node &start, const Node &cur, const SubMap &map, const PARActorSet &actorSet)
    {
        return !actorSet.isOccupied(cur.i, cur.j);
    };

    std::vector<int> actorsInConnectedComponents(actorSet.getConnectedComponentsCount());
    for(int i = 0; i < actorSet.getActorCount(); ++i)
    {
        PARActor actor = actorSet.getActor(i);
        ++actorsInConnectedComponents[actorSet.getConnectedComponent(actor.getCur_i(), actor.getCur_j())];
    }

    int m = map.GetEmptyCellCount() - actorSet.getActorCount();
    for(int i = 0; i < map.GetHeight(); ++i)
    {
        for(int j = 0; j < map.GetWidth(); ++j)
        {
            if(map.CellIsObstacle(i, j))
            {
                continue;
            }
            Node pos(i, j);
            auto subgraphs = actorSet.getSubgraphs(pos.i, pos.j);
            if(subgraphs.empty())
            {
                continue;
            }
            int subgraph = subgraphs[0];
            auto successors = search->findSuccessors(pos, map);
            int totalCount = actorSet.getComponentSize(pos.i, pos.j) - actorsInConnectedComponents[actorSet.getConnectedComponent(pos.i, pos.j)];
            int throughPos = 0;
            bool hasSuccessorsInOtherSubgraph = false;
            for(auto neigh : successors)
            {
                auto neighSubgraphs = actorSet.getSubgraphs(neigh.i, neigh.j);
                if(neighSubgraphs.empty() || neighSubgraphs[0] != subgraph)
                {
                    hasSuccessorsInOtherSubgraph = true;
                    int throughNeigh = getReachableNodesCount(map, actorSet, neigh, isUnoccupied, {pos});
                    int m1 = totalCount - throughNeigh;
                    if(m1 >= 1 && m1 < m && actorSet.isOccupied(i, j))
                    {
                        actorSet.setActorSubgraph(actorSet.getActorId(pos.i, pos.j), subgraph);
                    }
                    auto isGoal = [](const Node &start, const Node &cur, const SubMap &map, const PARActorSet &actorSet)
                    {
                        return map.GetCellDegree(cur.i, cur.j) == 1 || !actorSet.getSubgraphs(cur.i, cur.j).empty();
                    };
                    ISearch dijkstraSearch;
                    SearchResult searchResult = dijkstraSearch.startSearch(map, actorSet, neigh.i, neigh.j, 0, 0, isGoal, true, true, -1, {
                            pos});
                    auto path = *searchResult.lppath;
                    int actorCount = 0;
                    for(auto node : path)
                    {
                        if(actorSet.isOccupied(node.i, node.j))
                        {
                            if(actorCount >= m1 - 1)
                            {
                                break;
                            }
                            actorSet.setActorSubgraph(actorSet.getActorId(node.i, node.j), subgraph);
                            ++actorCount;
                        }
                    }
                    throughPos += throughNeigh;
                }
            }
            if(actorSet.isOccupied(i, j) && (!hasSuccessorsInOtherSubgraph || totalCount - throughPos >= 1))
            {
                actorSet.setActorSubgraph(actorSet.getActorId(pos.i, pos.j), subgraph);
            }
        }
    }
}

void PushAndRotate::getPriorities(const SubMap &map, PARActorSet &actorSet)
{
    std::unordered_map<Node, int> goalPositions;
    for(int i = 0; i < actorSet.getActorCount(); ++i)
    {
        goalPositions[actorSet.getActor(i).getGoalPosition()] = i;
    }

    auto isGoal = [](const Node &start, const Node &cur, const SubMap &map, const PARActorSet &actorSet)
    {
        return !actorSet.getSubgraphs(cur.i, cur.j).empty();
    };
    for(int i = 0; i < map.GetHeight(); ++i)
    {
        for(int j = 0; j < map.GetWidth(); ++j)
        {
            if(map.CellIsObstacle(i, j))
            {
                continue;
            }

            auto subgraphs = actorSet.getSubgraphs(i, j);
            if(subgraphs.empty())
            {
                continue;
            }
            int subgraph = subgraphs[0];
            auto successors = search->findSuccessors(Node(i, j), map);
            for(auto neigh : successors)
            {
                auto neighSubgraphs = actorSet.getSubgraphs(neigh.i, neigh.j);
                if(neighSubgraphs.empty() || neighSubgraphs[0] != subgraph)
                {
                    ISearch dijkstraSearch;
                    SearchResult searchResult = dijkstraSearch.startSearch(map, actorSet, neigh.i, neigh.j, 0, 0, isGoal, true, true, -1,
                            {Node(i, j)});
                    if(!searchResult.pathfound)
                    {
                        continue;
                    }
                    auto path = *searchResult.lppath;
                    path.push_front(Node(i, j));
                    for(auto node : path)
                    {
                        auto it = goalPositions.find(node);
                        if(it == goalPositions.end())
                        {
                            break;
                        }
                        PARActor actor = actorSet.getActor(it->second);
                        int actorSubgraph = actor.getSubgraph();
                        if(actorSubgraph != -1)
                        {
                            if(actorSubgraph != subgraph)
                            {
                                actorSet.setPriority(subgraph, actorSubgraph);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
}


PARSearchResult PushAndRotate::startSearch(const SubMap &map, const PARConfig &config, PARActorSet &actorSet)
{
    //std::cout << actorSet.getActorCount() << std::endl;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    getSubgraphs(map, actorSet);

    PARActorSet goalActorSet = actorSet;
    for(int i = 0; i < actorSet.getActorCount(); ++i)
    {
        Node goal = actorSet.getActor(i).getGoalPosition();
        goalActorSet.setActorPosition(i, goal);
    }
    assignToSubgraphs(map, actorSet);
    assignToSubgraphs(map, goalActorSet);
    for(int i = 0; i < actorSet.getActorCount(); ++i)
    {
        if(actorSet.getActor(i).getSubgraph() != goalActorSet.getActor(i).getSubgraph())
        {
            result.pathfound = false;
            return result;
        }
    }
    getPriorities(map, actorSet);

    result.pathfound = solve(map, config, actorSet);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    if(elapsedMilliseconds > config.maxTime)
    {
        result.pathfound = false;
    }
    if(result.pathfound)
    {
        getParallelPaths(actorSet);
        result.actorsMoves = &actorsMoves;
        result.actorsPaths = &actorsPaths;
        result.time = static_cast<double>(elapsedMilliseconds) / 1000;
    }
    return result;
}


void PushAndRotate::getPaths(PARActorSet &actorSet)
{


    actorsPaths.resize(actorSet.getActorCount());
    std::vector<Node> actorPositions;
    for (int i = 0; i < actorSet.getActorCount(); ++i) {
        Node startPosition = actorSet.getActor(i).getStartPosition();
        actorPositions.push_back(startPosition);
        actorsPaths[i].push_back(startPosition);
    }
    for (int i = 0; i < actorsMoves.size(); ++i) {
        actorPositions[actorsMoves[i].id].i += actorsMoves[i].di;
        actorPositions[actorsMoves[i].id].j += actorsMoves[i].dj;
        for (int j = 0; j < actorSet.getActorCount(); ++j) {
            actorsPaths[j].push_back(actorPositions[j]);
        }
    }
}