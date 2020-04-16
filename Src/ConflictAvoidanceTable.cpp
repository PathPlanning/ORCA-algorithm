#include "ConflictAvoidanceTable.h"

int convolution(int i, int j, const SubMap &map, int time, bool withTime)
{
    int res = withTime ? map.GetWidth() * map.GetHeight() * time : 0;
    return res + i * map.GetWidth() + j;
}


void ConflictAvoidanceTable::addActorPosition(int i, int j, int time, const SubMap& map)
{
    int conv = convolution(i, j, map, time, true);
    if (agentsCount.find(conv) == agentsCount.end())
    {
        agentsCount[conv] = 1;
    }
    else
    {
        ++agentsCount[conv];
    }
}

void ConflictAvoidanceTable::addActorPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const SubMap& map)
{
    for (auto it = start; it != end; ++it)
    {
        addActorPosition(it->i, it->j, it->depth, map);
    }
}

void ConflictAvoidanceTable::removeActorPosition(int i, int j, int time, const SubMap& map)
{
    int conv = convolution(i, j, map, time, true);
    --agentsCount[conv];
}

void ConflictAvoidanceTable::removeActorPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const SubMap& map)
{
    for (auto it = start; it != end; ++it)
    {
        removeActorPosition(it->i, it->j, it->depth, map);
    }
}

int ConflictAvoidanceTable::getActorCount(int i, int j, int time, const SubMap& map) const
{
    int conv = convolution(i, j, map, time, true);
    if (agentsCount.find(conv) == agentsCount.end())
    {
        return 0;
    }
    return agentsCount.at(conv);
}
