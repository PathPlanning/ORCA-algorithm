#include <unordered_map>
#include <list>

#include "Geom.h"
#include "SubMap.h"

#ifndef ORCA_CONFLICTAVOIDANCETABLE_H
#define ORCA_CONFLICTAVOIDANCETABLE_H

class ConflictAvoidanceTable
{
    public:
        std::unordered_map<int, int> agentsCount;

        void addActorPosition(int i, int j, int time, const SubMap& map);
        void removeActorPosition(int i, int j, int time, const SubMap& map);
        void addActorPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const SubMap& map);
        void removeActorPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const SubMap& map);
        int getActorCount(int i, int j, int time, const SubMap& map) const;
};

#endif //ORCA_CONFLICTAVOIDANCETABLE_H
