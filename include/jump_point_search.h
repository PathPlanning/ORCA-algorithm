#include <memory>
#include <list>

#include "updatable_map.h"
#include "EnvironmentOptions.h"
#include "Geom.h"

#ifndef ORCASTAR_JPS_PLANNER_H
#define ORCASTAR_JPS_PLANNER_H

class JumpPointSearch
{
    public:
        JumpPointSearch() = default;
        ~JumpPointSearch() = default;
        JumpPointSearch & operator = (const JumpPointSearch &obj);
		bool StartSearch(	std::pair<int64_t, int64_t> start_cell,
				   			std::pair<int64_t, int64_t> goal_cell,
				   			std::shared_ptr<UpdatableMap> map_ptr,
				   			std::shared_ptr<EnvironmentOptions> options_ptr,
				   			std::list<Point>& path);
        
    private:
        bool FindNeighbors(int64_t move_i, int64_t move_j, Node curNode);
        void FindJP(int64_t move_i, int64_t move_j, Node cur_node,std::list<Node> &successors);
        int  FindDirection(int64_t current_i, int64_t parent_i);
        std::list<Node> FindSuccessors(Node cur_node);
        float ComputeHFromCellToCell(int64_t i1, int64_t j1, int64_t i2, int64_t j2) const;
        Node FindMin();
        bool StopCriterion() const;
        void MakePrimaryPath(Node last_node, std::list<Point>& path);
        float Distance(int64_t i1, int64_t j1, int64_t i2, int64_t j2) const;
        void AddToOpen(Node new_node);
    
     
        std::shared_ptr<UpdatableMap> 		map;
        std::shared_ptr<EnvironmentOptions> options;
        
        std::pair<int64_t, int64_t> 		start;
        std::pair<int64_t, int64_t> 		goal;

        std::unordered_map<int64_t, Node>   close;
        std::vector<std::list<Node>>    	open;
        size_t 								open_size;
};


#endif //ORCASTAR_JPS_PLANNER_H
