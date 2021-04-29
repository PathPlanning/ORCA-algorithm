#include "global_path_planner.h"

#include <utility>

bool JPSPlanner::FindPath(Point start, Point goal)
{
	this->start = start;
	this->goal = goal;
	path.clear();
	auto start_cell = map->FindCellForPoint(start);
	auto goal_cell = map->FindCellForPoint(goal);
	
	path_created = search.StartSearch(start_cell, goal_cell, map, options, path);
	
	map.reset();
	options.reset();
		
	return path_created;
}

bool JPSPlanner::GetNextTarget(Point curr, Point &next)
{
	if(path_created)
	{
		if(path.size() == 0)
		{
			path.push_back(goal);
		}
		
		if(path.size() > 1)
		{
			float sqDistToCurr = (path.front() - curr).SquaredEuclideanNorm();
			float sqDelta = options->delta * options->delta;
			if(sqDistToCurr < sqDelta)
			{
				path.pop_front();
				next = path.front();
				return true;
			}
		}
		
		Node currNode = map->FindCellForPoint(curr);
		
		Node nextNode = map->FindCellForPoint(path.front());
		if(std::next(path.begin()) != path.end())
		{
			Node nextNextNode = map->FindCellForPoint(*std::next(path.begin()));
			if(currNode == nextNode || map->CheckVisibility(currNode.i, currNode.j, nextNode.i, nextNode.j, options->cutcorners))
			{
				next = path.front();
				if(currNode == nextNextNode || map->CheckVisibility(currNode.i, currNode.j, nextNextNode.i, nextNextNode.j, options->cutcorners))
				{
					path.pop_front();
					next = path.front();
				}
				return true;
			}
		}
		else
		{
			if(currNode == nextNode || map->CheckVisibility(currNode.i, currNode.j, nextNode.i, nextNode.j, options->cutcorners))
			{
				next = path.front();
				return true;
			}
		}


//        Node currNode = map->GetClosestNode(curr), nextNode = map->CreateNodeForPoint(currPath.front());
//        if(currNode == nextNode || visChecker.checkLine(currNode.i, currNode.j, nextNode.i, nextNode.j, *map))
//        {
//            next = currPath.front();
//            return true;
//        }
		
		Point last = path.front();
		path.pop_front();

//        bool isLastAccessible = SearchPath(map->GetClosestNode(curr), map->CreateNodeForPoint(last));
//        if(isLastAccessible)
//        {
//            next = currPath.front();
//            return true;
//        }
		path.clear();
		bool isGoalAccessible = search.StartSearch(map->FindCellForPoint(curr), map->FindCellForPoint(goal), map, options, path);
		if(isGoalAccessible)
		{
			next = path.front();
			return true;
		}
	}
	return false;
}

JPSPlanner::JPSPlanner(std::shared_ptr<UpdatableMap> map_ptr, std::shared_ptr<EnvironmentOptions> options_ptr)
{
	if(map_ptr and options_ptr)
	{
		map = std::move(map_ptr);
		options = std::move(options_ptr);
		path_created = false;
	}
	assert("map and options ptr should be transferred to planner");
}

JPSPlanner* JPSPlanner::Clone()
{
	return new JPSPlanner(*this);
}

JPSPlanner::JPSPlanner()
{
	path_created =  false;
}

JPSPlanner::JPSPlanner(const JPSPlanner& obj)
{
	path = obj.path;
	search = obj.search;
	start = obj.start;
	goal = obj.goal;
	path_created = obj.path_created;
	map = obj.map;
	options = obj.options;
}
