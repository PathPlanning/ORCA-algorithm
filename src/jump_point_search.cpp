#include "jump_point_search.h"

JumpPointSearch &JumpPointSearch::operator=(const JumpPointSearch &obj)
{
    // TODO
    return *this;
}

bool JumpPointSearch::FindPath(Point start, Point goal, std::weak_ptr<UpdatableMap> map_ptr,
                               std::weak_ptr<EnvironmentOptions> options_ptr, std::list<Point>& path)
{
    map = map_ptr.lock();
    options = options_ptr.lock();
    if(map and options)
    {
		this->start = start;
		this->goal = goal;
        start_cell = map->FindCellForPoint(start);
        goal_cell = map->FindCellForPoint(goal);
        
        path_created = StartSearch(path);

        map.reset();
        options.reset();
        
        return path_created;
    }
    assert("map and options ptr should be transferred to planner");
    return false;
}

bool JumpPointSearch::FindNeighbors(int64_t move_i, int64_t move_j, Node cur_node)
{
    while(map->CellOnGrid(cur_node.i, cur_node.j) && map->CellIsTraversable(cur_node.i, cur_node.j))
    {
        if(goal_cell.first == cur_node.i && goal_cell.second == cur_node.j) //goal location is found
            return true;
        if(options->cutcorners)
        {
            if(move_i == 0 && map->CellOnGrid(cur_node.i, cur_node.j+move_j))
            {
                if(map->CellOnGrid(cur_node.i + 1, cur_node.j))
                    if(map->CellIsTraversable(cur_node.i + 1, cur_node.j+move_j) &&
                        map->CellIsObstacle(cur_node.i + 1, cur_node.j))
                        return true;
                if(map->CellOnGrid(cur_node.i - 1,cur_node.j))
                    if(map->CellIsTraversable(cur_node.i - 1, cur_node.j+move_j) &&
                        map->CellIsObstacle(cur_node.i - 1, cur_node.j))
                        return true;
            }
            if(move_j == 0 && map->CellOnGrid(cur_node.i + move_i, cur_node.j))
            {
                if(map->CellOnGrid(cur_node.i, cur_node.j + 1))
                    if(map->CellIsTraversable(cur_node.i + move_i, cur_node.j + 1) &&
                        map->CellIsObstacle(cur_node.i, cur_node.j + 1))
                        return true;
                if(map->CellOnGrid(cur_node.i, cur_node.j - 1))
                    if(map->CellIsTraversable(cur_node.i + move_i, cur_node.j - 1) &&
                        map->CellIsObstacle(cur_node.i, cur_node.j - 1))
                        return true;
            }
        }
        else
        {
            if(move_i == 0 && map->CellOnGrid(cur_node.i, cur_node.j - move_j))
            {
                if(map->CellOnGrid(cur_node.i + 1, cur_node.j))
                    if(map->CellIsTraversable(cur_node.i + 1, cur_node.j) &&
                        map->CellIsObstacle(cur_node.i + 1, cur_node.j - move_j))
                        return true;
                if(map->CellOnGrid(cur_node.i - 1,cur_node.j))
                    if(map->CellIsTraversable(cur_node.i - 1, cur_node.j) &&
                        map->CellIsObstacle(cur_node.i - 1, cur_node.j - move_j))
                        return true;
            }
            if(move_j == 0 && map->CellOnGrid(cur_node.i - move_i, cur_node.j))
            {
                if(map->CellOnGrid(cur_node.i, cur_node.j + 1))
                    if(map->CellIsTraversable(cur_node.i, cur_node.j + 1) &&
                        map->CellIsObstacle(cur_node.i - move_i, cur_node.j + 1))
                        return true;
                if(map->CellOnGrid(cur_node.i, cur_node.j - 1))
                    if(map->CellIsTraversable(cur_node.i, cur_node.j - 1) &&
                        map->CellIsObstacle(cur_node.i - move_i, cur_node.j - 1))
                        return true;
            }
        }
        cur_node.i += move_i;
        cur_node.j += move_j;
    }
    return false;
}

void JumpPointSearch::FindJP(int64_t move_i, int64_t move_j, Node cur_node, std::list<Node> &successors)
{
    bool findOK = false;
    while (!findOK)
    {
        if ( map->CellOnGrid(cur_node.i + move_i, cur_node.j + move_j)) {
            if (!options->cutcorners)
            {
                if (move_i != 0 && move_j != 0)
                    if (!map->CellIsTraversable(cur_node.i, cur_node.j + move_j) ||
                        !map->CellIsTraversable(cur_node.i + move_i, cur_node.j))
                        return;
            }
            else if (!options->allowsqueeze)
            {
                if (move_i != 0 && move_j != 0)
                    if (!map->CellIsTraversable(cur_node.i, cur_node.j + move_j) &&
                        !map->CellIsTraversable(cur_node.i + move_i, cur_node.j))
                        return;
            }
            if (map->CellIsTraversable(cur_node.i + move_i, cur_node.j + move_j))
            {
                cur_node.i += move_i;
                cur_node.j += move_j;
                if (move_i == 0 || move_j == 0)
                    cur_node.g += 1;
                else
                    cur_node.g += sqrt(2);
            }
            else
                return;
        }
        else
            return;
        
        if (goal_cell.first == cur_node.i && goal_cell.second == cur_node.j)
            findOK = true;

        if (options->cutcorners)
        {
            if (move_i == 0)
            { //straight move along j
                if (map->CellOnGrid(cur_node.i + 1, cur_node.j + move_j))
                    if (map->CellIsTraversable(cur_node.i + 1, cur_node.j + move_j) &&
                        map->CellIsObstacle(cur_node.i + 1, cur_node.j))
                        findOK = true;
                if (map->CellOnGrid(cur_node.i - 1, cur_node.j + move_j))
                    if (map->CellIsTraversable(cur_node.i - 1, cur_node.j + move_j) &&
                        map->CellIsObstacle(cur_node.i - 1, cur_node.j))
                        findOK = true;
            } else if (move_j == 0)
            { //straight move along i
                if (map->CellOnGrid(cur_node.i + move_i, cur_node.j + 1))
                    if (map->CellIsTraversable(cur_node.i + move_i, cur_node.j + 1) &&
                        map->CellIsObstacle(cur_node.i, cur_node.j + 1))
                        findOK = true;
                if (map->CellOnGrid(cur_node.i + move_i, cur_node.j - 1))
                    if (map->CellIsTraversable(cur_node.i + move_i, cur_node.j - 1) &&
                        map->CellIsObstacle(cur_node.i, cur_node.j - 1))
                        findOK = true;
            }
            else
            { //diagonal move
                if (map->CellOnGrid(cur_node.i - move_i, cur_node.j + move_j))
                    if (map->CellIsObstacle(cur_node.i - move_i, cur_node.j) &&
                        map->CellIsTraversable(cur_node.i - move_i, cur_node.j + move_j))
                        findOK = true;
                if (!findOK && map->CellOnGrid(cur_node.i + move_i, cur_node.j - move_j))
                    if (map->CellIsObstacle(cur_node.i, cur_node.j - move_j) &&
                        map->CellIsTraversable(cur_node.i + move_i, cur_node.j - move_j))
                        findOK = true;
                if (!findOK)
                    if (FindNeighbors(move_i, 0, cur_node))
                        findOK = true;
                if (!findOK)
                    if (FindNeighbors(0, move_j, cur_node))
                        findOK = true;
            }
        }
        else
        {
            if (move_i == 0)
            { //straight move along j
                if (map->CellOnGrid(cur_node.i + 1, cur_node.j))
                    if (map->CellIsTraversable(cur_node.i + 1, cur_node.j) &&
                        map->CellIsObstacle(cur_node.i + 1, cur_node.j - move_j))//check forced neighbor
                        findOK = true;
                if (map->CellOnGrid(cur_node.i - 1, cur_node.j))
                    if (map->CellIsTraversable(cur_node.i - 1, cur_node.j) &&
                        map->CellIsObstacle(cur_node.i - 1, cur_node.j - move_j))
                        findOK = true;
            }
            else if (move_j == 0)
            { //straight move along i
                if (map->CellOnGrid(cur_node.i, cur_node.j + 1))
                    if (map->CellIsTraversable(cur_node.i, cur_node.j + 1) &&
                        map->CellIsObstacle(cur_node.i - move_i, cur_node.j + 1))
                        findOK = true;
                if (map->CellOnGrid(cur_node.i, cur_node.j - 1))
                    if (map->CellIsTraversable(cur_node.i, cur_node.j - 1) &&
                        map->CellIsObstacle(cur_node.i - move_i, cur_node.j - 1))
                        findOK = true;
            }
            else
            { //diagonal move
                if (FindNeighbors(move_i, 0, cur_node))//looking for forced neighbor along i
                    findOK = true;
                if (!findOK)
                    if (FindNeighbors(0, move_j, cur_node))//looking for forced neighbor along j
                        findOK = true;
            }
        }
    }
    if (close.find(cur_node.i * map->GridWidth() + cur_node.j) == close.end())
        successors.push_front(cur_node);
    return;
}

int JumpPointSearch::FindDirection(int64_t current_i, int64_t parent_i)
{
    if(current_i < parent_i)
        return -1;
    else if(current_i > parent_i)
        return 1;
    else
        return 0;
}

std::list<Node> JumpPointSearch::FindSuccessors(Node cur_node)
{
    int move_i = 0, move_j = 0;
    std::list<Node> successors;
    

    if(cur_node.i == start_cell.first && cur_node.j == start_cell.second) //if cur_node is the start location, then look for jump points in all directions
        for(int n = -1; n <= 1; n++)
            for(int m = -1; m <= 1; m++)
                if(n != 0 || m != 0)
                    FindJP(n, m, cur_node, successors);
    if(cur_node.i != start_cell.first || cur_node.j != start_cell.second)
    {
        move_i = FindDirection(cur_node.i, cur_node.parent->i);
        move_j = FindDirection(cur_node.j, cur_node.parent->j);
        FindJP(move_i, move_j, cur_node, successors);//continue to look for jump points in the same direction
        
        if(move_i != 0 && move_j != 0)
        { //if curNoode is a diagonal jump point
            if(map->CellIsObstacle(cur_node.i - move_i, cur_node.j))
                FindJP(-move_i, move_j, cur_node, successors);
            if(map->CellIsObstacle(cur_node.i, cur_node.j - move_j))
                FindJP(move_i, -move_j, cur_node, successors);
            FindJP(move_i, 0, cur_node, successors);//look for jump point in straight direction along i
            FindJP(0, move_j, cur_node, successors);//the same check along j
        }
        
        if(options->cutcorners)
        { //original JPS, when cutcorners is allowed
            if(move_i == 0)
            {
                if(map->CellOnGrid(cur_node.i - move_j, cur_node.j))
                    if(map->CellIsObstacle(cur_node.i - move_j, cur_node.j))
                        FindJP(-move_j, move_j, cur_node, successors);
                if(map->CellOnGrid(cur_node.i+move_j, cur_node.j))
                    if(map->CellIsObstacle(cur_node.i + move_j, cur_node.j))
                        FindJP(move_j, move_j, cur_node, successors);
            }
            else if(move_j == 0) {
                if(map->CellOnGrid(cur_node.i, cur_node.j - move_i))
                    if(map->CellIsObstacle(cur_node.i, cur_node.j - move_i))
                        FindJP(move_i, -move_i, cur_node, successors);
                if(map->CellOnGrid(cur_node.i, cur_node.j + move_i))
                    if(map->CellIsObstacle(cur_node.i, cur_node.j + move_i))
                        FindJP(move_i, move_i, cur_node, successors);
            }
        }
        else
        { //cutcorners disallowed
            if(move_i == 0)
            {
                if(map->CellOnGrid(cur_node.i - move_j, cur_node.j))
                    if(map->CellIsObstacle(cur_node.i - move_j, cur_node.j - move_j))
                    {
                        FindJP(-move_j, move_j, cur_node, successors);
                        FindJP(-move_j, 0, cur_node, successors);
                    }
                if(map->CellOnGrid(cur_node.i + move_j, cur_node.j))
                    if(map->CellIsObstacle(cur_node.i + move_j, cur_node.j - move_j))
                    {
                        FindJP(move_j, 0, cur_node, successors);
                        FindJP(move_j, move_j, cur_node, successors);
                    }
            }
            else if(move_j == 0)
            {
                if(map->CellOnGrid(cur_node.i, cur_node.j-move_i))
                    if(map->CellIsObstacle(cur_node.i - move_i, cur_node.j - move_i))
                    {
                        FindJP(0, -move_i, cur_node, successors); //additional check
                        FindJP(move_i, -move_i, cur_node, successors);
                    }
                if(map->CellOnGrid(cur_node.i, cur_node.j + move_i))
                    if(map->CellIsObstacle(cur_node.i - move_i, cur_node.j + move_i))
                    {
                        FindJP(0, move_i, cur_node, successors); //additional check
                        FindJP(move_i, move_i, cur_node, successors);
                    }
            }
        }
    }
    
    return successors;
}

bool JumpPointSearch::StartSearch(std::list<Point>& path)
{
    if(path_created)
    {
        close.clear();
        for(auto &l : open)
        {
            l.clear();
        }
        open_size = 0;
    }
    
    open.resize(map->GridHeight());
    
    
    Node cur_node;
    cur_node.i = start_cell.first;
    cur_node.j = start_cell.second;
    cur_node.g = 0;
    cur_node.H = ComputeHFromCellToCell(cur_node.i, cur_node.j, goal_cell.first, goal_cell.second);
    cur_node.F = options->hweight * cur_node.H;
    cur_node.parent = nullptr;
    AddToOpen(cur_node);
    size_t close_size = 0;
    bool pathfound = false;
    while (!StopCriterion())
    {
        cur_node = FindMin();
        close.insert({cur_node.i * map->GridWidth() + cur_node.j, cur_node});
        close_size++;
        open[cur_node.i].pop_front();
        open_size--;
        if (cur_node.i == goal_cell.first and cur_node.j == goal_cell.second)
        {
            pathfound = true;
            break;
        }
        std::list<Node> successors = FindSuccessors(cur_node);
        std::list<Node>::iterator it = successors.begin();
        auto parent = &(close.find(cur_node.i * map->GridWidth() + cur_node.j)->second);
        while (it != successors.end())
        {
            it->parent = parent;
            it->H = ComputeHFromCellToCell(it->i, it->j, goal_cell.first, goal_cell.second);
            it->F = it->g + options->hweight * it->H;
            AddToOpen(*it);
            it++;
        }
    }
    if(pathfound)
	{
		MakePrimaryPath(cur_node, path);
	}
    return pathfound;
}

float JumpPointSearch::ComputeHFromCellToCell(int64_t i1, int64_t j1, int64_t i2, int64_t j2) const
{
    switch (options->metrictype)
    {
        case CN_SP_MT_EUCL:
            return static_cast<float>(sqrt((i2 - i1)*(i2 - i1)+(j2 - j1)*(j2 - j1)));
        case CN_SP_MT_DIAG:
            return static_cast<float>(abs(abs(i2 - i1) - abs(j2 - j1)) + sqrt(2) * (std::min(abs(i2 - i1),abs(j2 - j1))));
        case CN_SP_MT_MANH:
            return (abs(i2 - i1) + abs(j2 - j1));
        case CN_SP_MT_CHEB:
            return std::max(abs(i2 - i1),abs(j2 - j1));
        default:
            return 0;
    }
}

Node JumpPointSearch::FindMin()
{
    Node min;
    min.F = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < open.size(); i++)
    {
        if(!open[i].empty() && open[i].begin()->F <= min.F)
        {
            if(open[i].begin()->F == min.F)
            {
                if((options->breakingties == CN_SP_BT_GMAX && open[i].begin()->g >= min.g)
                   || (options->breakingties == CN_SP_BT_GMIN && open[i].begin()->g <= min.g))
                {
                    min = *open[i].begin();
                }
            }
            else
            {
                min = *open[i].begin();
            }
        }
    }
    return min;
}

bool JumpPointSearch::StopCriterion() const
{
    if (open_size == 0)
    {
        return true;
    }
    return false;
}

void JumpPointSearch::MakePrimaryPath(Node last_node, std::list<Point>& path)
{
	
	// TODO
//	Node cur_node = last_node;
//	path.push_back(map->CenterPosition(cur_node.i, cur_node.j));
//	while(cur_node.parent != nullptr)
//	{
//		cur_node = *cur_node.parent;
//		if(!map->CheckVisibility(last_node.i, last_node.j, cur_node.i, cur_node.j, options->cutcorners))
//		{
//			path.push_back(map->CenterPosition(cur_node.i, cur_node.j));
//			new_hppath.push_back(*std::prev(node));
//		}
//	}
//
//	sr.pathlength += static_cast<float>(Theta::distance(new_hppath.back().i, new_hppath.back().j, sr.hppath->back().i, sr.hppath->back().j));
//	new_hppath.push_back(sr.hppath->back());
//	*sr.hppath = new_hppath;
}

float JumpPointSearch::Distance(int64_t i1, int64_t j1, int64_t i2, int64_t j2) const
{
    return static_cast<float>(sqrt(pow(i1 - i2, 2.0f) + pow(j1 - j2, 2.0f)));
}


void JumpPointSearch::AddToOpen(Node new_node)
{
    std::list<Node>::iterator iter, pos;
    
    if (open[new_node.i].size() == 0) {
        open[new_node.i].push_back(new_node);
        open_size++;
        return;
    }
    
    pos = open[new_node.i].end();
    bool posFound = false;
    for (iter = open[new_node.i].begin(); iter != open[new_node.i].end(); ++iter)
    {
        if (!posFound && iter->F >= new_node.F)
        {
            if(iter->F == new_node.F)
            {
                if((options->breakingties == CN_SP_BT_GMAX && new_node.g >= iter->g)
                   || (options->breakingties == CN_SP_BT_GMIN && new_node.g <= iter->g))
                {
                    pos = iter;
                    posFound = true;
                }
            }
            else
            {
                pos = iter;
                posFound = true;
            }
        }
        if (iter->j == new_node.j)
        {
            if (new_node.F >= iter->F)
                return;
            else
            {
                if (pos == iter)
                {
                    iter->F = new_node.F;
                    iter->g = new_node.g;
                    iter->parent = new_node.parent;
                    return;
                }
                open[new_node.i].erase(iter);
                open_size--;
                break;
            }
        }
    }
    open_size++;
    open[new_node.i].insert(pos, new_node);
}