#include "SubMap.h"


SubMap::SubMap()
{
    cellSize = 0.0;
    size = {0,0};
    origin = {0,0};
    divKoef = 0;
    fullMap = nullptr;
    emptyCells = 0;
}


SubMap::SubMap(const Map *map, Node mapOrigin, std::pair<int, int> mapSize, int divK)
{
    this->cellSize = map->GetCellSize() / divK;
    origin = mapOrigin;
    size = {mapSize.first * divK, mapSize.second * divK};
    divKoef = divK;
    fullMap = map;
    emptyCells = 0;
    for(int i = 0; i < size.first; i++)
    {
        for(int j = 0; j < size.second; j++)
        {
            if(CellOnGrid(i, j) && CellIsTraversable(i, j))
            {
                emptyCells++;
            }
        }
    }
    emptyCells *= (divK * divK);
}


SubMap::SubMap(const SubMap &obj)
{
    this->cellSize = obj.cellSize;
    origin = obj.origin;
    size = obj.size;
    divKoef = obj.divKoef;
    fullMap = obj.fullMap;
    emptyCells = obj.emptyCells;
}


SubMap::~SubMap()
{
    fullMap = nullptr;
}


bool SubMap::CellIsObstacle(int i, int j) const
{
    int iM = origin.i + i / divKoef;
    int jM = origin.j + j / divKoef;
    return CellOnGrid(i,j) && fullMap->CellIsObstacle(iM, jM);
}


bool SubMap::CellIsTraversable(int i, int j) const
{
    int iM = origin.i + i / divKoef;
    int jM = origin.j + j / divKoef;
    return CellOnGrid(i,j) && fullMap->CellIsTraversable(iM, jM);
}


bool SubMap::CellOnGrid(int i, int j) const
{
    return (i < size.first && i >= 0 && j < size.second && j >= 0);
}


int SubMap::GetHeight() const
{
    return size.first;
}


int SubMap::GetWidth() const
{
    return size.second;
}


float SubMap::GetCellSize() const
{
    return cellSize;
}


Node SubMap::GetClosestNode(const Point &point) const
{
    Node res;
    res.i = fullMap->GetHeight() * divKoef - 1 - (int) (point.Y() / cellSize);
    res.j = (int) (point.X() / cellSize);

    res.i = res.i - origin.i * divKoef;
    res.j = res.j - origin.j * divKoef;

    if(res.i < 0)
    {
        res.i = 0;
    }
    if(res.i > size.first - 1)
    {
        res.i = size.first - 1;
    }
    if(res.j < 0)
    {
        res.j = 0;
    }
    if(res.j > size.second - 1)
    {
        res.j = size.second - 1;
    }

    return res;
}


Point SubMap::GetPoint(const Node &node) const
{
    Point originPoint = fullMap->GetPoint(origin) + Point(-fullMap->GetCellSize()/2, fullMap->GetCellSize()/2);

    return {(originPoint.X() + node.j * cellSize + cellSize/2), (originPoint.Y() - (node.i * cellSize + cellSize/2))};
}

SubMap &SubMap::operator =(const SubMap &obj)
{
    if(this != &obj)
    {
        this->cellSize = obj.cellSize;
        origin = obj.origin;
        size = obj.size;
        divKoef = obj.divKoef;
        fullMap = obj.fullMap;
        emptyCells = obj.emptyCells;
    }
    return *this;
}

bool SubMap::CellIsTraversable(int i, int j, const std::unordered_set<Node> &occupiedNodes) const
{
    return CellIsTraversable(i, j) && occupiedNodes.find(Node(i, j)) == occupiedNodes.end();
}


int SubMap::GetCellDegree(int i, int j) const
{
    int degree = 0;
    for (int di = -1; di <= 1; ++di)
    {
        for (int dj = -1; dj <= 1; ++dj)
        {
            if ((di == 0) ^ (dj == 0))
            {
                if (CellOnGrid(i + di, j + dj) && !CellIsObstacle(i + di, j + dj))
                {
                    ++degree;
                }
            }
        }
    }
    return degree;
}

int SubMap::GetEmptyCellCount() const
{
    return emptyCells;
}

Node SubMap::FindAvailableNode(Node start, std::unordered_map<int, Node> occupied)
{
    if(occupied.find(start.i * GetWidth() + start.j) == occupied.end() && CellIsTraversable(start.i, start.j))
    {
        return start;
    }
    std::list<Node> open = std::list<Node>();
    std::unordered_map<int, Node> close = std::unordered_map<int, Node>();

    open.push_back(start);

    while(!open.empty())
    {
        Node curr = open.front();
        close.insert({curr.i * GetWidth() + curr.j, curr});
        open.pop_front();

        if(CellIsTraversable(curr.i, curr.j) && occupied.find(curr.i * GetWidth() + curr.j) == occupied.end())
        {
            return curr;
        }

        std::vector<Node> successors = {Node(curr.i + 1, curr.j), Node(curr.i - 1, curr.j), Node(curr.i, curr.j + 1), Node(curr.i, curr.j - 1)};

        for(auto &s : successors)
        {
            if(CellOnGrid(s.i,s.j))
            {
                if(CellIsObstacle(curr.i,curr.j) || CellIsTraversable(s.i,s.j))
                {
                    if(close.find(s.i * GetWidth() + s.j) == close.end() && find(open.begin(),open.end(), s) == open.end())
                    {
                        open.push_back(s);
                    }

                }

            }
        }

    }
    return Node(-1,-1);
}

Node SubMap::FindCloseToPointAvailableNode(Point pos, std::unordered_map<int, Node> occupied)
{
    auto start = this->GetClosestNode(pos);


    if(occupied.find(start.i * GetWidth() + start.j) == occupied.end() && CellIsTraversable(start.i, start.j))
    {
        return start;
    }

    start.H = (this->GetPoint(start) - pos).SquaredEuclideanNorm();

    auto cmp = [](Node left, Node right) { return (left.H > right.H); };

    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);

    std::unordered_map<int, Node> openDupl = std::unordered_map<int, Node>();

    std::unordered_map<int, Node> close = std::unordered_map<int, Node>();

    open.push(start);
    openDupl.insert({start.i * GetWidth() + start.j, start});

    while(!open.empty())
    {
        Node curr = open.top();
        close.insert({curr.i * GetWidth() + curr.j, curr});
        open.pop();
        openDupl.erase(curr.i * GetWidth() + curr.j);

        if(CellIsTraversable(curr.i, curr.j) && occupied.find(curr.i * GetWidth() + curr.j) == occupied.end())
        {
            return curr;
        }

        std::vector<Node> successors = {Node(curr.i + 1, curr.j), Node(curr.i - 1, curr.j), Node(curr.i, curr.j + 1), Node(curr.i, curr.j - 1)};


//            while(!open.empty())
//            {
//                std::cout << open.top().H << " ";
//                open.pop();
//            }
//            std::cout << '\n';


        for(auto &s : successors)
        {
            if(CellOnGrid(s.i,s.j))
            {
                if(CellIsObstacle(curr.i,curr.j) || CellIsTraversable(s.i,s.j))
                {
                    if(close.find(s.i * GetWidth() + s.j) == close.end() && openDupl.find(s.i * GetWidth() + s.j) == openDupl.end())
                    {
                        s.H = (this->GetPoint(s) - pos).SquaredEuclideanNorm();
                        open.push(s);
                        openDupl.insert({s.i * GetWidth() + s.j, s});
                    }
                }

            }
        }

    }
    return Node(-1,-1);
}
