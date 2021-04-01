#include "../include/Map.h"
#include <iostream>

Map::Map()
{
    cellSize = 1;
    height = 0;
    width = 0;
    grid = nullptr;
    obstacles = nullptr;
}

Map::Map(float cellSize, std::vector<std::vector<int>> &grid, std::vector<std::vector<Point>> &obstacles)
{
    this->cellSize = cellSize;
    this->grid = new std::vector<std::vector<int>>(grid);
    this->obstacles = new std::vector<std::vector<ObstacleSegment>>();

    height = this->grid->size();
    width = 0;
    if(height)
    {
        width = (*this->grid)[0].size();
    }

    int idCounter = 0;

    for(auto obstacle : obstacles)
    {
        if(obstacle.size() < 2)
        {
            continue;
        }

        bool rCvx = true, lCvx;
        std::vector<ObstacleSegment> tmpObstacle;
        for(int i = 0; i < obstacle.size(); i++)
        {
            Vertex left = obstacle[(i == 0 ? obstacle.size() - 1  : i - 1)];
            Vertex right = obstacle[i];
            Vertex next = obstacle[(i == obstacle.size() - 1 ? 0 : i + 1)];
            
            lCvx = rCvx;
            rCvx = true;
            if (obstacle.size() > 2)
            {
                rCvx = (left-next).Det(right - left) >= 0.0f;
            }
            right.SetConvex(rCvx);
            left.SetConvex(lCvx);

            tmpObstacle.emplace_back(ObstacleSegment(idCounter, left, right));
            idCounter++;
        }
        tmpObstacle[0].left.SetConvex(rCvx);
        this->obstacles->push_back(tmpObstacle);
        tmpObstacle.clear();
    }

    for(auto &obstacle : (*this->obstacles))
    {
        for(int i = 0; i < obstacle.size(); i++)
        {
            obstacle[i].next = (i == obstacle.size()-1) ? &obstacle.at(0) : &obstacle.at(i+1);
            obstacle[i].prev = (i == 0) ? &obstacle.at(obstacle.size()-1) : &obstacle.at(i-1);
        }
    }
}

Map::Map(const Map &obj)
{
    cellSize = obj.cellSize;
    grid = (obj.grid == nullptr) ? nullptr : new std::vector<std::vector<int>>(*obj.grid);
    obstacles = (obj.obstacles == nullptr) ? nullptr : new std::vector<std::vector<ObstacleSegment>>(*obj.obstacles);
    height = obj.height;
    width = obj.width;
}

Map::~Map()
{
    if(grid != nullptr)
    {
        delete grid;
        grid = nullptr;
    }

    if(obstacles != nullptr)
    {
        delete obstacles;
        obstacles = nullptr;
    }
}

bool Map::CellIsObstacle(int i, int j) const
{
    return ((*grid)[i][j] != CN_GC_NOOBS);
}

bool Map::CellIsTraversable(int i, int j) const
{
    return ((*grid)[i][j] == CN_GC_NOOBS);
}

bool Map::CellOnGrid(int i, int j) const
{
    return (i < height && i >= 0 && j < width && j >= 0);
}

unsigned int Map::GetHeight() const
{
    return height;
}

unsigned int Map::GetWidth() const
{
    return width;
}

float Map::GetCellSize() const
{
    return cellSize;
}

Node Map::GetClosestNode(const Point &point) const
{
    Node res;
    res.i = height - 1 - (int) (point.Y() / cellSize);
    res.j = (int) (point.X() / cellSize);

    if(res.i < 0)
    {
        res.i = 0;
    }
    if(res.i > height - 1)
    {
        res.i = height - 1;
    }
    if(res.j < 0)
    {
        res.j = 0;
    }
    if(res.j > width - 1)
    {
        res.j = width - 1;
    }

    return res;
}

Point Map::GetPoint(const Node &node) const
{
    return {(node.j * cellSize + cellSize/2), (height - 1 - node.i) * cellSize + cellSize/2};
}

const std::vector<std::vector<ObstacleSegment>> &Map::GetObstacles() const
{
    return *obstacles;
}

Map &Map::operator =(const Map &obj)
{
    if(this != &obj)
    {
        cellSize = obj.cellSize;
        height = obj.height;
        width = obj.width;

        if(grid != nullptr)
        {
            delete grid;
        }
        grid = (obj.grid == nullptr) ? nullptr : new std::vector<std::vector<int>>(*obj.grid);

        if(obstacles != nullptr)
        {
            delete obstacles;
        }
        obstacles = (obj.obstacles == nullptr) ? nullptr : new std::vector<std::vector<ObstacleSegment>>(*obj.obstacles);
    }
    return *this;
}

std::vector<ObstacleSegment> Map::GetCloseObstacles(const Point &point, float spacing) const
{
    std::vector<ObstacleSegment> obstacles;
    
    float max_x, min_x, max_y, min_y;
    max_x = point.x + spacing;
    max_y = point.y + spacing;
    min_x = point.x - spacing;
    min_y = point.y - spacing;
    
    auto bottom_left_cell = GetClosestNode({min_x, min_y});
    auto top_right_cell = GetClosestNode({max_x, max_y});
    
    auto obst_cells = std::unordered_map<int, std::vector<ObstacleSegment>>();
    
    auto half_cell = cellSize * 0.5;
    size_t obst_id = 0;
    for(auto i = top_right_cell.i; i <= bottom_left_cell.i; i++)
    {
        for(auto j = bottom_left_cell.j; j <= top_right_cell.j; j++)
        {
            if((*grid)[i][j])
            {
                obst_cells.emplace(i * width + j, std::vector<ObstacleSegment>());
                
                int n_i, n_j;
                auto cell_center = GetPoint({i, j});
                auto cell_bl = Point(cell_center.x - half_cell, cell_center.y - half_cell);
                auto cell_br = Point(cell_center.x + half_cell, cell_center.y - half_cell);
                auto cell_tl = Point(cell_center.x - half_cell, cell_center.y + half_cell);
                auto cell_tr = Point(cell_center.x + half_cell, cell_center.y + half_cell);
                

                auto neighbours = {std::pair<int, int>(i+1, j),
                                   std::pair<int, int>(i, j+1),
                                   std::pair<int, int>(i-1, j),
                                   std::pair<int, int>(i, j-1)};
    
                Point corners[] = {cell_br, cell_tr, cell_tl, cell_bl};
                ObstacleSegment a;
                size_t side = 0;
                Vertex left, right = Vertex(cell_bl);
                for(auto& n_cell : neighbours)
                {
                    n_i = n_cell.first, n_j = n_cell.second;
                    
                    left = right;
                    right = corners[side];
                    
                    if(CellOnGrid(n_i, n_j) and n_i <= bottom_left_cell.i and n_i >= top_right_cell.i and
                       n_j >= bottom_left_cell.j and n_j <= top_right_cell.j)
                    {
                        obst_cells[i * width + j].emplace_back(ObstacleSegment(obst_id, left, right));
                        obst_id ++;
                    }
                    else
                    {
                        obst_cells[i * width + j].emplace_back(ObstacleSegment(-1, Point(), Point()));
                    }
                    side++;
                }
            }
    

            for(auto& [pos_hash, pos_segm] : obst_cells)
            {
                for(size_t side = 0; side < 4; side++)
                {
                    if(pos_segm[side].id != -1)
                    {
                        size_t next_side = (side == 3) ? 0 : side + 1;
                        if(pos_segm[next_side].id != -1)
                        {
                            // TODO obst_cells as Map class field
                            pos_segm[side].next = &pos_segm[next_side]
                        }
                        else if( /* TODO other cases*/)
                        {
                        
                        }
                        
                    }
                    
                    obstacles.
                }
                
            }
//            Vertex left = obstacle[(i == 0 ? obstacle.size() - 1  : i - 1)];
//            Vertex right = obstacle[i];
//            Vertex next = obstacle[(i == obstacle.size() - 1 ? 0 : i + 1)];
//            lCvx = rCvx;
//            rCvx = true;
//            if (obstacle.size() > 2)
//            {
//                rCvx = (left-next).Det(right - left) >= 0.0f;
//            }
//            right.SetConvex(rCvx);
//            left.SetConvex(lCvx);
//
//            tmpObstacle.emplace_back(ObstacleSegment(idCounter, left, right));
        }
    }
    return std::vector<ObstacleSegment>();
}









