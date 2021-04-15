#include "updatable_map.h"

#include <utility>

UpdatableMap::UpdatableMap()
{
    this->origin = {0, 0};
    this->cell_size = 1.0f;
    this->height = 0;
    this->width = 0;
}

UpdatableMap &UpdatableMap::operator=(const UpdatableMap &obj)
{
    if(this != &obj)
    {
        cell_size = obj.cell_size;
        height = obj.height;
        width = obj.width;
        grid = obj.grid;
        obstacle_segments = obj.obstacle_segments;
    }
    return *this;
}


bool UpdatableMap::Update(std::vector<std::vector<bool>> grid, int64_t origin_i, int64_t origin_j, float c_size)
{
    this->grid = std::move(grid);
    this->origin = {origin_i, origin_j};
    this->cell_size = c_size;
    this->height = static_cast<int64_t>(this->grid.size());
    this->width = (height > 0) ? static_cast<int64_t>(this->grid[0].size()) : 0;
}

bool UpdatableMap::CellIsObstacle(int64_t i, int64_t j) const
{
    return grid[i][j] != CN_GC_NOOBS;
}

bool UpdatableMap::CellOnGrid(int64_t i, int64_t j) const
{
    return (i < height) and (i >= 0) and (j < width) and (j >= 0);;
}

bool UpdatableMap::CellIsTraversable(int64_t i, int64_t j) const
{
    return grid[i][j] == CN_GC_NOOBS;
}

int64_t UpdatableMap::GridHeight() const
{
    return height;
}

int64_t UpdatableMap::GridWidth() const
{
    return width;
}

float UpdatableMap::CellSize() const
{
    return cell_size;
}

Node UpdatableMap::CreateNodeForPoint(const Point &point) const
{
    int64_t res_i = origin.first - std::lround(point.Y() / cell_size);
    int64_t res_j = std::lround(point.x / cell_size) + origin.second;
    
    return {static_cast<int>(res_i), static_cast<int>(res_j)};
}

Point UpdatableMap::CenterPosition(int64_t i, int64_t j) const
{
    float x = cell_size * static_cast<float>(origin.first - i);
    float y = cell_size * static_cast<float>(j - origin.second);
    
    return {x, y};
}

std::unordered_map<size_t, ObstacleSegment> &UpdatableMap::GetCloseObstacles(const Point &point, float spacing)
{
    obstacle_segments.clear();
    
    auto next_in_line_ij_hash = [](char side, int64_t ij, int64_t width) -> int64_t
    {
        switch (side)
        {
            case 0:  return  ij + 1;
            case 1:  return  ij - width;
            case 2:  return  ij - 1;
            case 3:  return  ij + width;
            default: return -1;
        }
    };
    
    auto next_diagonal_ij_hash = [](char side, int64_t ij, int64_t width) -> int64_t
    {
        switch (side)
        {
            case 0:  return  ij + width + 1;
            case 1:  return  ij - width + 1;
            case 2:  return  ij - width - 1;
            case 3:  return  ij + width - 1;
            default: return -1;
        }
    };
    
    float max_x, min_x, max_y, min_y;
    max_x = point.x + spacing;
    max_y = point.y + spacing;
    min_x = point.x - spacing;
    min_y = point.y - spacing;
    
    auto bottom_left_cell = CreateNodeForPoint({min_x, min_y});
    auto top_right_cell = CreateNodeForPoint({max_x, max_y});
    
    auto obst_cells = std::unordered_map<int64_t, std::vector<ObstacleSegment>>();
    
    float half_cell = cell_size * 0.5f;
    size_t obst_id = 0;
    for(auto i = top_right_cell.i; i <= bottom_left_cell.i; i++)
    {
        for(auto j = bottom_left_cell.j; j <= top_right_cell.j; j++)
        {
            if (CellIsTraversable(i, j))
            {
                obst_cells.emplace(i * width + j, std::vector<ObstacleSegment>());
                obst_cells[i * width + j].reserve(4);
    
                int64_t n_i, n_j;
                auto cell_center = CenterPosition(i, j);
                auto cell_bl = Point(cell_center.x - half_cell, cell_center.y - half_cell);
                auto cell_br = Point(cell_center.x + half_cell, cell_center.y - half_cell);
                auto cell_tl = Point(cell_center.x - half_cell, cell_center.y + half_cell);
                auto cell_tr = Point(cell_center.x + half_cell, cell_center.y + half_cell);
                
                auto neighbours = {std::pair<int64_t, int64_t>(i + 1, j),
                                   std::pair<int64_t, int64_t>(i, j + 1),
                                   std::pair<int64_t, int64_t>(i - 1, j),
                                   std::pair<int64_t, int64_t>(i, j - 1)};
                
                Point corners[] = {cell_br, cell_tr, cell_tl, cell_bl};
                ObstacleSegment a;
                
                Vertex left, right = Vertex(cell_bl);
                char side = 0;
                for (auto &n_cell : neighbours)
                {
                    n_i = n_cell.first, n_j = n_cell.second;
                    left = right;
                    right = corners[side];
                    
                    if (CellOnGrid(n_i, n_j) and n_i <= bottom_left_cell.i and n_i >= top_right_cell.i and
                        n_j >= bottom_left_cell.j and n_j <= top_right_cell.j)
                    {
                        if (CellIsTraversable(n_i, n_j))
                        {
                            obst_cells[i * width + j].emplace_back(ObstacleSegment(obst_id, left, right));
                            obstacle_segments.emplace(obst_id, ObstacleSegment(obst_id, left, right));
                            obst_id++;
                        }
                        else
                        {
                            obst_cells[i * width + j].emplace_back(ObstacleSegment(-1, Point(), Point()));
                        }
                    }
                    else
                    {
                        obst_cells[i * width + j].emplace_back(ObstacleSegment(obst_id, left, right));
                        obstacle_segments.emplace(obst_id, ObstacleSegment(obst_id, left, right));
                        obst_id++;
                    }
                    side++;
                }
            }
        }
    }
    
    for(auto& [pos_hash, pos_segm] : obst_cells)
    {
        for(char side = 0; side < 4; side++)
        {
            if(pos_segm[side].id != -1)
            {
                int next_side = (side == 3) ? 0 : side + 1;
                int prev_side = (side - 1 > 0) ? side -1 : 3;
                if(pos_segm[next_side].id != -1)
                {
                    obstacle_segments[pos_segm[side].id].next = &obstacle_segments[pos_segm[next_side].id];
                    obstacle_segments[pos_segm[next_side].id].prev = &obstacle_segments[pos_segm[side].id];
                }
                else if(obst_cells[next_in_line_ij_hash(side, pos_hash, width)][side].id != -1)
                {
                    obstacle_segments[pos_segm[side].id].next = &obstacle_segments[obst_cells[next_in_line_ij_hash(side, pos_hash, width)][side].id];
                    obstacle_segments[pos_segm[next_side].id].prev = &obstacle_segments[pos_segm[side].id];
                    
                }
                else if(obst_cells[next_diagonal_ij_hash(side, pos_hash, width)][prev_side].id != -1)
                {
                    obstacle_segments[pos_segm[side].id].next = &obstacle_segments[obst_cells[next_diagonal_ij_hash(side, pos_hash, width)][prev_side].id];
                    obstacle_segments[obst_cells[next_diagonal_ij_hash(side, pos_hash, width)][prev_side].id].prev = &obstacle_segments[pos_segm[side].id];
                }
                else
                {
                    std::cerr << "WRONG! OBSTACLE ERROR\n";
                    exit(EXIT_FAILURE);
                }
                auto left = obstacle_segments[pos_segm[side].id].left;
                auto right = obstacle_segments[pos_segm[side].id].right;
                auto next = obstacle_segments[pos_segm[side].id].next->right;
                bool cvx = (left - next).Det(right - left) >= 0.0;
                
                obstacle_segments[pos_segm[side].id].right.SetConvex(cvx);
                obstacle_segments[pos_segm[side].id].next->left.SetConvex(cvx);
            }
        }
    }
    return obstacle_segments;
}

