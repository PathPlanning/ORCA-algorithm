#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <cstddef>
#include <iostream>
#include "cassert"

#include "Geom.h"

#ifndef ORCASTAR_UPDATABLE_MAP_H
#define ORCASTAR_UPDATABLE_MAP_H


class UpdatableMap
{
    public:
        UpdatableMap();
        UpdatableMap(const UpdatableMap &obj) = default;
        ~UpdatableMap() = default;
        UpdatableMap & operator = (const UpdatableMap &obj);
        
        bool Update(std::vector<std::vector<bool>> grid, int64_t origin_i, int64_t origin_j, float c_size, int64_t offset);
        bool CellIsObstacle(int64_t i, int64_t j, bool offset = true) const;
        bool CellOnGrid(int64_t i, int64_t j) const;
        bool CellIsTraversable(int64_t i, int64_t j, bool offset = true) const;
        int64_t GridHeight() const;
        int64_t GridWidth() const;
        float CellSize() const;
        std::pair<int64_t, int64_t> FindCellForPoint(const Point &point) const;
        [[deprecated]]
        Node CreateNodeForPoint(const Point &point) const;
        Point CenterPosition(int64_t i, int64_t j) const;
        void GetCloseObstacles(const Point &point, float spacing, std::unordered_map<size_t, ObstacleSegment>& obst_seg);
		bool CheckVisibility(int64_t i1, int64_t j1, int64_t i2, int64_t j2, bool cutcorners, bool offset = true);
        
    private:
        std::pair<int64_t, int64_t> origin;
        float cell_size;
        int64_t height;
        int64_t width;
        int64_t offset;
        std::vector<std::vector<bool>> grid;
        std::vector<std::vector<bool>> offset_grid;
        
//        std::unordered_map<size_t, ObstacleSegment> obstacle_segments;
};


#endif //ORCASTAR_UPDATABLE_MAP_H
