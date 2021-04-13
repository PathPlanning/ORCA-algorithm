#include <string>
#include <vector>
#include <unordered_map>

#include "Geom.h"

#ifndef ORCASTAR_UPDATABLE_MAP_H
#define ORCASTAR_UPDATABLE_MAP_H


class UpdatableMap
{
    public:
        UpdatableMap();
        UpdatableMap(const UpdatableMap &obj);
        ~UpdatableMap();
        
        
        bool Update(std::vector<std::vector<bool>> grid, size_t origin_i, size_t origin_j, float c_size);
        bool CellIsObstacle(int i, int j) const;
        bool CellOnGrid(int i, int j) const;
        bool CellIsTraversable(int i, int j) const;
        unsigned int GetHeight() const;
        unsigned int GetWidth() const;
        float GetCellSize() const;
        
        Node GetClosestNode(const Point &point) const;
        Point GetPoint(const Node &node) const;
        
        std::unordered_map<size_t, ObstacleSegment>& GetCloseObstacles(const Point &point, float spacing);
        
        UpdatableMap & operator = (const UpdatableMap &obj);
    
    private:
        std::pair<size_t, size_t> origin;
        float cell_size;
        size_t height;
        size_t width;
        std::vector<std::vector<bool>> grid;  // TODO Change to smart pointer or not dynamic at all and make it updatable
        std::unordered_map<size_t, ObstacleSegment> obstacle_segments;
};


#endif //ORCASTAR_UPDATABLE_MAP_H
