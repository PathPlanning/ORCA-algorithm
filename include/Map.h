#include <string>
#include <vector>
#include <unordered_map>

#include "Geom.h"


#ifndef ORCA_MAP_H
#define ORCA_MAP_H



class Map
{
    public:
        Map();
        Map(float cellSize, std::vector<std::vector<int>> &grid, std::vector<std::vector<Point>> &obstacles);
        Map(const Map &obj);
        ~Map();

        bool CellIsObstacle(int i, int j) const;
        bool CellOnGrid(int i, int j) const;
        bool CellIsTraversable(int i, int j) const;
        unsigned int GetHeight() const;
        unsigned int GetWidth() const;
        float GetCellSize() const;

        Node GetClosestNode(const Point &point) const;
        Point GetPoint(const Node &node) const;
    
        std::unordered_map<size_t, ObstacleSegment>& GetCloseObstacles(const Point &point, float spacing);

        [[deprecated]] const std::vector<std::vector<ObstacleSegment>> & GetObstacles() const;

        Map & operator = (const Map &obj);

    private:
        float cellSize;
        unsigned int height;
        unsigned int width;
        std::vector<std::vector<int>> *grid;  // TODO Change to smart pointer or not dynamic at all and make it updatable
        
        std::vector<std::vector<ObstacleSegment>> *obstacles; // TODO Delete
        
        std::unordered_map<size_t, ObstacleSegment> obstacle_segments;
};


#endif //ORCA_MAP_H
