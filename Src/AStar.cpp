#include "AStar.h"


Astar::Astar(bool WithTime, double HW, bool BT)
{
    hweight = HW;
    breakingties = BT;
    withTime = WithTime;
}

double Astar::computeHFromCellToCell(int i1, int j1, int i2, int j2)
{
    auto it = perfectHeuristic.find(std::make_pair(Node(i1, j1), Node(i2, j2)));
    if (it != perfectHeuristic.end())
    {
        return it->second;
    }
    return metric(i1, j1, i2, j2) * hweight;
}

double Astar::euclideanDistance(int x1, int y1, int x2, int y2)
{
    double res = sqrt(double((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
    return res;
}

double Astar::manhattanDistance(int x1, int y1, int x2, int y2)
{
    return abs(x1 - x2) + abs(y1 - y2);
}

double Astar::chebyshevDistance(int x1, int y1, int x2, int y2)
{
    return std::max(abs(x1 - x2), abs(y1 - y2));
}

double Astar::diagonalDistance(int x1, int y1, int x2, int y2)
{
    return (sqrt(2.0) - 1) * std::min(abs(x1 - x2), abs(y1 - y2)) + std::max(abs(x1 - x2), abs(y1 - y2));
}

double Astar::metric(int x1, int y1, int x2, int y2)
{
    return manhattanDistance(x1, y1, x2, y2);
}
