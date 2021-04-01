#include "../include/DirectPlanner.h"


bool DirectPlanner::GetNext(const Point &curr, Point &next)
{
    next = this->glGoal;
    return true;
}

bool DirectPlanner::CreateGlobalPath()
{
    return true;
}

DirectPlanner::DirectPlanner(const Map &map, const EnvironmentOptions &options, const Point &start, const Point &goal,
                             const float &radius) : PathPlanner(map, options, start, goal, radius) {}

DirectPlanner::DirectPlanner(const DirectPlanner &obj) : PathPlanner(obj) {}

DirectPlanner::~DirectPlanner() {}

DirectPlanner *DirectPlanner::Clone() const
{
    return new DirectPlanner(*this);
}

DirectPlanner &DirectPlanner::operator =(const DirectPlanner &obj)
{
    if(this != &obj)
    {
        PathPlanner::operator =(obj);
    }
    return *this;
}

void DirectPlanner::AddPointToPath(Point p)
{
    glGoal = p;
}

Point DirectPlanner::PullOutNext()
{
    return glGoal;
}

Point DirectPlanner::GetPastPoint()
{
    return glStart;
}

