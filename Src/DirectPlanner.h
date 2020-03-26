#include "PathPlanner.h"
#include "Geom.h"


#ifndef ORCA_DIRECTPLANNER_H
#define ORCA_DIRECTPLANNER_H


class DirectPlanner : public PathPlanner
{
    public:
        DirectPlanner(const Map &map, const EnvironmentOptions &options, const Point &start, const Point &goal, const float &radius);
        DirectPlanner(const DirectPlanner &obj);
        ~DirectPlanner() override;

        bool GetNext(const Point &curr, Point &next) override;
        bool CreateGlobalPath() override;
        DirectPlanner* Clone() const override;
        DirectPlanner & operator = (const DirectPlanner &obj);
};

#endif //ORCA_DIRECTPLANNER_H
