#include <memory>
#include <list>

#include "Geom.h"
#include "updatable_map.h"
#include "jump_point_search.h"
#include "EnvironmentOptions.h"
#include "LineOfSight.h"

#ifndef ORCASTAR_GLOBAL_PATH_PLANNER_H
#define ORCASTAR_GLOBAL_PATH_PLANNER_H

class GlobalPathPlanner
{
    public:
        virtual ~GlobalPathPlanner() = default;
        virtual bool GetNextTarget(Point curr, Point &next) = 0;
		virtual bool FindPath(Point start, Point goal) = 0;
		virtual GlobalPathPlanner* Clone() = 0;
};

class JPSPlanner : public GlobalPathPlanner
{
	public:
		JPSPlanner();
		JPSPlanner(const JPSPlanner& obj);
		JPSPlanner(std::shared_ptr<UpdatableMap> map_ptr, std::shared_ptr<EnvironmentOptions> options_ptr);
		~JPSPlanner() override = default;

		bool FindPath(Point start, Point goal) override;
		bool GetNextTarget(Point curr, Point &next) override;
		JPSPlanner* Clone() override;
		
	private:
		std::list<Point> path;
		JumpPointSearch search;
		std::shared_ptr<UpdatableMap> map;
		std::shared_ptr<EnvironmentOptions> options;
		Point start;
		Point goal;
		bool path_created;
};


#endif //ORCASTAR_GLOBAL_PATH_PLANNER_H
