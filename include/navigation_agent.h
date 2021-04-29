#include <vector>
#include <memory>

#include "ORCAAgent.h"
#include "EnvironmentOptions.h"
#include "updatable_map.h"
#include "global_path_planner.h"
#include "utils.h"

#ifndef ORCASTAR_NAVIGATION_AGENT_H
#define ORCASTAR_NAVIGATION_AGENT_H

class NavigationAgent : public ORCAAgent
{

public:
    NavigationAgent();
    NavigationAgent(const int &id, const EnvironmentOptions &options, AgentParam param);
    NavigationAgent(const NavigationAgent &obj);
    ~NavigationAgent();
    
    NavigationAgent* Clone() const override;
    bool operator == (const NavigationAgent &another) const;
    bool operator != (const NavigationAgent &another) const;
    NavigationAgent &operator = (const NavigationAgent &obj);

    void ComputeNewVelocity() override;
    void ApplyNewVelocity() override;
    bool UpdatePrefVelocity() override;
    bool UpdateEnvironmentInfo(std::vector<std::vector<bool>> &grid, size_t origin_i, size_t origin_j, float c_size);
    void AddNeighbour(Agent &neighbour, float distSq);
    void UpdateNeighbourObst();
    void UpdateGoal(Point goal);
    
    bool isFinished();
    bool CreateGlobalPath();
    
private:
    std::shared_ptr<UpdatableMap> upd_map;
    std::unordered_map<size_t, ObstacleSegment> obstacles_segments;
    std::unique_ptr<GlobalPathPlanner> global_planner;
    
    

};



#endif //ORCASTAR_NAVIGATION_AGENT_H
