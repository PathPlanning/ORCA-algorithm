#include "navigation_agent.h"


NavigationAgent::NavigationAgent()
{
    //TODO
}


NavigationAgent::NavigationAgent(const int &id, const EnvironmentOptions &options, AgentParam param)
{
    //TODO
}

NavigationAgent::NavigationAgent(const NavigationAgent &obj)
{
    //TODO
}

NavigationAgent::~NavigationAgent()
{
    //TODO
}

NavigationAgent *NavigationAgent::Clone() const
{
    //TODO
    return nullptr;
}

bool NavigationAgent::operator==(const NavigationAgent &another) const
{
    //TODO
    return false;
}

bool NavigationAgent::operator!=(const NavigationAgent &another) const
{
    //TODO
    return false;
}

NavigationAgent &NavigationAgent::operator=(const NavigationAgent &obj)
{
    //TODO
    return *this->Clone();
}

void NavigationAgent::ComputeNewVelocity()
{
    //TODO
}

void NavigationAgent::ApplyNewVelocity()
{
    //TODO
}

bool NavigationAgent::UpdatePrefVelocity()
{
    //TODO
}

bool NavigationAgent::UpdateEnvironmentInfo(std::vector<std::vector<bool>> &grid, size_t origin_i, size_t origin_j,
                                            float c_size, Point position, Point target)
{
    //TODO
    return false;
}

void NavigationAgent::AddNeighbour(Agent &neighbour, float distSq)
{
    // TODO
}

void NavigationAgent::UpdateNeighbourObst()
{
    // TODO
}

bool NavigationAgent::isFinished()
{
    // TODO
    return false;
}

bool NavigationAgent::InitPath()
{
    // TODO
    return false;
}















