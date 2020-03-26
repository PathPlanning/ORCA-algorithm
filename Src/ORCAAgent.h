#include "Agent.h"

#ifndef ORCA_ORCAAGENT_H
#define ORCA_ORCAAGENT_H


class ORCAAgent : public Agent
{

public:
    ORCAAgent();
    ORCAAgent(const int &id, const Point &start, const Point &goal, const Map &map, const EnvironmentOptions &options,
              AgentParam param);
    ORCAAgent(const ORCAAgent &obj);
    ~ORCAAgent();


    ORCAAgent* Clone() const override ;
    void ComputeNewVelocity() override;
    void ApplyNewVelocity() override;
    bool UpdatePrefVelocity() override;

    bool operator == (const ORCAAgent &another) const;
    bool operator != (const ORCAAgent &another) const;
    ORCAAgent &operator = (const ORCAAgent &obj);

private:
    float fakeRadius;
};


#endif //ORCA_ORCAAGENT_H
