#include "Agent.h"
#include <random>

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


    ORCAAgent* clone() const override ;
    void computeNewControl() override;
    void applyNewControl() override;
    bool prepareBeforeStep() override;

    bool operator == (const ORCAAgent &another) const;
    bool operator != (const ORCAAgent &another) const;
    ORCAAgent &operator = (const ORCAAgent &obj);

private:
    float fakeRadius;
};


#endif //ORCA_ORCAAGENT_H
