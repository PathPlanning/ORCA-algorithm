#include "Mission.h"


Mission::Mission(std::string fileName, unsigned int agentsNum, unsigned int stepsTh)
{
    taskReader = new XMLReader(fileName);
    agents = vector<Agent*>();
    this->agentsNum = agentsNum;
    stepsTreshhold = stepsTh;

    map = nullptr;
    options = nullptr;
    missionResult = Summary();
    resultsLog = std::unordered_map<int, std::pair<bool, int>>();
    resultsLog.reserve(agentsNum);

#ifndef NDEBUG
    taskLogger = new XMLLogger(XMLLogger::GenerateLogFileName(fileName, agentsNum), fileName);
    stepsLog = std::unordered_map<int, std::vector<Point>>();
    stepsLog.reserve(agentsNum);
#endif

    collisionsCount = 0;
    collisionsObstCount = 0;
    stepsCount = 0;


}


Mission::Mission(const Mission &obj)
{
    agents = obj.agents;
    map = obj.map;
    options = obj.options;
    missionResult = obj.missionResult;
    resultsLog = obj.resultsLog;
    collisionsCount = obj.collisionsCount;
    collisionsObstCount = obj.collisionsObstCount;
    stepsCount = obj.stepsCount;
    taskReader = (obj.taskReader == nullptr) ? nullptr : obj.taskReader->Clone();

#ifndef NDEBUG
    taskLogger = (obj.taskLogger == nullptr) ? nullptr : obj.taskLogger->Clone();
    stepsLog = obj.stepsLog;
#endif

}


Mission::~Mission()
{
    for(auto &agent : agents)
    {
        if(agent != nullptr)
        {
            delete agent;
            agent = nullptr;
        }
    }

    if(map != nullptr)
    {
        delete map;
        map = nullptr;
    }

    if(options != nullptr)
    {
        delete options;
        options = nullptr;
    }

    if(taskReader != nullptr)
    {
        delete taskReader;
        taskReader = nullptr;
    }

#ifndef NDEBUG
    if(taskLogger != nullptr)
    {
        delete taskLogger;
        taskLogger = nullptr;
    }
#endif
}


bool Mission::ReadTask()
{
    return taskReader->ReadData() && taskReader->GetMap(&map) && taskReader->GetAgents(agents, this->agentsNum) && taskReader->GetEnvironmentOptions(&options);
}


Summary Mission::StartMission()
{
    std::cout<<"Start\n";

    auto startpnt = std::chrono::high_resolution_clock::now();
    for(auto agent : agents)
    {
        bool found = agent->InitPath();
        if(!found)
        {
            std::cout<<agent->GetID()<< " "<< "Path not found\n";
        }
        resultsLog.insert({agent->GetID(), {false, 0}});

#ifndef NDEBUG
        stepsLog.insert({agent->GetID(), std::vector<Point>()});
        stepsLog[agent->GetID()].push_back({agent->GetPosition()});
#endif
        agent->UpdatePrefVelocity();
    }

    do
    {
        AssignNeighbours();

        for(auto &agent : agents)
        {
            agent->ComputeNewVelocity();
        }

        UpdateSate();

    }
    while(!IsFinished() && stepsCount < stepsTreshhold);

    auto endpnt = std::chrono::high_resolution_clock::now();
    long long int res = std::chrono::duration_cast<std::chrono::milliseconds>(endpnt - startpnt).count();

    float stepsSum = 0;
    float rate = 0;
    for(auto &node : resultsLog)
    {

        if(!node.second.first)
        {
            node.second.second = stepsCount;
        }
        else
        {
            rate++;
        }
        stepsSum += node.second.second;
    }
    for(auto &agent : agents)
    {
        collisionsCount += agent->GetCollision().first;
        collisionsObstCount += agent->GetCollision().second;
    }

    missionResult.successRate = rate * 100 / agentsNum;
    missionResult.runTime = ((float) res) / 1000;
    missionResult.collisions = collisionsCount / 2;
    missionResult.flowTime = stepsSum * options->timestep;
    missionResult.makeSpan = stepsCount * options->timestep;
    missionResult.collisionsObst = collisionsObstCount;

    std::cout<<"End\n";
    return missionResult;
}


#ifndef NDEBUG
bool Mission::SaveLog()
{
    taskLogger->SetResults(stepsLog, resultsLog);
    taskLogger->SetSummary(missionResult);
    return taskLogger->GenerateLog() && (stepsCount > 0);
}
#endif


void Mission::UpdateSate()
{
    for(auto &agent : agents)
    {
        agent->ApplyNewVelocity();
        Point newPos = agent->GetPosition() + (agent->GetVelocity() * options->timestep);
        agent->SetPosition(newPos);

#ifndef NDEBUG
        stepsLog[agent->GetID()].push_back(newPos);
#endif

        agent->UpdatePrefVelocity();
    }
    stepsCount++;
}


void Mission::AssignNeighbours()
{
    float distSq, rSightSq;
    for(auto &agent : agents)
    {
        rSightSq = agent->GetSightRadius() * agent->GetSightRadius();
        for(auto &another : agents)
        {
            if(agent != another &&
                (distSq = (agent->GetPosition()-another->GetPosition()).SquaredEuclideanNorm()) < rSightSq)
            {
                agent->AddNeighbour(*another, distSq);
            }
        }
        agent->UpdateNeighbourObst();
    }
}


bool Mission::IsFinished()
{
    bool result = true;
    for(auto &agent : agents)
    {
        bool localres = agent->isFinished();
        resultsLog[agent->GetID()].first = agent->isFinished() && resultsLog[agent->GetID()].first;
        if(localres && !resultsLog[agent->GetID()].first)
        {
            resultsLog[agent->GetID()].first = true;
            resultsLog[agent->GetID()].second = stepsCount;
        }
        result = result && localres;
    }

    return result;
}

Mission &Mission::operator = (const Mission &obj)
{
    if(this != &obj)
    {
        stepsCount = obj.stepsCount;
        stepsTreshhold = obj.stepsTreshhold;
        agentsNum = obj.agentsNum;
        collisionsCount = obj.collisionsCount;
        collisionsObstCount = obj.collisionsObstCount;
        taskReader = obj.taskReader;
        missionResult = obj.missionResult;
        resultsLog = obj.resultsLog;

        vector<Agent *> tmpAgents = vector<Agent *>(obj.agents.size());
        for(int i = 0; i < obj.agents.size(); i++)
        {
            tmpAgents.push_back(new Agent(*obj.agents[i]));
        }

        for(auto &agent : agents)
        {
            delete agent;
        }

        agents = tmpAgents;

        if(map != nullptr)
        {
            delete map;
        }
        map = (obj.map == nullptr) ? nullptr : new Map(*obj.map);

        if(options != nullptr)
        {
            delete options;
        }
        options = (obj.options == nullptr) ? nullptr : new EnvironmentOptions(*obj.options);

        if(taskReader != nullptr)
        {
            delete taskReader;
        }
        taskReader = (obj.taskReader == nullptr) ? nullptr : obj.taskReader->Clone();

#ifndef NDEBUG
        if(taskLogger != nullptr)
        {
            delete taskLogger;
        }
        taskLogger = (obj.taskLogger == nullptr) ? nullptr : obj.taskLogger->Clone();
        stepsLog = obj.stepsLog;
#endif

    }

    return *this;
}
