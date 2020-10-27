#include "Mission.h"


Mission::Mission(std::string fileName, unsigned int agentsNum, unsigned int stepsTh, bool time, size_t timeTh)
{
    taskReader = new XMLReader(fileName);
    agents = vector<Agent*>();
    this->agentsNum = agentsNum;
    stepsTreshhold = stepsTh;
    isTimeBounded = time;
    timeTreshhold = timeTh;

    map = nullptr;
    options = nullptr;
    missionResult = Summary();
    resultsLog = std::unordered_map<int, std::pair<bool, int>>();
    resultsLog.reserve(agentsNum);

#if FULL_LOG
    taskLogger = new XMLLogger(XMLLogger::GenerateLogFileName(fileName, agentsNum), fileName);
    stepsLog = std::unordered_map<int, std::vector<Point>>();
    stepsLog.reserve(agentsNum);

    goalsLog = std::unordered_map<int, std::vector<Point>>();
    goalsLog.reserve(agentsNum);
#endif

    collisionsCount = 0;
    collisionsObstCount = 0;
    stepsCount = 0;

#if PAR_LOG
    string tmpPAR = fileName.erase(fileName.find_last_of("."));
    PARLog = MAPFInstancesLogger(tmpPAR);
#endif

}


Mission::Mission(const Mission &obj)
{
#if PAR_LOG
    PARLog = obj.PARLog;
#endif
    agents = obj.agents;
    map = obj.map;
    options = obj.options;
    missionResult = obj.missionResult;
    resultsLog = obj.resultsLog;
    collisionsCount = obj.collisionsCount;
    collisionsObstCount = obj.collisionsObstCount;
    stepsCount = obj.stepsCount;
    taskReader = (obj.taskReader == nullptr) ? nullptr : obj.taskReader->Clone();


#if FULL_LOG
    taskLogger = (obj.taskLogger == nullptr) ? nullptr : obj.taskLogger->Clone();
    stepsLog = obj.stepsLog;
    goalsLog = obj.goalsLog;

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

#if FULL_LOG
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
#if FULL_OUTPUT
    std::cout<<"Start\n";
#endif

    auto startpnt = std::chrono::high_resolution_clock::now();
    for(auto agent : agents)
    {
#if PAR_LOG
        dynamic_cast<ORCAAgentWithECBS*>(agent)->SetMAPFInstanceLoggerRef(&PARLog);
#endif
        bool found = agent->InitPath();
#if FULL_OUTPUT
        if(!found)
        {
            std::cout<<agent->GetID()<< " "<< "Path not found\n";
        }
#endif
        resultsLog.insert({agent->GetID(), {false, 0}});

#if FULL_LOG
        stepsLog.insert({agent->GetID(), std::vector<Point>()});
        stepsLog[agent->GetID()].push_back({agent->GetPosition()});
        goalsLog[agent->GetID()].push_back(agent->GetPosition());
#endif

    }
    bool needToStop;
    do
    {
        AssignNeighbours();

        for(auto &agent : agents)
        {
            agent->UpdatePrefVelocity();
        }

        for(auto &agent : agents)
        {

            agent->ComputeNewVelocity();

        }

        UpdateSate();
        auto checkpnt = std::chrono::high_resolution_clock::now();
        size_t nowtime = std::chrono::duration_cast<std::chrono::milliseconds>(checkpnt - startpnt).count();

        needToStop = (isTimeBounded) ? nowtime >= timeTreshhold : stepsCount >= stepsTreshhold;

        //std::cout << std::endl;
    }
    while(!IsFinished() && !needToStop);

    auto endpnt = std::chrono::high_resolution_clock::now();
    size_t res = std::chrono::duration_cast<std::chrono::milliseconds>(endpnt - startpnt).count();

    float stepsSum = 0;
    float rate = 0;
    float meanMAPFTime = 0.0;
    int initCount = 0, uniCount = 0, updCount = 0, ECBSCount = 0, PARCount = 0;

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
        auto tmpPARAgent = dynamic_cast<ORCAAgentWithPAR*> (agent);
        if(tmpPARAgent != nullptr)
        {
            auto statMAPF = tmpPARAgent->GetMAPFStatistics();
            meanMAPFTime += statMAPF[CNS_MAPF_COMMON_TIME];
            initCount += static_cast<int>(statMAPF[CNS_MAPF_INIT_COUNT]);
            uniCount += static_cast<int>(statMAPF[CNS_MAPF_UNITE_COUNT]);
            updCount += static_cast<int>(statMAPF[CNS_MAPF_UPDATE_COUNT]);
        }
        else
        {
            auto tmpPARnECBSAgent = dynamic_cast<ORCAAgentWithPARAndECBS*> (agent);
            if(tmpPARnECBSAgent != nullptr)
            {
                auto statMAPF = tmpPARnECBSAgent->GetMAPFStatistics();
                meanMAPFTime += statMAPF[CNS_MAPF_COMMON_TIME];
                initCount += static_cast<int>(statMAPF[CNS_MAPF_INIT_COUNT]);
                uniCount += static_cast<int>(statMAPF[CNS_MAPF_UNITE_COUNT]);
                updCount += static_cast<int>(statMAPF[CNS_MAPF_UPDATE_COUNT]);
                ECBSCount += static_cast<int>(statMAPF[CNS_MAPF_ECBS_COUNT]);
                PARCount += static_cast<int>(statMAPF[CNS_MAPF_PAR_COUNT]);
            }
        }
    }

    meanMAPFTime = (initCount + uniCount + updCount == 0) ? 0.0 : meanMAPFTime / (initCount + uniCount + updCount);

    missionResult[CNS_SUM_SUCCESS_RATE] = std::to_string(rate * 100 / agentsNum);
    missionResult[CNS_SUM_RUN_TIME] = std::to_string(((float) res) / 1000);
    missionResult[CNS_SUM_COLLISIONS] = std::to_string(collisionsCount / 2);
    missionResult[CNS_SUM_FLOW_TIME] = std::to_string(stepsSum * options->timestep);
    missionResult[CNS_SUM_MAKESPAN] = std::to_string(stepsCount * options->timestep);
    missionResult[CNS_SUM_COLLISIONS_OBS] = std::to_string(collisionsObstCount);

    missionResult[CNS_SUM_MAPF_MEAN_TIME] = std::to_string(meanMAPFTime);
    missionResult[CNS_SUM_MAPF_INIT_COUNT] = std::to_string(initCount);
    missionResult[CNS_SUM_MAPF_UNITE_COUNT] = std::to_string(uniCount);
    missionResult[CNS_SUM_MAPF_UPDATE_COUNT] = std::to_string(updCount);

    missionResult[CNS_SUM_MAPF_ECBS_COUNT] = std::to_string(ECBSCount);
    missionResult[CNS_SUM_MAPF_PAR_COUNT] = std::to_string(PARCount);
#if FULL_OUTPUT
    std::cout<<"End\n";
#endif
    return missionResult;
}


#if FULL_LOG
bool Mission::SaveLog()
{
    taskLogger->SetResults(stepsLog, goalsLog, resultsLog);
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

#if FULL_LOG
        stepsLog[agent->GetID()].push_back(newPos);
        goalsLog[agent->GetID()].push_back(agent->GetNext());
#endif


    }
    stepsCount++;
//    if(stepsCount % 100 == 0)
//    {
//        cout<<stepsCount<<"\n";
//    }
}


void Mission::AssignNeighbours()
{
    for(auto &agent : agents)
    {

        for(auto &neighbour : agents)
        {
            if(agent != neighbour)
            {
                float distSq = (agent->GetPosition() - neighbour->GetPosition()).SquaredEuclideanNorm();
                agent->AddNeighbour(*neighbour, distSq);
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
#if PAR_LOG
        PARLog = obj.PARLog;
#endif

        vector<Agent *> tmpAgents = vector<Agent *>(obj.agents.size());
        for(int i = 0; i < obj.agents.size(); i++)
        {
            tmpAgents.push_back(obj.agents[i]->Clone());
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

#if FULL_LOG
        if(taskLogger != nullptr)
        {
            delete taskLogger;
        }
        taskLogger = (obj.taskLogger == nullptr) ? nullptr : obj.taskLogger->Clone();
        stepsLog = obj.stepsLog;
        goalsLog = obj.goalsLog;
#endif

    }
    return *this;
}
