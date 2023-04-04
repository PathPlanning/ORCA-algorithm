#include "Mission.h"


Mission::Mission(std::string fileName, unsigned int agentsNum, unsigned int stepsTh, bool time, size_t timeTh, bool speedStop)
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
    auto found = file.find_last_of(".");
    string tmpPAR = file.erase(found);
    std::string piece = "_" + std::to_string(agentsNum);
    tmpPAR.insert(found, piece);

    PARLog = MAPFInstancesLogger(tmpPAR);
#endif

    commonSpeedsBuffer = std::vector<std::list<float>>(agentsNum, std::list<float>(1000, 1.0));
    allStops = false;
    stopByMeanSpeed = speedStop;
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
    commonSpeedsBuffer = obj.commonSpeedsBuffer;
    allStops = obj.allStops;
    stopByMeanSpeed = obj.stopByMeanSpeed;
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
    return taskReader->readData() && taskReader->getMap(&map) && taskReader->getAgents(agents, this->agentsNum) &&
		   taskReader->getEnvironmentOptions(&options);
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
        dynamic_cast<ORCAAgentWithPARAndECBS*>(agent)->SetMAPFInstanceLoggerRef(&PARLog);
#endif
        bool found = agent->initAgent();
#if FULL_OUTPUT
        if(!found)
        {
            std::cout<<agent->GetID()<< " "<< "Path not found\n";
        }
#endif
        resultsLog.insert({agent->getID(), {false, 0}});

#if FULL_LOG
        stepsLog.insert({agent->getID(), std::vector<Point>()});
        stepsLog[agent->getID()].push_back({agent->getPosition()});
        goalsLog[agent->getID()].push_back(agent->getPosition());
#endif

    }
    bool needToStop, needToStopByTime, needToStopBySteps, needToStopBySpeed;
    do
    {
        AssignNeighbours();
        
        for(auto &agent : agents)
        {
			agent->prepareBeforeStep();
        }
        
        for(auto &agent : agents)
        {
			agent->computeNewControl();
        }
        
        UpdateSate();
        auto checkpnt = std::chrono::high_resolution_clock::now();
        size_t nowtime = std::chrono::duration_cast<std::chrono::milliseconds>(checkpnt - startpnt).count();

        needToStopBySpeed = (stopByMeanSpeed and allStops);
        needToStopByTime = (isTimeBounded and nowtime >= timeTreshhold);
        needToStopBySteps = (!isTimeBounded and stepsCount >= stepsTreshhold);
        needToStop = needToStopBySpeed or needToStopByTime or needToStopBySteps;

    }
    while(!IsFinished() && !needToStop);

    auto endpnt = std::chrono::high_resolution_clock::now();
    size_t res = std::chrono::duration_cast<std::chrono::milliseconds>(endpnt - startpnt).count();

    float stepsSum = 0;
    float rate = 0;
    float MAPFTime = 0.0;
    int initCount = 0, uniCount = 0, updCount = 0, ECBSCount = 0, PARCount = 0, successCount = 0, unsuccessCount = 0, flowtimeMAPF = 0;

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
        collisionsCount += agent->getCollisionCount().first;
        collisionsObstCount += agent->getCollisionCount().second;

		auto tmpAMAPFAgent = dynamic_cast<AgentGoalExchange*>(agent);
		if (tmpAMAPFAgent != nullptr) {
			size_t agent_messages_count, agent_exchange_count;
			std::tie(agent_messages_count, agent_exchange_count) = tmpAMAPFAgent->getAMAPFStat();
			messages_count += agent_messages_count;
			exchange_count += agent_exchange_count;
		}

        auto tmpPARAgent = dynamic_cast<ORCAAgentWithPAR*> (agent);
        if(tmpPARAgent != nullptr)
        {
            auto statMAPF = tmpPARAgent->GetMAPFStatistics();
            MAPFTime += statMAPF[CNS_MAPF_COMMON_TIME];
            initCount += static_cast<int>(statMAPF[CNS_MAPF_INIT_COUNT]);
            uniCount += static_cast<int>(statMAPF[CNS_MAPF_UNITE_COUNT]);
            updCount += static_cast<int>(statMAPF[CNS_MAPF_UPDATE_COUNT]);
            successCount += static_cast<int>(statMAPF[CNS_MAPF_SUCCESS_COUNT]);
            unsuccessCount += static_cast<int>(statMAPF[CNS_MAPF_UNSUCCESS_COUNT]);
            flowtimeMAPF += static_cast<int>(statMAPF[CNS_MAPF_FLOWTIME]);
        }
        else
        {
            auto tmpPARnECBSAgent = dynamic_cast<ORCAAgentWithPARAndECBS*> (agent);
            if(tmpPARnECBSAgent != nullptr)
            {
                //tmpPARnECBSAgent->PrintMAPFMemberStat();
                auto statMAPF = tmpPARnECBSAgent->GetMAPFStatistics();
                MAPFTime += statMAPF[CNS_MAPF_COMMON_TIME];
                initCount += static_cast<int>(statMAPF[CNS_MAPF_INIT_COUNT]);
                uniCount += static_cast<int>(statMAPF[CNS_MAPF_UNITE_COUNT]);
                updCount += static_cast<int>(statMAPF[CNS_MAPF_UPDATE_COUNT]);
                ECBSCount += static_cast<int>(statMAPF[CNS_MAPF_ECBS_COUNT]);
                PARCount += static_cast<int>(statMAPF[CNS_MAPF_PAR_COUNT]);
                successCount += static_cast<int>(statMAPF[CNS_MAPF_SUCCESS_COUNT]);
                unsuccessCount += static_cast<int>(statMAPF[CNS_MAPF_UNSUCCESS_COUNT]);
                flowtimeMAPF += static_cast<int>(statMAPF[CNS_MAPF_FLOWTIME]);
            }
        }
    }



    missionResult[CNS_SUM_SUCCESS_RATE] = std::to_string(rate * 100 / agentsNum);
    missionResult[CNS_SUM_RUN_TIME] = std::to_string(((float) res) / 1000);
    missionResult[CNS_SUM_COLLISIONS] = std::to_string(collisionsCount / 2);
    missionResult[CNS_SUM_FLOW_TIME] = std::to_string(stepsSum * options->timestep);
    missionResult[CNS_SUM_MAKESPAN] = std::to_string(stepsCount * options->timestep);
    missionResult[CNS_SUM_COLLISIONS_OBS] = std::to_string(collisionsObstCount);

//    missionResult[CNS_SUM_MAPF_MEAN_TIME] = std::to_string(MAPFTime);
//    missionResult[CNS_SUM_MAPF_INIT_COUNT] = std::to_string(initCount);
//    missionResult[CNS_SUM_MAPF_UNITE_COUNT] = std::to_string(uniCount);
//    missionResult[CNS_SUM_MAPF_UPDATE_COUNT] = std::to_string(updCount);
//
//    missionResult[CNS_SUM_MAPF_ECBS_COUNT] = std::to_string(ECBSCount);
//    missionResult[CNS_SUM_MAPF_PAR_COUNT] = std::to_string(PARCount);
//
//    missionResult[CNS_SUM_MAPF_FLOWTIME] = std::to_string(flowtimeMAPF);
//    missionResult[CNS_SUM_MAPF_SUCCESS_COUNT] = std::to_string(successCount);
//    missionResult[CNS_SUM_MAPF_UNSUCCESS_COUNT] = std::to_string(unsuccessCount);

	missionResult[CNS_AMAPF_MESSAGES] = std::to_string(messages_count);
	missionResult[CNS_AMAPF_EXCHANGE] = std::to_string(exchange_count);



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
    size_t i = 0;
    allStops = true;

    for(auto &agent : agents)
    {
		agent->applyNewControl();
        Point newPos = agent->getPosition() + (agent->getVelocity() * options->timestep);
		agent->updatePosition(newPos);
        commonSpeedsBuffer[i].pop_front();
        commonSpeedsBuffer[i].push_back(agent->getVelocity().EuclideanNorm());

        float sum = 0.0f;
        float c = 0.0f;
        float y, t;
        float mean;
        for(auto speed : commonSpeedsBuffer[i])
        {
            y = speed - c;
            t = sum + y;
            c = (t - sum) - y;
            sum = t;
        }
        mean = sum / commonSpeedsBuffer[i].size();

        if (mean >= 0.0001)
        {
            allStops = false;
        }

#if FULL_LOG
        stepsLog[agent->getID()].push_back(newPos);
        goalsLog[agent->getID()].push_back(agent->getNextGoalPoint());
#endif
        i++;
    }

    stepsCount++;
}


void Mission::AssignNeighbours()
{
    for(auto &agent : agents)
    {

        for(auto &neighbour : agents)
        {
            if(agent != neighbour)
            {
                float distSq = (agent->getPosition() - neighbour->getPosition()).SquaredEuclideanNorm();
				agent->addNeighbour(*neighbour, distSq);
            }
        }
		agent->updateObstacleInformation();
    }
}


bool Mission::IsFinished()
{
    bool result = true;
    for(auto &agent : agents)
    {
        bool localres = agent->isFinished();
        resultsLog[agent->getID()].first = localres && resultsLog[agent->getID()].first;
        if(localres && !resultsLog[agent->getID()].first)
        {
            resultsLog[agent->getID()].first = true;
            resultsLog[agent->getID()].second = stepsCount;
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
            tmpAgents.push_back(obj.agents[i]->clone());
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
        commonSpeedsBuffer = obj.commonSpeedsBuffer;
        allStops = obj.allStops;
        stopByMeanSpeed = obj.stopByMeanSpeed;
    }
    return *this;
}
