#include "Mission.h"


Mission::Mission(string taskFile, string commLogFile, string logFile, int agentsTreshhold, bool *fileOpened)
{
    this->logFile = logFile;
    float maxlength = 0;
    this->agNumTreshhold = agentsTreshhold;
    this->fileName = taskFile;
    commonLog.open(commLogFile, ios_base::app);

    if(!ReadMissionFromFile())
    {
        *fileOpened = false;
        return;
    }
    vector<pair<float, float>> starts, goals;
    ofstream pre_log("aaa.txt", ios_base::app);
    double sum = 0;
    for(auto &agent : agents)
    {
        auto tmpstart = agent.first.GetPosition().GetPair();
        auto tmpgoal = agent.second.GetPair();
        starts.push_back(tmpstart);
        goals.push_back(tmpgoal);
        float tmplength = sqrt((tmpstart.first - tmpgoal.first) * (tmpstart.first - tmpgoal.first) + (tmpstart.second - tmpgoal.second) * (tmpstart.second - tmpgoal.second));
        sum += tmplength;
        if(tmplength > maxlength)
        {
            maxlength = tmplength;
        }
    }
    pre_log<<sum<<"\t"<<maxlength<<"\n";
    #ifndef NDEBUG
        log = new XmlLogger(agentNumber, defaultRadius, defaultMaxSpeed, defaultAgentsMaxNum, defaultTimeBoundary, defaultSightRadius, starts, goals);
        log->WriteAlgorithmParam(timeStep, delta);
        stepsLog = vector<vector<pair<float, float>>> (agentNumber);
    #endif

    step = 0;
    results = vector<pair<bool, int>>(agentNumber);
    stepsTreshhold = 100 * (int)round(maxlength/(defaultMaxSpeed * timeStep));
    collision = 0;
    *fileOpened = true;
    pre_log.close();
}

bool Mission::isFinished()
{
    bool result = true;
    int i = 0;
    for(auto &agent : agents)
    {
        bool localres = ((agent.first.GetPosition() - agent.second).EuclideanNorm() < delta);
        results[i].first = localres && results[i].first;
        if(localres && !results[i].first)
        {
            results[i].first = true;
            results[i].second = step;
        }
        result = result && localres;
        i++;
    }

    return result;
}

Mission::~Mission()
{
    //delete log;
}

void Mission::StartMission()
{
    std::cout<<"Start\n";
    auto startpnt = std::chrono::high_resolution_clock::now();
    do
    {

        for(auto &agent : agents)
        {

            Vector goalVector = agent.second - agent.first.GetPosition();
            if(goalVector.SquaredEuclideanNorm() > 1.0f)
            {
                goalVector = (goalVector/goalVector.EuclideanNorm()) * agent.first.GetMaxSpeed();
            }

            agent.first.SetPrefVelocity(goalVector);

            for(auto &nagent : agents)
            {
                if(agent != nagent)
                    agent.first.AddNeighbour(nagent.first);
            }
        }

        for(auto &agent : agents)
        {
            agent.first.CalculateVelocity(&collision);

        }

        int i = 0;
        for(auto &agent : agents)
        {

            agent.first.UpdateVelocity();
            Point tmppos = agent.first.GetPosition() + (agent.first.GetVelocity() * timeStep);
            agent.first.SetPosition(tmppos);

            #ifndef NDEBUG
                stepsLog[i].push_back({agent.first.GetPosition().GetX(), agent.first.GetPosition().GetY()});
            #endif

            i++;
        }
        step++;
    }
    while(!isFinished() && step < stepsTreshhold);

    auto endpnt = std::chrono::high_resolution_clock::now();
    long long int res = std::chrono::duration_cast<std::chrono::milliseconds>(endpnt - startpnt).count();
    double summ = 0;
    double rate = 0;
    for(auto &node : results)
    {
        summ += node.second;
        if(!node.first)
        {
            node.second = step;
        }
        else
        {
            rate++;
        }

    }
    rate = rate * 100 / results.size();
    commonLog<<rate<<"\t";
    commonLog<< ((float) res) / 1000 <<"\t";
    commonLog<<summ<<"\t";
    commonLog<<step <<"\t";
    commonLog<<collision/2<<"\n";
    commonLog.close();

    #ifndef NDEBUG
        log->Save(stepsLog, results, ((float) res) / 1000, logFile);
    #endif

    std::cout<<"Succsess\n";
}

bool Mission::ReadMissionFromFile()
{
    float stx, sty, gx, gy;
    int id, counter = 0;
    XMLDocument doc;
    if(doc.LoadFile(fileName.c_str()))
    {
        std::cout << "File opening error\n";
        return false;
    }

    XMLElement * root = doc.FirstChildElement();
    if (root == nullptr)
    {
        std::cout << "Root not found\n";
        return false;
    }

    // Чтение параметров по умолчанию для агентов
    XMLElement *tmpElement = root->FirstChildElement("default_parameters");
    if (tmpElement == nullptr)
    {
        std::cout << "Default parameters not found\n";
        return false;
    }

    tmpElement->QueryFloatAttribute("size", &defaultRadius);
    tmpElement->QueryFloatAttribute("movespeed", &defaultMaxSpeed);
    tmpElement->QueryIntAttribute("agentsmaxnum", &defaultAgentsMaxNum);
    tmpElement->QueryFloatAttribute("timeboundary", &defaultTimeBoundary);
    tmpElement->QueryFloatAttribute("sightradius", &defaultSightRadius);

    tmpElement = root->FirstChildElement("algorithm");
    if (tmpElement == nullptr)
    {
        std::cout << "Algorithm parameters not found\n";
        return false;
    }
    tmpElement->QueryFloatAttribute("timestep", &timeStep);
    tmpElement->QueryFloatAttribute("delta", &delta);

    // Чтение информации об агентах
    //TODO не дефолтные параметры агентов
    tmpElement = root->FirstChildElement("agents");
    if (tmpElement == nullptr)
    {
        std::cout << "Agents parameters not found\n";
        return false;
    }



    tmpElement->QueryIntAttribute("number", &agentNumber);
    if(agentNumber > agNumTreshhold)
    {
        agentNumber = agNumTreshhold;
    }

    for(auto e = tmpElement->FirstChildElement("agent"); e != NULL && counter < agentNumber; e = e->NextSiblingElement("agent"), counter++)
    {
        e->QueryIntAttribute("id", &id);
        e->QueryFloatAttribute("start.x", &stx);
        e->QueryFloatAttribute("start.y", &sty);
        e->QueryFloatAttribute("goal.x", &gx);
        e->QueryFloatAttribute("goal.y", &gy);
        agents.push_back({Agent(defaultRadius, defaultMaxSpeed, defaultAgentsMaxNum, defaultTimeBoundary, timeStep, defaultSightRadius, id), Point(gx,gy)});
        agents[agents.size()-1].first.SetPosition(Point(stx, sty));
    }



    return true;
}
