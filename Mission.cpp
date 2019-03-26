#include "Mission.h"


Mission::Mission(string filename)
{
    double maxlength = 0;
    this->fileName = filename;
    if(!ReadMissionFromFile())
    {
        exit(-1);
    }
    vector<pair<double, double>> starts, goals;
    for(auto &agent : agents)
    {
        auto tmpstart = agent.first.GetPosition().GetPair();
        auto tmpgoal = agent.second.GetPair();
        starts.push_back(tmpstart);
        goals.push_back(tmpgoal);
        double tmplength = sqrt((tmpstart.first - tmpgoal.first) * (tmpstart.first - tmpgoal.first) + (tmpstart.second - tmpgoal.second) * (tmpstart.second - tmpgoal.second));
        if(tmplength > maxlength)
        {
            maxlength = tmplength;
        }
    }

    log = new XmlLogger(agentNumber, defaultRadius, defaultMaxSpeed, defaultAgentsMaxNum, defaultTimeBoundary, defaultSightRadius, starts, goals);
    log->WriteAlgorithmParam(timeStep, delta);
    step = 0;
    stepsLog = vector<vector<pair<double, double>>> (agentNumber);
    results = vector<pair<bool, int>>(agentNumber);
    stepsTreshhold = 10 * (int)round(maxlength/(defaultMaxSpeed * timeStep));

}

bool Mission::isFinished()
{
    bool result = true;
    int i = 0;
    for(auto it = agents.begin(); it != agents.end(); ++it, i++)
    {
        bool localres = (((*it).first.GetPosition() - (*it).second).EuclideanNorm() < delta);
        if(localres && !results[i].first)
        {
            results[i].first = true;
            results[i].second = step;
            (*it).first.Stop();
        }
        result = result && localres;
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
            goalVector = (goalVector/goalVector.EuclideanNorm());// * agent.first.GetMaxSpeed();
            agent.first.SetPrefVelocity(goalVector);

            for(auto &nagent : agents)
            {
                if(agent != nagent)
                    agent.first.AddNeighbour(nagent.first);
            }
        }

        for(auto &agent : agents)
        {
            agent.first.CalculateVelocity();

        }

        int i = 0;
        for(auto &agent : agents)
        {
            agent.first.UpdateVelocity();
            Point tmppos = agent.first.GetPosition() + agent.first.GetVelocity() * timeStep;
            agent.first.SetPosition(tmppos);
            stepsLog[i].push_back({agent.first.GetPosition().GetX(), agent.first.GetPosition().GetY()});
            i++;

        }

        step++;

    }
    while(!isFinished() && step < stepsTreshhold);
    auto endpnt = std::chrono::high_resolution_clock::now();
    long long int res = std::chrono::duration_cast<std::chrono::milliseconds>(endpnt - startpnt).count();
    for(auto &node : results)
    {
        if(!node.first)
        {
            node.second = step;
        }
    }
    log->Save(stepsLog, results, ((double) res) / 1000);
    std::cout<<"Succsess\n";
}

bool Mission::ReadMissionFromFile()
{
    double stx, sty, gx, gy;
    int id;
    XMLDocument doc;
    if(doc.LoadFile(fileName.c_str()))
    {
        std::cout << "File opening error\n";
        return false;
    }

    XMLNode * root = doc.FirstChild();
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
    }

    tmpElement->QueryDoubleAttribute("size", &defaultRadius);
    tmpElement->QueryDoubleAttribute("movespeed", &defaultMaxSpeed);
    tmpElement->QueryIntAttribute("agentsmaxnum", &defaultAgentsMaxNum);
    tmpElement->QueryDoubleAttribute("timeboundary", &defaultTimeBoundary);
    tmpElement->QueryDoubleAttribute("sightradius", &defaultSightRadius);

    // Чтение информации об агентах
    //TODO не дефолтные параметры агентов
    tmpElement = root->FirstChildElement("agents");
    if (tmpElement == nullptr)
    {
        std::cout << "Agents parameters not found\n";
        return false;
    }

    tmpElement->QueryIntAttribute("number", &agentNumber);

    for(auto e = tmpElement->FirstChildElement("agent"); e != NULL; e = e->NextSiblingElement("agent"))
    {
        e->QueryIntAttribute("id", &id);
        e->QueryDoubleAttribute("start.x", &stx);
        e->QueryDoubleAttribute("start.y", &sty);

        e->QueryDoubleAttribute("goal.x", &gx);
        e->QueryDoubleAttribute("goal.y", &gy);
        agents.push_back({Agent(defaultRadius, defaultMaxSpeed, defaultAgentsMaxNum, defaultTimeBoundary, defaultSightRadius, id), Point(gx,gy)});
        agents[agents.size()-1].first.SetPosition(Point(stx, sty));
    }


    tmpElement = root->FirstChildElement("algorithm");
    if (tmpElement == nullptr)
    {
        std::cout << "Algorithm parameters not found\n";
        return false;
    }
    tmpElement->QueryDoubleAttribute("timestep", &timeStep);
    tmpElement->QueryDoubleAttribute("delta", &delta);
    return true;
}
