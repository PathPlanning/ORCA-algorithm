#include "Mission.h"


Mission::Mission(string filename)
{
    this->fileName = filename;
    if(!ReadMissionFromFile())
    {
        cout<<"Error\n";
        exit(-1);
    }
    vector<pair<double, double>> starts, goals;
    for(auto &agent : agents)
    {
        starts.push_back({agent.first.GetPosition().GetPair()});
        goals.push_back(agent.second.GetPair());
    }

    log =  new XmlLogger(agentNumber, defaultRadius, starts, goals);
    step = 0;

}

bool Mission::isFinished()
{
    bool result = true;
    for(auto it = agents.begin(); it != agents.end(); ++it)
    {
        bool localres = (abs((*it).first.GetPosition().GetX() - (*it).second.GetX()) < delta) && (abs((*it).first.GetPosition().GetY() - (*it).second.GetY()) < delta);
        if(localres)
        {
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
    //TODO Добавить лимит по шагам
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
            log->WriteStep(step, i, agent.first.GetPosition().GetX(), agent.first.GetPosition().GetY());
            agent.first.SetPosition(tmppos);
            i++;

        }

        step++;

    }
    while(!isFinished());
    std::cout<<"Succsess\n";
    log->Save();
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
