
#include "XmlLogger.h"

XmlLogger::XmlLogger(int num, int r, double maxspeed, int neighborsNum, double timeBoundary, double sightradius, vector<pair<double, double>> start, vector<pair<double, double>> goal)
{
    doc = new XMLDocument();
    this->num = num;
    this->radius = r;
    root = doc->NewElement("root");
    doc->InsertFirstChild(root);
    XMLElement *defparam = doc->NewElement("default_parameters");
    defparam->SetAttribute("size", radius);
    defparam->SetAttribute("movespeed", maxspeed); //TODO
    defparam->SetAttribute("agentsmaxnum", neighborsNum);
    defparam->SetAttribute("timeboundary", timeBoundary);
    defparam->SetAttribute("sightradius", sightradius);

    XMLElement *agents = doc->NewElement("agents");
    agents->SetAttribute("number", num);
    XMLElement *ag;
    //steps = doc->NewElement("Steps");



    for(int i = 0; i < num; i++)
    {
        std::string name(std::to_string(i));
        ag = doc->NewElement("agent");
        ag->SetAttribute("id", i);
        ag->SetAttribute("start.x", start[i].first);
        ag->SetAttribute("start.y", start[i].second);
        ag->SetAttribute("goal.x", goal[i].first);
        ag->SetAttribute("goal.y", goal[i].second);
        agents->InsertEndChild(ag);
        XMLElement *tmpag = doc->NewElement(name.c_str());

        agentsteps.push_back(tmpag);
        //steps->InsertEndChild(tmpag);
    }

    root->InsertEndChild(defparam);
    root->InsertEndChild(agents);
    //root->InsertEndChild(steps);
}

//void XmlLogger::WriteStep(int step, int agentnum, double x, double y)
//{
//    if(agentnum >= num)
//        return;
//
//    XMLElement *tmpag = agentsteps[agentnum];
//
//    std::string stepname("Step");
//    XMLElement *tmpstep = doc->NewElement(stepname.c_str());
//    tmpstep->SetAttribute("num", step);
//    tmpstep->SetAttribute("X", x);
//    tmpstep->SetAttribute("Y", y);
//    tmpag->InsertEndChild(tmpstep);
//
//}


//
//void XmlLogger::WriteSummary()
//{
//   // <summary agentssolved="100.000000%"  flowtime="13719.313" avgduration="214.36427" makespan="492.27024218944149" time="0.37608677978818322"/>
//}


void XmlLogger::WriteAlgorithmParam(double timestep, double delta)
{
    XMLElement *tmpalg = doc->NewElement("algorithm");
    tmpalg->SetAttribute("timestep", timestep);
    tmpalg->SetAttribute("delta", delta);
    root->InsertEndChild(tmpalg);
}


void XmlLogger::Save(vector<vector<pair<double, double>>> resultSteps, vector<pair<bool, int>> results, double time)
{
    XMLElement *tmpsum, *tmpagent, *tmppath, *tmpstep;
    double rate = 0;
    int i = 0, j = 0, stepmax = 0;
    tmpsum = doc->NewElement("summary");
    root->InsertEndChild(tmpsum);
    for(auto &agent : resultSteps)
    {
        j = 0;
        tmpagent = doc->NewElement("agent");
        tmpagent->SetAttribute("number", i);
        tmppath = doc->NewElement("path");
        tmppath->SetAttribute("pathfound", results[i].first);
        tmppath->SetAttribute("steps", results[i].second);
        rate += results[i].first;
        //TODO pathfound="true" duration="346.31885627123097" nodescreated="607" time="0.0041928069354103346"

        for(auto &step : agent)
        {
            tmpstep = doc->NewElement("step");
            //<section number="0" start.x="13" start.y="43" finish.x="13" finish.y="43" duration="4.1054588307284714"/>
            tmpstep->SetAttribute("number", j);
            tmpstep->SetAttribute("x", step.first);
            tmpstep->SetAttribute("y", step.second);
            j++;
            tmppath->InsertEndChild(tmpstep);
        }



        tmpagent->InsertEndChild(tmppath);
        root->InsertEndChild(tmpagent);
        i++;
    }

    rate = rate * 100 / results.size();
    tmpsum->SetAttribute("agentssolved", rate);
    tmpsum->SetAttribute("maxsteps", j);
    tmpsum->SetAttribute("runtime", time);
    XMLError eResult = doc->SaveFile("resultlog.xml");
}


