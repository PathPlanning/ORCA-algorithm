 #include "XmlLogger.h"


XmlLogger::XmlLogger(int num, float r, float maxspeed, int neighborsNum, float timeBoundary, float sightradius, vector<pair<float, float>> start, vector<pair<float, float>> goal)
{
    doc = new XMLDocument();
    this->num = num;
    this->radius = r;
    root = doc->NewElement("root");
    doc->InsertFirstChild(root);
    XMLElement *defparam = doc->NewElement("default_parameters");
    defparam->SetAttribute("size", radius);
    defparam->SetAttribute("movespeed", maxspeed);
    defparam->SetAttribute("agentsmaxnum", neighborsNum);
    defparam->SetAttribute("timeboundary", timeBoundary);
    defparam->SetAttribute("sightradius", sightradius);

    XMLElement *agents = doc->NewElement("agents");
    agents->SetAttribute("number", num);
    XMLElement *ag;

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
    }
    root->InsertEndChild(defparam);
    root->InsertEndChild(agents);

}


void XmlLogger::WriteAlgorithmParam(float timestep, float delta)
{
    XMLElement *tmpalg = doc->NewElement("algorithm");
    tmpalg->SetAttribute("timestep", timestep);
    tmpalg->SetAttribute("delta", delta);
    root->InsertEndChild(tmpalg);
}


void XmlLogger::Save(vector<vector<pair<float, float>>> resultSteps, vector<pair<bool, int>> results, float time)
{
    XMLElement *tmpsum, *tmpagent, *tmppath, *tmpstep;
    float rate = 0;
    int i = 0, j = 0;
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
        //TODO  переделать time на процессорное время

        for(auto &step : agent)
        {
            tmpstep = doc->NewElement("step");
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
    doc->SaveFile("resultlog.xml");
}


