//
// Created by Stepan on 26/02/2019.
//

#include "XmlLogger.h"

XmlLogger::XmlLogger(int num, int r, std::vector<std::pair<double, double>> start, std::vector<std::pair<double, double>> goal)
{
    doc = new XMLDocument();
    this->num = num;
    this->radius = r;
    root = doc->NewElement("Root");
    doc->InsertFirstChild(root);

    XMLElement *agents = doc->NewElement("Agents");
    agents->SetAttribute("Number", num);
    agents->SetAttribute("Radius", radius);
    XMLElement *ag;
    steps = doc->NewElement("Steps");
    for(int i = 0; i < num; i++)
    {
        std::string name("A" + std::to_string(i));
        ag = doc->NewElement(name.c_str());
        ag->SetAttribute("Xstart", start[i].first);
        ag->SetAttribute("Ystart", start[i].second);
        ag->SetAttribute("Xgoal", goal[i].first);
        ag->SetAttribute("Ygoal", goal[i].second);
        agents->InsertEndChild(ag);
        XMLElement *tmpag = doc->NewElement(name.c_str());
        agentsteps.push_back(tmpag);
        steps->InsertEndChild(tmpag);
    }


    root->InsertEndChild(agents);
    root->InsertEndChild(steps);
}

void XmlLogger::WriteStep(int step, int agentnum, double x, double y)
{
    if(agentnum >= num)
        return;

    XMLElement *tmpag = agentsteps[agentnum];

    std::string stepname("Step");
    XMLElement *tmpstep = doc->NewElement(stepname.c_str());
    tmpstep->SetAttribute("num", step);
    tmpstep->SetAttribute("X", x);
    tmpstep->SetAttribute("Y", y);
    tmpag->InsertEndChild(tmpstep);

}

void XmlLogger::Save()
{
    XMLError eResult = doc->SaveFile("SavedData.xml");
}


