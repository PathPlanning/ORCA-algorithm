#include <string>
#include <sstream>

#include "tinyxml2.h"
#include "Reader.h"
#include "xml_tags.h"
#include "Const.h"
#include "Agent.h"
#include "ORCAAgent.h"
#include "ORCADDAgent.h"
#include "ThetaStar.h"
#include "DirectPlanner.h"
#include "ORCAAgentWithPAR.h"
#include "ORCAAgentWithECBS.h"
#include "ORCAAgentWithPARAndECBS.h"
#include "ORCAAgentWithReturning.h"



#ifndef ORCA_XMLREADER_H
#define ORCA_XMLREADER_H

using namespace tinyxml2;

class XMLReader : public Reader
{
    public:
        XMLReader();
        XMLReader(const std::string &fileName);
        XMLReader(const XMLReader &obj);
        ~XMLReader() override;

        bool ReadData() override;
        bool GetMap(Map **map) override;
        bool GetEnvironmentOptions(EnvironmentOptions **envOpt) override;
        bool GetAgents(std::vector<Agent *> &agents, const int &numThreshold) override;
        XMLReader* Clone() const override;

        XMLReader & operator = (const XMLReader & obj);



    private:
        std::string fileName;

        XMLDocument *doc;
        XMLElement *root;

        std::vector<Agent *> *allAgents;
        Map *map;
        EnvironmentOptions *options;
        std::vector<std::vector<int>> *grid;
        std::vector<std::vector<Point>> *obstacles;
        int plannertype;

        bool ReadMap();
        bool ReadAgents();
        bool ReadAlgorithmOptions();

};


#endif //ORCA_XMLREADER_H
