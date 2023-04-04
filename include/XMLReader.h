#include <string>
#include <sstream>
#include "tinyxml2.h"
#include "Reader.h"
#include "Const.h"
#include "Agent.h"
#import "ORCAAgent.h"
#import "ORCADDAgent.h"
#include "ThetaStar.h"
#include "DirectPlanner.h"
#include "ORCAAgentWithPAR.h"
#include "ORCAAgentWithECBS.h"
#include "ORCAAgentWithPARAndECBS.h"
#include "ORCAAgentWithReturning.h"
#include "agent_goal_exchange.h"



#ifndef ORCA_XMLREADER_H
#define ORCA_XMLREADER_H



using namespace tinyxml2;

class XMLReader : public Reader
{
    public:
        XMLReader();
        XMLReader(const std::string &file_name);
        XMLReader(const XMLReader &obj);
        ~XMLReader() override;

        bool readData() override;
        bool getMap(Map **map) override;
        bool getEnvironmentOptions(EnvironmentOptions **env_opt) override;
        bool getAgents(std::vector<Agent *> &agents, const int &num_threshold) override;



        XMLReader* Clone() const override;

        XMLReader & operator = (const XMLReader & obj);



    private:
        std::string file;

        XMLDocument *doc;
        XMLElement *root;

        std::vector<Agent *> *all_agents;
        Map *map;
        EnvironmentOptions *options;
        std::vector<std::vector<int>> *grid;
        std::vector<std::vector<Point>> *obstacles;
		std::vector<Point> all_goals;
        int planner_type;
		std::string agent_type_str;

        bool readMap();
        bool readAgents();
        bool readAlgorithmOptions();
};


#endif //ORCA_XMLREADER_H
