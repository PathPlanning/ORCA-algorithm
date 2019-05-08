#include <iostream>
#include "Agent.h"
#include "Point.h"
#include "XmlLogger.h"
#include "Mission.h"
#include <fstream>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char* argv[])
{
    //vector<int> numAg = {15,30,45,60,75,90,105,120,135,150,165,180,195,210};
    vector<int> numAg = {210};
    string log;
    if(argc < 2)
    {
        std::cout<<"Pathfinding log file (XML) is not specified!"<<std::endl;
        log = "logfile.txt";
    }
    else
    {
        log = string(argv[1]);
    }

    ofstream pre_log(log);
    pre_log<< "Success\tRuntime\tFlowtime\tMakespan\n";
    pre_log.close();

    string tmpinp = "_task.xml";
    for(auto &num : numAg)
    {
        cout<<"Number of agents: "<<num<<"\n";
        for(int i = 0; i < 50; i++)
        {
            string inp = std::to_string(i) + tmpinp;
            cout<<"File: "<<inp<<"\n";
            Mission mission(inp, log, num);
            mission.StartMission();
        }
    }


    return 0;
}