#include <iostream>
#include <string>
#include <ostream>

#include "include/Mission.h"

#define STEP_MAX            10000
#define IS_TIME_BOUNDED     true
#define STOP_BY_SPEED       false
#define TIME_MAX            1000 * 60 * 0.2


int main(int argc, char* argv[])
{

    if(argc != 3)
    {
        std::cout<<"Error! Invalid number of arguments.\n";
        return -1;
    }

    string taskfile = string(argv[1]);
    int num = 0;
    try
    {
        string tmp = string(argv[2]);
        num = stoi(tmp);
        if(num < 1)
        {
            throw std::invalid_argument("Number of agents must be positive");
        }
    }
    catch(std::invalid_argument)
    {
        std::cout<<"Error! Invalid number of agents."<<std::endl;
        return -1;
    }

    
    Mission task = Mission(taskfile, num, STEP_MAX, IS_TIME_BOUNDED, TIME_MAX, STOP_BY_SPEED);
    if(task.ReadTask())
    {
        auto summary = task.StartMission();

        std::cout << "\nSuccess\tRuntime\tMakespan\tFlowtime\tCollisions\tCollisionsObst\tMeanMAPFTime\tInits\tUpdates\tUnites\tECBS\tPnR\tSuccessMAPF\tUnsuccessMAPF\tFlowtimeMAPF\n";
        std::cout << summary[CNS_SUM_SUCCESS_RATE] << "\t";
        std::cout << summary[CNS_SUM_RUN_TIME] << "\t";
        std::cout << summary[CNS_SUM_MAKESPAN] << "\t";
        std::cout << summary[CNS_SUM_FLOW_TIME] << "\t";
        std::cout << summary[CNS_SUM_COLLISIONS] << "\t";
        std::cout << summary[CNS_SUM_COLLISIONS_OBS] << "\t";
        std::cout << summary[CNS_SUM_MAPF_MEAN_TIME] << "\t";
        std::cout << summary[CNS_SUM_MAPF_INIT_COUNT] << "\t";
        std::cout << summary[CNS_SUM_MAPF_UPDATE_COUNT] << "\t";
        std::cout << summary[CNS_SUM_MAPF_UNITE_COUNT] << "\t";
        std::cout << summary[CNS_SUM_MAPF_ECBS_COUNT] << "\t";
        std::cout << summary[CNS_SUM_MAPF_PAR_COUNT] << "\t";
        std::cout << summary[CNS_SUM_MAPF_SUCCESS_COUNT] << "\t";
        std::cout << summary[CNS_SUM_MAPF_UNSUCCESS_COUNT] << "\t";
        std::cout << summary[CNS_SUM_MAPF_FLOWTIME] << "\n";

#if FULL_LOG
        task.SaveLog();
#endif

    }
    else
    {
        std::cout<<"Error during task execution\n";
        return -1;
    }

    return 0;
}
