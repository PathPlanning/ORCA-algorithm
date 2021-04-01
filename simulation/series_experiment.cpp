#include <iostream>
#include <string>
#include <ostream>
#include <sstream>

#include "include/Mission.h"

#define RESULT_FILE         "result.txt"
#define INPUT_FILE_PART     "_task.xml"
#define STEP_MAX            20000
#define IS_TIME_BOUNDED     false
#define STOP_BY_SPEED       true
#define TIME_MAX            1000 * 60 * 1


int main(int argc, char* argv[])
{
    if(argc != 6)
    {
        std::cout<<"Error! Invalid number of arguments.\n";
        return -1;
    }

    vector<int> numAg = vector<int>();
    int sup, step, inf, tasknum;

    try
    {
        string tmp = string(argv[1]);
        inf = stoi(tmp);
        tmp = string(argv[2]);
        step = stoi(tmp);
        tmp = string(argv[3]);
        sup = stoi(tmp);
        tmp = string(argv[4]);
        tasknum = stoi(tmp);

        if(inf < 1 || sup < inf || step < 1 || tasknum < 1)
        {
            throw std::invalid_argument("Invalid parameters.");
        }
    }
    catch(std::invalid_argument)
    {
        std::cout<<"Error! Invalid parameters."<<std::endl;
        return -1;
    }

    string path = string(argv[5]);

    std::cout<<"The number of agents will be set to the following values: ";
    for(int i = inf; i <= sup; i+= step)
    {
        numAg.push_back(i);
        cout<<i<<" ";
    }
    cout<<"\n";
    ofstream pre_log(path + "/" + RESULT_FILE);
    if(!pre_log)
    {
        std::cout<<"Error! Сannot open or create file "<< path + RESULT_FILE <<std::endl;
        return -1;
    }

//    pre_log<< "Success\tRuntime\tMakespan\tFlowtime\tCollisions\tCollisionsObst\n";

    for(auto &num : numAg)
    {
        cout<<"Number of agents: "<<num<<"\n";
        for(int i = 0; i < tasknum; i++)
        {
            string taskfile = path + "/" +std::to_string(i) + INPUT_FILE_PART;
            cout<<"File: "<<taskfile<<"\n";
            Mission *task = new Mission(taskfile, num, STEP_MAX, IS_TIME_BOUNDED, TIME_MAX, STOP_BY_SPEED);

            if(task->ReadTask())
            {
                auto summary = task->StartMission();
                std::stringstream sumStream;
//                std::cout << "\nSuccess\tRuntime\tMakespan\tFlowtime\tCollisions\tCollisionsObst\tMeanMAPFTime\tInits\tUpdates\tUnites\tECBS\tPnR\n";
                sumStream << summary[CNS_SUM_SUCCESS_RATE] << "\t";
                sumStream << summary[CNS_SUM_RUN_TIME] << "\t";
                sumStream << summary[CNS_SUM_MAKESPAN] << "\t";
                sumStream << summary[CNS_SUM_FLOW_TIME] << "\t";
                sumStream << summary[CNS_SUM_COLLISIONS] << "\t";
                sumStream << summary[CNS_SUM_COLLISIONS_OBS] << "\t";
                sumStream << summary[CNS_SUM_MAPF_MEAN_TIME] << "\t";
                sumStream << summary[CNS_SUM_MAPF_INIT_COUNT] << "\t";
                sumStream << summary[CNS_SUM_MAPF_UPDATE_COUNT] << "\t";
                sumStream << summary[CNS_SUM_MAPF_UNITE_COUNT] << "\t";
                sumStream << summary[CNS_SUM_MAPF_ECBS_COUNT] << "\t";
                sumStream << summary[CNS_SUM_MAPF_PAR_COUNT] << "\t";
                sumStream << summary[CNS_SUM_MAPF_SUCCESS_COUNT] << "\t";
                sumStream << summary[CNS_SUM_MAPF_UNSUCCESS_COUNT] << "\t";
                sumStream << summary[CNS_SUM_MAPF_FLOWTIME] << "\n";
                std::cout << sumStream.str();
                pre_log << sumStream.str();

#if FULL_LOG
                task->SaveLog();
#endif
                delete task;
            }
            else
            {
                std::cout<<"Error during task execution\n\n";
                delete task;
                continue;
            }

        }
    }

    pre_log.close();
    return 0;
}