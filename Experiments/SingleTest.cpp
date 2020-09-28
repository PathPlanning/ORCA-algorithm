#include <iostream>
#include <string>
#include <ostream>

#include "Mission.h"

#define STEP_MAX            12800
#define IS_TIME_BOUNDED     false
#define TIME_MAX            1000 * 60 * 4
#define RESULT_FILE         "result.txt"


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
    
    ofstream pre_log(RESULT_FILE, std::ofstream::app);
    
    Mission task = Mission(taskfile, num, STEP_MAX, IS_TIME_BOUNDED, TIME_MAX);
    if(task.ReadTask())
    {
        std::string result = task.StartMission().ToString();
        std::cout << "\nSuccess\tRuntime\tMakespan\tFlowtime\tCollisions\tCollisionsObst\n";
        std::cout << result;
        pre_log << result;
#if FULL_LOG
        task.SaveLog();
#endif

    }
    else
    {
        std::cout<<"Error during task execution\n";
        pre_log << -1;
        return -1;
    }
    pre_log.close();
    return 0;
}
