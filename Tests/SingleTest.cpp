#include <iostream>
#include <string>

#include "Mission.h"

#define STEP_MAX 5000

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


    Mission task = Mission(taskfile, num, STEP_MAX);
    if(task.ReadTask())
    {
        std::string result = task.StartMission().ToString();
        std::cout << "\nSuccess\tRuntime\tFlowtime\tMakespan\tCollisions\tCollisionsObst\n";
        std::cout << result;

#ifndef NDEBUG
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
