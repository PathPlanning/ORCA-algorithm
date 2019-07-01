# ORCA Algorithm

Implementation of ORCA algorithm

## Description
The ORCA algorithm is a decentralized collision avoidance algorithm in a multi-agent environment. The main idea of ​​the algorithm is iterative selection of new agent speed, close to a certain preferred velocity. Selection of the new speed is based on ORCA principle.

Optimal Reciprocal Collision Avoidance (ORCA) — the principle, which provides a sufficient condition for multiple robots to avoid collisions among one another, and thus can guarantee collision-free navigation [[1](http://gamma.cs.unc.edu/ORCA/)]. 
The principle is based on the concept of velocity obstacles, which are used to search for a new  speed of agent so that during the time **_t_** there is no collision with other agents. In the process of searching for a new velocity, algorithm creates a set of n-1 linear constraints(where n is the number of agents that the current one takes into account). A new velocity (Vnew) that satisfies these constraints and is close to the preferred velocity, are searched using an linear programming. The preferred velocity is selected so that the agent makes a move to the target point. The agent is a disk of radius r centered at p. For each neighboring agent (located at a distance R or less), their position and current speed are known. At each simulation step, for each agent, a new velocities are searched, after which the global simulation time is changed to dt and the position of all agents is changed to dt * Vnew (own for each agent).

![Block scheme](/images/ORCA-scheme.png)

More information about ORCA algorithm you can find at ORCA creators web page [[2](http://gamma.cs.unc.edu/RVO2/)].

## Build

To build the project you can use CMake, CMakeLists file is available in the repo. Please note that the code relies on C++11 standart. Make sure that your compiler supports it. If you only need a summary of the results, then use the following commands to build:
#### MacOS
```bash
git clone https://github.com/PathPlanning/ORCA-alorithm.git
cd ORCA-alorithm
mkdir Release
cd Release
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
#### Windows
> in progress

If you need a full log about every task, then use the following commands to build (performance may decrease):
#### MacOS
```sh
git clone https://github.com/PathPlanning/ORCA-alorithm.git
cd ORCA-alorithm
mkdir Debug
cd Debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```
#### Windows
> in progress

## Launch
Use the following command to launch:
#### MacOS
```sh
.\ORCA
```
#### Windows
> in progress

To run the compiled file and get a result you need to pass a correct input XML-file(s). The task files must be in the same directory as the executable file, and must also be named according to the following pattern:
```
*number*_task.xml
```
Moreover, the numbering of tasks should form a sequence of numbers from **_0_** to **_n-1_**, where **_n_** is the total number of tasks. The maximum number of tasks is 100.
For example:
```
0_task.xml
1_task.xml
2_task.xml
3_task.xml
```

To set additional parameters it is possible to use command line arguments (All arguments are required when using additional parameters).
#### MacOS
```sh
.\ORCA [filename Nmin Nstep Nmax]
```
#### Windows
> in progress

where 
- `filename` — name of the file, which contain general log;
- `Nmin` — the initial number of agents at which tasks will run;
- `Nstep` — step of changing the number of agents when restarting tasks;
- `Nmax` — the final number of agents at which tasks will run.

If the number of agents in the task is less than the required value, it will be started with the number of agents specified in the task

## Input and Output files
### Input files
Input files are an XML files with a specific structure.  
Input file should contain:

* Mandatory tag `<default_parameters>`. It describes parameters of agents and agent's perception.
    * `agentsmaxnum` — mandatory attribute that define a number of neighbors, that the agent takes into account;
    * `movespeed` - mandatory attribute that define maximum speed of agent;
    * `sightradius` - mandatory attribute that define the radius in which the agent takes neighbors into account;
    * `size` - mandatory attribute that define size of the agent (radius of the agent);
    * `timeboundary` - mandatory attribute that define the time within which the algorithm ensures collision avoidance

* Mandatory tag `<algorithm>`. It describes the parameters of the algorithm.
    * `delta` - mandatory attribute that define the distance between the center of the agent and the finish, which is enough to reach the finish
    * `timestep` - mandatory attribute that define the time step of simulation.
* Mandatory tag `<agents>`. It describes the parameters of the agents.
    * `number` - mandatory attribute that define the number of agents;
    * `<agent>` - mandatory tags which define parameters of each agent.
        * `id` - defines the identifier of agent
        * `start.x` - defines the coordinate of start position on the x-axis 
        * `start.y` - defines the coordinate of start position on the y-axis 
        * `goal.x` - defines the coordinate of finish position on the x-axis 
        * `goal.y` - defines the coordinate of finish position on the y-axis 
  
Examples locates in directory [Task examples](https://github.com/haiot4105/ORCA-alorithm/tree/master/Task%20examples).
### Output files
#### General log
Contains the main information about the execution of tasks. For example:
```
Success	Runtime	Flowtime    Makespan Collisions
100	0.013	2050	    243     0
100	0.012	1892	    189     0
100	0.011	2486	    277     0
```
There are 4 columns:
* `Success` -  shows the percent of agents, which succsed their tasks; 
* `Runtime` -  shows the time of running of task;
* `Flowtime` - shows the sum of steps of all agents;
* `Makespan` - shows the maximum value of steps of amoung all agents;
* `Collisions` - shows the number of collisions between agents while execution of task;

#### Local log

Contains the full information about the execution of each task. 
Includes same tags as input file, summary (same as general log, but without Flowtime attribute) and information about steps of each agent.

Summary example:
```xml
<summary Success="100" Makespan="242" Runtime="0.0049999999"/>
```
Agent's path example:
```xml
<agent number="0">
    <path pathfound="true" steps="4">
        <step number="0" x="33.18066" y="9.1728058"/>
        <step number="1" x="33.36132" y="9.3456116"/>
        <step number="2" x="33.541981" y="9.5184174"/>
        <step number="3" x="33.722641" y="9.6912231"/>
    </path>
</agent>
```
## Links
1. [Van Den Berg J. et al. Reciprocal n-body collision avoidance //Robotics research. – Springer, Berlin, Heidelberg, 2011. – p. 3-19.](http://gamma.cs.unc.edu/ORCA/)
2. [ORCA creators webpage](http://gamma.cs.unc.edu/RVO2/)
