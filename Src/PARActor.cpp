#include "PARActor.h"

int PARActor::getStart_i() const
{
    return start_i;
}

int PARActor::getStart_j() const
{
    return start_j;
}

int PARActor::getGoal_i() const
{
    return goal_i;
}

int PARActor::getGoal_j() const
{
    return goal_j;
}

int PARActor::getCur_i() const
{
    return cur_i;
}

int PARActor::getCur_j() const
{
    return cur_j;
}


int PARActor::getId() const
{
    return id;
}

int PARActor::getSubgraph() const {
    return subgraph;
}

Node PARActor::getStartPosition() const {
    return Node(start_i, start_j);
}

Node PARActor::getGoalPosition() const {
    return Node(goal_i, goal_j);
}

Node PARActor::getCurPosition() const {
    return Node(cur_i, cur_j);
}

void PARActor::setCurPosition(int i, int j) {
    cur_i = i;
    cur_j = j;
}

void PARActor::setSubgraph(int Subgraph) {
    subgraph = Subgraph;
}

