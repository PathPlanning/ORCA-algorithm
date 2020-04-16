#include <tuple>
#include <set>

#ifndef ORCA_CONSTRAINTS_H
#define ORCA_CONSTRAINTS_H

struct Constraint
{
    int     i, j;
    int     time;
    int     actorId;
    int     prev_i, prev_j;
    bool    goalNode;

    Constraint(int x = 0, int y = 0, int Time = 0, int ActorId = 0, int PrevI = -1, int PrevJ = -1, bool GoalNode = false)
    {
        i = x;
        j = y;
        time = Time;
        actorId = ActorId;
        prev_i = PrevI;
        prev_j = PrevJ;
        goalNode = GoalNode;
    }

    bool operator== (const Constraint &other) const
    {
        return i == other.i && j == other.j && time == other.time &&
               prev_i == other.prev_i && prev_j == other.prev_j && goalNode == other.goalNode;
    }
    bool operator!= (const Constraint &other) const
    {
        return !(*this == other);
    }

    bool operator< (const Constraint &other) const
    {
        return std::tuple<int, int, int, int, int, bool>(i, j, time, prev_i, prev_j, goalNode) <
               std::tuple<int, int, int, int, int, bool>(other.i, other.j, other.time, other.prev_i, other.prev_j, other.goalNode);
    }
};


class ConstraintsSet
{
    public:
        void addNodeConstraint(int i, int j, int time, int actorId);
        void addGoalNodeConstraint(int i, int j, int time, int actorId);
        void addEdgeConstraint(int i, int j, int time, int actorId, int prevI, int prevJ);
        void addConstraint(Constraint &constraint);
        ConstraintsSet getActorConstraints(int actorId) const;

        bool hasNodeConstraint(int i, int j, int time, int actorId) const;
        bool hasFutureConstraint(int i, int j, int time, int actorId) const;
        bool hasEdgeConstraint(int i, int j, int time, int actorId, int prevI, int prevJ) const;
    private:
        std::set<Constraint> nodeConstraints;
        std::set<Constraint> edgeConstraints;
        std::set<Constraint> goalNodeConstraints;
};



#endif //ORCA_CONSTRAINTS_H
