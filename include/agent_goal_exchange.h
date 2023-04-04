#include <set>
#include <unordered_set>
#include <algorithm>
#include <cstdlib>
#include <random>
#include <functional>

#include "Agent.h"

#ifndef ORCASTAR_AGENT_GOAL_EXCHANGE_H
#define ORCASTAR_AGENT_GOAL_EXCHANGE_H


class AgentGoalExchange : public Agent
{
    public:
		AgentGoalExchange();
		AgentGoalExchange(const int &id, const Point &start, const Map &map, const EnvironmentOptions &options,
                         AgentParam param);
		AgentGoalExchange(const AgentGoalExchange &obj);
        ~AgentGoalExchange();
		AgentGoalExchange &operator = (const AgentGoalExchange &obj);
		AgentGoalExchange* clone() const override ;

        bool operator == (const AgentGoalExchange &another) const;
        bool operator != (const AgentGoalExchange &another) const;

		void setGoalList(const std::vector<Point> &goal_list);

        void computeNewControl() override;
        void applyNewControl() override;
        bool prepareBeforeStep() override;
        void addNeighbour(Agent &neighbour, float dist_sq) override;
		bool initAgent() override;

		std::tuple<size_t, size_t> getAMAPFStat() const;


    private:
		auto requestJoinToGroup(bool transmit_to_neighbours) -> std::tuple<std::unordered_set<AgentGoalExchange *>, std::unordered_set<Point>>;
		std::vector <std::pair<float, Agent*>>& requestNeighbours();

		bool tryExchangeGoal();
		bool goal_was_exchanged;

		std::tuple<bool, Point, bool> requestForExchanging(Point other_goal, Point other_position, bool other_finished);
        void sendGroupRequestForExchanging();

		std::set<AgentGoalExchange *> createGroup();

        float buffer_radius;
		std::unordered_set<Point> goals;
		bool in_group;
		size_t messages_count;
		size_t exchange_count;

		bool finished;
};

template<>
struct std::hash<AgentGoalExchange>
{
	std::size_t operator()(AgentGoalExchange const& agent) const noexcept
	{
		return std::hash<int>{}(agent.getID());
	}
};

#endif //ORCASTAR_AGENT_GOAL_EXCHANGE_H
