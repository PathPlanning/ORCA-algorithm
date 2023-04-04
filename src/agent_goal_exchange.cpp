#include "agent_goal_exchange.h"


AgentGoalExchange::AgentGoalExchange() : Agent() {
	buffer_radius = 0;
	goals = std::unordered_set<Point>();
	goal_was_exchanged = false;
	messages_count = 0;
	exchange_count = 0;
}


AgentGoalExchange::AgentGoalExchange(const int &id, const Point &start, const Map &map,
									 const EnvironmentOptions &options, AgentParam param) : Agent(id, start, Point(),
																								  map, options, param) {
	buffer_radius = param.rEps + param.radius;
	goals = std::unordered_set<Point>();
	goal_was_exchanged = false;
	messages_count = 0;
	exchange_count = 0;
	finished = false;
}


AgentGoalExchange::AgentGoalExchange(const AgentGoalExchange &obj) : Agent(obj) {
	buffer_radius = obj.buffer_radius;
	goals = obj.goals;
	goal_was_exchanged = obj.goal_was_exchanged;
	messages_count = obj.messages_count;
	exchange_count = obj.exchange_count;
	finished = obj.finished;
}


AgentGoalExchange::~AgentGoalExchange() = default;


AgentGoalExchange &AgentGoalExchange::operator=(const AgentGoalExchange &obj) {

	if (this != &obj) {
		Agent::operator=(obj);
		buffer_radius = obj.buffer_radius;
		goals = obj.goals;
		goal_was_exchanged = obj.goal_was_exchanged;
		messages_count = obj.messages_count;
		exchange_count = obj.exchange_count;
		finished = obj.finished;

	}
	return *this;
}


AgentGoalExchange *AgentGoalExchange::clone() const {
	return new AgentGoalExchange(*this);
}


bool AgentGoalExchange::operator==(const AgentGoalExchange &another) const {
	return this->id == another.id;
}


bool AgentGoalExchange::operator!=(const AgentGoalExchange &another) const {
	return this->id != another.id;
}


void AgentGoalExchange::computeNewControl() {

//	if (finished) {
//		ORCALines.clear();
//		Neighbours.clear();
//		this->newV = Point(0,0);
//	}

	ORCALines.clear();

	// Получение ORCA-линий препятсвий
	for (int i = 0; i < NeighboursObst.size(); i++) {
		Line line;

		Vertex *left = &(NeighboursObst[i].second.left);
		Vertex *right = &(NeighboursObst[i].second.right);

		Vector lRelativePosition = *left - position;
		Vector rRelativePosition = *right - position;


		bool alreadyCovered = false;

		for (int j = 0; j < ORCALines.size(); j++) {
			if ((lRelativePosition * invTimeBoundaryObst - ORCALines[j].liesOn).Det(ORCALines[j].dir) -
				invTimeBoundaryObst * buffer_radius >= -CN_EPS &&
				(rRelativePosition * invTimeBoundaryObst - ORCALines[j].liesOn).Det(ORCALines[j].dir) -
				invTimeBoundaryObst * buffer_radius >= -CN_EPS) {
				alreadyCovered = true;
				break;
			}
		}
		if (alreadyCovered)
			continue;

		float lSqDist = lRelativePosition.SquaredEuclideanNorm();
		float rSqDist = rRelativePosition.SquaredEuclideanNorm();

		float sqFakeRadius = buffer_radius * buffer_radius;
		float sqTrueRadius = param.radius * param.radius;

		Vector obstacleVector = *right - *left;
		float s = -lRelativePosition.ScalarProduct(obstacleVector) / obstacleVector.SquaredEuclideanNorm();
		float lineSqDist = (-lRelativePosition - obstacleVector * s).SquaredEuclideanNorm();

		if ((s < 0.0f && lSqDist < sqTrueRadius) || (s > 1.0f && rSqDist < sqTrueRadius) ||
			(s >= 0.0f && s < 1.0f && lineSqDist < sqTrueRadius)) {
			collisionsObst++;
		}


		if (s < 0.0f && lSqDist < sqFakeRadius) {

			if (left->IsConvex()) {
				line.liesOn = Point();
				line.dir = Point(-lRelativePosition.Y(), lRelativePosition.X()) /
						   sqrt(lSqDist); // Построение единичного вектора, нормального к относительному положению
				ORCALines.push_back(line);
			}

			continue;
		}
		else if (s > 1.0f && rSqDist < sqFakeRadius) {

			if (right->IsConvex() && rRelativePosition.Det(NeighboursObst[i].second.next->dir) >= 0.0f) {
				line.liesOn = Point();
				line.dir = Point(-rRelativePosition.Y(), rRelativePosition.X()) / sqrt(rSqDist);
				ORCALines.push_back(line);
			}

			continue;
		}
		else if (s >= 0.0f && s < 1.0f && lineSqDist < sqFakeRadius) {
			line.liesOn = Point();
			line.dir = -(NeighboursObst[i].second.dir);
			ORCALines.push_back(line);
			continue;
		}


		Vector lLegDirection, rLegDirection;

		if (s < 0.0f && lineSqDist <= sqFakeRadius) {

			if (!left->IsConvex()) {
				continue;
			}

			right = left;

			float leg1 = sqrt(lSqDist - sqFakeRadius);

			lLegDirection = Point(lRelativePosition.X() * leg1 - lRelativePosition.Y() * buffer_radius,
								  lRelativePosition.X() * buffer_radius + lRelativePosition.Y() * leg1) / lSqDist;
			rLegDirection = Point(lRelativePosition.X() * leg1 + lRelativePosition.Y() * buffer_radius,
								  -lRelativePosition.X() * buffer_radius + lRelativePosition.Y() * leg1) / lSqDist;
		}
		else if (s > 1.0f && lineSqDist <= sqFakeRadius) {

			if (!right->IsConvex()) {
				continue;
			}

			left = right;

			float leg2 = std::sqrt(rSqDist - sqFakeRadius);
			lLegDirection = Point(rRelativePosition.X() * leg2 - rRelativePosition.Y() * buffer_radius,
								  rRelativePosition.X() * buffer_radius + rRelativePosition.Y() * leg2) / rSqDist;
			rLegDirection = Point(rRelativePosition.X() * leg2 + rRelativePosition.Y() * buffer_radius,
								  -rRelativePosition.X() * buffer_radius + rRelativePosition.Y() * leg2) / rSqDist;
		}
		else {
			if (left->IsConvex()) {
				float leg1 = std::sqrt(lSqDist - sqFakeRadius);
				lLegDirection = Point(lRelativePosition.X() * leg1 - lRelativePosition.Y() * buffer_radius,
									  lRelativePosition.X() * buffer_radius + lRelativePosition.Y() * leg1) / lSqDist;
			}
			else {
				lLegDirection = -NeighboursObst[i].second.dir;
			}

			if (right->IsConvex()) {
				float leg2 = std::sqrt(rSqDist - sqFakeRadius);
				rLegDirection = Point(rRelativePosition.X() * leg2 + rRelativePosition.Y() * buffer_radius,
									  -rRelativePosition.X() * buffer_radius + rRelativePosition.Y() * leg2) / rSqDist;
			}
			else {
				rLegDirection = NeighboursObst[i].second.dir;
			}
		}

		ObstacleSegment *leftNeighbor = NeighboursObst[i].second.prev;

		bool isLLegForeign = false, isRLegForeign = false;

		if (left->IsConvex() && lLegDirection.Det(-leftNeighbor->dir) >= 0.0f) {
			lLegDirection = -leftNeighbor->dir;
			isLLegForeign = true;
		}

		if (right->IsConvex() && rLegDirection.Det(NeighboursObst[i].second.next->dir) <= 0.0f) {
			rLegDirection = NeighboursObst[i].second.next->dir;
			isRLegForeign = true;
		}

		Point leftCutoff = (*left - position) * invTimeBoundaryObst;
		Point rightCutoff = (*right - position) * invTimeBoundaryObst;
		Vector cutoffVec = rightCutoff - leftCutoff;

		const float t = (right == left ? 0.5f : ((currV - leftCutoff).ScalarProduct(cutoffVec)) /
												cutoffVec.SquaredEuclideanNorm());
		const float tLeft = ((currV - leftCutoff).ScalarProduct(lLegDirection));
		const float tRight = ((currV - rightCutoff).ScalarProduct(rLegDirection));

		if ((t < 0.0f && tLeft < 0.0f) || (left == right && tLeft < 0.0f && tRight < 0.0f)) {
			Vector unitW = (currV - leftCutoff) / (currV - leftCutoff).EuclideanNorm();

			line.dir = Vector(unitW.Y(), -unitW.X());
			line.liesOn = leftCutoff + unitW * buffer_radius * invTimeBoundaryObst;
			ORCALines.push_back(line);
			continue;
		}
		else if (t > 1.0f && tRight < 0.0f) {
			Vector unitW = (currV - rightCutoff) / (currV - rightCutoff).EuclideanNorm();

			line.dir = Vector(unitW.Y(), -unitW.X());
			line.liesOn = rightCutoff + unitW * buffer_radius * invTimeBoundaryObst;
			ORCALines.push_back(line);
			continue;
		}

		float cutoffSqDist = ((t < 0.0f || t > 1.0f || right == left) ? std::numeric_limits<float>::infinity() : (
				currV - (leftCutoff + cutoffVec * t)).SquaredEuclideanNorm());
		float lLegSqDist = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : (currV - (leftCutoff +
																								lLegDirection *
																								tLeft)).SquaredEuclideanNorm());
		float rLegSqDist = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : (currV - (rightCutoff +
																								 rLegDirection *
																								 tRight)).SquaredEuclideanNorm());

		if (cutoffSqDist <= lLegSqDist && cutoffSqDist <= rLegSqDist) {
			line.dir = -NeighboursObst[i].second.dir;
			line.liesOn = leftCutoff + Point(-line.dir.Y(), line.dir.X()) * buffer_radius * invTimeBoundaryObst;
			ORCALines.push_back(line);
			continue;
		}
		else if (lLegSqDist <= rLegSqDist) {
			if (isLLegForeign) {
				continue;
			}

			line.dir = lLegDirection;
			line.liesOn = leftCutoff + Point(-line.dir.Y(), line.dir.X()) * buffer_radius * invTimeBoundaryObst;;
			ORCALines.push_back(line);
			continue;
		}
		else {
			if (isRLegForeign) {
				continue;
			}

			line.dir = -rLegDirection;
			line.liesOn = rightCutoff + Point(-line.dir.Y(), line.dir.X()) * buffer_radius * invTimeBoundaryObst;
			ORCALines.push_back(line);
			continue;
		}
	}

	size_t numObstLines = ORCALines.size();

	//Получение ORCA-линий агентов
	//std::sort(Neighbours.begin(),Neighbours.end(), Compare);

	Line currline;
	AgentGoalExchange *curragent;
	Vector u, w;

	unsigned long minMaxNum = (param.agentsMaxNum < Neighbours.size()) ? param.agentsMaxNum : Neighbours.size();

	for (unsigned long i = 0; i < minMaxNum; i++) {
		auto Neighbour = Neighbours[i];
		curragent = dynamic_cast<AgentGoalExchange *>(Neighbour.second);
		auto circlecenter = curragent->position - this->position; //(P_b - P_a)
		auto relvelocity = this->currV - curragent->currV; //(V_a - V_b)

		float radiussum = buffer_radius + curragent->buffer_radius; //(R_a + R_b)
		float radiussum2 = radiussum * radiussum;
		float distSq = circlecenter.SquaredEuclideanNorm();
		float trueSqRadSum = (param.radius + curragent->param.radius) * (param.radius + curragent->param.radius);

		if (distSq < trueSqRadSum) {
			collisions++;
		}

		if (distSq >= radiussum2) {
			w = relvelocity - (circlecenter *
							   invTimeBoundary); //w -- вектор на плоскости скоростей от центра малой окружности (основания VO) до скорости другого агента относительно этого
			float sqwlength = w.SquaredEuclideanNorm();
			float wproj = w.ScalarProduct(circlecenter);

			// если эти условия выполняются, то вектор w отложенный из центра окружности-основания VO будет своим концом ближе к
			// этой самой окружности, а значит и ближайшая точка на границе VO -- это какая-то точка на этой окружнрости

			if (wproj < 0.0f && (wproj * wproj) > sqwlength * radiussum2) {
				const float wlength = std::sqrt(sqwlength);
				const Vector nw = w / wlength;
				currline.dir = Vector(nw.Y(), -nw.X());
				u = nw * (radiussum * invTimeBoundary - wlength);
			}
			else {
				//иначе проекция на стороны VO
				//длина проекции вектора относительных положений на сторону VO

				float leg = std::sqrt(distSq - radiussum2);

				if (circlecenter.Det(w) > 0.0f) //если точка ближе к левой стороне VO
				{
					currline.dir = Vector(circlecenter.X() * leg - circlecenter.Y() * radiussum,
										  circlecenter.X() * radiussum + circlecenter.Y() * leg) / distSq;
				}
				else //если точка ближе к правой стороне VO
				{
					currline.dir = -(Vector(circlecenter.X() * leg + circlecenter.Y() * radiussum,
											-circlecenter.X() * radiussum + circlecenter.Y() * leg) / distSq);
				}

				float rvproj = relvelocity.ScalarProduct(currline.dir);

				u = currline.dir * rvproj - relvelocity;
			}
		}
		else {
			const float invTimeStep = 1.0f / options->timestep;

			Vector w = relvelocity - circlecenter * invTimeStep;
			float wlength = w.EuclideanNorm();
			Vector wn = w / wlength;
			currline.dir = Vector(wn.Y(), -wn.X());
			u = wn * (radiussum * invTimeStep - wlength);
		}

		currline.liesOn = this->currV + u * 0.5f;
		ORCALines.push_back(currline);
		Neighbours.pop_back();
	}

	auto lineFail = Utils::linearProgram2(ORCALines, param.maxSpeed, this->prefV, false, this->newV);
	if (lineFail < this->ORCALines.size()) {
		Utils::linearProgram3(ORCALines, numObstLines, lineFail, param.maxSpeed, this->newV);
	}

	Neighbours.clear();
}


void AgentGoalExchange::applyNewControl() {

	currV = newV;
	speedSaveBuffer.pop_front();
	speedSaveBuffer.push_back(currV.EuclideanNorm());

	// Kahan summation algorithm
	float sum = 0.0f;
	float c = 0.0f;
	float y, t;
	for (auto speed: speedSaveBuffer) {
		y = speed - c;
		t = sum + y;
		c = (t - sum) - y;
		sum = t;
	}
	meanSavedSpeed = sum / speedSaveBuffer.size();
}


bool AgentGoalExchange::prepareBeforeStep() {

//	if (id == 3) {
//		std::cout << "id " << id << " goals: ";
//		for (auto &g: goals) {
//			std::cout << g.ToString() << " ";
//		}
//		std::cout << std::endl;
//	}

	Vector global_goal_vector = goal - position;
	float global_goal_dist = global_goal_vector.EuclideanNorm();

	if (global_goal_dist < options->delta) {
		finished = true;
	}

	bool goal_exchanged_now = tryExchangeGoal();
	if (goal_was_exchanged or goal_exchanged_now) {
		exchange_count++;
		goal_was_exchanged = false;
		planner->changeGlobalGoal(goal);
		if (not planner->CreateGlobalPath()) {
			return false;
		}
	}

	Point next;
	if (not planner->GetNext(position, next)) {
		nextForLog = position;
		prefV = Point();
		return false;
	}

	nextForLog = goal;
	Vector goalVector = next - position;
	float dist = goalVector.EuclideanNorm();


	if ((options->trigger == MAPFTriggers::COMMON_POINT && CommonPointMAPFTrigger(dist)) ||
		(options->trigger == MAPFTriggers::SPEED_BUFFER && SingleNeighbourMeanSpeedMAPFTrigger())) {
		sendGroupRequestForExchanging();
	}

	if (next == goal && dist < options->delta) {
		prefV = Point();
		return true;
	}

	if (dist > CN_EPS) {
		goalVector = (goalVector / dist) * param.maxSpeed;
	}

	prefV = goalVector;
//	std::random_device rd{};
//	std::mt19937 gen{rd()};
//
//	std::normal_distribution<> d{0.0, 0.03};
//
//	float rand_x = d(gen);
//	float rand_y = d(gen);
//
//	prefV = prefV + Point(rand_x, rand_y);
	return true;
}


void AgentGoalExchange::addNeighbour(Agent &neighbour, float dist_sq) {
	float sightSq = param.sightRadius * param.sightRadius;

	if (!(dist_sq < sightSq)) {
		return;
	}

	int i = 0;
	auto tmpit = Neighbours.begin();

	while (tmpit != Neighbours.end() && i < param.agentsMaxNum && Neighbours[i].first < dist_sq) {
		i++;
		tmpit++;
	}

	if (i < param.agentsMaxNum) {
		Neighbours.insert(tmpit, std::pair<float, Agent *>(dist_sq, &neighbour));
	}
}


std::vector<std::pair<float, Agent *>> &AgentGoalExchange::requestNeighbours() {
	return Neighbours;
}


void AgentGoalExchange::sendGroupRequestForExchanging() {


//	std::set<AgentGoalExchange *> close_agents = createGroup();
//
//
//
//
//	//close_agents.insert(this);
//	auto buffSize = speedSaveBuffer.size();
//	speedSaveBuffer = std::list<float>(buffSize, param.maxSpeed);
//
//
//
////    std::uniform_int_distribution<int> uid(0, close_agents.size() - 1);
////    auto a = uid(gen);
////    int i = 0;
//	for (auto &el: close_agents) {
//		auto buffSize = speedSaveBuffer.size();
//		el->speedSaveBuffer = std::list<float>(buffSize, param.maxSpeed);
////        auto a = uid(gen);
////        std::cout << a << "\n";
////        if(i != )
////        {
//		el->ReturnTosPastPoint();
////        }
////        i++;
//	}
}


void AgentGoalExchange::setGoalList(const std::vector<Point> &goal_list) {
	goals = std::unordered_set<Point>(goal_list.begin(), goal_list.end());
}

bool AgentGoalExchange::initAgent() {

	if (goals.empty()) return false;

	auto closest_point = [this](const Point &a, const Point &b) {
		return (a - this->position).SquaredEuclideanNorm() < (b - this->position).SquaredEuclideanNorm();
	};

	goal = *std::min_element(goals.begin(), goals.end(), closest_point);
	planner->changeGlobalGoal(goal);
	return Agent::initAgent();
}

std::set<AgentGoalExchange *> AgentGoalExchange::createGroup() {

	std::set<AgentGoalExchange *> close_agents;
	for (auto &n_dist_agent: Neighbours) {
		auto neighbour = dynamic_cast<AgentGoalExchange *>(n_dist_agent.second);
		close_agents.insert(neighbour);
		for (auto &nn_dist_agent: neighbour->requestNeighbours()) {
			auto neighbour_of_neighbour = dynamic_cast<AgentGoalExchange *>(nn_dist_agent.second);
			close_agents.insert(neighbour_of_neighbour);
		}
	}
	return close_agents;
}

auto AgentGoalExchange::requestJoinToGroup(bool transmit_to_neighbours) -> std::tuple<std::unordered_set<AgentGoalExchange *>, std::unordered_set<Point>>
{
	std::unordered_set<AgentGoalExchange *> close_agents;
	std::unordered_set<Point> group_goals = {goal};
	for (auto &n_dist_agent: Neighbours) {
		auto neighbour = dynamic_cast<AgentGoalExchange *>(n_dist_agent.second);
		close_agents.insert(neighbour);
	}

	if (not transmit_to_neighbours) {
		return {close_agents, group_goals};
	}

	for (auto &close_agent : close_agents) {
		std::unordered_set<AgentGoalExchange *> new_close_agents;
		std::unordered_set<Point> new_group_goals;
		std::tie(new_close_agents, new_group_goals) = close_agent->requestJoinToGroup(false);
		close_agents.insert(new_close_agents.begin(), new_close_agents.end());
		group_goals.insert(new_group_goals.begin(), new_group_goals.end());
	}

	return {close_agents, group_goals};
}

std::tuple<bool, Point, bool> AgentGoalExchange::requestForExchanging(Point other_goal, Point other_position, bool other_finished) {

	auto closest_point = [this](const Point &a, const Point &b) {
		return (a - this->position).SquaredEuclideanNorm() < (b - this->position).SquaredEuclideanNorm();
	};


	double my_dist_sq_my_goal = (position - goal).SquaredEuclideanNorm();
	double other_dist_sq_my_goal = (other_position - goal).SquaredEuclideanNorm();
	if (goal == other_goal) {

		if (finished and not other_finished) {
			return {true, other_goal, true};
		}


		if ((finished or not other_finished) and my_dist_sq_my_goal < other_dist_sq_my_goal) {
			return {true, other_goal, false};
		}

		auto old_goal = goal;


		if (finished and other_finished) {
			goals.erase(goal);
			finished = false;
		}

		goals.erase(goal);
		goal = *std::min_element(goals.begin(), goals.end(), closest_point);
		goal_was_exchanged = true;



		if (not other_finished and other_dist_sq_my_goal >= options->delta * options->delta) {
			goals.insert(old_goal);
		}
		return {false, old_goal, false};
	}

	if (other_finished or finished) {
		return {false, other_goal, false};
	}

	double my_dist_sq_other_goal = (position - other_goal).SquaredEuclideanNorm();
	double other_dist_sq_other_goal = (other_position - other_goal).SquaredEuclideanNorm();

	if ((my_dist_sq_other_goal < my_dist_sq_my_goal and other_dist_sq_my_goal < other_dist_sq_other_goal) or
		(my_dist_sq_other_goal < other_dist_sq_other_goal and other_dist_sq_my_goal < my_dist_sq_my_goal)) {
		auto old_goal = goal;

		if (goals.find(other_goal) != goals.end()) {
			goal = other_goal;
		}
		else {
			goals.erase(old_goal);
			goal = *std::min_element(goals.begin(), goals.end(), closest_point);
			goals.insert(old_goal);
		}

		goal_was_exchanged = true;

		return {true, old_goal, true};
	}
	return {false, other_goal, false};
}

bool AgentGoalExchange::tryExchangeGoal() {

	Point new_goal = goal;
	bool was_exchanged = false;
	bool other_finished = false;
	for (auto &n_dist_agent: Neighbours) {
		auto neighbour = dynamic_cast<AgentGoalExchange *>(n_dist_agent.second);
		bool exchange;
		Point tmp_goal;
		messages_count++;



		std::tie(exchange, tmp_goal, other_finished) = neighbour->requestForExchanging(new_goal, position, finished);
		was_exchanged = was_exchanged || exchange;

		if (exchange) {
			if (tmp_goal != new_goal and goals.find(tmp_goal) != goals.end()) {
				new_goal = tmp_goal;
			}
			else {
				if (tmp_goal == new_goal and other_finished) {
						goals.erase(new_goal);
						finished = false;
				}


				auto closest_point = [this](const Point &a, const Point &b) {
					return (a - this->position).SquaredEuclideanNorm() < (b - this->position).SquaredEuclideanNorm();
				};

				new_goal = *std::min_element(goals.begin(), goals.end(), closest_point);
			}
		}
	}

	goal = new_goal;
	return was_exchanged;
}

std::tuple<size_t, size_t> AgentGoalExchange::getAMAPFStat() const {
	return {messages_count, exchange_count};
}
