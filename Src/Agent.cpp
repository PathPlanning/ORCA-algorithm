#include "Agent.h"


bool linearProgram1(const std::vector<Line> &lines, unsigned long int curr, float radius, const Vector &optVelocity,
                    bool directionOpt, Vector &result);

unsigned long int
linearProgram2(const std::vector<Line> &lines, float radius, const Vector &optVelocity, bool directionOpt,
               Vector &result);

void
linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, float radius, Vector &result);


bool Compare( std::pair<float, Agent *> a, std::pair<float, Agent *> b)
{
    return (a.first > b.first);
}


bool CompareObst( std::pair<float, ObstacleSegment> a, std::pair<float, ObstacleSegment> b)
{
    return (a.first < b.first);
}


float SqPointSegDistance(Point L1, Point L2, Point P)
{
    auto v = L2 - L1;
    auto w = P - L1;
    float c1, c2;
    if ( ( c1 = w.ScalarProduct(v)) <= 0.0f)
    {
        return (P - L1).SquaredEuclideanNorm();
    }
    if ( (c2 = v.ScalarProduct(v)) <= c1 )
    {
        return (P - L2).SquaredEuclideanNorm();
    }

    float b = c1 / c2;
    Point Pb = L1 + v * b;
    return (P - Pb).SquaredEuclideanNorm();


}


void Agent::ComputeNewVelocity()
{
    ORCALines.clear();


    // Получение ORCA-линий препятсвий
    for (int i = 0; i < NeighboursObst.size(); i++)
    {
        Line line;

        Vertex *left = &(NeighboursObst[i].second.left);
        Vertex *right = &(NeighboursObst[i].second.right);

        Vector lRelativePosition = *left - position;
        Vector rRelativePosition = *right - position;


        bool alreadyCovered = false;

        for (int j = 0; j < ORCALines.size(); j++)
        {
            if ((lRelativePosition * invTimeBoundaryObst - ORCALines[j].liesOn).Det(ORCALines[j].dir) - invTimeBoundaryObst * param.radius >= -CN_EPS &&
                (rRelativePosition * invTimeBoundaryObst - ORCALines[j].liesOn).Det(ORCALines[j].dir) - invTimeBoundaryObst * param.radius >= -CN_EPS)
            {
                alreadyCovered = true;
                break;
            }
        }
        if (alreadyCovered)
            continue;

        float lSqDist = lRelativePosition.SquaredEuclideanNorm();
        float rSqDist = rRelativePosition.SquaredEuclideanNorm();

        float sqRadius = param.radius * param.radius;

        Vector obstacleVector = *right - *left;
        float s = -lRelativePosition.ScalarProduct(obstacleVector) / obstacleVector.SquaredEuclideanNorm();
        float lineSqDist = (-lRelativePosition - obstacleVector * s).SquaredEuclideanNorm();



        if (s < 0.0f && lSqDist <= sqRadius)
        {

            if (left->IsConvex())
            {
                line.liesOn = Point();
                line.dir = Point(-lRelativePosition.Y(), lRelativePosition.X()) / sqrt(lSqDist); // Построение единичного вектора, нормального к относительному положению
                ORCALines.push_back(line);
                collisionsObst++;
            }

            continue;
        }
        else if (s > 1.0f && rSqDist <= sqRadius)
        {

            if (right->IsConvex() && rRelativePosition.Det(NeighboursObst[i].second.next->dir) >= 0.0f)
            {
                line.liesOn = Point();
                line.dir = Point(-rRelativePosition.Y(), rRelativePosition.X()) / sqrt(rSqDist);
                ORCALines.push_back(line);
                collisionsObst++;
            }

            continue;
        }
        else if (s >= 0.0f && s < 1.0f && lineSqDist <= sqRadius)
        {
            line.liesOn = Point();
            line.dir = -(NeighboursObst[i].second.dir);
            ORCALines.push_back(line);
            collisionsObst++;
            continue;
        }


        Vector lLegDirection, rLegDirection;

        if (s < 0.0f && lineSqDist <= sqRadius)
        {

            if (!left->IsConvex())
            {
                continue;
            }

            right = left;

            float leg1 = sqrt(lSqDist - sqRadius);

            lLegDirection = Point(lRelativePosition.X() * leg1 - lRelativePosition.Y() * param.radius, lRelativePosition.X() * param.radius + lRelativePosition.Y() * leg1) / lSqDist;
            rLegDirection = Point(lRelativePosition.X() * leg1 + lRelativePosition.Y() * param.radius, -lRelativePosition.X() * param.radius + lRelativePosition.Y() * leg1) / lSqDist;
        }
        else if (s > 1.0f && lineSqDist <= sqRadius)
        {

            if (!right->IsConvex())
            {
                continue;
            }

            left = right;

            float leg2 = std::sqrt(rSqDist - sqRadius);
            lLegDirection = Point(rRelativePosition.X() * leg2 - rRelativePosition.Y() * param.radius, rRelativePosition.X() * param.radius + rRelativePosition.Y() * leg2) / rSqDist;
            rLegDirection = Point(rRelativePosition.X() * leg2 + rRelativePosition.Y() * param.radius, -rRelativePosition.X() * param.radius + rRelativePosition.Y() * leg2) / rSqDist;
        }
        else
        {
            if (left->IsConvex())
            {
                float leg1 = std::sqrt(lSqDist - sqRadius);
                lLegDirection = Point(lRelativePosition.X() * leg1 - lRelativePosition.Y() * param.radius, lRelativePosition.X() * param.radius + lRelativePosition.Y() * leg1) / lSqDist;
            }
            else
            {
                lLegDirection = -NeighboursObst[i].second.dir;
            }

            if (right->IsConvex())
            {
                float leg2 = std::sqrt(rSqDist - sqRadius);
                rLegDirection = Point(rRelativePosition.X() * leg2 + rRelativePosition.Y() * param.radius, -rRelativePosition.X() * param.radius + rRelativePosition.Y() * leg2) / rSqDist;
            }
            else
            {
                rLegDirection = NeighboursObst[i].second.dir;
            }
        }

        ObstacleSegment *leftNeighbor = NeighboursObst[i].second.prev;

        bool isLLegForeign = false, isRLegForeign = false;

        if (left->IsConvex() && lLegDirection.Det(-leftNeighbor->dir) >= 0.0f)
        {
            lLegDirection = -leftNeighbor->dir;
            isLLegForeign = true;
        }

        if (right->IsConvex() && rLegDirection.Det(NeighboursObst[i].second.next->dir) <= 0.0f)
        {
            rLegDirection = NeighboursObst[i].second.next->dir;
            isRLegForeign = true;
        }

        Point leftCutoff = (*left - position) * invTimeBoundaryObst;
        Point rightCutoff = (*right - position) * invTimeBoundaryObst;
        Vector cutoffVec = rightCutoff - leftCutoff;

        const float t = (right == left ? 0.5f : ((currV - leftCutoff).ScalarProduct(cutoffVec)) / cutoffVec.SquaredEuclideanNorm());
        const float tLeft = ((currV - leftCutoff).ScalarProduct(lLegDirection));
        const float tRight = ((currV - rightCutoff).ScalarProduct(rLegDirection));

        if ((t < 0.0f && tLeft < 0.0f) || (left == right && tLeft < 0.0f && tRight < 0.0f))
        {
            Vector unitW = (currV - leftCutoff)/(currV - leftCutoff).EuclideanNorm();

            line.dir = Vector(unitW.Y(), -unitW.X());
            line.liesOn = leftCutoff + unitW * param.radius *  invTimeBoundaryObst;
            ORCALines.push_back(line);
            continue;
        }
        else if (t > 1.0f && tRight < 0.0f)
        {
            Vector unitW = (currV - rightCutoff)/(currV - rightCutoff).EuclideanNorm();

            line.dir = Vector(unitW.Y(), -unitW.X());
            line.liesOn = rightCutoff + unitW * param.radius * invTimeBoundaryObst ;
            ORCALines.push_back(line);
            continue;
        }

        float cutoffSqDist = ((t < 0.0f || t > 1.0f || right == left) ? std::numeric_limits<float>::infinity() : (currV - (leftCutoff + cutoffVec * t)).SquaredEuclideanNorm());
        float lLegSqDist = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : (currV - (leftCutoff + lLegDirection * tLeft)).SquaredEuclideanNorm());
        float rLegSqDist = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : (currV - (rightCutoff + rLegDirection * tRight)).SquaredEuclideanNorm());

        if (cutoffSqDist <= lLegSqDist && cutoffSqDist <= rLegSqDist)
        {
            line.dir = -NeighboursObst[i].second.dir;
            line.liesOn = leftCutoff + Point(-line.dir.Y(), line.dir.X()) * param.radius * invTimeBoundaryObst;
            ORCALines.push_back(line);
            continue;
        }
        else if (lLegSqDist <= rLegSqDist)
        {
            if (isLLegForeign)
            {
                continue;
            }

            line.dir = lLegDirection;
            line.liesOn = leftCutoff + Point(-line.dir.Y(), line.dir.X()) * param.radius * invTimeBoundaryObst;;
            ORCALines.push_back(line);
            continue;
        }
        else
        {
            if (isRLegForeign)
            {

                continue;
            }

            line.dir = -rLegDirection;
            line.liesOn = rightCutoff + Point(-line.dir.Y(), line.dir.X()) * param.radius * invTimeBoundaryObst;
            ORCALines.push_back(line);
            continue;
        }
    }


    size_t numObstLines = ORCALines.size();

    //Получение ORCA-линий агентов
    //std::sort(Neighbours.begin(),Neighbours.end(), Compare);

    Line currline;
    Agent curragent;
    Vector u, w;

    unsigned long minMaxNum = (param.agentsMaxNum < Neighbours.size()) ? param.agentsMaxNum : Neighbours.size();

    for(unsigned long i = 0; i < minMaxNum; i++)
    {
        auto Neighbour = Neighbours[i];
        curragent = *Neighbour.second;
        auto circlecenter = curragent.position - this->position; //(P_b - P_a)
        auto relvelocity = this->currV - curragent.currV; //(V_a - V_b)

        float radiussum = param.radius + curragent.param.radius; //(R_a + R_b)
        float radiussum2 = radiussum * radiussum;
        float distSq = circlecenter.SquaredEuclideanNorm();

        if(distSq > radiussum2)
        {
            w = relvelocity - (circlecenter * invTimeBoundary); //w -- вектор на плоскости скоростей от центра малой окружности (основания VO) до скорости другого агента относительно этого
            float sqwlength = w.SquaredEuclideanNorm();
            float wproj = w.ScalarProduct(circlecenter);

            // если эти условия выполняются, то вектор w отложенный из центра окружности-основания VO будет своим концом ближе к
            // этой самой окружности, а значит и ближайшая точка на границе VO -- это какая-то точка на этой окружнрости

            if(wproj < 0.0f && (wproj * wproj) > sqwlength * radiussum2)
            {
                const float wlength = std::sqrt(sqwlength);
                const Vector nw = w / wlength;
                currline.dir = Vector(nw.Y(), -nw.X());
                u = nw * (radiussum * invTimeBoundary - wlength);
            }
            else
            {
                //иначе проекция на стороны VO
                //длина проекции вектора относительных положений на сторону VO

                float leg = std::sqrt(distSq - radiussum2);

                if(circlecenter.Det(w) > 0.0f) //если точка ближе к левой стороне VO
                {
                    currline.dir = Vector(circlecenter.X() * leg - circlecenter.Y() * radiussum, circlecenter.X() * radiussum + circlecenter.Y() * leg) / distSq;
                }
                else //если точка ближе к правой стороне VO
                {
                    currline.dir = -(Vector(circlecenter.X() * leg + circlecenter.Y() * radiussum, -circlecenter.X() * radiussum + circlecenter.Y() * leg) / distSq);
                }

                float rvproj = relvelocity.ScalarProduct(currline.dir);

                u = currline.dir * rvproj - relvelocity;
            }
        }
        else
        {
            collisions++;
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

    auto lineFail = linearProgram2(ORCALines, param.maxSpeed, this->prefV, false, this->newV);
    if(lineFail < this->ORCALines.size())
    {
        linearProgram3(ORCALines, numObstLines, lineFail, param.maxSpeed, this->newV);
    }

    Neighbours.clear();
}


Agent::Agent()
{
    id = -1;
    start = Point();
    goal = Point();
    planner = nullptr;
    options = nullptr;
    map = nullptr;
    Neighbours = std::vector <std::pair<float, Agent*>>();
    NeighboursObst = std::vector <std::pair<float, ObstacleSegment>>();
    ORCALines = std::vector <Line>();
    position = Point();
    prefV = Point();
    newV = Point();
    currV = Point();
    param = AgentParam();
    invTimeBoundaryObst = 0;
    invTimeBoundary = 0;
    collisions = 0;
    collisionsObst = 0;
    maxSqObstDist = 0;
}


Agent::Agent(const int &id, const Point &start, const Point &goal, const Map &map, const EnvironmentOptions &options, AgentParam param)
{
    this->id = id;
    this->start = start;
    this->goal = goal;
    this->map = &map;
    this->options = &options;
    this->param = param;

    Neighbours = std::vector <std::pair<float, Agent*>>();
    Neighbours.reserve((unsigned int)param.agentsMaxNum);
    NeighboursObst = std::vector <std::pair<float, ObstacleSegment>>();
    ORCALines = std::vector <Line>();
    ORCALines.reserve((unsigned int)param.agentsMaxNum);
    position = start;
    prefV = Point();
    newV = Point();
    currV = Point();
    invTimeBoundaryObst = 1/param.timeBoundaryObst;
    invTimeBoundary = 1/param.timeBoundary;
    collisions = 0;
    collisionsObst = 0;
    maxSqObstDist = std::pow((param.timeBoundaryObst * param.maxSpeed + param.radius), 2.0f);

}


Agent::Agent(const Agent &obj)
{
    this->id = obj.id;
    this->start = obj.start;
    this->goal = obj.goal;
    this->map = obj.map;
    this->options = obj.options;
    this->param = obj.param;

    if(obj.planner != nullptr)
    {
        this->planner = obj.planner->Clone();
    }

    Neighbours = obj.Neighbours;
    NeighboursObst = obj.NeighboursObst;
    ORCALines = obj.ORCALines;
    position = obj.position;
    prefV = obj.prefV;
    newV = obj.newV;
    currV = obj.currV;
    invTimeBoundaryObst = obj.invTimeBoundaryObst;
    invTimeBoundary = obj.invTimeBoundary;
    collisions = obj.collisions;
    collisionsObst = obj.collisionsObst;
    maxSqObstDist = obj.maxSqObstDist;
}


Agent::~Agent()
{
    if(planner != nullptr)
    {
        delete planner;
        planner = nullptr;
    }
    options= nullptr;
    map = nullptr;
    Neighbours.clear();
}


bool Agent::isFinished()
{
    return ((this->position - this->goal).EuclideanNorm() < options->delta);
}


bool Agent::operator == (const Agent &another) const
{
    return this->id == another.id;
}


bool Agent::operator != (const Agent &another) const
{
    return this->id != another.id;
}


void Agent::AddNeighbour(Agent &neighbour, float distSq)
{
    int i = 0;
    auto tmpit = Neighbours.begin();
    while(tmpit != Neighbours.end() && i < param.agentsMaxNum && Neighbours[i].first < distSq)
    {
        i++;
        tmpit++;
    }
    if(i < param.agentsMaxNum)
    {
        Neighbours.insert(tmpit,std::pair<float, Agent *>(distSq, &neighbour));
    }
    //Neighbours.emplace_back(std::pair<float, Agent *>(distSq, &neighbour));
}



void Agent::ApplyNewVelocity()
{
    currV = newV;
}


void Agent::SetPosition(const Point &pos)
{
    position = pos;
}


Point Agent::GetPosition() const
{
    return position;
}


bool Agent::UpdatePrefVelocity()
{
    Point next;
    if (planner->GetNext(position, next))
    {
        Vector goalVector = next - position;
        float dist = goalVector.EuclideanNorm();
        if(next == goal && dist < options->delta)
        {
            prefV = Point();
            return true;
        }

        if(dist > CN_EPS)
        {
            goalVector = (goalVector/dist) * param.maxSpeed;
        }

        prefV = goalVector;
        return true;
    }
    prefV = Point();
    return false;
}


float Agent::GetSightRadius() const
{
    return param.sightRadius;
}


void Agent::UpdateNeighbourObst()
{
    NeighboursObst.clear();
    std::vector<std::vector<ObstacleSegment>> tmpObstacles = map->GetObstacles();
    float distSq = 0;

    for(int i = 0; i < tmpObstacles.size(); i++)
    {
        for(int j = 0; j < tmpObstacles[i].size(); j++)
        {
            distSq = SqPointSegDistance(tmpObstacles[i][j].left, tmpObstacles[i][j].right, position);
            if(distSq < maxSqObstDist)
            {
                NeighboursObst.push_back({distSq, tmpObstacles[i][j]});
            }
        }
    }

    std::sort(NeighboursObst.begin(), NeighboursObst.end(), CompareObst);

}


Point Agent::GetVelocity() const
{
    return currV;
}


std::pair<unsigned int, unsigned int> Agent::GetCollision() const
{
    return {collisions, collisionsObst};
}


bool Agent::InitPath()
{
    return planner->CreateGlobalPath();
}


int Agent::GetID() const
{
    return id;
}

float Agent::GetRadius() const
{
    return param.radius;
}


Agent &Agent::operator = (const Agent &obj)
{

    if(this != &obj)
    {
        start = obj.start;
        goal = obj.goal;
        id = obj.id;
        position = obj.position;
        prefV = obj.prefV;
        newV = obj.newV;
        currV = obj.currV;
        param = obj.param;
        invTimeBoundaryObst = obj.invTimeBoundaryObst;
        invTimeBoundary = obj.invTimeBoundary;
        maxSqObstDist = obj.maxSqObstDist;
        collisions = obj.collisions;
        collisionsObst = obj.collisionsObst;
        ORCALines = obj.ORCALines;
        NeighboursObst = obj.NeighboursObst;
        Neighbours = obj.Neighbours;
        options = obj.options;
        map = obj.map;
        if(planner != nullptr)
        {
            delete planner;
            planner = nullptr;
        }
        planner = (obj.planner == nullptr) ? nullptr : obj.planner->Clone();;
    }
    return *this;
}

float Agent::GetDistToGoal() const
{
    return (position - goal).EuclideanNorm();
}


bool linearProgram1(const std::vector<Line> &lines, unsigned long curr, float radius, const Vector &optVelocity,
                    bool directionOpt, Vector &result)
{
    float dotProduct = lines[curr].liesOn.ScalarProduct(lines[curr].dir);
    float discriminant = dotProduct * dotProduct + radius * radius - lines[curr].liesOn.SquaredEuclideanNorm();

    if(discriminant < 0.0f)
    {
        // Максимальная скорость не позволяет удовлетворить это условие
        return false;
    }

    float sqrtDiscriminant = std::sqrt(discriminant);
    float tLeft = -dotProduct - sqrtDiscriminant;
    float tRight = -dotProduct + sqrtDiscriminant;

    for(int i = 0; i < curr; ++i)
    {

        const float denominator = lines[curr].dir.Det(lines[i].dir);
        const float numerator = lines[i].dir.Det(lines[curr].liesOn - lines[i].liesOn);

        if(std::fabs(denominator) <= CN_EPS)
        {
            // Текущая и сравниваемая линии параллельны
            if(numerator < 0.0f)
            {
                return false;
            }
            else
            {
                continue;
            }
        }

        const float t = numerator / denominator;

        if(denominator >= 0.0f)
        {
            /* Line i bounds line lineNo on the right. */
            tRight = std::min(tRight, t);
        }
        else
        {
            /* Line i bounds line lineNo on the left. */
            tLeft = std::max(tLeft, t);
        }

        if(tLeft > tRight)
        {
            return false;
        }
    }

    if(directionOpt)
    {
        /* Optimize direction. */
        if(optVelocity.ScalarProduct(lines[curr].dir) > 0.0f)
        {
            /* Take right extreme. */
            result = lines[curr].liesOn + lines[curr].dir * tRight;
        }
        else
        {
            /* Take left extreme. */
            result = lines[curr].liesOn + lines[curr].dir * tLeft;
        }
    }
    else
    {
        /* Optimize closest point. */
        const float t = lines[curr].dir.ScalarProduct(optVelocity - lines[curr].liesOn);

        if(t < tLeft)
        {
            result = lines[curr].liesOn + lines[curr].dir * tLeft;
        }
        else if(t > tRight)
        {
            result = lines[curr].liesOn + lines[curr].dir * tRight;
        }
        else
        {
            result = lines[curr].liesOn + lines[curr].dir * t;
        }
    }

    return true;
}


unsigned long int
linearProgram2(const std::vector<Line> &lines, float radius, const Vector &optVelocity, bool directionOpt,
               Vector &result)
{
    if(directionOpt)
    {
        result = optVelocity * radius;
    }
    else if(optVelocity.SquaredEuclideanNorm() > radius * radius)
    {
        result = (optVelocity / optVelocity.EuclideanNorm()) * radius;
    }
    else
    {
        result = optVelocity;
    }

    for(unsigned long int i = 0; i < lines.size(); ++i)
    {

        if(lines[i].dir.Det(lines[i].liesOn - result) > 0.0f)
        {
            const Vector tempResult = result;

            if(!linearProgram1(lines, i, radius, optVelocity, directionOpt, result))
            {
                result = tempResult;
                return i;
            }
        }
    }

    return lines.size();
}


void
linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, float radius, Vector &result)
{
    float distance = 0.0f;

    for(size_t i = beginLine; i < lines.size(); ++i)
    {
        if(lines[i].dir.Det(lines[i].liesOn - result) > distance)
        {
            /* Result does not satisfy constraint of line i. */
            std::vector<Line> projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));

            for(size_t j = numObstLines; j < i; ++j)
            {
                Line line;

                float determinant = lines[i].dir.Det(lines[j].dir);

                if(std::fabs(determinant) <= CN_EPS)
                {
                    /* Line i and line j are parallel. */
                    if(lines[i].dir.ScalarProduct(lines[j].dir) > 0.0f)
                    {
                        /* Line i and line j point in the same direction. */
                        continue;
                    }
                    else
                    {
                        /* Line i and line j point in opposite direction. */
                        line.liesOn = (lines[i].liesOn + lines[j].liesOn) * 0.5f;
                    }
                }
                else
                {
                    line.liesOn = lines[i].liesOn + lines[i].dir * (lines[j].dir.Det(lines[i].liesOn - lines[j].liesOn) / determinant);
                }


                line.dir = (lines[j].dir - lines[i].dir);
                line.dir = line.dir / line.dir.EuclideanNorm();
                projLines.push_back(line);
            }

            const Vector tempResult = result;

            if(linearProgram2(projLines, radius, Vector(-lines[i].dir.Y(), lines[i].dir.X()), true, result) < projLines.size())
            {
                result = tempResult;
            }

            distance = lines[i].dir.Det(lines[i].liesOn - result);
        }
    }
}