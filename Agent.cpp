#include "Agent.h"
#include <iostream>


bool linearProgram1(const std::vector<Line> &lines, unsigned long int curr, float radius, const Vector &optVelocity,
                    bool directionOpt, Vector &result);

unsigned long int
linearProgram2(const std::vector<Line> &lines, float radius, const Vector &optVelocity, bool directionOpt,
               Vector &result);

void
linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, float radius, Vector &result);


    bool Compare(pair<float, Agent *> a, pair<float, Agent *> b)
    {
        return (a.first > b.first);
    }




Agent::Agent()
{
    radius = 1;
    maxSpeed = 10;
    timeBoundary = 15;
    agentsMaxNum = std::numeric_limits<int>::max();
    sightRadius = std::numeric_limits<float>::max();
    Neighbours =  vector <pair<float, Agent*>>();
    ORCALines = vector<Line>();
    position = Point(radius, radius);
    prefV = Point();
    newV = Point();
    currV = Point();

}


Agent::Agent(float radius, float maxspeed, int agentsmaxnum, float timeboundary, float sightradius, int id)
{
    this->radius = radius;
    this->maxSpeed = maxspeed;
    this->agentsMaxNum = agentsmaxnum;
    this->timeBoundary = timeboundary;
    this->sightRadius = sightradius;
    this->id = id;

    Neighbours =  vector <pair<float, Agent*>>();
    ORCALines = vector<Line>();
    position = Point(radius, radius);
    prefV = Point();
    newV = Point();
    currV = Point();
    count = 0;

}


Agent::Agent(const Agent &obj)
{
    radius = obj.radius;
    maxSpeed = obj.maxSpeed;
    timeBoundary = obj.timeBoundary;
    agentsMaxNum = obj.agentsMaxNum;
    Neighbours = obj.Neighbours;
    ORCALines = obj.ORCALines;
    position = obj.position;
    prefV = obj.prefV;
    newV = obj.newV;
    currV = obj.currV;
    sightRadius = obj.sightRadius;
    id = obj.id;
}

void Agent::CalculateVelocity()
{
    //TODO препятствия

    //Получение ORCA-линий агентов
    std::make_heap(Neighbours.begin(),Neighbours.end(), Compare);
    ORCALines.clear();
    Line currline;
    Agent curragent;
    Vector u, w;
    const float invtimeBoundary = 1.0f / timeBoundary;
    int minMaxNum = (agentsMaxNum < Neighbours.size()) ? agentsMaxNum : Neighbours.size();

    for(int i = 0; i < minMaxNum; i++)
    {
        auto Neighbour = Neighbours.front();
        pop_heap(Neighbours.begin(), Neighbours.end(), Compare);
        curragent = *Neighbour.second;
        auto circlecenter = curragent.position - this->position; //(P_b - P_a)
        auto relvelocity = this->currV - curragent.currV; //(V_a - V_b)

        float radiussum = this->radius + curragent.radius; //(R_a + R_b)
        float radiussum2 = radiussum * radiussum;
        float distSq = circlecenter.SquaredEuclideanNorm();

        if(distSq > radiussum2)
        {
            w = relvelocity - (circlecenter * invtimeBoundary); //w -- вектор на плоскости скоростей от центра малой окружности (основания VO) до скорости другого агента относительно этого
            float sqwlength = w.SquaredEuclideanNorm();
            float wproj = w.ScalarProduct(circlecenter);

            // если эти условия выполняются, то вектор w отложенный из центра окружности-основания VO будет своим концом ближе к
            // этой самой окружности, а значит и ближайшая точка на границе VO -- это какая-то точка на этой окружнрости

            if(wproj < 0.0f && (wproj * wproj) > sqwlength * radiussum2)
            {
                const float wlength = std::sqrt(sqwlength);
                const Vector nw = w / wlength;
                currline.dir = Vector(nw.GetY(), -nw.GetX());
                u = nw * (radiussum * invtimeBoundary - wlength);
            }
            else
            {
                //иначе проекция на стороны VO
                //длина проекции вектора относительных положений на сторону VO

                float leg = std::sqrt(distSq - radiussum2);

                if(circlecenter.Det(w) > 0.0f) //если точка ближе к левой стороне VO
                {
                    currline.dir = Vector(circlecenter.GetX() * leg - circlecenter.GetY() * radiussum, circlecenter.GetX() * radiussum + circlecenter.GetY() * leg) / distSq;
                }
                else //если точка ближе к правой стороне VO
                {
                    currline.dir = -(Vector(circlecenter.GetX() * leg + circlecenter.GetY() * radiussum, -circlecenter.GetX() * radiussum + circlecenter.GetY() * leg) / distSq);
                }

                float rvproj = relvelocity.ScalarProduct(currline.dir);

                u = currline.dir * rvproj - relvelocity;
            }
        }
        else
        {

            const float invTimeStep = 1.0f / 0.25;

            Vector w = relvelocity - circlecenter * invTimeStep;
            float wlength = w.EuclideanNorm();
            Vector wn = w / wlength;
            currline.dir = Vector(wn.GetY(), -wn.GetX());
            u = wn * (radiussum * invTimeStep - wlength);
        }

        currline.liesOn = this->currV + u * 0.5f;
        ORCALines.push_back(currline);
        Neighbours.pop_back();
    }

    auto lineFail = linearProgram2(ORCALines, this->maxSpeed, this->prefV, false, this->newV);
    if(lineFail < this->ORCALines.size())
    {
        linearProgram3(ORCALines, 0, lineFail, this->maxSpeed, this->newV);
    }
    Neighbours.clear();
}


void Agent::AddNeighbour(Agent &neighbour)
{
    float dist = (this->position - neighbour.position).SquaredEuclideanNorm();
    if(dist < sightRadius * sightRadius)
    {
        Neighbours.push_back(std::pair<float, Agent *>(dist, &neighbour));
    }
}


Point Agent::GetPosition()
{
    return position;
}


void Agent::SetPrefVelocity(const Velocity &newpref)
{
    this->prefV = newpref;
}


void Agent::SetPosition(Point pos)
{
    this->position = pos;
}


Point Agent::GetVelocity()
{
    return currV;
}


void Agent::UpdateVelocity()
{
    this->currV = this->newV;
}


float Agent::GetMaxSpeed()
{
    return maxSpeed;
}

bool Agent::operator ==(const Agent &another) const
{
    return this->id == another.id;
}

bool Agent::operator !=(const Agent &another) const
{
    return this->id != another.id;
}

int Agent::GetID()
{
    return id;
}

Point Agent::GetPrefVelocity()
{
    return prefV;
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

        if(std::fabs(denominator) <= eps)
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

                if(std::fabs(determinant) <= eps)
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

            if(linearProgram2(projLines, radius, Vector(-lines[i].dir.GetY(), lines[i].dir.GetX()), true, result) < projLines.size())
            {
                result = tempResult;
            }

            distance = lines[i].dir.Det(lines[i].liesOn - result);
        }
    }
}

