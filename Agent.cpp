
#include "Agent.h"
#include <iostream>

bool linearProgram1(const std::vector<Line> &lines, unsigned long int curr, double radius, const Vector &optVelocity, bool directionOpt, Vector &result);
unsigned long int linearProgram2(const std::vector<Line> &lines, double radius, const Vector &optVelocity, bool directionOpt, Vector &result);
void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, double radius, Vector &result);


Agent::Agent()
{
    radius = 1;
    maxSpeed = 10;
    timeBoundary = 15;
    agentsMaxNum = std::numeric_limits<int>::max();
    sightRadius = std::numeric_limits<double>::max();
    Neighbours = multimap <double, Agent*>();
    ORCALines = vector <Line>();
    position = Point(radius, radius);
    prefV = Point();
    newV = Point();
    currV = Point();
    finished = false;
}


Agent::Agent(double radius, double maxspeed, int agentsmaxnum, double timeboundary, double sightradius, int id)
{
    this->radius = radius;
    this->maxSpeed = maxspeed;
    this->agentsMaxNum = agentsmaxnum;
    this->timeBoundary = timeboundary;
    this->sightRadius = sightradius;
    this->id = id;

    Neighbours = multimap <double, Agent*>();
    ORCALines = vector <Line>();
    position = Point(radius, radius);
    prefV = Point();
    newV = Point();
    currV = Point();
    finished = false;
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
    finished = obj.finished;
    sightRadius = obj.sightRadius;
}

void Agent::CalculateVelocity()
{
    if(finished)
    {
        return;
    }

    //TODO доделывать

    //Получение ORCA-линий
    ORCALines.clear();
    Line currline;
    Agent curragent;
    Vector u, w;

    if(Neighbours.size() > agentsMaxNum)
    {
        Neighbours.erase(std::next(Neighbours.begin(), agentsMaxNum), Neighbours.end());
    }

    for(auto &Neighbour : Neighbours)
    {

        curragent = *Neighbour.second;
        auto circlecenter = curragent.position - this->position; //(P_b - P_a)
        auto relvelocity = this->currV - curragent.currV; //(V_a - V_b)
        auto radiussum = this->radius + curragent.radius; //(R_a + R_b)
        double radiussum2 = radiussum * radiussum;

        if (circlecenter.SquaredEuclideanNorm() > radiussum2)
        {
            w = relvelocity - (circlecenter / this->timeBoundary); //w -- вектор на плоскости скоростей от центра малой окружности (основания VO) до скорости другого агента относительно этого
            double sqwlength = w.SquaredEuclideanNorm();
            double wproj = w.ScalarProduct( circlecenter );


            // если эти условия выполняются, то вектор w отложенный из центра окружности-основания VO будет своим концом ближе к
            // этой самой окружности, а значит и ближайшая точка на границе VO -- это какая-то точка на этой окружнрости

            if(wproj < 0 && (wproj * wproj) > sqwlength * radiussum2)
            {
                double wlength = std::sqrt( sqwlength );
                Vector nw = w / wlength;
                currline.dir = Vector( nw.GetX(), -nw.GetY());
                u = nw * (radiussum / timeBoundary - wlength);
            }
            else
            {
                //иначе проекция на стороны VO

                //длина проекции вектора относительных положений на сторону VO
                double distsq = circlecenter.SquaredEuclideanNorm();
                double leg = std::sqrt( distsq - radiussum2 );

                if(circlecenter.Det( w ) > 0) //если точка ближе к левой стороне VO
                {
                    currline.dir = Vector( circlecenter.GetX() * leg - circlecenter.GetY() * radiussum, circlecenter.GetX() * radiussum + circlecenter.GetY() * leg ) / distsq;
                }
                else //если точка ближе к правой стороне VO
                {
                    currline.dir = (Vector( circlecenter.GetX() * leg + circlecenter.GetY() * radiussum, -circlecenter.GetX() * radiussum + circlecenter.GetY() * leg ) / distsq) * -1;
                }

                double rvproj = relvelocity.ScalarProduct( currline.dir );

                u = currline.dir * rvproj - relvelocity;
            }
        }
        else
        {
            /* Vector from cutoff center to relative velocity. */
            Vector w = relvelocity - circlecenter / timeBoundary;

            double wlength = w.EuclideanNorm();
            Vector wn = w / wlength;

            currline.dir = Vector(wn.GetY(), -wn.GetX());
            u = wn * (radiussum / timeBoundary - wlength);
        }

        currline.liesOn = this->currV + u / 2;

        ORCALines.push_back(currline);
    }
    auto lineFail = linearProgram2(ORCALines, this->maxSpeed, this->prefV, false, this->newV);

    if (lineFail < this->ORCALines.size())
    {
        linearProgram3(ORCALines, 0, lineFail, this->maxSpeed, this->newV);
    }
    Neighbours.clear();
}


void Agent::AddNeighbour(Agent &neighbour)
{
    double dist = (this->position - neighbour.position).EuclideanNorm();
    if(dist < sightRadius)
    {
        Neighbours.insert(std::pair<double, Agent*>(dist, &neighbour));
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


void Agent::Stop()
{
    this->currV = Point();
    this->prefV = Point();
    this->newV = Point();
    finished = true;
}

double Agent::GetMaxSpeed()
{
    return maxSpeed;
}

bool Agent::operator ==(const Agent &another) const
{
        return this->id == another.id;
}


bool linearProgram1(const std::vector<Line> &lines, unsigned long curr, double radius, const Vector &optVelocity, bool directionOpt, Vector &result)
{
    double dotProduct = lines[curr].liesOn.ScalarProduct(lines[curr].dir);
    double discriminant = dotProduct * dotProduct + radius * radius - lines[curr].liesOn.SquaredEuclideanNorm();

    if (discriminant < 0)
    {
        // Максимальная скорость не позволяет удовлетворить это условие
        return false;
    }

    double sqrtDiscriminant = std::sqrt(discriminant);
    double tLeft = -dotProduct - sqrtDiscriminant;
    double tRight = -dotProduct + sqrtDiscriminant;

    for (int i = 0; i < curr; ++i)
    {

        double denominator = lines[curr].dir.Det(lines[i].dir);
        double numerator = lines[i].dir.Det(lines[curr].liesOn - lines[i].liesOn);

        if (std::fabs(denominator) <= eps)
        {
            // Текущая и сравниваемая линии параллельны
            if (numerator < 0)
            {
                return false;
            }
            else
            {
                continue;
            }
        }

        double t = numerator / denominator;

        if (denominator >= 0.0f)
        {
            /* Line i bounds line lineNo on the right. */
            tRight = std::min(tRight, t);
        }
        else
        {
            /* Line i bounds line lineNo on the left. */
            tLeft = std::max(tLeft, t);
        }

        if (tLeft > tRight)
        {
            return false;
        }
    }

    if (directionOpt) {
        /* Optimize direction. */
        if (optVelocity.ScalarProduct(lines[curr].dir) > 0.0f)
        {
            /* Take right extreme. */
            result = lines[curr].liesOn + lines[curr].dir * tRight;
        }
        else {
            /* Take left extreme. */
            result = lines[curr].liesOn +   lines[curr].dir * tLeft;
        }
    }
    else
    {
        /* Optimize closest point. */
        double t = lines[curr].dir.ScalarProduct(optVelocity - lines[curr].liesOn);

        if (t < tLeft)
        {
            result = lines[curr].liesOn + lines[curr].dir * tLeft;
        }
        else if (t > tRight)
        {
            result = lines[curr].liesOn + lines[curr].dir * tRight ;
        }
        else
        {
            result = lines[curr].liesOn + lines[curr].dir * t;
        }
    }

    return true;
}

unsigned long int linearProgram2(const std::vector<Line> &lines, double radius, const Vector &optVelocity, bool directionOpt, Vector &result)
{
    if (directionOpt)
    {
        result = optVelocity * radius;
    }
    else if (optVelocity.SquaredEuclideanNorm() > radius * radius)
    {
        result = optVelocity/optVelocity.EuclideanNorm() * radius;
    }
    else
    {
        result = optVelocity;
    }

    for (unsigned long int i = 0; i < lines.size(); ++i)
    {

        if (lines[i].dir.Det(lines[i].liesOn - result) > 0)
        {
            Vector tempResult = result;

            if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result))
            {
                result = tempResult;
                return i;
            }
        }
    }

    return lines.size();
}


void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, double radius, Vector &result)
{
    double distance = 0.0f;

    for (size_t i = beginLine; i < lines.size(); ++i) {
        if (lines[i].dir.Det(lines[i].liesOn - result) > distance) {
            /* Result does not satisfy constraint of line i. */
            std::vector<Line> projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));

            for (size_t j = numObstLines; j < i; ++j) {
                Line line;

                double determinant = lines[i].dir.Det(lines[j].dir);

                if (std::fabs(determinant) <= eps) {
                    /* Line i and line j are parallel. */
                    if (lines[i].dir.ScalarProduct(lines[j].dir) > 0.0f) {
                        /* Line i and line j point in the same direction. */
                        continue;
                    }
                    else {
                        /* Line i and line j point in opposite direction. */
                        line.liesOn =  (lines[i].liesOn + lines[j].liesOn) * 0.5;
                    }
                }
                else {
                    line.liesOn = lines[i].liesOn + lines[i].dir * (lines[j].dir.Det(lines[i].liesOn - lines[j].liesOn) / determinant);
                }

                line.dir = (lines[j].dir - lines[i].dir)/(lines[j].dir - lines[i].dir).EuclideanNorm();
                projLines.push_back(line);
            }

            const Vector tempResult = result;

            if (linearProgram2(projLines, radius, Vector(-lines[i].dir.GetY(), lines[i].dir.GetX()), true, result) < projLines.size())
            {
                result = tempResult;
            }

            distance = lines[i].dir.Det(lines[i].liesOn - result);
        }
    }
}

