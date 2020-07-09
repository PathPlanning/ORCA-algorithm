#include "ORCAAgentWithPAR.h"


ORCAAgentWithPAR::ORCAAgentWithPAR() : Agent()
{
    fakeRadius = 0;
    PARAgents = std::set<ORCAAgentWithPAR *>();
    inPARMode = false;
    moveToPARPos = false;
    PARExec = false;
    PARMap = SubMap();
    conf.maxTime = CN_DEFAULT_PARMAXTIME;
    PARVis = false;
    PARExec = false;
    PARStart = Point(-1,-1);
    PARGoal = Point(-1,-1);
    buffPar = std::vector<Point>();
    PARUnion = false;
    notPARVis = false;
    PARcommon = Point(-1,-1);
}


ORCAAgentWithPAR::ORCAAgentWithPAR(const int &id, const Point &start, const Point &goal, const Map &map,
                     const EnvironmentOptions &options, AgentParam param) : Agent(id, start, goal, map, options, param)
{
    fakeRadius = param.rEps + param.radius;
    PARAgents = std::set<ORCAAgentWithPAR *>();
    inPARMode = false;
    moveToPARPos = false;
    PARVis = false;
    PARExec = false;
    PARMap = SubMap();
    conf.maxTime = CN_DEFAULT_PARMAXTIME;
    PARStart = Point(-1,-1);
    PARGoal = Point(-1,-1);
    PARsearch = Astar(false);
    PARSolver = PushAndRotate(&PARsearch);
    buffPar = std::vector<Point>();
    PARUnion = false;
    notPARVis = false;
    PARcommon = Point(-1,-1);
}


ORCAAgentWithPAR::ORCAAgentWithPAR(const ORCAAgentWithPAR &obj) : Agent(obj)
{
    fakeRadius = obj.fakeRadius;
    PARAgents = obj.PARAgents;
    inPARMode = obj.inPARMode;
    moveToPARPos = obj.moveToPARPos;
    PARMap = obj.PARMap;
    PARSet = obj.PARSet;
    conf = obj.conf;
    PARVis = obj.PARVis;
    PARExec = obj.PARExec;
    PARStart = obj.PARStart;
    PARGoal = obj.PARGoal;
    PARsearch = obj.PARsearch;
    PARSolver = obj.PARSolver;
    buffPar = obj.buffPar;
    PARUnion = obj.PARUnion;
    notPARVis = obj.notPARVis;
    PARcommon = obj.PARcommon;
    currPARPos = obj.currPARPos;

}


ORCAAgentWithPAR::~ORCAAgentWithPAR() = default;



ORCAAgentWithPAR& ORCAAgentWithPAR::operator = (const ORCAAgentWithPAR &obj)
{

    if(this != &obj)
    {
        Agent::operator=(obj);
        fakeRadius = obj.fakeRadius;
        PARAgents = obj.PARAgents;
        inPARMode = obj.inPARMode;
        moveToPARPos = obj.moveToPARPos;
        PARMap = obj.PARMap;
        PARSet = obj.PARSet;
        conf = obj.conf;
        PARVis = obj.PARVis;
        PARExec = obj.PARExec;
        PARStart = obj.PARStart;
        PARGoal = obj.PARGoal;
        PARsearch = obj.PARsearch;
        PARSolver = obj.PARSolver;
        buffPar = obj.buffPar;
        PARUnion = obj.PARUnion;
        notPARVis = obj.notPARVis;
        PARcommon = obj.PARcommon;
        currPARPos = obj.currPARPos;
    }
    return *this;
}


void ORCAAgentWithPAR::ComputeNewVelocity()
{
    if(PARExec)
    {
     // Collision count

        unsigned long minMaxNum = (param.agentsMaxNum < Neighbours.size()) ? param.agentsMaxNum : Neighbours.size();

        for(unsigned long i = 0; i < minMaxNum; i++)
        {
            auto curragent = dynamic_cast<ORCAAgentWithPAR *>(Neighbours[i].second);
            if((curragent->position - position).SquaredEuclideanNorm() <
                (param.radius + curragent->param.radius) * (param.radius + curragent->param.radius))
            {
                collisions++;
            }
        }

        for (int i = 0; i < NeighboursObst.size(); i++)
        {
            Vertex *left = &(NeighboursObst[i].second.left);
            Vertex *right = &(NeighboursObst[i].second.right);

            Vector lRelativePosition = *left - position;
            Vector rRelativePosition = *right - position;

            float lSqDist = lRelativePosition.SquaredEuclideanNorm();
            float rSqDist = rRelativePosition.SquaredEuclideanNorm();

            float sqTrueRadius = param.radius * param.radius;

            Vector obstacleVector = *right - *left;
            float s = -lRelativePosition.ScalarProduct(obstacleVector) / obstacleVector.SquaredEuclideanNorm();
            float lineSqDist = (-lRelativePosition - obstacleVector * s).SquaredEuclideanNorm();

            if ((s < 0.0f && lSqDist < sqTrueRadius) || (s > 1.0f && rSqDist < sqTrueRadius) ||
                (s >= 0.0f && s < 1.0f && lineSqDist < sqTrueRadius))
            {
                collisionsObst++;
            }
        }




        if(currPARPos < (*PARres.actorsPaths)[PARActorId].size())
        {
            auto currGoal =  PARMap.GetPoint((*PARres.actorsPaths)[PARActorId][currPARPos]);
            float dist = (currGoal - position).EuclideanNorm();
            newV = (currGoal - position);
            if(dist > 1)
            {
                newV = newV/dist;
            }
            else if(dist < 0.01)
            {
                newV = Point();
            }
        }
        else
        {
            // TODO
            newV = Point();
        }

        return;
    }




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
            if ((lRelativePosition * invTimeBoundaryObst - ORCALines[j].liesOn).Det(ORCALines[j].dir) - invTimeBoundaryObst * fakeRadius >= -CN_EPS &&
                (rRelativePosition * invTimeBoundaryObst - ORCALines[j].liesOn).Det(ORCALines[j].dir) - invTimeBoundaryObst * fakeRadius >= -CN_EPS)
            {
                alreadyCovered = true;
                break;
            }
        }
        if (alreadyCovered)
            continue;

        float lSqDist = lRelativePosition.SquaredEuclideanNorm();
        float rSqDist = rRelativePosition.SquaredEuclideanNorm();

        float sqFakeRadius = fakeRadius * fakeRadius;
        float sqTrueRadius = param.radius * param.radius;

        Vector obstacleVector = *right - *left;
        float s = -lRelativePosition.ScalarProduct(obstacleVector) / obstacleVector.SquaredEuclideanNorm();
        float lineSqDist = (-lRelativePosition - obstacleVector * s).SquaredEuclideanNorm();

        if ((s < 0.0f && lSqDist < sqTrueRadius) || (s > 1.0f && rSqDist < sqTrueRadius) ||
            (s >= 0.0f && s < 1.0f && lineSqDist < sqTrueRadius))
        {
            collisionsObst++;
        }


        if (s < 0.0f && lSqDist < sqFakeRadius)
        {

            if (left->IsConvex())
            {
                line.liesOn = Point();
                line.dir = Point(-lRelativePosition.Y(), lRelativePosition.X()) / sqrt(lSqDist); // Построение единичного вектора, нормального к относительному положению
                ORCALines.push_back(line);
            }

            continue;
        }
        else if (s > 1.0f && rSqDist < sqFakeRadius)
        {

            if (right->IsConvex() && rRelativePosition.Det(NeighboursObst[i].second.next->dir) >= 0.0f)
            {
                line.liesOn = Point();
                line.dir = Point(-rRelativePosition.Y(), rRelativePosition.X()) / sqrt(rSqDist);
                ORCALines.push_back(line);
            }

            continue;
        }
        else if (s >= 0.0f && s < 1.0f && lineSqDist < sqFakeRadius)
        {
            line.liesOn = Point();
            line.dir = -(NeighboursObst[i].second.dir);
            ORCALines.push_back(line);
            continue;
        }


        Vector lLegDirection, rLegDirection;

        if (s < 0.0f && lineSqDist <= sqFakeRadius)
        {

            if (!left->IsConvex())
            {
                continue;
            }

            right = left;

            float leg1 = sqrt(lSqDist - sqFakeRadius);

            lLegDirection = Point(lRelativePosition.X() * leg1 - lRelativePosition.Y() * fakeRadius, lRelativePosition.X() * fakeRadius + lRelativePosition.Y() * leg1) / lSqDist;
            rLegDirection = Point(lRelativePosition.X() * leg1 + lRelativePosition.Y() * fakeRadius, -lRelativePosition.X() * fakeRadius + lRelativePosition.Y() * leg1) / lSqDist;
        }
        else if (s > 1.0f && lineSqDist <= sqFakeRadius)
        {

            if (!right->IsConvex())
            {
                continue;
            }

            left = right;

            float leg2 = std::sqrt(rSqDist - sqFakeRadius);
            lLegDirection = Point(rRelativePosition.X() * leg2 - rRelativePosition.Y() * fakeRadius, rRelativePosition.X() * fakeRadius + rRelativePosition.Y() * leg2) / rSqDist;
            rLegDirection = Point(rRelativePosition.X() * leg2 + rRelativePosition.Y() * fakeRadius, -rRelativePosition.X() * fakeRadius + rRelativePosition.Y() * leg2) / rSqDist;
        }
        else
        {
            if (left->IsConvex())
            {
                float leg1 = std::sqrt(lSqDist - sqFakeRadius);
                lLegDirection = Point(lRelativePosition.X() * leg1 - lRelativePosition.Y() * fakeRadius, lRelativePosition.X() * fakeRadius + lRelativePosition.Y() * leg1) / lSqDist;
            }
            else
            {
                lLegDirection = -NeighboursObst[i].second.dir;
            }

            if (right->IsConvex())
            {
                float leg2 = std::sqrt(rSqDist - sqFakeRadius);
                rLegDirection = Point(rRelativePosition.X() * leg2 + rRelativePosition.Y() * fakeRadius, -rRelativePosition.X() * fakeRadius + rRelativePosition.Y() * leg2) / rSqDist;
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
            line.liesOn = leftCutoff + unitW * fakeRadius *  invTimeBoundaryObst;
            ORCALines.push_back(line);
            continue;
        }
        else if (t > 1.0f && tRight < 0.0f)
        {
            Vector unitW = (currV - rightCutoff)/(currV - rightCutoff).EuclideanNorm();

            line.dir = Vector(unitW.Y(), -unitW.X());
            line.liesOn = rightCutoff + unitW * fakeRadius * invTimeBoundaryObst ;
            ORCALines.push_back(line);
            continue;
        }

        float cutoffSqDist = ((t < 0.0f || t > 1.0f || right == left) ? std::numeric_limits<float>::infinity() : (currV - (leftCutoff + cutoffVec * t)).SquaredEuclideanNorm());
        float lLegSqDist = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : (currV - (leftCutoff + lLegDirection * tLeft)).SquaredEuclideanNorm());
        float rLegSqDist = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : (currV - (rightCutoff + rLegDirection * tRight)).SquaredEuclideanNorm());

        if (cutoffSqDist <= lLegSqDist && cutoffSqDist <= rLegSqDist)
        {
            line.dir = -NeighboursObst[i].second.dir;
            line.liesOn = leftCutoff + Point(-line.dir.Y(), line.dir.X()) * fakeRadius * invTimeBoundaryObst;
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
            line.liesOn = leftCutoff + Point(-line.dir.Y(), line.dir.X()) * fakeRadius * invTimeBoundaryObst;;
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
            line.liesOn = rightCutoff + Point(-line.dir.Y(), line.dir.X()) * fakeRadius * invTimeBoundaryObst;
            ORCALines.push_back(line);
            continue;
        }
    }


    size_t numObstLines = ORCALines.size();

    //Получение ORCA-линий агентов
    //std::sort(Neighbours.begin(),Neighbours.end(), Compare);

    Line currline;
    ORCAAgentWithPAR *curragent;
    Vector u, w;

    unsigned long minMaxNum = (param.agentsMaxNum < Neighbours.size()) ? param.agentsMaxNum : Neighbours.size();

    for(unsigned long i = 0; i < minMaxNum; i++)
    {
        auto Neighbour = Neighbours[i];
        curragent = dynamic_cast<ORCAAgentWithPAR *>(Neighbour.second);
        auto circlecenter = curragent->position - this->position; //(P_b - P_a)
        auto relvelocity = this->currV - curragent->currV; //(V_a - V_b)

        float radiussum = fakeRadius + curragent->fakeRadius; //(R_a + R_b)
        float radiussum2 = radiussum * radiussum;
        float distSq = circlecenter.SquaredEuclideanNorm();
        float trueSqRadSum = (param.radius + curragent->param.radius) * (param.radius + curragent->param.radius);

        if(distSq < trueSqRadSum )
        {
            collisions++;
        }

        if(distSq >= radiussum2)
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
    if(lineFail < this->ORCALines.size())
    {
        Utils::linearProgram3(ORCALines, numObstLines, lineFail, param.maxSpeed, this->newV);
    }

    Neighbours.clear();
}


void ORCAAgentWithPAR::ApplyNewVelocity()
{
    PARVis = false;
    notPARVis = false;
    PARUnion = false;
    currV = newV;
}


bool ORCAAgentWithPAR::UpdatePrefVelocity()
{
    Point next;
    if(inPARMode)
    {
        if(PARUnion)
        {
            UnitePAR();
            prefV = Point();
            return true;
        }

        if(notPARVis)
        {
            UpdatePAR();
            prefV = Point();
            return true;
        }

        if(PARExec)
        {
            bool allOnPos = true;
            bool allFin = true;

            for(auto& ag : PARAgents)
            {
                int agPARId = ag->PARActorId;
                auto agPARPos = (currPARPos < (*PARres.actorsPaths)[agPARId].size()) ? currPARPos : (*PARres.actorsPaths)[agPARId].size() - 1;
                auto agCurrGoal =  PARMap.GetPoint((*PARres.actorsPaths)[agPARId][agPARPos]);
                if((ag->position - agCurrGoal).EuclideanNorm() > 0.01 || !ag->PARExec)
                {
                    allOnPos = false;
                }

                if((ag->position - ag->PARGoal).EuclideanNorm() > 0.01 && ag->inPARMode)
                {
                    allFin = false;
                }
            }

            if(allOnPos)
            {
                currPARPos++;
            }



            if(allFin)
            {
                inPARMode = false;
                PARExec = false;
                moveToPARPos = false;

                PARStart = Point(-1,-1);
                PARGoal = Point(-1,-1);
                prefV = Point();
                return true;
            }
            else
            {
                prefV = Point();
                return true;
            }
        }

        if(moveToPARPos)
        {
            float distSq = (position-PARStart).SquaredEuclideanNorm();
            if(distSq < options->delta * options->delta)
            {
                prefV = Point();
                PARExec = true;

                bool flag1 = false, flag2 = true;
                for(auto &a : PARAgents)
                {
                    if(a == this)
                    {
                        flag1 = true;
                    }

                    if(!flag1 && !a->PARExec)
                    {
                        flag2 = false;
                    }


                    float distSq2 = (a->position - a->PARStart).SquaredEuclideanNorm();
                    if(distSq2 >= (options->delta * options->delta))
                    {
                        PARExec = false;
                        break;
                    }
                }

                if(PARExec && flag2)
                {
                    nextForLog = PARGoal;
                    moveToPARPos = false;
                    currPARPos = 0;

                    return true;
                }
                PARExec = false;
                return true;
            }
            else
            {

                nextForLog = PARStart;
                Vector goalVector = PARStart - position;
                float dist = goalVector.EuclideanNorm();

                if(dist > 1)
                {
                    goalVector = (goalVector/dist);
                }
                prefV = goalVector;
                return true;
            }
        }
    }
    else
    {
        if(PARVis)
        {
            prefV = Point();
            return true;
        }
        if(planner->GetNext(position, next))
        {
            nextForLog = next;
            Vector goalVector = next - position;
            float dist = goalVector.EuclideanNorm();
            if(Neighbours.size() >= param.PARNum && dist < param.sightRadius)
            {
                PreparePARExecution(next);
                prefV = Point();
                return true;
            }
            else
            {
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
        }
        nextForLog = position;
        prefV = Point();
        return false;
    }
}


bool ORCAAgentWithPAR::operator == (const ORCAAgentWithPAR &another) const
{
    return this->id == another.id;
}


bool ORCAAgentWithPAR::operator != (const ORCAAgentWithPAR &another) const
{
    return this->id != another.id;
}


ORCAAgentWithPAR* ORCAAgentWithPAR::Clone() const
{
    return new ORCAAgentWithPAR(*this);
}


void ORCAAgentWithPAR::AddNeighbour(Agent &neighbour, float distSq)
{
    float sightSq = param.sightRadius * param.sightRadius;

    if(distSq >= sightSq)
    {
        return;
    }
    auto tmpAgentPAR = dynamic_cast<ORCAAgentWithPAR*>(&neighbour);
    if(tmpAgentPAR->inPARMode)
    {
        PARVis = true;

        if(inPARMode)
        {
            if( !tmpAgentPAR->PARUnion && PARAgents.find(tmpAgentPAR) == PARAgents.end())
            {
                if(distSq < (param.radius + tmpAgentPAR->param.radius + 0.25) * (param.radius + tmpAgentPAR->param.radius + 0.25))
                {
                    PARUnion = true;
                }
            }
        }

    }
    else if(inPARMode)
    {
        if(distSq < (param.radius + tmpAgentPAR->param.radius + 0.25) * (param.radius + tmpAgentPAR->param.radius + 0.25))
        {
            notPARVis = true;
        }

    }

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

}


std::set<ORCAAgentWithPAR *> ORCAAgentWithPAR::GetAgentsForCentralizedPlanning()
{
    return PARAgents;
}


void ORCAAgentWithPAR::SetAgentsForCentralizedPlanning(std::set<ORCAAgentWithPAR *> agents)
{
    PARAgents = agents;
}


void ORCAAgentWithPAR::PreparePARExecution(Point common)
{
    PARAgents.clear();
    PARAgents.insert(this);
    for (auto &el : Neighbours)
    {
        ORCAAgentWithPAR * neighbour = dynamic_cast<ORCAAgentWithPAR *>(el.second);
        if(!neighbour->inPARMode && !neighbour->PARVis)
        {
            PARAgents.insert(neighbour);
            for(auto &el2 : neighbour->GetNeighbours())
            {
                auto agent = dynamic_cast<ORCAAgentWithPAR *>(el2.second);
                if(!agent->inPARMode && !agent->PARVis)
                {
                    PARAgents.insert(agent);
                }
                else
                {
                    PARVis = true;
                    PARAgents.clear();
                    return;
                }
            }
        }
        else
        {
            PARVis = true;
            PARAgents.clear();
            return;
        }
    }

    for(auto &ag : PARAgents)
    {
        if(ag != this)
        {
            ag->SetAgentsForCentralizedPlanning(PARAgents);
        }
        if (!ag->ComputePAREnv(common))
        {
            return;
        }
    }

    for(auto &ag : PARAgents)
    {
        if(!ag->ComputePAR())
        {
            for(auto &ag1 : PARAgents)
            {
                while(!ag1->buffPar.empty())
                {
                    ag1->planner->AddPointToPath(ag1->buffPar.back());
                    ag1->buffPar.pop_back();
                }

                ag1->PARStart = Point(-1,-1);
                ag1->PARGoal = Point(-1,-1);
            }
            return;
        }
    }

    // TODO Save here
    PARLog->SaveInstance(PARSet, PARMap);


    for(auto &ag : PARAgents)
    {
        ag->inPARMode = true;
        ag->moveToPARPos = true;
        ag->PARVis = false;
        ag->PARExec = false;
        ag->PARcommon = common;
    }
}


std::vector<std::pair<float, Agent *>>& ORCAAgentWithPAR::GetNeighbours()
{
    return Neighbours;
}


bool ORCAAgentWithPAR::ComputePAREnv(Point common, std::vector<std::pair<Point, ORCAAgentWithPAR*>> oldGoals)
{

    float minX = map->GetWidth() * map->GetCellSize(), minY = map->GetHeight() * map->GetCellSize(), maxX = 0, maxY = 0;

    for(auto &ag : PARAgents)
    {
        if((ag->GetPosition().X() - ag->param.sightRadius) < minX)
        {
            minX = (ag->GetPosition().X() - ag->param.sightRadius);
        }
        if((ag->GetPosition().Y() - ag->param.sightRadius) < minY)
        {
            minY = (ag->GetPosition().Y() - ag->param.sightRadius);
        }

        if((ag->GetPosition().X() + ag->param.sightRadius) > maxX)
        {
            maxX = (ag->GetPosition().X() + ag->param.sightRadius);
        }
        if((ag->GetPosition().Y() + ag->param.sightRadius) > maxY)
        {
            maxY = (ag->GetPosition().Y() + ag->param.sightRadius);
        }
    }

    Node nodeLeftTop = map->GetClosestNode(Point(minX, maxY));
    Node nodeRightBottom = map->GetClosestNode(Point(maxX, minY));
    int width = nodeRightBottom.j - nodeLeftTop.j + 1;
    int height = nodeRightBottom.i - nodeLeftTop.i + 1;
    PARMap = SubMap(map, nodeLeftTop, {height,width}, 1);

    // Starts and goals
    std::unordered_map<int, Node> starts;
    std::unordered_map<int, Node> goals;
    for(auto &og : oldGoals)
    {
        Node tmpOldGoal = PARMap.GetClosestNode(og.first);
        if(goals.find(tmpOldGoal.i * PARMap.GetWidth() + tmpOldGoal.j) == goals.end())
        {
            goals.insert({tmpOldGoal.i * PARMap.GetWidth() + tmpOldGoal.j, tmpOldGoal});
        }
        else
        {
            og.second->PARGoal = Point(-1,-1);
        }

    }

    PARSet = PARActorSet();
    PARSet.clear();
    Point tmpGoalPoint;
    for(auto &ag : PARAgents)
    {
        Node tmpStart, tmpGoal;

        if(ag->PARGoal.X() < 0)
        {
            ag->buffPar.clear();
            tmpGoalPoint = ag->PullOutIntermediateGoal(common);
            tmpGoal = PARMap.GetClosestNode(tmpGoalPoint);
            tmpGoal = PARMap.FindAvailableNode(tmpGoal, goals);

            if(tmpGoal.i < 0)
            {
                for(auto &ag1 : PARAgents)
                {
                    while(!ag1->buffPar.empty())
                    {
                        ag1->planner->AddPointToPath(ag1->buffPar.back());
                        ag1->buffPar.pop_back();
                    }
                    ag1->PARStart = Point(-1,-1);
                    ag1->PARGoal = Point(-1,-1);
                    ag1->inPARMode = false;
                    ag1->moveToPARPos = false;
                    ag1->PARVis = false;
                    ag1->PARExec = false;
                }

                return false;
            }

            ag->PARGoal = PARMap.GetPoint(tmpGoal);
        }
        else
        {
            tmpGoal = PARMap.GetClosestNode(ag->PARGoal);
        }


        if(ag->PARStart.X() < 0)
        {
            tmpStart = PARMap.GetClosestNode(ag->GetPosition());
            tmpStart = PARMap.FindAvailableNode(tmpStart, starts);

            if(tmpStart.i < 0)
            {
                for(auto &ag1 : PARAgents)
                {
                    while(!ag1->buffPar.empty())
                    {
                        ag1->planner->AddPointToPath(ag1->buffPar.back());
                        ag1->buffPar.pop_back();
                    }
                    ag1->PARStart = Point(-1,-1);
                    ag1->PARGoal = Point(-1,-1);
                    ag1->inPARMode = false;
                    ag1->moveToPARPos = false;
                    ag1->PARVis = false;
                    ag1->PARExec = false;
                }
                return false;
            }
            ag->PARStart = PARMap.GetPoint(tmpStart);
        }
        else
        {
            tmpStart = PARMap.GetClosestNode(ag->PARStart);
        }

        starts.insert({tmpStart.i * PARMap.GetWidth() + tmpStart.j, tmpStart});
        goals.insert({tmpGoal.i * PARMap.GetWidth() + tmpGoal.j, tmpGoal});

        PARSet.addActor(tmpStart.i, tmpStart.j, tmpGoal.i, tmpGoal.j);
        if(ag == this)
        {
            PARActorId = PARSet.getActorId(tmpStart.i, tmpStart.j);
            currPARPos = 0;
        }
    }
    return true;
}


Point ORCAAgentWithPAR::PullOutIntermediateGoal(Point common)
{
    Point nextPoint = planner->PullOutNext();
    buffPar.push_back(nextPoint);
    if(nextPoint == common)
    {
        Point nextNextPoint = planner->PullOutNext();
        buffPar.push_back(nextNextPoint);
        return nextNextPoint;
    }

    return nextPoint;
}


bool ORCAAgentWithPAR::ComputePAR()
{
    PARSolver.clear();
    conf.maxTime = CN_DEFAULT_PARMAXTIME;
//    PARMap.printSubMap();
    PARres = PARSolver.startSearch(PARMap, conf,PARSet);

    return PARres.pathfound;
}


bool ORCAAgentWithPAR::UnitePAR()
{
    std::vector<std::pair<Point, ORCAAgentWithPAR*>> goals;
    PARUnion = false;
    for(auto &ag : PARAgents)
    {
        ag->PARStart = Point(-1,-1);
        goals.push_back({ag->PARGoal, ag});

        ag->PARUnion = false;
    }


    for(auto &el : Neighbours)
    {
        ORCAAgentWithPAR * neighbour = dynamic_cast<ORCAAgentWithPAR *>(el.second);
        if(neighbour->inPARMode && PARAgents.find(neighbour) == PARAgents.end())
        {
            for(auto& nn : neighbour->PARAgents)
            {
                while(!nn->buffPar.empty())
                {
                    nn->planner->AddPointToPath(nn->buffPar.back());
                    nn->buffPar.pop_back();
                }

                nn->PARStart = Point(-1,-1);
                nn->PARGoal = Point(-1,-1);
                nn->PARUnion = false;
            }

            PARAgents.insert(neighbour->PARAgents.begin(), neighbour->PARAgents.end());
        }
    }


    for(auto &ag : PARAgents)
    {
        if(ag != this)
        {
            ag->SetAgentsForCentralizedPlanning(PARAgents);
        }
        if (!ag->ComputePAREnv(PARcommon, goals))
        {
            return false;
        }
    }

    for(auto &ag : PARAgents)
    {
        if(!ag->ComputePAR())
        {
            for(auto &ag1 : PARAgents)
            {
                while(!ag1->buffPar.empty())
                {
                    ag1->planner->AddPointToPath(ag1->buffPar.back());
                    ag1->buffPar.pop_back();
                }

                ag1->PARStart = Point(-1,-1);
                ag1->PARGoal = Point(-1,-1);
                ag1->inPARMode = false;
                ag1->moveToPARPos = false;
                ag1->PARVis = false;
                ag1->PARExec = false;
            }
            return false;
        }
    }

    for(auto &ag : PARAgents)
    {
        ag->inPARMode = true;
        ag->moveToPARPos = true;
        ag->PARVis = false;
        ag->PARExec = false;
        ag->PARcommon = PARcommon;
        ag->currPARPos = 0;
    }
  //  std::cout<<"Agent No "<< id <<". End unite PAR\n";
    return true;

}


bool ORCAAgentWithPAR::UpdatePAR()
{
    std::vector<std::pair<Point, ORCAAgentWithPAR*>> goals;
    for(auto &ag : PARAgents)
    {
        ag->PARStart = Point(-1,-1);
        goals.push_back({ag->PARGoal, ag});

    }

    std::set<ORCAAgentWithPAR*> tmpAgents;

    for(auto &ag : PARAgents)
    {

        auto N = ag->GetNeighbours();
        for(auto &el : N)
        {
            ORCAAgentWithPAR * neighbour = dynamic_cast<ORCAAgentWithPAR *>(el.second);
            if(!neighbour->inPARMode)
            {
                neighbour->PARStart = Point(-1,-1);
                neighbour->PARGoal = Point(-1,-1);
                tmpAgents.insert(neighbour);
            }
        }
        ag->notPARVis = false;
    }

    PARAgents.insert(tmpAgents.begin(), tmpAgents.end());

    for(auto &ag : PARAgents)
    {
        if(ag != this)
        {
            ag->SetAgentsForCentralizedPlanning(PARAgents);
        }

        if (!ag->ComputePAREnv(PARcommon, goals))
        {
            return false;
        }
    }

    for(auto &ag : PARAgents)
    {
        if(!ag->ComputePAR())
        {
            for(auto &ag1 : PARAgents)
            {
                while(!ag1->buffPar.empty())
                {
                    ag1->planner->AddPointToPath(ag1->buffPar.back());
                    ag1->buffPar.pop_back();
                }

                ag1->PARStart = Point(-1,-1);
                ag1->PARGoal = Point(-1,-1);
                ag1->inPARMode = false;
                ag1->moveToPARPos = false;
                ag1->PARVis = false;
                ag1->PARExec = false;
            }
            return false;
        }
    }

    for(auto &ag : PARAgents)
    {
        ag->inPARMode = true;
        ag->moveToPARPos = true;
        ag->PARVis = false;
        ag->PARExec = false;
        ag->PARcommon = PARcommon;
        ag->currPARPos = 0;
    }

    return true;
}


bool ORCAAgentWithPAR::isPARMember() const
{
    return inPARMode;
}

void ORCAAgentWithPAR::SetPARInstanceLoggerRef(PARInstancesLogger *log)
{
    PARLog = log;
}