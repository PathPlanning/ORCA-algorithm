#include "gtest/gtest.h"
#include "ORCAAgentWithPAR.h"



class TestORCAAgentWithPAR : public ::testing::Test
{
    protected:
        void SetUp()
        {
            grid =  {{0,0,0,0,0,0,0,0},
                     {0,0,0,0,0,0,0,0},
                     {0,0,1,0,1,1,0,0},
                     {0,0,1,0,0,1,0,0},
                     {0,0,1,1,0,0,0,0},
                     {0,0,0,0,0,0,0,0},
                     {0,0,0,0,0,0,0,0}};
            obstacles =
                    {
                            {{2,2},{4,2},{4, 3},{3, 3},{3, 5},{2, 5}},
                            {{5,3},{6, 3},{6, 5},{4, 5},{4, 4},{5, 4}},
                            {{0,0},{0, 7}, {8, 7}, {8, 0}}
                    };


            map = new Map(1, grid, obstacles);
            opt = EnvironmentOptions(CN_SP_MT_EUCL , 0, false, false, 1, 0.1, 0.1);
            param = AgentParam(3.0, 5.4, 33.0, 0.3, 0.19, 1, 10, 2);

            starts[0] = Point(0.5, 0.5);
            starts[1] = Point(0.5, 6.5);

            goals[0] = Point(0.5, 6.5);
            goals[1] = Point(0.5, 0.5);





            for(int i = 0; i < 2; i++)
            {
                underTest[i] = new ORCAAgentWithPAR(i, starts[i], goals[i], *map, opt,param);
                underTest[i]->SetPlanner(ThetaStar(*map, opt, starts[i], goals[i], 0.3));
            }


            emptygrid =     {{0,0,0},
                             {0,0,0},
                             {0,0,0}};
            emptyobstacles = {};
            emptymap = new Map(1, emptygrid, emptyobstacles);



            starts[2] = Point(0.5, 0.5);
            starts[3] = Point(2.5, 2.5);
            starts[4] = Point(0.5, 2.5);
            starts[5] = Point(2.5, 0.5);

            goals[2] = Point(2.5, 2.5);
            goals[3] = Point(0.5, 0.5);
            goals[4] = Point(2.5, 0.5);
            goals[5] = Point(0.5, 2.5);

            for(int i = 2; i < 6; i++)
            {
                underTest[i] = new ORCAAgentWithPAR(i, starts[i], goals[i], *emptymap, opt,param);
                underTest[i]->SetPlanner(ThetaStar(*emptymap, opt, starts[i], goals[i], 0.3));
                underTest[i]->InitPath();
            }
        }

        void TearDown()
        {
            delete map;
            delete emptymap;

            for(int i = 0 ; i < 6; i++)
            {
                delete underTest[i];
            }

        }

        Agent *underTest [6];
        Point starts [6];
        Point goals [6];
        Map *map;
        Map *emptymap;
        std::vector<std::vector<int>> grid;
        std::vector<std::vector<int>> emptygrid;
        std::vector<std::vector<Point>> obstacles;
        std::vector<std::vector<Point>> emptyobstacles;
        EnvironmentOptions opt;
        AgentParam param;

};


TEST_F(TestORCAAgentWithPAR, NotPARTest)
{
    for(int i = 0; i < 2; i++)
    {
        ASSERT_TRUE(underTest[i]->GetPosition() == starts[i]);
    }

    ASSERT_TRUE(underTest[0]->InitPath());
    ASSERT_TRUE(underTest[1]->InitPath());

    underTest[0]->UpdatePrefVelocity();
    underTest[0]->UpdateNeighbourObst();
    underTest[0]->ComputeNewVelocity();
    underTest[0]->ApplyNewVelocity();
    Point v = underTest[0]->GetVelocity();
    v = v/v.EuclideanNorm();

    ASSERT_NEAR(v.X(), 0.0, 0.01);
    ASSERT_NEAR(v.Y(), 1.0, 0.01);


    underTest[0]->SetPosition(goals[0]);
    underTest[0]->UpdatePrefVelocity();
    underTest[0]->UpdateNeighbourObst();
    underTest[0]->AddNeighbour(*underTest[1], 0.0);
    underTest[0]->ComputeNewVelocity();

    ASSERT_EQ(underTest[0]->GetCollision().first, 1);
    ASSERT_TRUE(underTest[0]->isFinished());

    float offset = 0.2f;
    int colObsOld = 0;
    for(int i = 0; i < obstacles[0].size(); i++)
    {
        Point pos = Point(obstacles[0][i].X() + offset, obstacles[0][i].X() + offset);
        underTest[0]->SetPosition(pos);
        underTest[0]->UpdatePrefVelocity();
        underTest[0]->UpdateNeighbourObst();
        underTest[0]->ComputeNewVelocity();

        int colObs = underTest[0]->GetCollision().second;
        ASSERT_EQ(colObs, colObsOld + 2);
        colObsOld = colObs;
    }


    for(int i = 0; i < obstacles[0].size(); i++)
    {
        int f = (i == 0) ? obstacles[0].size() - 1 : i - 1;
        int s = i;
        float x = ((obstacles[0][f].X() + offset) + (obstacles[0][s].X() + offset))/2;
        float y = ((obstacles[0][f].Y() + offset) + (obstacles[0][s].Y() + offset))/2;
        Point pos = Point(x,y);
        underTest[0]->SetPosition(pos);
        underTest[0]->UpdatePrefVelocity();
        underTest[0]->UpdateNeighbourObst();
        underTest[0]->ComputeNewVelocity();

        int colObs = underTest[0]->GetCollision().second;
        ASSERT_EQ(colObs, colObsOld + 1);
        colObsOld = colObs;
    }


    underTest[0]->SetPosition(Point(0.5, 2.5));
    underTest[1]->SetPosition(Point(0.5, 4.5));
    underTest[0]->UpdatePrefVelocity();
    underTest[1]->UpdatePrefVelocity();
    underTest[0]->AddNeighbour(*underTest[1], 4.0);
    underTest[1]->AddNeighbour(*underTest[0], 4.0);
    underTest[0]->UpdateNeighbourObst();
    underTest[1]->UpdateNeighbourObst();
    underTest[0]->ComputeNewVelocity();
    underTest[1]->ComputeNewVelocity();
    underTest[0]->ApplyNewVelocity();
    underTest[1]->ApplyNewVelocity();

    ASSERT_NE(underTest[0]->GetVelocity().EuclideanNorm(), 0.0);
    ASSERT_NE(underTest[1]->GetVelocity().EuclideanNorm(), 0.0);


    for(float time = 0; time < param.timeBoundary; time += opt.timestep)
    {
        Point pos1 = underTest[0]->GetPosition() + underTest[0]->GetVelocity() * time;
        Point pos2 = underTest[1]->GetPosition() + underTest[1]->GetVelocity() * time;

        float dist = (pos1-pos2).EuclideanNorm();
        float rsum = underTest[0]->GetRadius() + underTest[1]->GetRadius();
        ASSERT_GT(dist, rsum);
    }

}




TEST_F(TestORCAAgentWithPAR, PARTest)
{

    underTest[2]->UpdateNeighbourObst();
    underTest[2]->AddNeighbour(*underTest[3], 8.0);
    underTest[2]->AddNeighbour(*underTest[4], 4.0);
    underTest[2]->UpdatePrefVelocity();


    for(int i = 2; i < 5; i++)
    {

        ASSERT_TRUE(dynamic_cast<ORCAAgentWithPAR*>(underTest[i])->isPARMember());
    }

    ASSERT_FALSE(dynamic_cast<ORCAAgentWithPAR*>(underTest[5])->isPARMember());

    underTest[5]->SetPosition({1,1});
    underTest[2]->AddNeighbour(*underTest[5], 0.5);
    underTest[2]->UpdatePrefVelocity();
    for(int i = 2; i < 6; i++)
    {
        ASSERT_TRUE(dynamic_cast<ORCAAgentWithPAR*>(underTest[i])->isPARMember());

    }

    for(int i = 2; i < 5; i++)
    {
        underTest[i]->SetPosition(starts[i]);
    }
    underTest[5]->SetPosition({emptymap->GetPoint(emptymap->GetClosestNode({1,1}))});

    for(int i = 2; i < 6; i++)
    {
        underTest[i]->UpdatePrefVelocity();
    }

    for(int i = 2; i < 6; i++)
    {
        underTest[i]->UpdatePrefVelocity();
    }

    for(int i = 2; i < 6; i++)
    {
        underTest[i]->SetPosition(goals[i]);
    }

    for(int i = 2; i < 6; i++)
    {
        underTest[i]->UpdatePrefVelocity();
    }

    for(int i = 2; i < 6; i++)
    {
        ASSERT_FALSE(dynamic_cast<ORCAAgentWithPAR*>(underTest[i])->isPARMember());
    }




}