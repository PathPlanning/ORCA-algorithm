#include "gtest/gtest.h"
#include "Map.h"


class TestMap : public ::testing::Test
{
    protected:
        void SetUp()
        {
            grid ={{0,0,0,0,0,0,0,0},
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

            underTest = new Map(1, grid, obstacles);
        }
        void TearDown()
        {
            delete underTest;
        }
        Map *underTest;
        std::vector<std::vector<int>> grid;
        std::vector<std::vector<Point>> obstacles;

};


TEST_F(TestMap, GridTest)
{
    EXPECT_EQ(underTest->GetHeight(), grid.size());
    EXPECT_EQ(underTest->GetWidth(), grid[0].size());
    for(int i = 0; i < grid.size(); i++)
    {
        for(int j = 0; j < grid[i].size(); j++)
        {
            EXPECT_EQ(underTest->CellIsObstacle(i, j), grid[i][j]);
            EXPECT_NE(underTest->CellIsTraversable(i, j), grid[i][j]);
        }
    }

    ASSERT_TRUE(underTest->CellOnGrid(0,0));
    ASSERT_FALSE(underTest->CellOnGrid(0,110));
}


TEST_F(TestMap, ObstacleTest)
{
    int i = 0;
    EXPECT_EQ(obstacles.size(), underTest->GetObstacles().size());
    for(auto &o : obstacles)
    {
        EXPECT_EQ(o.size(), underTest->GetObstacles()[i].size());
        i++;
    }
}



TEST_F(TestMap, PointNodeTest)
{
    for(float n = 0.5; n < 7; n++)
    {
        Point a = {n, n};
        Point b = underTest->GetPoint(underTest->GetClosestNode(a));
        ASSERT_FLOAT_EQ(a.X(), b.X());
        ASSERT_FLOAT_EQ(a.Y(), b.Y());
    }

    Node a = underTest->GetClosestNode(Point(-1,-1));
    EXPECT_EQ(a.i, 6);
    EXPECT_EQ(a.j, 0);
    a = underTest->GetClosestNode(Point( 1.5 ,7.5));
    EXPECT_EQ(a.i, 0);
    EXPECT_EQ(a.j, 1);

}
