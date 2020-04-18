#include "gtest/gtest.h"
#include "SubMap.h"






class TestSubMap : public ::testing::Test
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

            base = new Map(1, grid, obstacles);
            origin = {2, 2};
            underTest = new SubMap(base, origin , {3, 4}, 1);
        }
        void TearDown()
        {
            delete base;
            delete underTest;
        }
        Map *base;
        SubMap *underTest;
        std::vector<std::vector<int>> grid;
        std::vector<std::vector<Point>> obstacles;
        Node origin;
};


TEST_F(TestSubMap, GridTest)
{


    EXPECT_EQ(underTest->GetHeight(), 3);
    EXPECT_EQ(underTest->GetWidth(), 4);
    EXPECT_EQ(underTest->GetEmptyCellCount(), 5);
    for(int i = 0; i < underTest->GetHeight(); i++)
    {
        for(int j = 0; j < underTest->GetWidth(); j++)
        {
            EXPECT_EQ(underTest->CellIsObstacle(i, j), base->CellIsObstacle(origin.i + i, origin.j + j));
        }
    }
}


TEST_F(TestSubMap, PointNodeTest)
{
    for(float n = 2.5; n < 5; n++)
    {
        Point a = {n, n};
        Point b = underTest->GetPoint(underTest->GetClosestNode(a));
        ASSERT_FLOAT_EQ(a.X(), b.X());
        ASSERT_FLOAT_EQ(a.Y(), b.Y());
    }

    Node a = underTest->GetClosestNode(Point(1.5,1.5));
    EXPECT_EQ(a.i, 2);
    EXPECT_EQ(a.j, 0);
    a = underTest->GetClosestNode(Point( 2.5 ,5.5));
    EXPECT_EQ(a.i, 0);
    EXPECT_EQ(a.j, 0);


    std::unordered_map<int, Node> occupied;

    Node n = {0,1};


    Node r = underTest->FindAvailableNode(n, {});
    EXPECT_EQ(r.i, n.i);
    EXPECT_EQ(r.j, n.j);

    occupied.insert({ n.i * underTest->GetWidth() + n.j, n});
    r = underTest->FindAvailableNode(n, occupied);

    ASSERT_FALSE(r == n);
    ASSERT_TRUE(underTest->CellIsTraversable(r.i,r.j));
    ASSERT_TRUE(underTest->CellOnGrid(r.i,r.j));

    Node tmp = {1,1};
    occupied.insert({ tmp.i * underTest->GetWidth() + tmp.j, tmp});
    tmp = {1,2};
    occupied.insert({ tmp.i * underTest->GetWidth() + tmp.j, tmp});
    tmp = {2,2};
    occupied.insert({ tmp.i * underTest->GetWidth() + tmp.j, tmp});

    r = underTest->FindAvailableNode(n, occupied);
    ASSERT_TRUE(r == Node(2, 3));

    tmp = {2,3};
    occupied.insert({ tmp.i * underTest->GetWidth() + tmp.j, tmp});

    r = underTest->FindAvailableNode(n, occupied);
    ASSERT_TRUE(r == Node(-1, -1));

}
