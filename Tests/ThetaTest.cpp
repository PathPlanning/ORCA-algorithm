#include "gtest/gtest.h"
#include "ThetaStar.h"


class TestTheta : public ::testing::Test
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
            s1 = {0.5,0.5};
            s2 = {2.5,5.5};
            g1 = {0.5, 6.5};
            g2 = {3.5, 3.5};
            underTest1 = new ThetaStar(*map, opt, s1, g1, 0.5);
            underTest1->CreateGlobalPath();

            underTest2 = new ThetaStar(*map, opt, s2, g2, 0.5);
            underTest2->CreateGlobalPath();

            underTest3 = new ThetaStar(*map, opt, s2, g2, 1);
            underTest3->CreateGlobalPath();

        }
        void TearDown()
        {
            delete map;
            delete underTest1;
            delete underTest2;
            delete underTest3;
        }

        ThetaStar *underTest1;
        ThetaStar *underTest2;
        ThetaStar *underTest3;
        Point s1,s2,g1,g2;
        Map *map;
        std::vector<std::vector<int>> grid;
        std::vector<std::vector<Point>> obstacles;
        EnvironmentOptions opt;

};


TEST_F(TestTheta, PathTest)
{
    Point next;
    underTest1->GetNext(s1, next);
    EXPECT_FLOAT_EQ(next.X(), g1.X());
    EXPECT_FLOAT_EQ(next.Y(), g1.Y());

    underTest2->GetNext(s2, next);
    EXPECT_FLOAT_EQ(next.X(), 3.5);
    EXPECT_FLOAT_EQ(next.Y(), 5.5);

    underTest2->GetNext(next, next);
    EXPECT_FLOAT_EQ(next.X(), 3.5);
    EXPECT_FLOAT_EQ(next.Y(), 3.5);

    ASSERT_FALSE(underTest3->GetNext(s2, next));

}

TEST_F(TestTheta, OtherTest)
{
    Point inp = {1,1};
    underTest1->AddPointToPath(inp);
    Point res = underTest1->PullOutNext();

    ASSERT_TRUE(inp == res);
}
