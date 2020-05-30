
#include <ros/ros.h> 

#include <gtest/gtest.h>


TEST(TestSuiteTestSuite, testSuiteTest) {
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}