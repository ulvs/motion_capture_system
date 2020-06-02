
#include <string>
#include <ros/ros.h> 
#include <gtest/gtest.h>
#include "gmock/gmock.h"
#include <mocap_qualisys/RTProtocol.h>
#include <mocap_qualisys/QualisysDriver.h>



ros::NodeHandle* nh_ptr;

/* Check if the tests are working at all */
TEST(TestSuiteTestSuite, testSuiteTest) {
    EXPECT_TRUE(true);
}

TEST(QualisysSuite, contructor) {
    mocap::QualisysDriver driver(*nh_ptr);
    EXPECT_FALSE(driver.isInitialized());
}

TEST(QualisysSuite, testInit) {
    nh_ptr->setParam("server_address", "127.0.0.1");
    mocap::TestQualisysDriver driver(*nh_ptr);
    EXPECT_FALSE(driver.isInitialized());
    driver.init();
    EXPECT_TRUE(driver.isInitialized());
}

TEST(QualisysSuite, testRun) {
    nh_ptr->setParam("server_address", "127.0.0.1");
    mocap::TestQualisysDriver driver(*nh_ptr);
    driver.init();
    EXPECT_TRUE(driver.run());
}

TEST(QualisysSuite, testSubjectList) {
    nh_ptr->setParam("server_address", "127.0.0.1");
    std::vector<std::string> model_list({"mockModel1", "mockModel3"});
    nh_ptr->setParam("model_list", model_list);
    mocap::TestQualisysDriver driver(*nh_ptr);
    driver.init();
    EXPECT_TRUE(driver.run());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    nh_ptr = &nh;
    return RUN_ALL_TESTS();
}