
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
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
    mocap::TestQualisysDriver driver(*nh_ptr);
    EXPECT_FALSE(driver.isInitialized());
    ASSERT_TRUE(driver.init());
    EXPECT_TRUE(driver.isInitialized());
}

TEST(QualisysSuite, testRun) {
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_TRUE(MockCRTProtocol::current_protocol->is_connected);
    EXPECT_TRUE(driver.run());
}

TEST(ParameterResponseAddress, serverAddress){
    std::string prev_address;
    nh_ptr->param("server_address", prev_address, std::string("127.0.0.1"));
    std::string address("192.0.0.1");
    nh_ptr->setParam("server_address", address);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(address, MockCRTProtocol::current_protocol->server_address);
    nh_ptr->setParam("server_address", prev_address);
    ROS_ERROR("Previous address: %s", prev_address.c_str());
}

TEST(ParameterResponseAddress, noServerAddress){
    std::string prev_address;
    nh_ptr->param("server_address", prev_address, std::string("127.0.0.1"));
    nh_ptr->deleteParam("server_address");
    mocap::TestQualisysDriver driver(*nh_ptr);
    EXPECT_FALSE(driver.init());
    nh_ptr->setParam("server_address", prev_address);
    ROS_ERROR("Previous address: %s", prev_address.c_str());
}

TEST(ParameterResponsePort, basePortDefault){
    nh_ptr->deleteParam("server_base_port");
    short default_port = 22222;
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(default_port, MockCRTProtocol::current_protocol->base_port);
}

TEST(ParameterResponsePort, basePortSet){
    short port = 20000;
    nh_ptr->setParam("server_base_port", port);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(port, MockCRTProtocol::current_protocol->base_port);
    nh_ptr->deleteParam("server_base_port");
}

TEST(ParameterResponseProtocol, udpRandomPortNoParamSet){
    nh_ptr->deleteParam("udp_port");
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_TRUE(MockCRTProtocol::current_protocol->is_udp);
    EXPECT_FALSE(MockCRTProtocol::current_protocol->is_tcp);
}

TEST(ParameterResponseProtocol, udpRandomPortParam0){
    // Protocol chould be UDP and port number will be random
    short port = 0;
    nh_ptr->setParam("udp_port", port);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_TRUE(MockCRTProtocol::current_protocol->is_udp);
    nh_ptr->deleteParam("udp_port");
}

TEST(ParameterResponseProtocol, udpSpecificPort){
    // Specific UDP port
    short port = 20000;
    nh_ptr->setParam("udp_port", port);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_TRUE(MockCRTProtocol::current_protocol->is_udp);
    EXPECT_EQ(port, MockCRTProtocol::current_protocol->udp_port);
    nh_ptr->deleteParam("udp_port");
}

TEST(ParameterResponseProtocol, tcpRandomPort){
    // Specific UDP port
    short port = -1;
    nh_ptr->setParam("udp_port", port);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_FALSE(MockCRTProtocol::current_protocol->is_udp);
    EXPECT_TRUE(MockCRTProtocol::current_protocol->is_tcp);
    nh_ptr->deleteParam("udp_port");
}

TEST(ParameterResponseProtocol, noParamSet){
    nh_ptr->deleteParam("qtm_protocol_version");
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(mocap::DEFAULT_PROTOCOL_VERSION, 
              MockCRTProtocol::current_protocol->minor_version);
}

TEST(ParameterResponseProtocol, specific){
    int version = 17;
    nh_ptr->setParam("qtm_protocol_version", version);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(version, 
              MockCRTProtocol::current_protocol->minor_version);
    nh_ptr->deleteParam("qtm_protocol_version");       
}


TEST(ParameterResponseProtocol, tooLow){
    int version = 7;
    nh_ptr->setParam("qtm_protocol_version", version);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(mocap::DEFAULT_PROTOCOL_VERSION,
              MockCRTProtocol::current_protocol->minor_version);
    nh_ptr->deleteParam("qtm_protocol_version");       
}

TEST(ParameterResponseRate, NotSet){
    nh_ptr->deleteParam("frame_rate");
    unsigned int fps = MockCRTProtocol::current_protocol->GetSystemFrequency();
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    double dt = MockCRTProtocol::current_protocol->dt();
    EXPECT_EQ(fps, MockCRTProtocol::current_protocol->frame_rate);
    EXPECT_NEAR(driver.get_dt(), dt, 1E-7);   
}

TEST(ParameterResponseRate, SetExact){
    unsigned int fps = MockCRTProtocol::current_protocol->GetSystemFrequency();
    nh_ptr->setParam("frame_rate", (signed int)fps);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    double dt = MockCRTProtocol::current_protocol->dt();
    EXPECT_EQ(fps, MockCRTProtocol::current_protocol->frame_rate);
    EXPECT_NEAR(driver.get_dt(), dt, 1E-7);
    nh_ptr->deleteParam("frame_rate");
}

TEST(ParameterResponseRate, SetHigh){
    unsigned int fps = MockCRTProtocol::current_protocol->GetSystemFrequency() + 1;
    nh_ptr->setParam("frame_rate", (signed int)fps);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    double dt = MockCRTProtocol::current_protocol->dt();
    EXPECT_EQ(fps - 1, MockCRTProtocol::current_protocol->frame_rate);
    EXPECT_NEAR(driver.get_dt(), dt, 1E-7);
    nh_ptr->deleteParam("frame_rate");
}

TEST(ParameterResponseRate, SetHalf){
    //nh.param("frame_rate", unsigned_frame_rate, 0);
    unsigned int fps = MockCRTProtocol::current_protocol->GetSystemFrequency()/2;
    nh_ptr->setParam("frame_rate", (signed int)fps);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    double dt = MockCRTProtocol::current_protocol->dt();
    EXPECT_EQ(fps,
              MockCRTProtocol::current_protocol->frame_rate);
    EXPECT_NEAR(driver.get_dt(), dt, 1E-7);
    nh_ptr->deleteParam("frame_rate");
}


/*
TEST(ParameterResponse, ){
    nh.param("max_accel", max_accel, 10.0);
}
*/

/*
TEST(ParameterResponse, ){
    nh.param("publish_tf", publish_tf, false);
}
*/

class SubTracker {
    public:
        int pose_called = 0;
        int odom_called = 0;
        double hz = -1.0;
        double pose_x = 0.0;
        double heading_w = 0.0;
        geometry_msgs::PoseStamped last_pose_msg;
        nav_msgs::Odometry last_odom_msg;
    protected:
        float time_last_pose_received = -1.0;
        ros::Subscriber pose_sub;
        
        void measurePublishingFrequency(){
            double now = ros::Time::now().toSec();
            if (time_last_pose_received > 0) {
                double delta = now - time_last_pose_received;
                if (hz < 0){ hz = delta;} 
                else {hz = 0.9*hz + 0.1*delta;}
            }
            time_last_pose_received = now;
        }

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
            pose_called++;
            measurePublishingFrequency();
            pose_x = msg->pose.position.x;
            heading_w = msg->pose.orientation.w;
            last_pose_msg = *msg;
        }

        ros::Subscriber odom_sub;
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
            odom_called++;
            last_odom_msg = *msg;
        }
    public:
        void init(std::string topic_base){
            pose_sub = nh_ptr->subscribe(
                topic_base + "/pose",
                100,
                &SubTracker::poseCallback,
                this);
            odom_sub = nh_ptr->subscribe(
                topic_base + "/odom",
                100,
                &SubTracker::odomCallback,
                this);
        }
        bool comparePose(const Eigen::Vector3f& in_pos, 
                         const MockCRTProtocol::RotMatrix& in_heading){
            Eigen::Quaternionf quat_in_heading(in_heading);
            Eigen::Quaternionf msg_heading(
                last_pose_msg.pose.orientation.w,
                last_pose_msg.pose.orientation.x,
                last_pose_msg.pose.orientation.y,
                last_pose_msg.pose.orientation.z);
            Eigen::Vector3f msg_pos(
                last_pose_msg.pose.position.x,
                last_pose_msg.pose.position.y,
                last_pose_msg.pose.position.z);
            float precision = 1E-7;
            EXPECT_NEAR(msg_pos.x(), in_pos.x(), precision);
            EXPECT_NEAR(msg_pos.y(), in_pos.y(), precision);
            EXPECT_NEAR(msg_pos.z(), in_pos.z(), precision);
            EXPECT_NEAR(msg_heading.x(), quat_in_heading.x(), precision);
            EXPECT_NEAR(msg_heading.y(), quat_in_heading.y(), precision);
            EXPECT_NEAR(msg_heading.z(), quat_in_heading.z(), precision);
            EXPECT_NEAR(msg_heading.w(), quat_in_heading.w(), precision);
            return (quat_in_heading.isApprox(msg_heading, precision)) 
                    && (in_pos.isApprox(msg_pos, precision));
        }
};
class SubscriberFixture : public::testing::Test {
    protected:
    SubTracker pose0_counter;
    SubTracker pose1_counter;
    SubTracker pose2_counter;
    SubTracker pose3_counter;

    void SetUp() override {
        pose0_counter.init("mockModel0");
        pose1_counter.init("mockModel1");
        pose2_counter.init("mockModel2");
        pose3_counter.init("mockModel3");
    }

    ros::Time getStamp(){
        /*Get time stamp from latest message*/
        return pose0_counter.last_pose_msg.header.stamp;
    }
    std::string getFrame(){
        /*Get time stamp from latest message*/
        return pose0_counter.last_pose_msg.header.frame_id;
    }
};

TEST_F(SubscriberFixture, testSubjectList) {
    /* Only follow a few of the available bodies */
    std::vector<std::string> model_list({"mockModel0", "mockModel2"});
    nh_ptr->setParam("model_list", model_list);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    ros::Time mock_time(ros::Time::now());
    uint32_t fps = 100;
    ros::Duration time_step(0, 1000000000/fps);
    int num_steps = 10;
    for (int i=0; i < num_steps; i++){
        mock_time += time_step;
        ros::Time::setNow(mock_time);
        driver.run();
        ros::spinOnce();
    }
    EXPECT_EQ(num_steps, pose0_counter.pose_called);
    EXPECT_EQ(0,         pose1_counter.pose_called);
    EXPECT_EQ(num_steps, pose2_counter.pose_called);
    EXPECT_EQ(0,         pose3_counter.pose_called);
    nh_ptr->deleteParam("model_list");
}

TEST_F(SubscriberFixture, publishing) {
    /* Test that the node is publsihing the correct data 
       to the right topics.
    */
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    ros::Time mock_time(ros::Time::now());
    uint32_t fps = 100;
    ros::Duration time_step(0, 1000000000/fps);
    int num_steps = 10;
    float dt = 1.0/fps;
    Eigen::Vector3f pos(0.0, 0.0, 0.0);
    MockCRTProtocol::RotMatrix heading;
    heading.setIdentity();
    for (int i=0; i < num_steps; i++){
        mock_time += time_step;
        ros::Time::setNow(mock_time);
        MockCRTProtocol::updateState(dt, pos, heading);
        driver.run();
        ros::spinOnce();
    }
    EXPECT_EQ(num_steps, pose0_counter.pose_called);
    EXPECT_EQ(num_steps, pose1_counter.pose_called);
    EXPECT_EQ(num_steps, pose2_counter.pose_called);
    EXPECT_EQ(num_steps, pose3_counter.pose_called);
    // It takes two steps for the KF to initialize and 
    // start publishing full odometry.
    EXPECT_EQ(num_steps-2, pose0_counter.odom_called);
    EXPECT_EQ(num_steps-2, pose1_counter.odom_called);
    EXPECT_EQ(num_steps-2, pose2_counter.odom_called);
    EXPECT_EQ(num_steps-2, pose3_counter.odom_called);
    EXPECT_TRUE(pose0_counter.comparePose(pos/1000, heading));
    EXPECT_NEAR(pos.x()/1000, pose0_counter.pose_x, 1E-7);
    Eigen::Quaternionf quat_heading(heading);
    EXPECT_NEAR(quat_heading.w(), pose0_counter.heading_w, 1E-7);
}

TEST_F(SubscriberFixture, frameIDnotSet) {
    nh_ptr->deleteParam("fixed_frame_id");
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    int num_steps = 10;
    for (int i=0; i < num_steps; i++){
        driver.run();
        ros::spinOnce();
    }
    EXPECT_EQ(std::string("mocap"), pose0_counter.last_pose_msg.header.frame_id);
}

TEST_F(SubscriberFixture, frameIDSet) {
    std::string mock_frame("mock_frame");
    nh_ptr->setParam("fixed_frame_id", mock_frame);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    int num_steps = 10;
    for (int i=0; i < num_steps; i++){
        driver.run();
        ros::spinOnce();
    }
    EXPECT_EQ(mock_frame, pose0_counter.last_pose_msg.header.frame_id);
    nh_ptr->deleteParam("fixed_frame_id");
}


TEST_F(SubscriberFixture, basicJitterCompensation) {
    /* This test is intended to check that the package
       can compensate for varying delays between the 
       QTM server and the computer where the node is runing.
    */
    ros::Time mock_time(ros::Time::now());
    ros::Time::setNow(mock_time);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    int fps = MockCRTProtocol::current_protocol->GetSystemFrequency();
    ros::Duration time_step(0, 1000000000/fps);
    int jitter_arr[] = {1000000, 10000000, 0};
    int num_steps = 200;
    ros::Time previous_stamp;
    for (int i=0; i<num_steps; i++){
        ros::Time::setNow(mock_time);
        driver.run();
        ros::spinOnce();
        ros::Duration jitter(0, jitter_arr[i%3] - jitter_arr[(i-1)%3]);
        mock_time += time_step + jitter;
        if (i > 0) {
            ros::Duration delta(getStamp() - previous_stamp);
            EXPECT_NEAR(time_step.nsec, delta.nsec, 1000);
        }
        previous_stamp = getStamp();
    }
    EXPECT_EQ(num_steps, pose0_counter.pose_called);
}

TEST_F(SubscriberFixture, delayCompensation) {
    /* Check if the constant delay compensation parameter is working
       Since there is no real dely in the test, the time stamps
       of the messsages are expected to be `delay_compensation`
       micro seconds older than ros::Time::now()
    */
    int delay_compensation = 5000; // micro seconds
    nh_ptr->setParam("delay_compensation", delay_compensation);
    ros::Time mock_time(ros::Time::now());
    ros::Time::setNow(mock_time);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    int fps = MockCRTProtocol::current_protocol->GetSystemFrequency();
    ros::Duration time_step(0, 1000000000/fps);
    int num_steps = 100;
    for (int i=0; i<num_steps; i++){
        ros::Time::setNow(mock_time);
        driver.run();
        ros::spinOnce();
        mock_time += time_step;
        ros::Duration delay = ros::Time::now() - getStamp();
        EXPECT_NEAR(delay_compensation, delay.nsec/1000, 1);
    }
    nh_ptr->deleteParam("delay_compensation");
}

TEST_F(SubscriberFixture, framesForJitterCompensation) {
    /* This test is intended to check that the package
       can use multiple frames for jitter compensation.
       Using the `jitter_comp_frames` parameter.
       The driver is expected to go through the 
       first `jitter_comp_frames`, and find the minimum
       latency by comparing the difference between 
       pairs of QTM packet time stamps and the correspoding
       duration between ros::Time::now() when each packet is
       received.
    */
    int comp_frames = 10; // micro seconds
    nh_ptr->setParam("jitter_comp_frames", comp_frames);
    ros::Time mock_time(ros::Time::now());
    ros::Time::setNow(mock_time);
    mocap::TestQualisysDriver driver(*nh_ptr);
    ASSERT_TRUE(driver.init());
    int fps = MockCRTProtocol::current_protocol->GetSystemFrequency();
    ros::Duration time_step(0, 1000000000/fps);
    // int jitter_arr[] = {1000000, 10000000, 0};
    int jitter_arr[] = {0, 0, 0};
    int pub_frames = 5; // Frames that should be published
    int num_steps = comp_frames + pub_frames;
    ros::Time previous_stamp;
    for (int i=0; i<num_steps; i++){
        ros::Time::setNow(mock_time);
        driver.run();
        ros::spinOnce();
        ros::Duration jitter(0, jitter_arr[i%3] - jitter_arr[(i-1)%3]);
        mock_time += time_step + jitter;
        if (i > comp_frames) {
            ros::Duration delta(getStamp() - previous_stamp);
            EXPECT_NEAR(time_step.nsec, delta.nsec, 1000);
            ros::Duration delay = ros::Time::now() - getStamp();
            EXPECT_EQ(0, delay.nsec/1000);
        }
        if (i >= comp_frames) previous_stamp = getStamp();
    }
    EXPECT_EQ(pub_frames, pose0_counter.pose_called);
    nh_ptr->deleteParam("jitter_comp_frames");
}

TEST_F(SubscriberFixture, dropOldFrame) {
    /* Check that the driver drops old frames.
       If a frame older than the most recent one
       arrives it should be dropped without being published.
    */
    ros::Time mock_time(ros::Time::now());
    ros::Time::setNow(mock_time);
    MockCRTProtocol* protocol_ptr;
    mocap::TestQualisysDriver driver(*nh_ptr);
    protocol_ptr = MockCRTProtocol::current_protocol;
    ASSERT_TRUE(driver.init());
    ros::Duration rt_time_step(0, 5000000);
    uint64_t time_step = protocol_ptr->GetSystemRate();
    driver.run();
    ros::spinOnce();
    // Make the time stamp of the QTM packet older
    protocol_ptr->spoof_packet.packet_header.time_stamp -= time_step;
    ros::Time::setNow(mock_time + rt_time_step);
    driver.run();
    ros::spinOnce();
    //Only one pose should have been published
    EXPECT_EQ(1, pose0_counter.pose_called);
}

TEST_F(SubscriberFixture, trackedUntracked) {
    /* Check that the driver drops old frames.
       If a frame older than the most recent one
       arrives it should be dropped without being published.
    */
    ros::Time mock_time(ros::Time::now());
    ros::Time::setNow(mock_time);
    MockCRTProtocol* protocol_ptr;
    mocap::TestQualisysDriver driver(*nh_ptr);
    protocol_ptr = MockCRTProtocol::current_protocol;
    ASSERT_TRUE(driver.init());
    ros::Duration rt_time_step(0, 5000000);
    uint64_t time_step = protocol_ptr->GetSystemRate();
    driver.run();
    ros::spinOnce();
    // Make the time stamp of the QTM packet older
    protocol_ptr->spoof_packet.packet_header.time_stamp -= time_step;
    ros::Time::setNow(mock_time + rt_time_step);
    driver.run();
    ros::spinOnce();
    //Only one pose should have been published
    EXPECT_EQ(1, pose0_counter.pose_called);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    ros::console::levels::Level level = ros::console::levels::Fatal; 
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level)){
        ros::console::notifyLoggerLevelsChanged();
    }
    nh_ptr = &nh;
    nh_ptr->setParam("server_address", "127.0.0.1");
    return RUN_ALL_TESTS();
}