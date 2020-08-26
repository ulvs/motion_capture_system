
#include <string>
#include <math.h>
#include <memory>
#include <ros/ros.h>
#include <ros/console.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include <gtest/gtest.h>
#include "gmock/gmock.h"
#include <mocap_qualisys/RTProtocol.h>
#include <mocap_qualisys/QualisysDriver.h>

ros::NodeHandle* nh_ptr;
typedef std::shared_ptr<CRTProtocol> ProtocolPtr;
typedef std::shared_ptr<MockCRTProtocol> MockProtocolPtr;

void resetParameters(){
    nh_ptr->setParam("server_address", "127.0.0.1");
    nh_ptr->deleteParam("server_base_port");
    nh_ptr->deleteParam("model_list");
    nh_ptr->deleteParam("frame_rate");
    nh_ptr->deleteParam("publish_tf");
    nh_ptr->deleteParam("fixed_frame_id");
    nh_ptr->deleteParam("udp_port");
    nh_ptr->deleteParam("qtm_protocol_version");
    nh_ptr->deleteParam("delay_compensation");
    nh_ptr->deleteParam("jitter_comp_frames");
}

/* Check if the tests are working at all */
TEST(TestSuiteTestSuite, testSuiteTest) {
    EXPECT_TRUE(true);
}

TEST(QualisysSuite, contructor) {
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    EXPECT_FALSE(driver.isInitialized());
}

TEST(QualisysSuite, testInit) {
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    EXPECT_FALSE(driver.isInitialized());
    ASSERT_TRUE(driver.init());
    EXPECT_TRUE(driver.isInitialized());
}

TEST(QualisysSuite, testRun) {
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_TRUE(prot_ptr->is_connected);
    EXPECT_TRUE(driver.run());
}

TEST(ParameterResponseAddress, serverAddress){
    std::string prev_address;
    nh_ptr->param("server_address", prev_address, std::string("127.0.0.1"));
    std::string address("192.0.0.1");
    nh_ptr->setParam("server_address", address);
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(address, prot_ptr->server_address);
    nh_ptr->setParam("server_address", prev_address);
    ROS_ERROR("Previous address: %s", prev_address.c_str());
}

TEST(ParameterResponseAddress, noServerAddress){
    std::string prev_address;
    nh_ptr->param("server_address", prev_address, std::string("127.0.0.1"));
    nh_ptr->deleteParam("server_address");
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    EXPECT_FALSE(driver.init());
    nh_ptr->setParam("server_address", prev_address);
}

TEST(ParameterResponsePort, basePortDefault){
    nh_ptr->deleteParam("server_base_port");
    short default_port = 22222;
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(default_port, prot_ptr->base_port);
}

TEST(ParameterResponsePort, basePortSet){
    short port = 20000;
    nh_ptr->setParam("server_base_port", port);
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(port, prot_ptr->base_port);
    nh_ptr->deleteParam("server_base_port");
}

TEST(ParameterResponseProtocol, udpRandomPortNoParamSet){
    nh_ptr->deleteParam("udp_port");
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_TRUE(prot_ptr->is_udp);
    EXPECT_FALSE(prot_ptr->is_tcp);
}

TEST(ParameterResponseProtocol, udpRandomPortParam0){
    // Protocol chould be UDP and port number will be random
    short port = 0;
    nh_ptr->setParam("udp_port", port);
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_TRUE(prot_ptr->is_udp);
    nh_ptr->deleteParam("udp_port");
}

TEST(ParameterResponseProtocol, udpSpecificPort){
    // Specific UDP port
    short port = 20000;
    nh_ptr->setParam("udp_port", port);
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_TRUE(prot_ptr->is_udp);
    EXPECT_EQ(port, prot_ptr->udp_port);
    nh_ptr->deleteParam("udp_port");
}

TEST(ParameterResponseProtocol, tcpRandomPort){
    // Specific UDP port
    short port = -1;
    nh_ptr->setParam("udp_port", port);
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_FALSE(prot_ptr->is_udp);
    EXPECT_TRUE(prot_ptr->is_tcp);
    nh_ptr->deleteParam("udp_port");
}

TEST(ParameterResponseProtocol, noParamSet){
    nh_ptr->deleteParam("qtm_protocol_version");
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(mocap::DEFAULT_PROTOCOL_VERSION, 
              prot_ptr->minor_version);
}

TEST(ParameterResponseProtocol, specific){
    int version = 17;
    nh_ptr->setParam("qtm_protocol_version", version);
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(version, 
              prot_ptr->minor_version);
    nh_ptr->deleteParam("qtm_protocol_version");       
}

TEST(ParameterResponseProtocol, tooLow){
    int version = 7;
    nh_ptr->setParam("qtm_protocol_version", version);
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(mocap::DEFAULT_PROTOCOL_VERSION,
              prot_ptr->minor_version);
    nh_ptr->deleteParam("qtm_protocol_version");       
}

TEST(ParameterResponseRate, NotSet){
    nh_ptr->deleteParam("frame_rate");
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    unsigned int fps = prot_ptr->system_frequency;
    ASSERT_TRUE(driver.init());
    double dt = prot_ptr->dt();
    EXPECT_EQ(fps, prot_ptr->frame_rate);
    EXPECT_NEAR(driver.getDt(), dt, 1E-7);   
}

TEST(ParameterResponseRate, SetExact){
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    unsigned int fps = prot_ptr->system_frequency;
    nh_ptr->setParam("frame_rate", (signed int)fps);
    ASSERT_TRUE(driver.init());
    double dt = prot_ptr->dt();
    EXPECT_EQ(fps, prot_ptr->frame_rate);
    EXPECT_NEAR(driver.getDt(), dt, 1E-7);
    nh_ptr->deleteParam("frame_rate");
}

TEST(ParameterResponseRate, SetHigh){
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    unsigned int fps = prot_ptr->system_frequency + 1;
    nh_ptr->setParam("frame_rate", (signed int)fps);
    ASSERT_TRUE(driver.init());
    double dt = prot_ptr->dt();
    EXPECT_EQ(fps - 1, prot_ptr->frame_rate);
    EXPECT_NEAR(driver.getDt(), dt, 1E-7);
    nh_ptr->deleteParam("frame_rate");
}

TEST(ParameterResponseRate, SetHalf){
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    unsigned int fps = prot_ptr->system_frequency/2;
    nh_ptr->setParam("frame_rate", (signed int)fps);
    ASSERT_TRUE(driver.init());
    double dt = prot_ptr->dt();
    EXPECT_EQ(fps,
              prot_ptr->frame_rate);
    EXPECT_NEAR(driver.getDt(), dt, 1E-7);
    nh_ptr->deleteParam("frame_rate");
}

TEST(ParameterResponseRate, SetOneBelow){
    MockProtocolPtr prot_ptr(new MockCRTProtocol);
    mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    unsigned int fps = prot_ptr->system_frequency - 1;
    nh_ptr->setParam("frame_rate", (signed int)fps);
    ASSERT_TRUE(driver.init());
    double dt = prot_ptr->dt();
    EXPECT_EQ(prot_ptr->system_frequency/2,
              prot_ptr->frame_rate);
    EXPECT_NEAR(driver.getDt(), dt, 1E-7);
    nh_ptr->deleteParam("frame_rate");
}


// /*
// TEST(ParameterResponse, ){
//     nh.param("max_accel", max_accel, 10.0);
// }
// */

// /*
// TEST(ParameterResponse, ){
//     nh.param("publish_tf", publish_tf, false);
// }
// */

class SubTracker {
    public:
        int pose_called = 0;
        int odom_called = 0;
        double hz = -1.0;
        double pose_x = 0.0;
        double heading_w = 0.0;
        geometry_msgs::PoseStamped last_pose_msg;
        nav_msgs::Odometry last_odom_msg;
        bool pose_received = false;
        bool odom_received = false;
    protected:
        ros::Time time_last_pose_received = ros::Time(0, 0);
        ros::Subscriber pose_sub;
        
        void measurePublishingFrequency(){
            ros::Time now = ros::Time::now();
            if (time_last_pose_received.toSec() > 0) {
                ros::Duration delta = now - time_last_pose_received;
                if (hz < 0){ hz = delta.toSec();} 
                else {hz = 0.9*hz + 0.1*delta.toSec();}
            }
            time_last_pose_received = now;
        }

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
            pose_called++;
            measurePublishingFrequency();
            pose_x = msg->pose.position.x;
            heading_w = msg->pose.orientation.w;
            last_pose_msg = *msg;
            pose_received = true;
        }

        ros::Subscriber odom_sub;
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
            odom_called++;
            last_odom_msg = *msg;
            odom_received = true;
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

typedef std::unique_ptr<mocap::QualisysDriver> DriverPtr; 
class SubscriberFixture : public::testing::Test {
    protected:
    MockProtocolPtr prot_ptr;
    DriverPtr driver;
    SubTracker pose0_counter;
    SubTracker pose1_counter;
    SubTracker pose2_counter;
    SubTracker pose3_counter;
    ros::Time mock_time;
    int fps;
    ros::Duration time_step;
    double dt;

    void SetUp() override {
        mock_time = ros::Time::now();
        ros::Time::setNow(mock_time);
        resetParameters();
        prot_ptr = MockProtocolPtr(new MockCRTProtocol);
        pose0_counter.init("mockModel0");
        pose1_counter.init("mockModel1");
        pose2_counter.init("mockModel2");
        pose3_counter.init("mockModel3");
        fps = prot_ptr->system_frequency;
    }

    void StartDriver() {
        driver = DriverPtr(new mocap::QualisysDriver(*nh_ptr, prot_ptr));
        ASSERT_TRUE(driver->init());
        time_step = ros::Duration(0, round(1000000000.0/fps));
        dt = time_step.toSec();
    }

    void Run() {
        driver->run();
        ros::spinOnce();   
    }

    void TearDown() override {
        resetParameters();
    }

    bool poseReceived(){
        return pose0_counter.pose_received;
    }

    bool odomReceived(){
        return pose0_counter.odom_received;
    }

    ros::Time getStamp(){
        /*Get time stamp from latest message*/
        return pose0_counter.last_pose_msg.header.stamp;
    }

    std::string getFrame(){
        /*Get frame_id from latest message*/
        return pose0_counter.last_pose_msg.header.frame_id;
    }

    ros::Time getOdomStamp(){
        /*Get time stamp from latest odom message*/
        return pose0_counter.last_odom_msg.header.stamp;
    }

    std::string getOdomFrame(){
        /*Get frame_id from latest odom message*/
        return pose0_counter.last_odom_msg.header.frame_id;
    }
};

TEST_F(SubscriberFixture, testSubjectList) {
    /* Only track selected bodies */ 
    std::vector<std::string> model_list({"mockModel0", "mockModel2"});
    nh_ptr->setParam("model_list", model_list);
    StartDriver();
    int num_steps = 10;
    for (int i=0; i < num_steps; i++){
        mock_time += time_step;
        ros::Time::setNow(mock_time);
        Run();
    }
    EXPECT_EQ(num_steps, pose0_counter.pose_called);
    EXPECT_EQ(0,         pose1_counter.pose_called);
    EXPECT_EQ(num_steps, pose2_counter.pose_called);
    EXPECT_EQ(0,         pose3_counter.pose_called);
}

TEST_F(SubscriberFixture, publishing) {
    /* Test that the node is publsihing the correct data 
       to the right topics.
    */
    // mocap::QualisysDriver driver(*nh_ptr, prot_ptr);
    // ASSERT_TRUE(driver.init());
    StartDriver();
    int num_steps = 10;
    Eigen::Vector3f pos(0.0, 0.0, 0.0);
    MockCRTProtocol::RotMatrix heading;
    heading.setIdentity();
    for (int i=0; i < num_steps; i++){
        mock_time += time_step;
        ros::Time::setNow(mock_time);
        MockCRTProtocol::updateState(dt, pos, heading);
        Run();
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
    StartDriver();
    int num_steps = 10;
    for (int i=0; i < num_steps; i++){
        Run();
    }
    EXPECT_EQ(std::string("mocap"), pose0_counter.last_pose_msg.header.frame_id);
}

TEST_F(SubscriberFixture, frameIDSet) {
    std::string mock_frame("mock_frame");
    nh_ptr->setParam("fixed_frame_id", mock_frame);
    StartDriver();
    int num_steps = 10;
    for (int i=0; i < num_steps; i++){
        Run();
    }
    EXPECT_EQ(mock_frame, pose0_counter.last_pose_msg.header.frame_id);
    EXPECT_EQ(mock_frame, pose0_counter.last_odom_msg.header.frame_id);
}


TEST_F(SubscriberFixture, basicJitterCompensation) {
    /* This test is intended to check that the package
       can compensate for varying delays between the 
       QTM server and the computer where the node is runing.
    */
    StartDriver();
    int jitter_arr[] = {1000000, 10000000, 0};
    int num_steps = 200;
    ros::Time previous_stamp;
    for (int i=0; i<num_steps; i++){
        ros::Time::setNow(mock_time);
        Run();
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
    StartDriver();
    int num_steps = 100;
    for (int i=0; i<num_steps; i++){
        ros::Time::setNow(mock_time);
        Run();
        mock_time += time_step;
        ros::Time now = ros::Time::now();
        ros::Duration pose_delay = now - getStamp();
        ros::Duration odom_delay = now - getOdomStamp();
        if (poseReceived()){
            EXPECT_NEAR(delay_compensation, pose_delay.nsec/1000, 1);
        }
        if (odomReceived()){
            EXPECT_NEAR(delay_compensation, odom_delay.nsec/1000, 1);
        }
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
    StartDriver();
    // int jitter_arr[] = {1000000, 10000000, 0};
    int jitter_arr[] = {0, 0, 0};
    int pub_frames = 5; // Frames that should be published
    int num_steps = comp_frames + pub_frames;
    ros::Time previous_stamp;
    for (int i=0; i<num_steps; i++){
        ros::Time::setNow(mock_time);
        Run();
        ros::Duration jitter(0, jitter_arr[i%3] - jitter_arr[(i-1)%3]);
        mock_time += time_step + jitter;
        if (i > comp_frames) {
            ros::Duration delta(getStamp() - previous_stamp);
            EXPECT_NEAR(time_step.nsec, delta.nsec, 1000);
            ros::Duration delay = ros::Time::now() - getStamp();
            EXPECT_EQ(0, round(delay.nsec/1000.0));
        }
        if (i >= comp_frames) previous_stamp = getStamp();
    }
    EXPECT_EQ(pub_frames, pose0_counter.pose_called);   
}

TEST_F(SubscriberFixture, dropOldFrame) {
    /* Check that the driver drops old frames.
       If a frame older than the most recent one
       arrives it should be dropped without being published.
    */
    StartDriver();
    ros::Duration rt_time_step(0, 5000000);
    uint64_t false_time_step = prot_ptr->frame_rate;
    Run();
    // Make the time stamp of the QTM packet older
    prot_ptr->spoof_packet.packet_header.time_stamp -= false_time_step;
    ros::Time::setNow(mock_time + rt_time_step);
    //Only one pose should have been published
    EXPECT_EQ(1, pose0_counter.pose_called);
}

TEST_F(SubscriberFixture, trackedUntracked) {
    /* Check that the driver keeps track of when a body is tracked
       and not tracked by the mocap system. 
       When not tracked the 6DOF component for the body will
       only contain NaN. 
       A pose should not be published and the KF should be reset.
       When the target is tracked again a pose should immediately 
       be published and the KF should start initializing.
    */
    
    ros::Time::setNow(mock_time);
    StartDriver();
    prot_ptr->setNextPacketNan();
    Run();
    int num_steps = 50;
    int nan_interval = 10;
    EXPECT_EQ(0, pose0_counter.pose_called);
    int expected_poses = 0;
    int expected_odoms = 0;
    ROS_FATAL("pose_x: %f", pose0_counter.pose_x);
    ROS_FATAL("w: %f", pose0_counter.heading_w);
    for (int i=0; i < num_steps; i++){
        if (i % nan_interval == 0 
            || i % nan_interval == 1 
            || i % nan_interval == 2 
            || i % nan_interval == 3){
            prot_ptr->setNextPacketNan();
        } else {
            expected_poses++;
        }
        if (i % nan_interval != 0 
            && i % nan_interval != 1 
            && i % nan_interval != 2 
            && i % nan_interval != 3
            && i % nan_interval != 4 
            && i % nan_interval != 5){
                expected_odoms++;
            }
        mock_time += time_step;
        ros::Time::setNow(mock_time);
        Run();
        EXPECT_EQ(expected_poses, pose0_counter.pose_called);
        EXPECT_EQ(expected_odoms, pose0_counter.odom_called);

    }
    // EXPECT_EQ(num_steps - nan_interval, pose0_counter.pose_called);
    // EXPECT_EQ(num_steps - nan_interval*3, pose0_counter.odom_called);
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
    resetParameters();
    return RUN_ALL_TESTS();
}