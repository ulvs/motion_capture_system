
#include <string>
#include <cstdint>
#include <limits>
#include <math.h>
#include <sstream>
#include <stdexcept>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "RTProtocol.h"
#include "RTPacket.h"


class MockCRTProtocol : public CRTProtocol{
    public:
    static const unsigned int NUM_6DOF_BODIES = 4;
    struct PacketHeader
    {
        uint32_t size;  //Size of packet in bytes
        uint32_t type = CRTPacket::EPacketType::PacketData; 
        uint64_t time_stamp; // in micro seconds
        uint32_t frame_number;
        uint32_t component_count = 1;   // Number of data components 
        // the 6DOF header and the data counts as one component.
        // If e.g. 3D components was also included they would be another component.
    };
    struct Packet6DOFData{
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        float rotation[9] = {1.0, 0.0, 0.0, 
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0}; 
    };
    struct Packet6DOFHeader{
        uint32_t component_size = sizeof(Packet6DOFData) * NUM_6DOF_BODIES; 
        uint32_t component_type = CRTPacket::EComponentType::Component6d;
        uint32_t body_count = NUM_6DOF_BODIES;
        uint16_t drop_rate = 0;
        uint16_t out_of_sync_rate = 0;
    };
    struct Packet6DOF{
        PacketHeader packet_header;
        Packet6DOFHeader header6DOF;
        Packet6DOFData data[NUM_6DOF_BODIES];
    };

    static float linear_velocity; //  mm/second
    static float angular_velocity; // rad/s;
    typedef Eigen::Matrix<float, 3, 3, Eigen::ColMajor> RotMatrix;
    static void updateState(float dt, Eigen::Vector3f &pos, RotMatrix &heading){
        Eigen::Vector3f vel_vector(linear_velocity, 0, 0);
        pos += heading * vel_vector * dt;
        Eigen::AngleAxis<float> rot(angular_velocity*dt, Eigen::Vector3f(0,0,1));
        heading *= rot.toRotationMatrix();
    }

    std::string maErrorStr;
    CRTProtocol::SSettings6DOF mvs6DOFSettings;
    CRTPacket  mockRTPacket;
    Packet6DOF spoof_packet;
    Packet6DOF  spoof_nan_packet;
    bool next_packet_is_nan = false;
    std::vector<char> mockDataBuff;
    ros::Time mock_time;

    /* Connection status variables */
    bool is_connected = false;
    bool has_camera_settings = false;
    bool has_6dof_data = false;
    bool is_streaming = false;
    std::string server_address = "";
    short base_port = 0;
    bool is_tcp = false;
    bool is_udp = false;
    short udp_port = 0;
    int minor_version = 1;

    const unsigned int system_frequency = 100;
    unsigned int GetSystemFrequency() const {
        if (has_camera_settings == false){
            return 0;
        } else {
            return system_frequency;
        }
    }

    unsigned int GetSystemRate(){
        double sys_freq = 1000000.0/GetSystemFrequency();
        sys_freq += 0.5 - (sys_freq<0);
        return (unsigned int)sys_freq;
    }

    void init_6DOF_packet(Packet6DOF& packet){
        packet.packet_header.size = sizeof(packet);
        packet.packet_header.time_stamp = 0;
        packet.packet_header.frame_number = 0;
        packet.packet_header.component_count = 1;
        Eigen::Matrix<float, 3, 3, Eigen::ColMajor> rot_matrix;
        rot_matrix.setIdentity();
        for (uint32_t i; i<NUM_6DOF_BODIES; i++) {
            packet.data[i].x = 0.0;
            packet.data[i].y = 0.0;
            packet.data[i].z = 0.0;
            memcpy(packet.data[i].rotation, rot_matrix.data(), sizeof(float)*9);
        }
    }

    void init_6DOF_nan_packet(Packet6DOF& packet){
        packet.packet_header.size = sizeof(packet);
        packet.packet_header.time_stamp = 0;
        packet.packet_header.frame_number = 0;
        packet.packet_header.component_count = 1;
        const int num_floats = 9;
        const float init_values[num_floats] = {
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
        };
        for (uint32_t i; i<NUM_6DOF_BODIES; i++) {
            packet.data[i].x = std::numeric_limits<float>::quiet_NaN();
            packet.data[i].y = std::numeric_limits<float>::quiet_NaN();
            packet.data[i].z = std::numeric_limits<float>::quiet_NaN();
            memcpy(packet.data[i].rotation, init_values, sizeof(float)*num_floats);
        }
    }

    void update_6DOF_packet(Packet6DOF& packet){
        packet.packet_header.time_stamp += GetSystemRate();
        packet.packet_header.frame_number += 1;
        float dt = 1.0/GetSystemFrequency();
        // Move the bodies
        for (uint32_t i=0; i<NUM_6DOF_BODIES; i++){
            // extract vectors/matrices that can be changed 
            Eigen::Vector3f pos_vector(
                packet.data[i].x,
                packet.data[i].y,
                packet.data[i].z);
            Eigen::Matrix<float, 3, 3, Eigen::ColMajor> heading_matrix(packet.data[i].rotation);
            updateState(dt, pos_vector, heading_matrix);
            // Put back into struct
            packet.data[i].x = pos_vector.x();
            packet.data[i].y = pos_vector.y();
            packet.data[i].z = pos_vector.z();
            memcpy(packet.data[i].rotation, heading_matrix.data(), sizeof(float)*9);
        }
    }

    int disconnect_calls = 0;
    bool Connect(
        const char* pServerAddr, 
        unsigned short nPort, 
        unsigned short* pnUDPServerPort = nullptr,
        int nMajorVersion  = MAJOR_VERSION, 
        int nMinorVersion = MINOR_VERSION, 
        bool bBigEndian = false) 
    {
        init_6DOF_packet(spoof_packet);
        init_6DOF_nan_packet(spoof_nan_packet);
        server_address = pServerAddr;
        base_port = nPort;
        if (pnUDPServerPort != nullptr){
            is_udp = true;
            udp_port = *pnUDPServerPort;
        } else {
            is_tcp = true;
        }
        minor_version = nMinorVersion;
        mock_time = ros::Time::now();
        is_connected = true;
        return true;
    }

    void Disconnect() {
        is_connected = false;
        disconnect_calls++;
    }

    bool Connected() {
        return is_connected;
    }

    bool Read6DOFSettings(bool &bDataAvailable){
        bDataAvailable = is_connected;
        if (!is_connected) {
            maErrorStr = "MOCK not initialized (in Read6DOFSettings)";
        } else {
            // Populate settings with mock models
            for (uint32_t i=0; i<NUM_6DOF_BODIES; i++){
                CRTProtocol::SSettings6DOFBody body_settings;
                std::stringstream name;
                name << "mockModel" << i;
                body_settings.oName = name.str();
                mvs6DOFSettings.bodySettings.push_back(body_settings);
            }
        }
        has_6dof_data = is_connected;
        return bDataAvailable;
    }

    bool ReadCameraSystemSettings(){
        if (!is_connected) {
            maErrorStr = "MOCK not initialized (in ReadCameraSystemSettings)";
        } 
        has_camera_settings = is_connected;
        return has_camera_settings;
    }

    unsigned int Get6DOFBodyCount() const {
        if (!has_6dof_data){
            return NUM_6DOF_BODIES;
        } else {
            return 0;
        }
    }

    const char* Get6DOFBodyName(unsigned int index) const {
        return mvs6DOFSettings.bodySettings.at(index).oName.c_str();
    }

    const char* GetErrorString(){
        return maErrorStr.c_str();
    }

    void setNextPacketNan(){
        next_packet_is_nan = true;
    }

    CRTPacket* GetRTPacket(){
        return &mockRTPacket;
    }

    unsigned int frame_rate = 0;

    bool StreamFrames(EStreamRate eRate, 
                        unsigned int nRateArg, 
                        unsigned short nUDPPort, 
                        const char* pUDPAddr,
                        unsigned int nComponentType,
                        const SComponentOptions& componentOptions = { }) {
        if (is_connected == false) {
            return false;
        }
        if(nComponentType != cComponent6d){
            throw std::runtime_error(
                "Invalid nComponentType argument passed to stream frames");
        }
        if (eRate == EStreamRate::RateAllFrames){
            frame_rate = system_frequency;
        }
        else if (nRateArg == system_frequency){
            frame_rate = nRateArg;
        } 
        else if (nRateArg > 0) {
            int divisor = 2;
            frame_rate = system_frequency;
            while (frame_rate >= nRateArg){
                frame_rate = system_frequency/divisor;
                divisor++;
            }
        } else {
            throw std::runtime_error("RateFrequency mode requested with nRateArg = 0");
        }
        is_streaming = true;
        return is_streaming;
    }

    double dt(){
        if (frame_rate > GetSystemFrequency()) {
            return 1.0/GetSystemFrequency();
        } else if (frame_rate == 0) {
            return 0.0; 
        } else {
            return 1.0/frame_rate;
        }
    }

    const static int len_delay = 10;
    static int delay_arr[len_delay];
    int ReceiveRTPacket(CRTPacket::EPacketType &eType, bool bSkipEvents = true, int nTimeout = cWaitForDataTimeout){
        if (!is_connected) {
            maErrorStr = "MOCK not connected (in ReceiveRTPacket)";
            return -1;
        } else if (!is_streaming) {
            maErrorStr = "MOCK not streaming (in ReceiveRTPacket)";
            return -1;
        }
        update_6DOF_packet(spoof_packet);
        char* ptr_begin;
        char* ptr_end;
        if (next_packet_is_nan){
            ROS_FATAL("IS NAN!");
            ptr_begin = (char*)&spoof_nan_packet;
            ptr_end = ptr_begin + sizeof(spoof_nan_packet);
            next_packet_is_nan = false;
        } else {
            ptr_begin = (char*)&spoof_packet;
            ptr_end = ptr_begin + sizeof(spoof_packet);
        }
        mockDataBuff = std::vector<char>(ptr_begin, ptr_end);
        mockRTPacket.SetData(mockDataBuff.data());
        eType = mockRTPacket.GetType(mockDataBuff.data(), false);
        return sizeof(spoof_packet);
    }
};

float MockCRTProtocol::linear_velocity = 1000.0;
float MockCRTProtocol::angular_velocity = M_PI/4.0;
