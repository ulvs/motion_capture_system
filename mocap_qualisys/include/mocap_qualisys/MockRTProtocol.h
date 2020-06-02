
#include <string>
#include <cstdint>
#include <sstream>
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
        uint32_t component_count= 1;   // Number of data components 
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
        //typedef rot_matrix Eigen::Matrix<float, 3, 3, Eigen::ColMajor>;
    private:
        bool is_connected = false;
        bool has_6dof_data = false;
        bool is_streaming = false;
        std::string maErrorStr;
        CRTProtocol::SSettings6DOF mvs6DOFSettings;
        CRTPacket  mockRTPacket;
        Packet6DOF spoof_packet;
        std::vector<char> mockDataBuff;
        ros::Time mock_time;

    private:
    void init_6DOF_packet(Packet6DOF& packet){
        packet.packet_header.size = sizeof(packet);
        packet.packet_header.time_stamp = 0;
        packet.packet_header.frame_number = 0;
        packet.packet_header.component_count = 1;

        for (uint32_t i; i<NUM_6DOF_BODIES; i++) {
            packet.data[i].x = 0.0;
            packet.data[i].y = 0.0;
            packet.data[i].z = 0.0;
        }
        
    }

    void update_6DOF_packet(Packet6DOF& packet){
        packet.packet_header.time_stamp += 1000000/GetSystemFrequency();
        packet.packet_header.frame_number += 1;
        float dt = 1.0/GetSystemFrequency();
        for (uint32_t i=0; i<NUM_6DOF_BODIES; i++){
            Eigen::Vector3f vel_vector(dt, 0, 0);
            Eigen::Vector3f pos_vector(
                packet.data[i].x, 
                packet.data[i].y, 
                packet.data[i].z);
            Eigen::AngleAxis<float> rot(3.141*dt*0.25, Eigen::Vector3f(0,0,1));
            Eigen::Matrix<float, 3, 3, Eigen::ColMajor> heading_matrix(packet.data[i].rotation);
            // update
            pos_vector += heading_matrix * vel_vector * dt;
            heading_matrix *= rot.toRotationMatrix();
            // Put back
            packet.data[i].x = pos_vector.x();
            packet.data[i].y = pos_vector.y();
            packet.data[i].z = pos_vector.z();
            memcpy(packet.data[i].rotation, heading_matrix.data(), sizeof(float)*9);
        }
        ros::Duration time_shift(0, 1000000000/GetSystemFrequency());
        mock_time += time_shift;
        ros::Time::setNow(mock_time);
    }


    public:
        int disconnect_calls = 0;
        //MockRTProtocol():CRTProtocol::CRTProtocol(){}
        
        bool Connect(
            const char* pServerAddr, 
            unsigned short nPort, 
            unsigned short* pnUDPServerPort = nullptr,
            int nMajorVersion  = MAJOR_VERSION, 
            int nMinorVersion = MINOR_VERSION, 
            bool bBigEndian = false) 
        {
            //ROS_WARN("Mock connection");
            init_6DOF_packet(spoof_packet);
            mock_time = ros::Time::now();
            is_connected = true;
            //mockRTPacket.SetEndianness(true);
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
            return is_connected;
        }

        unsigned int Get6DOFBodyCount() const {
            if (!has_6dof_data){
                return NUM_6DOF_BODIES;
            } else {
                return 0;
            }
        }

        const char* Get6DOFBodyName(unsigned int index){
            return mvs6DOFSettings.bodySettings.at(index).oName.c_str();
        }

        unsigned int GetSystemFrequency(){
            return 100;
        }

        const char* GetErrorString(){
            return maErrorStr.c_str();
        }

        CRTPacket* GetRTPacket(){
            return &mockRTPacket;
        }

        bool StreamFrames(EStreamRate eRate, unsigned int nRateArg, unsigned short nUDPPort, const char* pUDPAddr, const char* components){
            is_streaming = true;
            return is_streaming;
        } 

        bool StreamFrames(EStreamRate eRate, 
                          unsigned int nRateArg, 
                          unsigned short nUDPPort, 
                          const char* pUDPAddr,
                          unsigned int nComponentType,
                          const SComponentOptions& componentOptions = { }) {
            is_streaming = true;
            return is_streaming;
        }

        int ReceiveRTPacket(CRTPacket::EPacketType &eType, bool bSkipEvents = true, int nTimeout = cWaitForDataTimeout){
            if (!is_connected) {
                maErrorStr = "MOCK not connected (in ReceiveRTPacket)";
                return -1;
            } else if (!is_streaming) {
                maErrorStr = "MOCK not streaming (in ReceiveRTPacket)";
                return -1;
            }
            update_6DOF_packet(spoof_packet);
            char* ptr_begin = (char*)&spoof_packet;
            char* ptr_end = ptr_begin + sizeof(spoof_packet);
            mockDataBuff = std::vector<char>(ptr_begin, ptr_end);
            mockRTPacket.SetData(mockDataBuff.data());
            eType = mockRTPacket.GetType(mockDataBuff.data(), false); 
            return sizeof(spoof_packet);
        }
};
