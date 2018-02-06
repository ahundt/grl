#ifndef GRL_READ_DATA_FROM_BINARY
#define GRL_READ_DATA_FROM_BINARY

#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE

#include "flatbuffers/flatbuffers.h"
#include "grl/flatbuffer/JointState_generated.h"
#include "grl/flatbuffer/ArmControlState_generated.h"
#include "grl/flatbuffer/KUKAiiwa_generated.h"
#include "grl/flatbuffer/LinkObject_generated.h"
#include "grl/flatbuffer/Time_generated.h"
#include "grl/flatbuffer/flatbuffer.hpp"
#include "grl/flatbuffer/FusionTrack_generated.h"
#include "grl/flatbuffer/LogKUKAiiwaFusionTrack_generated.h"
#include <thirdparty/fbs_tk/fbs_tk.hpp>

#include <cstdio> // For printing and file access.
#include <iostream>
#include <cerrno>

#include <fstream>
#include <string>
#include <vector>
#include <set>
#include <iterator>
#include <algorithm>
namespace grl {
    struct kuka_tag {};
    struct fusiontracker_tag {};
    enum TimeType { device_time = 0, local_request_time = 1, local_receive_time = 2 };

    std::vector<int64_t> getTimeStamp(const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP, kuka_tag, TimeType time_type){
        auto states = kukaStatesP->states();
        std::vector<int64_t> timeStamp;
        std::size_t state_size = states->size();
        for(int i = 1; i<state_size; ++i){
            auto KUKAiiwaState = states->Get(i);
            auto FRIMessage = KUKAiiwaState->FRIMessage();
            auto timeEvent = FRIMessage->timeStamp();
            switch(time_type) {
                case device_time:{
                    timeStamp.push_back(timeEvent->device_time());
                    break;
                }
                case local_request_time:{
                    timeStamp.push_back(timeEvent->local_request_time());
                    break;
                }
                case local_receive_time:{
                    timeStamp.push_back(timeEvent->local_receive_time());
                    break;
                }
                default:{
                     std::perror("No time type!");
                }
            }
        }
        return timeStamp;
    }

    std::vector<int64_t> getTimeStamp(const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP, fusiontracker_tag, TimeType time_type){

        auto states = logKUKAiiwaFusionTrackP->states();
        std::vector<int64_t> timeStamp;

        std::size_t state_size = states->size();
        //Eigen::VectoXd time_stamp(state_size);
        for(int i = 0; i<state_size; ++i){
            auto kukaiiwaFusionTrackMessage = states->Get(i);
            auto timeEvent = kukaiiwaFusionTrackMessage->timeEvent();
            switch(time_type) {
                case device_time:{
                    timeStamp.push_back(timeEvent->device_time());
                    break;
                }
                case local_request_time:{
                    timeStamp.push_back(timeEvent->local_request_time());
                    break;
                }
                case local_receive_time:{
                    timeStamp.push_back(timeEvent->local_receive_time());
                    break;
                }
                default:{
                     std::perror("No time type!");
                }
            }
        }
        return timeStamp;
    }


    /// Return the joint_index th joint angle
    std::vector<double> getJointAngles(const grl::flatbuffer::KUKAiiwaStates* kukaStatesP, int joint_index){
        if(kukaStatesP == nullptr)
            return std::vector<double>();
        auto states = kukaStatesP->states();
        std::vector<double> jointPosition;
        std::size_t state_size = states->size();
        for(int i = 1; i<state_size; ++i){
            auto KUKAiiwaState = states->Get(i);
            auto FRIMessage = KUKAiiwaState->FRIMessage();
            auto joints_Position = FRIMessage->measuredJointPosition(); // flatbuffers::Vector<double> *
            jointPosition.push_back(joints_Position->Get(joint_index));
        }
        return jointPosition;
    }
    /// Return the joint_index th joint angle
   std::vector<std::vector<double>> getAllJointAngles(const grl::flatbuffer::KUKAiiwaStates* kukaStatesP){
        if(kukaStatesP == nullptr)
            return std::vector<std::vector<double>>();
        auto states = kukaStatesP->states();

        std::vector<std::vector<double>> allJointPosition;
        std::size_t state_size = states->size();
        for(int i = 1; i<state_size; ++i){
            auto KUKAiiwaState = states->Get(i);
            auto FRIMessage = KUKAiiwaState->FRIMessage();
            auto joints_Position = FRIMessage->measuredJointPosition(); // flatbuffers::Vector<double> *
            std::vector<double> oneJointPosition;
            for(int joint_index=0; joint_index<7; joint_index++){
                    oneJointPosition.push_back(joints_Position->Get(joint_index));
            }
            allJointPosition.push_back(oneJointPosition);
        }
        return allJointPosition;
    }

    // void writetoCSVforKuka(std::string binaryfilename, std::string csvfilename) {
    //     FILE* file = std::fopen(binaryfilename.c_str(), "rb");
    //     std::fseek(file, 0L, SEEK_END); // seek to end
    //     std::size_t length = std::ftell(file);
    //     std::fseek(file, 0L, SEEK_SET); // seek to start
    //     std::vector<uint8_t> buffer(length);
    //     std::fread(buffer.data(), sizeof(uint8_t), buffer.size(), file);
    //     std::fclose(file);
    //     auto kukaiiwaStates = grl::flatbuffer::GetKUKAiiwaStates(&buffer[0]);
    //     auto states = kukaiiwaStates->states();
    //     std::size_t state_size = states->size();
    //     std::cout<< "------Kuka_state_size: "<<state_size << std::endl;
    //     std::vector<int64_t> deviceTime = grl:: getTimeStamp_Kuka(kukaiiwaStates, "device_time");
    //     std::vector<int64_t> local_request_time = grl:: getTimeStamp_Kuka(kukaiiwaStates, "local_request_time");
    //     std::vector<int64_t> local_receive_time = grl:: getTimeStamp_Kuka(kukaiiwaStates, "local_receive_time");

    //     std::vector<double> jointAngles_0 = grl::getJointAngles(kukaiiwaStates, 0);
    //     std::vector<double> jointAngles_1 = grl::getJointAngles(kukaiiwaStates, 1);
    //     std::vector<double> jointAngles_2 = grl::getJointAngles(kukaiiwaStates, 2);
    //     std::vector<double> jointAngles_3 = grl::getJointAngles(kukaiiwaStates, 3);
    //     std::vector<double> jointAngles_4 = grl::getJointAngles(kukaiiwaStates, 4);
    //     std::vector<double> jointAngles_5 = grl::getJointAngles(kukaiiwaStates, 5);
    //     std::vector<double> jointAngles_6 = grl::getJointAngles(kukaiiwaStates, 6);

    //     std::vector<std::vector<double>> allJointAngles = grl::getAllJointAngles(kukaiiwaStates);
    //     // if(allJointAngles.size() > 0){
    //     //     std::cout<< "------States size: "<<allJointAngles.size() << std::endl;
    //     //     std::cout<< "------Joint Number: "<<allJointAngles[0].size() << std::endl;
    //     // }
    //     // create an ofstream for the file output (see the link on streams for more info)

    //     std::ofstream fs;
    //     // create a name for the file output

    //     fs.open(csvfilename, std::ofstream::out | std::ofstream::app);
    //      // write the file headers
    //     fs << "local_request_time" << ",local_receive_time" << ",Time_Difference" << ",Device_Time" << ",JointPosition_0" << ",JointPosition_1" <<",JointPosition_2"
    //       <<",JointPosition_3" <<",JointPosition_4" <<",JointPosition_5" <<",JointPosition_6" <<std::endl;
    //     for(int i=0; i<state_size; ++i) {
    //         // write the data to the output file
    //         fs<<local_request_time[i] << ","<<local_receive_time[i] <<","<<local_receive_time[i]-local_request_time[i] <<","<<deviceTime[i] << "," << jointAngles_0[i] << "," << jointAngles_1[i] <<"," << jointAngles_2[i] <<"," <<
    //             jointAngles_3[i] <<"," << jointAngles_4[i] << "," << jointAngles_5[i] << "," << jointAngles_6[i]<<std::endl;
    //     }
    //     fs.close();
    // }
}
#endif