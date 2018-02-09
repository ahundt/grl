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
#include <Eigen/Dense>

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
    typedef Eigen::Matrix<int64_t , Eigen::Dynamic , 1>  VectorXd;
    struct kuka_tag {};
    struct fusiontracker_tag {};
    enum TimeType { device_time = 0, local_request_time = 1, local_receive_time = 2 };

    grl::VectorXd getTimeStamp(const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP, kuka_tag, TimeType time_type){
        auto states = kukaStatesP->states();
        std::size_t state_size = states->size();
        grl::VectorXd timeStamp(state_size);
        for(int i = 0; i<state_size; ++i){
            auto KUKAiiwaState = states->Get(i);
            auto FRIMessage = KUKAiiwaState->FRIMessage();
            auto timeEvent = FRIMessage->timeStamp();
            switch(time_type) {
                case device_time:{
                    timeStamp(i) = timeEvent->device_time();
                    break;
                }
                case local_request_time:{
                    timeStamp(i) = timeEvent->local_request_time();
                    break;
                }
                case local_receive_time:{
                    timeStamp(i) = timeEvent->local_receive_time();
                    break;
                }
                default:{
                     std::perror("No time type!");
                }
            }
        }
        return timeStamp;
    }

    grl::VectorXd  getTimeStamp(const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP, fusiontracker_tag, TimeType time_type){

        auto states = logKUKAiiwaFusionTrackP->states();
        std::size_t state_size = states->size();
        grl::VectorXd timeStamp(state_size);
        //Eigen::VectoXd time_stamp(state_size);
        for(int i = 0; i<state_size; ++i){
            auto kukaiiwaFusionTrackMessage = states->Get(i);
            auto timeEvent = kukaiiwaFusionTrackMessage->timeEvent();
            switch(time_type) {
                case device_time:{
                    timeStamp(i) = timeEvent->device_time();
                    break;
                }
                case local_request_time:{
                    timeStamp(i) = timeEvent->local_request_time();
                    break;
                }
                case local_receive_time:{
                    timeStamp(i) = timeEvent->local_receive_time();
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
    Eigen::VectorXf getJointAnglesFromKUKA(const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP, int joint_index){
        // if(kukaStatesP == nullptr)
        //     return Eigen::VectorXf();
        auto states = kukaStatesP->states();

        std::size_t state_size = states->size();
        Eigen::VectorXf jointPosition(state_size);
        for(int i = 0; i<state_size; ++i){
            auto KUKAiiwaState = states->Get(i);
            auto FRIMessage = KUKAiiwaState->FRIMessage();
            auto joints_Position = FRIMessage->measuredJointPosition(); // flatbuffers::Vector<double> *
            jointPosition(i) = joints_Position->Get(joint_index);
        }
        return jointPosition;
    }
    /// Return the joint_index th joint angle

   Eigen::MatrixXf getAllJointAngles(const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP){
        // if(kukaStatesP == nullptr)
        //     return Eigen::MatrixXf();
        auto states = kukaStatesP->states();
        std::size_t state_size = states->size();
        Eigen::MatrixXf allJointPosition(state_size, 7);
        for(int i = 0; i<state_size; ++i){
            auto KUKAiiwaState = states->Get(i);
            auto FRIMessage = KUKAiiwaState->FRIMessage();
            auto joints_Position = FRIMessage->measuredJointPosition(); // flatbuffers::Vector<double> *
            Eigen::VectorXf oneStateJointPosition;
            for(int joint_index=0; joint_index<7; joint_index++){
                    oneStateJointPosition(i) = joints_Position->Get(joint_index);
            }
            allJointPosition.row(i) = oneStateJointPosition.transpose();
        }
        return allJointPosition;
    }

    void writetoCSVforKuka(std::string kukaBinaryfile, std::string csvfilename) {
        fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> KUKAiiwaStatesRoot = fbs_tk::open_root<grl::flatbuffer::KUKAiiwaStates>(kukaBinaryfile);
        auto states = KUKAiiwaStatesRoot->states();
        std::size_t kuka_state_size = states->size();
        std::cout<< "------Kuka_state_size: "<<kuka_state_size << std::endl;
        grl::VectorXd  kuka_deviceTime = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::device_time);
        grl::VectorXd  kuka_local_request_time = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::local_request_time);
        grl::VectorXd  kuka_local_receive_time = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::local_receive_time);
        auto initial_local_time = kuka_local_request_time(0);
        auto initial_device_time_kuka = kuka_deviceTime(0);

        kuka_local_request_time = kuka_local_request_time - initial_local_time * grl::VectorXd::Ones(kuka_state_size);
        kuka_local_receive_time = kuka_local_receive_time - initial_local_time * grl::VectorXd::Ones(kuka_state_size);
        kuka_deviceTime = kuka_deviceTime - initial_device_time_kuka * grl::VectorXd::Ones(kuka_state_size);

        Eigen::VectorXf jointAngles_0 = grl::getJointAnglesFromKUKA(KUKAiiwaStatesRoot, 0);
        Eigen::VectorXf jointAngles_1 = grl::getJointAnglesFromKUKA(KUKAiiwaStatesRoot, 1);
        Eigen::VectorXf jointAngles_2 = grl::getJointAnglesFromKUKA(KUKAiiwaStatesRoot, 2);
        Eigen::VectorXf jointAngles_3 = grl::getJointAnglesFromKUKA(KUKAiiwaStatesRoot, 3);
        Eigen::VectorXf jointAngles_4 = grl::getJointAnglesFromKUKA(KUKAiiwaStatesRoot, 4);
        Eigen::VectorXf jointAngles_5 = grl::getJointAnglesFromKUKA(KUKAiiwaStatesRoot, 5);
        Eigen::VectorXf jointAngles_6 = grl::getJointAnglesFromKUKA(KUKAiiwaStatesRoot, 6);

        // create an ofstream for the file output (see the link on streams for more info)

        std::ofstream fs;
        // create a name for the file output

        fs.open(csvfilename, std::ofstream::out | std::ofstream::app);
         // write the file headers
        fs << "local_request_time"
           << ",local_receive_time"
           << ",Time_Difference"
           << ",Device_Time"
           << ",JointPosition_0" << ",JointPosition_1" << ",JointPosition_2"
           << ",JointPosition_3" << ",JointPosition_4" << ",JointPosition_5" << ",JointPosition_6" << std::endl;
        for(int i=0; i<kuka_state_size; ++i) {
            // write the data to the output file
            fs << kuka_local_request_time[i] << ","
               << kuka_local_receive_time[i] << ","
               << kuka_local_receive_time[i] - kuka_local_request_time[i] << ","
               << kuka_deviceTime[i] << ","
               << jointAngles_0[i] << ","
               << jointAngles_1[i] << ","
               << jointAngles_2[i] << ","
               << jointAngles_3[i] << ","
               << jointAngles_4[i] << ","
               << jointAngles_5[i] << ","
               << jointAngles_6[i] << std::endl;
        }
        fs.close();
    }
}
#endif