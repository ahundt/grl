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
            Eigen::VectorXf oneStateJointPosition(7);
            for(int joint_index=0; joint_index<7; joint_index++){
                    oneStateJointPosition(joint_index) = joints_Position->Get(joint_index);
            }
            allJointPosition.row(i) = oneStateJointPosition.transpose();
        }
        return allJointPosition;
    }

    void writetoJointAngToCSV(std::string kukaBinaryfile, std::string csvfilename) {
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
    void writeTimeEventToCSV(
    std::string & csvfilename,
    grl::VectorXd device_time,
    grl::VectorXd local_request_time,
    grl::VectorXd local_receive_time){

        std::size_t size = device_time.size();
        // auto initial_local_time = local_request_time(0);
        auto initial_local_time = local_receive_time(0);
        auto initial_device_time = device_time(0);
        local_request_time = local_request_time - initial_local_time * grl::VectorXd::Ones(size);
        local_receive_time = local_receive_time - initial_local_time * grl::VectorXd::Ones(size);
        device_time = device_time - initial_device_time * grl::VectorXd::Ones(size);
        grl::VectorXd receive_request = local_receive_time - local_request_time;
        //  grl::VectorXd device_time_offset = local_receive_time - local_request_time;
        std::ofstream fs;
        // create a name for the file output
        fs.open( csvfilename, std::ofstream::out | std::ofstream::app);
         // write the file headers
        fs << "local_request_time_offset,"
           << "local_receive_time_X,"
           << "device_time_offset,"
           << "Y,"
           << "Receive-Request,"
           << "device_time_step,"
           << "receive_time_step,"
           << std::endl;
        int64_t device_time_step = 0;
        int64_t receive_time_step = 0;
        for(int i=0; i<size; ++i) {
            if(i>0) {
                device_time_step = device_time(i) - device_time(i-1);
                receive_time_step = local_receive_time(i) - local_receive_time(i-1);
            }
            // write the data to the output file
            fs << local_request_time(i)<< ","         // A
               << local_receive_time(i) <<","          // B
               << device_time(i) <<","                        // C
               << device_time(i) - local_receive_time(i) << ","
               << receive_request(i) << ","
               << device_time_step << ","
               << receive_time_step << ","
               << std::endl;  //D
        }
        fs.close();
}

    void writePoseToCSV(
        std::string & csvfilename,
        grl::VectorXd device_time,
        grl::VectorXd local_request_time,
        grl::VectorXd local_receive_time,
        Eigen::MatrixXd &poseEE){

        std::size_t time_size = device_time.size();
        std::size_t row_size = poseEE.rows();
        assert(time_size == row_size);

        // auto initial_local_time = local_request_time(0);
        auto initial_local_time = local_receive_time(0);
        auto initial_device_time = device_time(0);
        local_request_time = local_request_time - initial_local_time * grl::VectorXd::Ones(time_size);
        local_receive_time = local_receive_time - initial_local_time * grl::VectorXd::Ones(time_size);
        device_time = device_time - initial_device_time * grl::VectorXd::Ones(time_size);
        grl::VectorXd receive_request = local_receive_time - local_request_time;
        //  grl::VectorXd device_time_offset = local_receive_time - local_request_time;
        std::ofstream fs;
        // create a name for the file output
        fs.open( csvfilename, std::ofstream::out | std::ofstream::app);
         // write the file headers
        fs << "local_receive_time_X,"
           << "local_request_time_offset,"
           << "device_time_offset,"
           << "Y,"
           << "Receive-Request,"
           << "device_time_step,"
           << "receive_time_step,"
           << "X,"
           << "Y,"
           << "Z,"
           << "R,"
           << "P,"
           << "Y,"
           << std::endl;
        int64_t device_time_step = 0;
        int64_t receive_time_step = 0;
        for(int i=0; i<time_size; ++i) {
            if(i>0) {
                device_time_step = device_time(i) - device_time(i-1);
                receive_time_step = local_receive_time(i) - local_receive_time(i-1);
            }
            Eigen::RowVectorXd pose = poseEE.row(i);
            // write the data to the output file
            fs << local_receive_time(i) <<","          // B
               << local_request_time(i)<< ","         // A
               << device_time(i) <<","                        // C
               << device_time(i) - local_receive_time(i) << ","
               << receive_request(i) << ","
               << device_time_step << ","
               << receive_time_step << ","
               << pose(0) << ","
               << pose(1) << ","
               << pose(2) << ","
               << pose(3) << ","
               << pose(4) << ","
               << pose(5) << ","
               << std::endl;  //D
        }
        fs.close();
}

    void writetoCSVforFusionTrackKukaiiwa(
        std::string &fusiontrackBinaryfile,
        std::string &kukaBinaryfile,
        std::string& FTKUKA_CSVfilename,
        std::string& FT_CSVfilename,
        std::string& KUKA_CSVfilename) {
    fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> KUKAiiwaStatesRoot =
        fbs_tk::open_root<grl::flatbuffer::KUKAiiwaStates>(kukaBinaryfile);
    auto KUKA_states = KUKAiiwaStatesRoot->states();
    std::size_t kuka_state_size = KUKA_states->size();
    std::cout<< "------Kuka_state_size: "<< kuka_state_size << std::endl;

    grl::VectorXd kuka_device_time = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::device_time);
    grl::VectorXd kuka_local_request_time = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::local_request_time);
    grl::VectorXd kuka_local_receive_time = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::local_receive_time);
    writeTimeEventToCSV(KUKA_CSVfilename, kuka_device_time, kuka_local_request_time, kuka_local_receive_time);



    fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> FusionTrackStatesRoot = fbs_tk::open_root<grl::flatbuffer::LogKUKAiiwaFusionTrack>(fusiontrackBinaryfile);
    auto FT_states = FusionTrackStatesRoot->states();
    std::size_t FT_state_size = FT_states->size();
    std::cout<< "------FusionTrack State Size: "<< FT_state_size << std::endl;
    grl::VectorXd FT_device_time = grl:: getTimeStamp(FusionTrackStatesRoot, grl::fusiontracker_tag(), grl::TimeType::device_time);
    grl::VectorXd FT_local_request_time = grl::getTimeStamp(FusionTrackStatesRoot, grl::fusiontracker_tag(), grl::TimeType::local_request_time);
    grl::VectorXd FT_local_receive_time = grl::getTimeStamp(FusionTrackStatesRoot, grl::fusiontracker_tag(), grl::TimeType::local_receive_time);
    writeTimeEventToCSV(FT_CSVfilename, FT_device_time, FT_local_request_time, FT_local_receive_time);
    int kuka_index = 0;
    int FT_index = 0;
    // filter out the very beginning data, which can gurantee to record the data when the two devices work simultaniously.
    while(kuka_local_receive_time(kuka_index) < FT_local_receive_time(FT_index)){
        kuka_index++;
    }
    while(kuka_local_receive_time(kuka_index) > FT_local_receive_time(FT_index) && kuka_index == 0){
        FT_index++;
    }

    auto initial_local_time = std::min(FT_local_receive_time(FT_index), kuka_local_receive_time(kuka_index));
    auto initial_device_time_kuka = kuka_device_time(kuka_index);
    auto initial_device_time_FT = FT_device_time(FT_index);
    FT_local_request_time = FT_local_request_time - initial_local_time * grl::VectorXd::Ones(FT_state_size);
    FT_local_receive_time = FT_local_receive_time - initial_local_time * grl::VectorXd::Ones(FT_state_size);
    FT_device_time = FT_device_time - initial_device_time_FT * grl::VectorXd::Ones(FT_state_size);

    kuka_local_request_time = kuka_local_request_time - initial_local_time * grl::VectorXd::Ones(kuka_state_size);
    kuka_local_receive_time = kuka_local_receive_time - initial_local_time * grl::VectorXd::Ones(kuka_state_size);
    kuka_device_time = kuka_device_time - initial_device_time_kuka * grl::VectorXd::Ones(kuka_state_size);

    std::ofstream fs;
    // create a name for the file output
    fs.open( FTKUKA_CSVfilename, std::ofstream::out | std::ofstream::app);
    // write the file headers
    fs << "local_receive_time_offset_X,"
       << "FT_local_request_time,"
       << "KUKA_local_request_time,"
       << "FT_device_time_offset,"
       << "device_time_offset_kuka,"
       << "Y_FT,"
       << "Y_kuka"
       << std::endl;

    int64_t kuka_diff = kuka_device_time(kuka_index) - kuka_local_receive_time(kuka_index);
    int64_t FT_diff = FT_device_time(FT_index) - FT_local_receive_time(FT_index);
    while ( kuka_index < kuka_state_size && FT_index < FT_state_size )
    {
        if ( kuka_local_receive_time(kuka_index) < FT_local_receive_time(FT_index) ){
            // write the data to the output file
            fs << kuka_local_receive_time(kuka_index) <<","
               <<","
               << kuka_local_request_time(kuka_index) << ","
               << ","
               << kuka_device_time(kuka_index) <<","
               << ","
               << kuka_device_time(kuka_index) - kuka_local_receive_time(kuka_index) - kuka_diff
               << std::endl;
               kuka_index++;
        } else if( kuka_local_receive_time(kuka_index) > FT_local_receive_time(FT_index)) {
            // write the data to the output file
            fs << FT_local_receive_time(FT_index) <<","
               << FT_local_request_time(FT_index) << ","
               <<","
               << FT_device_time(FT_index) << ","
               << ","
               << FT_device_time(FT_index) - FT_local_receive_time(FT_index) - FT_diff << ","
               << std::endl;
            FT_index++;
        } else {
            fs << FT_local_receive_time(FT_index) <<","
               << FT_local_request_time(FT_index) << ","
               << kuka_local_request_time(kuka_index) << ","
               << FT_device_time(FT_index) << ","
               << kuka_device_time(kuka_index) <<","
               << FT_device_time(FT_index) - FT_local_receive_time(FT_index) - FT_diff << ","
               << kuka_device_time(kuka_index) - kuka_local_receive_time(kuka_index) - kuka_diff
               << std::endl;
            FT_index++;
            kuka_index++;
        }
    }
    fs.close();
}

}
#endif