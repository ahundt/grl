#ifndef GRL_READ_FUSIONTRACK_FROM_FLATBUFFER
#define GRL_READ_FUSIONTRACK_FROM_FLATBUFFER

#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE
#include "grl/flatbuffer/FusionTrack_generated.h"
#include "grl/flatbuffer/Time_generated.h"
#include "grl/flatbuffer/LogKUKAiiwaFusionTrack_generated.h"
#include <thirdparty/fbs_tk/fbs_tk.hpp>
#include <Eigen/Dense>

namespace grl {

    const grl::flatbuffer::LogKUKAiiwaFusionTrack* getLogKUKAiiwaFusionTrackFromFlatbuffer(const std::string filename){

        //std::cout << "CurrentWorkingDir: " << curWorkingDir << std::endl;
        if(filename.empty()) return nullptr;
        FILE* file = std::fopen(filename.c_str(), "rb");
        if(!file) {
            std::perror("File opening failed");
            return nullptr;
        }
        std::fseek(file, 0L, SEEK_END); // seek to end
        std::size_t length = std::ftell(file);
        std::fseek(file, 0L, SEEK_SET); // seek to start
        std::vector<uint8_t> buffer(length);
        std::fread(buffer.data(), sizeof(uint8_t), buffer.size(), file);
        std::fclose(file);
        return grl::flatbuffer::GetLogKUKAiiwaFusionTrack(&buffer[0]);
    }

    /// Return the joint_index th joint angle
    // std::vector<double> getGeoPositions(const grl::flatbuffer::LogKUKAiiwaFusionTrack* logKUKAiiwaFusionTrackP, int joint_index){
    //     if(kukaStatesP == nullptr)
    //         return std::vector<double>();
    //     auto states = kukaStatesP->states();
    //     std::vector<double> jointPosition;
    //     std::size_t state_size = states->size();
    //     for(int i = 1; i<state_size; ++i){
    //         auto KUKAiiwaState = states->Get(i);
    //         auto FRIMessage = KUKAiiwaState->FRIMessage();
    //         auto joints_Position = FRIMessage->measuredJointPosition(); // flatbuffers::Vector<double> *
    //         jointPosition.push_back(joints_Position->Get(joint_index));
    //     }
    //     return jointPosition;
    // }

    std::vector<int64_t> getTimeStamp_FusionTrack(const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP, std::string time_type){

        auto states = logKUKAiiwaFusionTrackP->states();
        std::vector<int64_t> timeStamp;
        
        std::size_t state_size = states->size();
        //Eigen::VectoXd time_stamp(state_size);
        for(int i = 0; i<state_size; ++i){
            auto kukaiiwaFusionTrackMessage = states->Get(i);
            auto timeEvent = kukaiiwaFusionTrackMessage->timeEvent();

            if(time_type =="device_time"){
                timeStamp.push_back(timeEvent->device_time());
                //time_stamp<<timeEvent->device_time();
            } else if (time_type == "local_request_time") {
                timeStamp.push_back(timeEvent->local_request_time());
            } else if (time_type == "local_receive_time"){
                timeStamp.push_back(timeEvent->local_receive_time());
            }
        }
        return timeStamp;
    }

    // void writetoCSVforFusionTrack(std::string binaryfilename, std::string csvfilename) {
    //     FILE* file = std::fopen(binaryfilename.c_str(), "rb");
    //     std::fseek(file, 0L, SEEK_END); // seek to end
    //     std::size_t length = std::ftell(file);
    //     std::fseek(file, 0L, SEEK_SET); // seek to start
    //     std::vector<uint8_t> buffer(length);
    //     std::fread(buffer.data(), sizeof(uint8_t), buffer.size(), file);
    //     std::fclose(file);
    //     auto logKUKAiiwaFusionTrack = grl::flatbuffer::GetLogKUKAiiwaFusionTrack(&buffer[0]);
    //     auto states = logKUKAiiwaFusionTrack->states();
    //     std::size_t state_size = states->size();
    //     std::cout<< "------FusionTrack State Size: "<<state_size << std::endl;
    //     std::vector<int64_t> deviceTime = grl:: getTimeStamp_FusionTrack(logKUKAiiwaFusionTrack, "device_time");
    //     std::vector<int64_t> local_request_time = grl:: getTimeStamp_FusionTrack(logKUKAiiwaFusionTrack, "local_request_time");
    //     std::vector<int64_t> local_receive_time = grl:: getTimeStamp_FusionTrack(logKUKAiiwaFusionTrack, "local_receive_time");

    //     // create an ofstream for the file output (see the link on streams for more info)

    //     std::ofstream fs;
    //     // create a name for the file output

    //     fs.open(csvfilename, std::ofstream::out | std::ofstream::app);
    //      // write the file headers
    //     fs << "local_request_time" << ",local_receive_time" << ",Time_Difference" << ",Device_Time" <<std::endl;
    //     for(int i=0; i<state_size; ++i) {
    //         // write the data to the output file
    //         fs<<local_request_time[i] << ","<<local_receive_time[i] <<","<<local_receive_time[i]-local_request_time[i] <<","<< deviceTime[i]<<std::endl;
    //     }
    //     fs.close();
    // }


}
#endif