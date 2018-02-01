
// Library includes
#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE
#include "flatbuffers/util.h"
#include "grl/kuka/readKukaiiwaFromFlatbuffer.hpp"
#include "grl/sensor/readFusionTrackFromFlatbuffer.hpp"
#include <thirdparty/fbs_tk/fbs_tk.hpp>
#include "grl/flatbuffer/FusionTrack_generated.h"
#include "grl/flatbuffer/Time_generated.h"
#include "grl/flatbuffer/LogKUKAiiwaFusionTrack_generated.h"



#include <vector>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include <Eigen/Dense>

void writetoCSVforFusionTrackKukaiiwa(std::string KUKAbinaryfilename, std::string FusionTrackbinaryfilename, std::string csvfilename);
int main(int argc, char* argv[])
{
    std::string kukaBinaryfile = "/home/chunting/src/V-REP_PRO_EDU_V3_4_0_Linux/2018_01_31_16_11_27_Kukaiiwa.iiwa";
    std::string fusiontrackBinaryfile = "/home/chunting/src/V-REP_PRO_EDU_V3_4_0_Linux/2018_01_31_16_10_58_FusionTrack.flik";
    std::string KukaFusionTrackCSVFile = "/home/chunting/src/V-REP_PRO_EDU_V3_4_0_Linux/TimeEventCombined.csv";
    writetoCSVforFusionTrackKukaiiwa(kukaBinaryfile, fusiontrackBinaryfile, KukaFusionTrackCSVFile);
    // fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> KUKAiiwaStatesRoot = fbs_tk::open_root<grl::flatbuffer::KUKAiiwaStates>(kukaBinaryfile);

    // auto states = KUKAiiwaStatesRoot->states();
    // std::size_t state_size = states->size();
    // std::cout<< "++++++Kuka_state_size: "<<state_size << std::endl;


    // grl::writetoCSVforKuka(kukaBinaryfile, kukaCSVFile);
    // std::string fusiontrackBinaryfile = "/home/chunting/src/V-REP_PRO_EDU_V3_4_0_Linux/2018_01_31_16_10_58_FusionTrack.flik";
    // std::string fusiontrackCSVFile = "/home/chunting/src/V-REP_PRO_EDU_V3_4_0_Linux/2018_01_31_16_10_58_FusionTrack.csv";
    // grl::writetoCSVforFusionTrack(fusiontrackBinaryfile, fusiontrackCSVFile);

    return 0;
}

void writetoCSVforFusionTrackKukaiiwa(std::string KUKAbinaryfilename, std::string FusionTrackbinaryfilename, std::string csvfilename) {
    fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> KUKAiiwaStatesRoot = fbs_tk::open_root<grl::flatbuffer::KUKAiiwaStates>(KUKAbinaryfilename);
    auto KUKA_states = KUKAiiwaStatesRoot->states();
    std::size_t kuka_state_size = KUKA_states->size();
    std::cout<< "------Kuka_state_size: "<<kuka_state_size << std::endl;
    std::vector<int64_t> kuka_deviceTime = grl:: getTimeStamp_Kuka(KUKAiiwaStatesRoot, "device_time");
    std::vector<int64_t> kuka_local_request_time = grl::getTimeStamp_Kuka(KUKAiiwaStatesRoot, "local_request_time");
    std::vector<int64_t> kuka_local_receive_time = grl::getTimeStamp_Kuka(KUKAiiwaStatesRoot, "local_receive_time");
    fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> FusionTrackStatesRoot = fbs_tk::open_root<grl::flatbuffer::LogKUKAiiwaFusionTrack>(FusionTrackbinaryfilename);

    auto FT_states = FusionTrackStatesRoot->states();
    std::size_t FT_state_size = FT_states->size();
    std::cout<< "------FusionTrack State Size: "<<FT_state_size << std::endl;
    std::vector<int64_t> FT_deviceTime = grl:: getTimeStamp_FusionTrack(FusionTrackStatesRoot, "device_time");
    std::vector<int64_t> FT_local_request_time = grl::getTimeStamp_FusionTrack(FusionTrackStatesRoot, "local_request_time");
    std::vector<int64_t> FT_local_receive_time = grl::getTimeStamp_FusionTrack(FusionTrackStatesRoot, "local_receive_time");
    std::vector<int64_t> local_request_time;
    std::vector<int64_t> local_receive_time_offset_FT;
    std::vector<int64_t> local_receive_time_offset_kuka;
    std::vector<int64_t> device_time_offset_kuka;
    std::vector<int64_t> device_time_offset_FT;
    int64_t initial_local_request_time = std::min(kuka_local_request_time[0], FT_local_request_time[0]);
    int64_t initial_local_receive_time_kuka = kuka_local_receive_time[0];
    int64_t initial_local_receive_time_FT = FT_local_receive_time[0];
    int64_t initial_device_time_kuka = kuka_deviceTime[0];
    int64_t initial_device_time_FT = FT_deviceTime[0];

    auto itkuka_local_request_time = kuka_local_request_time.begin();
    auto itFT_local_request_time = FT_local_request_time.begin();

    auto itkuka_local_request_timeEnd = kuka_local_request_time.end();
    auto itFT_local_request_timeEnd = FT_local_request_time.end();

    local_request_time.reserve(kuka_state_size+FT_state_size);
    local_receive_time_offset_FT.reserve(kuka_state_size+FT_state_size);
    local_receive_time_offset_kuka.reserve(kuka_state_size+FT_state_size);
    device_time_offset_kuka.reserve(kuka_state_size+FT_state_size);
    device_time_offset_FT.reserve(kuka_state_size+FT_state_size);
   
    int kuka_index = 0;
    int FT_index = 0; 

    while ( itkuka_local_request_time != itkuka_local_request_timeEnd && 
            itFT_local_request_time != itFT_local_request_timeEnd )
    {
        if ( *itkuka_local_request_time < *itFT_local_request_time ){
            local_request_time.push_back( *itkuka_local_request_time - initial_local_request_time);
            local_receive_time_offset_FT.push_back(FT_local_receive_time[FT_index] - initial_local_receive_time_FT);
            local_receive_time_offset_kuka.push_back(kuka_local_receive_time[kuka_index] - initial_local_receive_time_kuka);
            device_time_offset_kuka.push_back(kuka_deviceTime[kuka_index]-initial_device_time_kuka);
            device_time_offset_FT.push_back(FT_deviceTime[FT_index]-initial_device_time_FT);
            kuka_index++;
            itkuka_local_request_time++;
  
  
        } else if( *itkuka_local_request_time > *itFT_local_request_time ) {
            local_request_time.push_back( *itFT_local_request_time - initial_local_request_time );
            local_receive_time_offset_FT.push_back(FT_local_receive_time[FT_index] - initial_local_receive_time_FT);
            local_receive_time_offset_kuka.push_back(kuka_local_receive_time[kuka_index] - initial_local_receive_time_kuka);
            device_time_offset_kuka.push_back(kuka_deviceTime[kuka_index]-initial_device_time_kuka);
            device_time_offset_FT.push_back(FT_deviceTime[FT_index]-initial_device_time_FT);
            FT_index++;
            itFT_local_request_time++;
        } else {
            local_request_time.push_back( *itFT_local_request_time - initial_local_request_time );
            local_receive_time_offset_FT.push_back(FT_local_receive_time[FT_index] - initial_local_receive_time_FT);
            local_receive_time_offset_kuka.push_back(kuka_local_receive_time[kuka_index] - initial_local_receive_time_kuka);
            device_time_offset_kuka.push_back(kuka_deviceTime[kuka_index]-initial_device_time_kuka);
            device_time_offset_FT.push_back(FT_deviceTime[FT_index]-initial_device_time_FT);
            FT_index++;
            kuka_index++;
            itkuka_local_request_time++;
            itFT_local_request_time++;
        }
    }

    // copy rest of kuka_local_request_time array
    while ( itkuka_local_request_time != itkuka_local_request_timeEnd ){
        FT_index = FT_state_size-1;
        local_request_time.push_back( *itkuka_local_request_time - initial_local_request_time);
        local_receive_time_offset_FT.push_back(FT_local_receive_time[FT_index] - initial_local_receive_time_FT);
        local_receive_time_offset_kuka.push_back(kuka_local_receive_time[kuka_index] - initial_local_receive_time_kuka);
        device_time_offset_kuka.push_back(kuka_deviceTime[kuka_index]-initial_device_time_kuka);
        device_time_offset_FT.push_back(FT_deviceTime[FT_index]-initial_device_time_FT);
        kuka_index++;
        itkuka_local_request_time++;
    }
    

    // copy rest of FT_local_request_time array
    while ( itFT_local_request_time != itFT_local_request_timeEnd ) {
        kuka_index = kuka_state_size-1;
        local_request_time.push_back( *itFT_local_request_time - initial_local_request_time );
        local_receive_time_offset_FT.push_back(FT_local_receive_time[FT_index] - initial_local_receive_time_FT);
        local_receive_time_offset_kuka.push_back(kuka_local_receive_time[kuka_index] - initial_local_receive_time_kuka);
        device_time_offset_kuka.push_back(kuka_deviceTime[kuka_index]-initial_device_time_kuka);
        device_time_offset_FT.push_back(FT_deviceTime[FT_index]-initial_device_time_FT);
        FT_index++;
        itFT_local_request_time++;
    }
    // // create an ofstream for the file output (see the link on streams for more info)
    std::ofstream fs;
    // create a name for the file output
    fs.open( csvfilename, std::ofstream::out | std::ofstream::app);
     // write the file headers
    fs << "local_request_time" << ",local_receive_time_offset_FT" << ",local_receive_time_offset_kuka" <<",device_time_offset_FT" << ",device_time_offset_kuka" <<  std::endl;
    for(int i=0; i<FT_state_size+kuka_state_size-1; ++i) {
        // write the data to the output file
        fs<<local_request_time[i] << ","<<local_receive_time_offset_FT[i] <<","<<local_receive_time_offset_kuka[i] <<"," <<device_time_offset_FT[i]<<","<< device_time_offset_kuka[i] <<std::endl;
    }
    fs.close();
}


