
// Library includes
#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE
#include "flatbuffers/util.h"
#include "grl/flatbuffer/readDatafromBinary.hpp"
#include <thirdparty/fbs_tk/fbs_tk.hpp>
#include "grl/flatbuffer/FusionTrack_generated.h"
#include "grl/flatbuffer/Time_generated.h"
#include "grl/flatbuffer/LogKUKAiiwaFusionTrack_generated.h"
#include "grl/time.hpp"



#include <vector>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include <Eigen/Dense>

std::string foldname = "/home/chunting/src/V-REP_PRO_EDU_V3_4_0_Linux/";
std::string kukaBinaryfile = foldname + "2018_02_02_18_21_43_Kukaiiwa.iiwa";
std::string fusiontrackBinaryfile = foldname + "2018_02_02_18_21_58_FusionTrack.flik";
std::string CSVfilename = foldname + current_date_and_time_string() + "_FTKUKA.csv";
std::string FT_CSVfilename = foldname + current_date_and_time_string() + "_FT.csv";
std::string KUKA_CSVfilename = foldname + current_date_and_time_string() + "_KUKA.csv";
std::string KUKA_CSVfilename_Joint = foldname + current_date_and_time_string() + "_KUKA_Joint.csv";

void writetoCSV(std::string & csvfilename,  grl::VectorXd device_time,  grl::VectorXd local_request_time,  grl::VectorXd local_receive_time);
void writetoCSVforFusionTrackKukaiiwa();

int main(int argc, char* argv[])
{

    writetoCSVforFusionTrackKukaiiwa();
    grl::writetoCSVforKuka(kukaBinaryfile, KUKA_CSVfilename_Joint);
    return 0;
}

void writetoCSVforFusionTrackKukaiiwa() {
    fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> KUKAiiwaStatesRoot =
        fbs_tk::open_root<grl::flatbuffer::KUKAiiwaStates>(kukaBinaryfile);
    auto KUKA_states = KUKAiiwaStatesRoot->states();
    std::size_t kuka_state_size = KUKA_states->size();
    std::cout<< "------Kuka_state_size: "<< kuka_state_size << std::endl;

    grl::VectorXd kuka_device_time = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::device_time);
    grl::VectorXd kuka_local_request_time = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::local_request_time);
    grl::VectorXd kuka_local_receive_time = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::local_receive_time);
    writetoCSV(KUKA_CSVfilename, kuka_device_time, kuka_local_request_time, kuka_local_receive_time);



    fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> FusionTrackStatesRoot = fbs_tk::open_root<grl::flatbuffer::LogKUKAiiwaFusionTrack>(fusiontrackBinaryfile);
    auto FT_states = FusionTrackStatesRoot->states();
    std::size_t FT_state_size = FT_states->size();
    std::cout<< "------FusionTrack State Size: "<< FT_state_size << std::endl;
    grl::VectorXd FT_deviceTime = grl:: getTimeStamp(FusionTrackStatesRoot, grl::fusiontracker_tag(), grl::TimeType::device_time);
    grl::VectorXd FT_local_request_time = grl::getTimeStamp(FusionTrackStatesRoot, grl::fusiontracker_tag(), grl::TimeType::local_request_time);
    grl::VectorXd FT_local_receive_time = grl::getTimeStamp(FusionTrackStatesRoot, grl::fusiontracker_tag(), grl::TimeType::local_receive_time);
    writetoCSV(FT_CSVfilename, FT_deviceTime, FT_local_request_time, FT_local_receive_time);

    auto initial_local_time = std::min(FT_local_request_time(0), kuka_local_request_time(0));
    auto initial_device_time_kuka = kuka_device_time(0);
    auto initial_device_time_FT = FT_deviceTime(0);
    FT_local_request_time = FT_local_request_time - initial_local_time * grl::VectorXd::Ones(FT_state_size);
    FT_local_receive_time = FT_local_receive_time - initial_local_time * grl::VectorXd::Ones(FT_state_size);
    FT_deviceTime = FT_deviceTime - initial_device_time_FT * grl::VectorXd::Ones(FT_state_size);

    kuka_local_request_time = kuka_local_request_time - initial_local_time * grl::VectorXd::Ones(kuka_state_size);
    kuka_local_receive_time = kuka_local_receive_time - initial_local_time * grl::VectorXd::Ones(kuka_state_size);
    kuka_device_time = kuka_device_time - initial_device_time_kuka * grl::VectorXd::Ones(kuka_state_size);

    std::size_t states_size = kuka_state_size+FT_state_size;
    grl::VectorXd local_request_time_offset(states_size);
    grl::VectorXd local_receive_time_offset_FT(states_size);
    grl::VectorXd local_receive_time_offset_kuka(states_size);
    grl::VectorXd device_time_offset_kuka(states_size);
    grl::VectorXd device_time_offset_FT(states_size);
    grl::VectorXd device_type(states_size);

    // auto kuka_size = kuka_local_request_time.size();

    int kuka_index = 0;
    int FT_index = 0;
    int index = kuka_index + FT_index;
    // // while(kuka_local_request_time(kuka_index) > FT_local_request_time(FT_index)){
    // //     itFT_local_request_time++;
    // //     FT_index++;
    // // }

    // while ( kuka_index < kuka_state_size && FT_index < FT_state_size )
    // {
    //     if ( kuka_local_request_time(kuka_index) < FT_local_request_time(FT_index) ){
    //         local_request_time_offset(index) = kuka_local_request_time(kuka_index);
    //         local_receive_time_offset_FT(index) = FT_local_receive_time(FT_index);
    //         local_receive_time_offset_kuka(index) = kuka_local_receive_time(kuka_index);
    //         device_time_offset_kuka (index) =  kuka_device_time(kuka_index);
    //         device_time_offset_FT(index) = FT_deviceTime(FT_index);
    //         device_type(index) = 0;
    //         kuka_index++;


    //     } else if( kuka_local_request_time(kuka_index) > FT_local_request_time(FT_index)) {
    //         local_request_time_offset(index) = FT_local_request_time(FT_index);
    //         local_receive_time_offset_FT(index) = FT_local_receive_time(FT_index);
    //         local_receive_time_offset_kuka(index) =  kuka_local_receive_time(kuka_index);
    //         device_time_offset_kuka(index) = kuka_device_time(kuka_index);
    //         device_time_offset_FT(index) = FT_deviceTime(FT_index);
    //         device_type(index) = 1;
    //         FT_index++;
    //     } else {
    //         local_request_time_offset(index) = FT_local_request_time(FT_index);
    //         local_receive_time_offset_FT(index) = FT_local_receive_time(FT_index);
    //         local_receive_time_offset_kuka(index) =  kuka_local_receive_time(kuka_index);
    //         device_time_offset_kuka(index)=  kuka_device_time(kuka_index);
    //         device_time_offset_FT(index) = FT_deviceTime(FT_index);
    //         device_type(index) = 2;
    //         FT_index++;
    //         kuka_index++;
    //     }
    //     index++;
    // }


    // // // copy rest of kuka_local_request_time array
    // // while ( itkuka_local_request_time != itkuka_local_request_timeEnd ){
    // //     FT_index = FT_state_size-2;
    // //     local_request_time_offset << ( kuka_local_request_time(kuka_index);
    // //     local_receive_time_offset_FT << (FT_local_receive_time(FT_index);
    // //     local_receive_time_offset_kuka << (kuka_local_receive_time(kuka_index);
    // //     device_time_offset_kuka << (kuka_device_time(kuka_index);
    // //     device_time_offset_FT << (FT_deviceTime(FT_index);
    // //     kuka_index++;
    // //     itkuka_local_request_time++;
    // //     device_type << (0);
    // // }


    // // // copy rest of FT_local_request_time array
    // // while ( itFT_local_request_time != itFT_local_request_timeEnd ) {
    // //     kuka_index = kuka_state_size-2;
    // //     local_request_time_offset << ( FT_local_request_time(FT_index);
    // //     local_receive_time_offset_FT << (FT_local_receive_time(FT_index);
    // //     local_receive_time_offset_kuka << (kuka_local_receive_time(kuka_index);
    // //     device_time_offset_kuka << (kuka_device_time(kuka_index);
    // //     device_time_offset_FT << (FT_deviceTime(FT_index);
    // //     FT_index++;
    // //     itFT_local_request_time++;
    // //     device_type << (1);
    // //    // std::cout<<"kuka_local_receive_time: " << kuka_index <<"  "<<kuka_local_receive_time(kuka_index)  << std::endl;
    // // }
    // // // create an ofstream for the file output (see the link on streams for more info)


    // auto real_size = local_request_time_offset.size();
    // std::cout<< "------FT_KUKA: "<<real_size << std::endl;
    // std::ofstream fs;
    // // create a name for the file output
    // fs.open( CSVfilename, std::ofstream::out | std::ofstream::app);
    //  // write the file headers
    // fs << "local_request_time_X,"
    //    << "device type,"
    //    << "local_receive_time_offset_FT,"
    //    << "local_receive_time_offset_kuka,"
    //    << "device_time_offset_FT,"
    //    << "device_time_offset_kuka,"
    //    << "Y_FT,"
    //    << "Y_kuka"  << std::endl;
    // for(int i=0; i<real_size; ++i) {
    //     // write the data to the output file
    //     fs << local_request_time_offset(i) << ","                                      // A
    //        << device_type(i)<< ","                                                     // B
    //        << local_receive_time_offset_FT(i) <<","                                    // C
    //        << local_receive_time_offset_kuka(i) <<","                                  // D
    //        << device_time_offset_FT(i) <<","                                           // E
    //        << device_time_offset_kuka(i) <<","                                         // F
    //        << device_time_offset_FT(i) - local_request_time_offset(i) <<","            // G
    //        << device_time_offset_kuka(i) - local_request_time_offset(i) << std::endl;  //H
    // }
    // std::cout<< "local_request_time_offset: "<< local_request_time_offset[real_size-1] << std::endl;
    // std::cout<< "local_receive_time_offset_FT: "<< local_receive_time_offset_FT[real_size-1] << std::endl;
    // std::cout<< "local_receive_time_offset_kuka: "<< local_receive_time_offset_kuka[real_size-1]/10 << std::endl;
    // std::cout<< "device_time_offset_FT: "<< device_time_offset_FT[real_size-1] << std::endl;
    // std::cout<< "device_time_offset_kuka: "<< device_time_offset_kuka[real_size-1] << std::endl;

    // fs.close();
}
void writetoCSV(
    std::string & csvfilename,
    grl::VectorXd device_time,
    grl::VectorXd local_request_time,
    grl::VectorXd local_receive_time){

        std::size_t size = device_time.size();
        auto initial_local_time = local_request_time(0);
        auto initial_device_time = device_time(0);
        local_request_time = local_request_time - initial_local_time * grl::VectorXd::Ones(size);
        local_receive_time = local_receive_time - initial_local_time * grl::VectorXd::Ones(size);
        device_time = device_time - initial_device_time * grl::VectorXd::Ones(size);

        std::ofstream fs;
        // create a name for the file output
        fs.open( csvfilename, std::ofstream::out | std::ofstream::app);
         // write the file headers
        fs << "local_request_time_X,"
           << "local_receive_time_offset,"
           << "device_time_offset,"
           << "Y"  << std::endl;
        for(int i=0; i<size; ++i) {
            // write the data to the output file
            fs << local_request_time(i)<< ","         // A
               << local_receive_time(i) <<","          // B
               << device_time(i) <<","                        // C
               << device_time(i) - local_request_time(i) << std::endl;  //D
        }
        fs.close();
}







