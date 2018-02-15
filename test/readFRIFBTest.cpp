
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
std::string kukaBinaryfile = foldname + "2018_02_13_19_06_04_Kukaiiwa.iiwa";
std::string fusiontrackBinaryfile = foldname + "2018_02_13_19_05_49_FusionTrack.flik";
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
    grl::VectorXd FT_device_time = grl:: getTimeStamp(FusionTrackStatesRoot, grl::fusiontracker_tag(), grl::TimeType::device_time);
    grl::VectorXd FT_local_request_time = grl::getTimeStamp(FusionTrackStatesRoot, grl::fusiontracker_tag(), grl::TimeType::local_request_time);
    grl::VectorXd FT_local_receive_time = grl::getTimeStamp(FusionTrackStatesRoot, grl::fusiontracker_tag(), grl::TimeType::local_receive_time);
    writetoCSV(FT_CSVfilename, FT_device_time, FT_local_request_time, FT_local_receive_time);
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
    fs.open( CSVfilename, std::ofstream::out | std::ofstream::app);
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
void writetoCSV(
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







