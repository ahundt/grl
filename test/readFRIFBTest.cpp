
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

#include <chrono>
#include <thread>

std::string foldname = "/home/chunting/src/V-REP_PRO_EDU_V3_4_0_Linux/";
std::string kukaBinaryfile = foldname + "2018_02_20_20_27_40_Kukaiiwa.iiwa";
std::string fusiontrackBinaryfile = foldname + "2018_02_23_16_55_17_FusionTrack.flik";
std::string FTKUKA_CSVfilename = foldname + current_date_and_time_string() + "_FTKUKA.csv";
std::string FT_CSVfilename = foldname + current_date_and_time_string() + "_FT.csv";
std::string FT_Marker22_CSVfilename = foldname + current_date_and_time_string() + "_FT_Marker22.csv";
std::string FT_Marker55_CSVfilename = foldname + current_date_and_time_string() + "_FT_Marker55.csv";
std::string KUKA_CSVfilename = foldname + current_date_and_time_string() + "_KUKA.csv";
std::string KUKA_CSVfilename_Joint = foldname + current_date_and_time_string() + "_KUKA_Joint.csv";



int main(int argc, char* argv[])
{

    /// Write FT data to CSV
    fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> FusionTrackStatesRoot = fbs_tk::open_root<grl::flatbuffer::LogKUKAiiwaFusionTrack>(fusiontrackBinaryfile);
    grl::MatrixXd timeEventM_FT = grl::getTimeStamp(FusionTrackStatesRoot, grl::fusiontracker_tag());
    grl::regularizeTimeEvent(timeEventM_FT);

    uint32_t makerID_22 = 22;
    uint32_t makerID_55 = 55;
    Eigen::MatrixXd markerPose_22 = grl::getMarkerPose(FusionTrackStatesRoot, makerID_22, timeEventM_FT);
    Eigen::MatrixXd markerPose_55 = grl::getMarkerPose(FusionTrackStatesRoot, makerID_55, timeEventM_FT);
    std::vector<std::string> FT_Labels_Pose = getLabels(grl::fusiontracker_tag());
    if(markerPose_22.rows()>2){
         grl::writeMatrixToCSV(FT_Marker22_CSVfilename, FT_Labels_Pose, markerPose_22);
    }
    if(markerPose_55.rows()>2) {
        grl::writeMatrixToCSV(FT_Marker55_CSVfilename, FT_Labels_Pose, markerPose_55);
    }

    /// Write KUKA data to CSV
    fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> KUKAiiwaStatesRoot = fbs_tk::open_root<grl::flatbuffer::KUKAiiwaStates>(kukaBinaryfile);
    grl::MatrixXd timeEventM_Kuka = grl::getTimeStamp(KUKAiiwaStatesRoot,grl::kuka_tag());
    grl::regularizeTimeEvent(timeEventM_Kuka);

    Eigen::MatrixXd jointAngles = grl::getAllJointAngles(KUKAiiwaStatesRoot);
    std::vector<std::string> Kuka_Joint_Labels = getLabels(grl::kuka_tag());

    if(jointAngles.rows() == timeEventM_Kuka.rows()){
         Eigen::MatrixXd JointWithTime(jointAngles.rows(), jointAngles.cols()+timeEventM_Kuka.cols());
         JointWithTime << timeEventM_Kuka.cast<double>(), jointAngles;
         grl::writeMatrixToCSV(KUKA_CSVfilename_Joint, Kuka_Joint_Labels, JointWithTime);
    }

    ///



    // grl::writeFT_KUKATimeEventToCSV(fusiontrackBinaryfile, kukaBinaryfile, FTKUKA_CSVfilename, FT_CSVfilename, FT_PoseCSVfilename, KUKA_CSVfilename);
    // grl::writeJointAngToCSV(kukaBinaryfile, KUKA_CSVfilename_Joint);
    return 0;
}







