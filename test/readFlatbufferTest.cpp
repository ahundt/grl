
// Library includes
#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE
#include "flatbuffers/util.h"
#include "grl/flatbuffer/ParseflatbuffertoCSV.hpp"
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
/// Boost to create an empty folder
#include <boost/filesystem.hpp>

/// split -C 200m --numeric-suffixes 2018_02_28_16_39_13_FusionTrack.json 2018_02_28_16_39_13_FusionTrack

/// The command to get the json file from flatbuffer binary file, these two files should be located in the same folder.
/// flatc -I . --json LogKUKAiiwaFusionTrack.fbs -- 2018_03_14_22_06_29_FusionTrack.flik

int main(int argc, char* argv[])
{
    /// Define the file names
    std::string foldname = "/home/chunting/src/V-REP_PRO_EDU_V3_4_0_Linux/";
    std::string kukaTimeStamp = "2018_03_14_22_06_39_Kukaiiwa";
    std::string FTTimeStamp = "2018_03_14_22_06_29_FusionTrack";
    std::string kukaBinaryfile = foldname + kukaTimeStamp+ ".iiwa";
    std::string fusiontrackBinaryfile = foldname + FTTimeStamp +".flik";
    std::string foldtimestamp = current_date_and_time_string();
    boost::filesystem::path dir{foldname+foldtimestamp};
    boost::filesystem::create_directory(dir);

    std::string KUKA_TimeEvent_CSV = foldname + foldtimestamp + "/KUKA_TimeEvent.csv";
    std::string KUKA_Joint_CSV = foldname + foldtimestamp + "/KUKA_Joint.csv";
    std::string KUKA_Pose_CSV = foldname + foldtimestamp + "/KUKA_Pose.csv";
    std::string KUKA_Inverse_Pose_CSV = foldname + foldtimestamp + "/Inverse_KUKA_Pose.csv";
    std::string FudicialToRobotPose_CSV = foldname + foldtimestamp + "/FudicialToRobot_Pose.csv";
    std::string FudicialToFTPose_CSV = foldname + foldtimestamp + "/FudicialToFT_Pose.csv";

    std::string FT_TimeEvent_CSV = foldname + foldtimestamp + "/FT_TimeEvent.csv";
    std::string FT_Marker22_CSV = foldname + foldtimestamp + "/FT_Pose_Marker22.csv";
    std::string FT_Marker55_CSV = foldname + foldtimestamp + "/FT_Pose_Marker55.csv";
    std::string FT_Marker50000_CSV = foldname + foldtimestamp + "/FT_Pose_Marker50000.csv";

    std::string FTKUKA_TimeEvent_CSV = foldname + foldtimestamp + "/FTKUKA_TimeEvent.csv";

    // Put all the data into the same coordinate system --- Marker frame
    bool inMarkerFrame = false;    // Indicate the marker pose is in marker frame or tracker frame.
    /// Write FT data to CSV
    fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> logKUKAiiwaFusionTrackP = fbs_tk::open_root<grl::flatbuffer::LogKUKAiiwaFusionTrack>(fusiontrackBinaryfile);
    grl::MatrixXd timeEventM_FT = grl::getTimeStamp(logKUKAiiwaFusionTrackP, grl::fusiontracker_tag());
    grl::regularizeTimeEvent(timeEventM_FT);

    std::size_t FT_size = timeEventM_FT.rows();

    uint32_t markerID_22 = 22;
    uint32_t markerID_55 = 55;
    uint32_t markerID_50000 = 50000;
    timeEventM_FT = grl::MatrixXd::Zero(FT_size, timeEventM_FT.cols());
    Eigen::MatrixXd markerPose(FT_size, grl::col_Pose);
    // Get the pose of the marker 22.
    // Since the bad data is skipped, the timeEventM_FT also needs to be filled out with the orinial time stamp.

    int validsize_22 = grl::getMarkerPose(logKUKAiiwaFusionTrackP, markerID_22, timeEventM_FT, markerPose, inMarkerFrame);
    std::cout<<"validsize_22: " << validsize_22 << std::endl;
    std::vector<std::string> FT_Labels_Pose = grl::getLabels(grl::LabelsType::FT_Pose);
    if(validsize_22>2){
        grl::regularizeTimeEvent(timeEventM_FT);

        grl::writeTimeEventToCSV(FT_TimeEvent_CSV, timeEventM_FT);
        grl::writeMatrixToCSV(FT_Marker22_CSV, FT_Labels_Pose, timeEventM_FT, markerPose);
    }
    // Change it back to the original size
    timeEventM_FT.resize(FT_size, grl::col_timeEvent);
    markerPose.resize(FT_size, grl::col_Pose);
    // Get the pose of the bone marker
    int validsize_55 = grl::getMarkerPose(logKUKAiiwaFusionTrackP, markerID_55, timeEventM_FT, markerPose);

    std::cout<<"validsize_55: " << validsize_55 << std::endl;
    if(validsize_55>2) {
        grl::regularizeTimeEvent(timeEventM_FT);
        grl::writeMatrixToCSV(FT_Marker55_CSV, FT_Labels_Pose, timeEventM_FT, markerPose);
    }
     // Change it back to the original size
    timeEventM_FT.resize(FT_size, grl::col_timeEvent);
    markerPose.resize(FT_size, grl::col_Pose);
    // Get the pose of the bone marker
    int validsize_50000 = grl::getMarkerPose(logKUKAiiwaFusionTrackP, markerID_50000, timeEventM_FT, markerPose);

    std::cout<<"validsize_50000: " << validsize_50000 << std::endl;
    if(validsize_50000>2) {
        grl::regularizeTimeEvent(timeEventM_FT);
        grl::writeMatrixToCSV(FT_Marker50000_CSV, FT_Labels_Pose, timeEventM_FT, markerPose);
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Write KUKA data to CSV
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> kukaStatesP = fbs_tk::open_root<grl::flatbuffer::KUKAiiwaStates>(kukaBinaryfile);
    grl::MatrixXd timeEventM_Kuka = grl::getTimeStamp(kukaStatesP,grl::kuka_tag());
    grl::regularizeTimeEvent(timeEventM_Kuka);
    grl::writeTimeEventToCSV(KUKA_TimeEvent_CSV, timeEventM_Kuka);
    Eigen::MatrixXd jointAngles = grl::getAllJointAngles(kukaStatesP);
    std::vector<std::string> Kuka_Joint_Labels = grl::getLabels(grl::LabelsType::Joint);

    if(jointAngles.rows() == timeEventM_Kuka.rows()){
        // Write the joint angles into csv
        grl::writeMatrixToCSV(KUKA_Joint_CSV, Kuka_Joint_Labels, timeEventM_Kuka, jointAngles);
        std::vector<std::string> PK_Pose_Labels = grl::getLabels(grl::LabelsType::Kuka_Pose);
        // forward kinematic to get the of the end effector
        // markerPose = true;  // Get the marker pose directly, or get the end effector pose.
        bool markerPose = false;
        std::vector<sva::PTransformd> PoseEE = grl::getPoseEE(jointAngles, markerPose);
        // convert the sva::PTransformd to Plucker coordinate.
        Eigen::MatrixXd PKPose = grl::getPluckerPose(PoseEE);
        // Write the plucker coordinate into csv.
        grl::writeMatrixToCSV(KUKA_Pose_CSV, PK_Pose_Labels, timeEventM_Kuka, PKPose);
        // Invert the EE pose
        std::vector<sva::PTransformd> inversePose = grl::invertPose(PoseEE);
        // convert the sva::PTransformd to Plucker coordinate.
        PKPose = Eigen::MatrixXd::Zero(PKPose.rows(), PKPose.cols());
        PKPose = grl::getPluckerPose(inversePose);
        // Write the plucker coordinate into csv.

        grl::writeMatrixToCSV(KUKA_Inverse_Pose_CSV, PK_Pose_Labels, timeEventM_Kuka, PKPose);


        // // Pose of end-effector againt robot base frame
        // std::vector<sva::PTransformd> Fudicial_Robot_Pose = grl::getEEToFudicial22Matrix(PoseEE, inMarkerFrame);

        // PKPose = Eigen::MatrixXd::Zero(PKPose.rows(), PKPose.cols());
        // PKPose = grl::getPluckerPose(Fudicial_Robot_Pose);
        // grl::writeMatrixToCSV(FudicialToRobotPose_CSV, PK_Pose_Labels, timeEventM_Kuka, PKPose);

    }


    // grl::writeFTKUKATimeEventToCSV(FTKUKA_TimeEvent_CSV, logKUKAiiwaFusionTrackP, kukaStatesP);
    return 0;
}







