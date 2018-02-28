
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

/// split -C 307m --numeric-suffixes 2018_02_26_14_19_55_FusionTrack.json 2018_02_26_14_19_55_FusionTrack

int main(int argc, char* argv[])
{
    std::string foldname = "/home/chunting/src/V-REP_PRO_EDU_V3_4_0_Linux/";
    std::string kukaTimeStamp = "2018_02_26_14_23_14_Kukaiiwa";
    std::string FTTimeStamp = "2018_02_26_14_19_55_FusionTrack";
    std::string kukaBinaryfile = foldname + kukaTimeStamp+ ".iiwa";
    std::string fusiontrackBinaryfile = foldname + FTTimeStamp +".flik";
    std::string foldtimestamp = current_date_and_time_string();
    boost::filesystem::path dir{foldname+foldtimestamp};
    boost::filesystem::create_directory(dir);

    std::string KUKA_TimeEvent_CSVfilename = foldname + foldtimestamp + "/KUKA_TimeEvent.csv";
    std::string KUKA_Joint_CSVfilename = foldname + foldtimestamp + "/KUKA_Joint.csv";
    std::string KUKA_Pose_CSVfilename = foldname + foldtimestamp + "/KUKA_Pose.csv";

    std::string FT_TimeEvent_CSVfilename = foldname + foldtimestamp + "/FT_TimeEvent.csv";
    std::string FT_Marker22_CSVfilename = foldname + foldtimestamp + "/FT_Pose_Marker22.csv";
    std::string FT_Marker55_CSVfilename = foldname + foldtimestamp + "/FT_Pose_Marker55.csv";

    std::string FTKUKA_TimeEvent_CSVfilename = foldname + foldtimestamp + "/FTKUKA_TimeEvent.csv";



    /// Write FT data to CSV
    fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> logKUKAiiwaFusionTrackP = fbs_tk::open_root<grl::flatbuffer::LogKUKAiiwaFusionTrack>(fusiontrackBinaryfile);
    grl::MatrixXd timeEventM_FT = grl::getTimeStamp(logKUKAiiwaFusionTrackP, grl::fusiontracker_tag());
    //grl::regularizeTimeEvent(timeEventM_FT);

    std::size_t FT_size = timeEventM_FT.rows();

    uint32_t makerID_22 = 22;
    uint32_t makerID_55 = 55;
    //timeEvent_FT = grl::MatrixXd::Zero();
    Eigen::MatrixXd markerPose(FT_size, grl::col_FT_Pose);
    int validsize_22 = grl::getMarkerPose(logKUKAiiwaFusionTrackP, makerID_22, timeEventM_FT, markerPose);

    grl::regularizeTimeEvent(timeEventM_FT);
    std::vector<std::string> FT_Labels_Pose = grl::getLabels(grl::LabelsType::FT_Pose);
    if(validsize_22>2){
        grl::writeTimeEventToCSV(FT_TimeEvent_CSVfilename, timeEventM_FT);
        grl::writeMatrixToCSV(FT_Marker22_CSVfilename, FT_Labels_Pose, timeEventM_FT, markerPose);
    }

    timeEventM_FT.resize(FT_size, grl::col_timeEvent);

    markerPose.resize(FT_size, grl::col_FT_Pose);
    //timeEvent_FT = grl::MatrixXd::Zero();
    //markerPose = Eigen::MatrixXd::Zero();
    int validsize_55 = grl::getMarkerPose(logKUKAiiwaFusionTrackP, makerID_55, timeEventM_FT, markerPose);
    grl::regularizeTimeEvent(timeEventM_FT);

    std::cout<<"validsize_22: " << validsize_22 << "   validsize_55: " << validsize_55 << std::endl;


    if(validsize_55>2) {
        grl::writeMatrixToCSV(FT_Marker55_CSVfilename, FT_Labels_Pose, timeEventM_FT, markerPose);
    }

    // /// Write KUKA data to CSV
    fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> kukaStatesP = fbs_tk::open_root<grl::flatbuffer::KUKAiiwaStates>(kukaBinaryfile);
    grl::MatrixXd timeEventM_Kuka = grl::getTimeStamp(kukaStatesP,grl::kuka_tag());
    grl::regularizeTimeEvent(timeEventM_Kuka);
    grl::writeTimeEventToCSV(KUKA_TimeEvent_CSVfilename, timeEventM_Kuka);
    Eigen::MatrixXd jointAngles = grl::getAllJointAngles(kukaStatesP);
    std::vector<std::string> Kuka_Joint_Labels = grl::getLabels(grl::LabelsType::Joint);

    if(jointAngles.rows() == timeEventM_Kuka.rows()){
        grl::writeMatrixToCSV(KUKA_Joint_CSVfilename, Kuka_Joint_Labels, timeEventM_Kuka, jointAngles);
        std::vector<std::string> Kuka_Pose_Labels = grl::getLabels(grl::LabelsType::Kuka_Pose);
        Eigen::MatrixXd PoseEE = grl::getPoseEE(jointAngles);
        grl::writeMatrixToCSV(KUKA_Pose_CSVfilename, Kuka_Pose_Labels, timeEventM_Kuka, PoseEE);

    }


    // grl::writeFTKUKATimeEventToCSV(FTKUKA_TimeEvent_CSVfilename, logKUKAiiwaFusionTrackP, kukaStatesP);
    return 0;
}







