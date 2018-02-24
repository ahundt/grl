
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
std::string kukaBinaryfile = foldname + "2018_02_20_20_27_40_Kukaiiwa.iiwa";
std::string fusiontrackBinaryfile = foldname + "2018_02_20_21_18_39_FusionTrack.flik";
std::string FTKUKA_CSVfilename = foldname + current_date_and_time_string() + "_FTKUKA.csv";
std::string FT_CSVfilename = foldname + current_date_and_time_string() + "_FT.csv";
std::string FT_PoseCSVfilename = foldname + current_date_and_time_string() + "_FT_Pose.csv";
std::string KUKA_CSVfilename = foldname + current_date_and_time_string() + "_KUKA.csv";
std::string KUKA_CSVfilename_Joint = foldname + current_date_and_time_string() + "_KUKA_Joint.csv";
const char *Time_Labels[] = {"local_receive_time_X", "local_request_time_offset", "device_time_offset"};
const char *FT_Pose_Labels[] = {"Counter", "P_X", "P_Y", "P_Z", "Q_X", "Q_Y", "Q_Z", "Q_W"};
const char *FT_Labels[] = {Time_Labels[0], Time_Labels[1], Time_Labels[2], FT_Pose_Labels[0], FT_Pose_Labels[1],
                FT_Pose_Labels[2], FT_Pose_Labels[3], FT_Pose_Labels[4], FT_Pose_Labels[5], FT_Pose_Labels[6], FT_Pose_Labels[7]};



int main(int argc, char* argv[])
{
    fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> FusionTrackStatesRoot = fbs_tk::open_root<grl::flatbuffer::LogKUKAiiwaFusionTrack>(fusiontrackBinaryfile);
    grl::MatrixXd timeEventM_FT = grl::getRegularizedTimeStamp(FusionTrackStatesRoot,grl::fusiontracker_tag());
    uint32_t makerID = 22;
    Eigen::MatrixXd markerPose = grl::getMarkerPose(FusionTrackStatesRoot, makerID, timeEventM_FT);
    /// Combine two matrix into a new one.
    // assert(markerPose.rows() == timeEventM_FT.rows());
    // Eigen::MatrixXd FT_Matrix(markerPose.rows(), timeEventM_FT.cols()+markerPose.cols());
    // /// Cast grl::MatrixXd to Eigen::MatrixXd
    // FT_Matrix << timeEventM_FT.cast<double>(), markerPose;
    std::vector<std::string> FT_Labels_Pose(FT_Labels, grl::end(FT_Labels));
    grl::writeMatrixToCSV(FT_PoseCSVfilename, FT_Labels_Pose, markerPose);

    // fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> KUKAiiwaStatesRoot = fbs_tk::open_root<grl::flatbuffer::KUKAiiwaStates>(kukaBinaryfile);

    // grl::writeFT_KUKATimeEventToCSV(fusiontrackBinaryfile, kukaBinaryfile, FTKUKA_CSVfilename, FT_CSVfilename, FT_PoseCSVfilename, KUKA_CSVfilename);
    // grl::writeJointAngToCSV(kukaBinaryfile, KUKA_CSVfilename_Joint);
    return 0;
}







