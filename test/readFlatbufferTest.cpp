
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
    /// Define the csv file names
    std::string foldname = "/home/cjiao1/src/V-REP_PRO_EDU_V3_4_0_Linux/";
    std::string kukaTimeStamp = "2018_03_26_19_06_21_Kukaiiwa";
    std::string FTTimeStamp = "2018_03_26_19_06_21_FusionTrack";
    std::string kukaBinaryfile = foldname + kukaTimeStamp+ ".iiwa";
    std::string fusiontrackBinaryfile = foldname + FTTimeStamp +".flik";
    std::string foldtimestamp = current_date_and_time_string();
    boost::filesystem::path dir{foldname+foldtimestamp};
    boost::filesystem::create_directory(dir);
    
    std::string KUKA_FRI_CSVfilename = foldname + foldtimestamp + "/KUKA_FRIMessage.csv";
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

    auto strRobot = grl::getURDFModel();
    rbd::MultiBody mb = strRobot.mb;
    rbd::MultiBodyConfig mbc(mb);
    rbd::MultiBodyGraph mbg(strRobot.mbg);
    std::size_t nrJoints = mb.nrJoints();
    std::size_t nrBodies = strRobot.mb.nrBodies();
    std::vector<std::string> jointNames;
    std::cout<<"Joint Size: "<< nrJoints << std::endl;

    // Put all the data into the same coordinate system --- Marker frame
    bool inMarkerFrame = false;    // Indicate the marker pose is in marker frame or tracker frame.
    uint32_t markerID_22 = 22;
    uint32_t markerID_55 = 55;
    uint32_t markerID_50000 = 50000;
    /// Get the object pointer to read data from flatbuffer
    fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> logKUKAiiwaFusionTrackP = fbs_tk::open_root<grl::flatbuffer::LogKUKAiiwaFusionTrack>(fusiontrackBinaryfile);
    fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> kukaStatesP = fbs_tk::open_root<grl::flatbuffer::KUKAiiwaStates>(kukaBinaryfile);
    std::size_t FT_size = grl::getStateSize(logKUKAiiwaFusionTrackP);
    std::size_t kuka_size = grl::getStateSize(kukaStatesP);

    /// Create matrix to contain data
    Eigen::MatrixXd markerPose_FT(FT_size, grl::col_Pose);
    Eigen::MatrixXd markerTransform_FT  = Eigen::MatrixXd::Zero(FT_size, grl::Transform_Labels.size());
    grl::MatrixXd timeEventM_FT = grl::MatrixXd::Zero(FT_size, grl::Time_Labels.size());
    grl::MatrixXd timeEventM_Kuka = grl::MatrixXd::Zero(kuka_size, grl::Time_Labels.size());
    Eigen::VectorXd sequenceCounterVec =  Eigen::VectorXd::Zero(kuka_size);
    Eigen::VectorXd reflectedSequenceCounterVec =  Eigen::VectorXd::Zero(kuka_size);
    Eigen::MatrixXd measuredJointPosition = Eigen::MatrixXd::Zero(kuka_size, grl::jointNum);
    Eigen::MatrixXd measuredTorque = Eigen::MatrixXd::Zero(kuka_size, grl::jointNum);
    Eigen::MatrixXd commandedJointPosition = Eigen::MatrixXd::Zero(kuka_size, grl::jointNum);
    Eigen::MatrixXd commandedTorque = Eigen::MatrixXd::Zero(kuka_size, grl::jointNum);
    Eigen::MatrixXd externalTorque = Eigen::MatrixXd::Zero(kuka_size, grl::jointNum);
    Eigen::MatrixXd jointStateInterpolated = Eigen::MatrixXd::Zero(kuka_size, grl::jointNum);
    int ret = grl::getFRIMessage(kukaStatesP,
        timeEventM_Kuka, 
        sequenceCounterVec,
        reflectedSequenceCounterVec,
        measuredJointPosition, 
        measuredTorque, 
        commandedJointPosition, 
        commandedTorque, 
        externalTorque,
        jointStateInterpolated);
    int validsize_22 = grl::getMarkerPose(logKUKAiiwaFusionTrackP, markerID_22, timeEventM_FT, markerPose_FT);
   
    std::cout<< "------Kuka_size: "<< kuka_size << "     FT_size: "<< FT_size << std::endl;
    std::cout<<"validsize_22: " << validsize_22 << std::endl;
    std::size_t FT_time_size = timeEventM_FT.rows();
    std::size_t kuka_time_size = timeEventM_Kuka.rows();
    grl::MatrixXd timeEventM = grl::MatrixXd::Zero(FT_size, timeEventM_FT.cols());
    
    int kuka_index = 0;
    int FT_index = 0;

    // Skip the very beginning data,since the tracker starts to work once the scene is loaded in vrep.
    // But kuka starts to work only after clicking on the start button.
    // To combine the time from two devices, they should have the same starting time point.
    while(kuka_index<kuka_time_size && timeEventM_Kuka(kuka_index, grl::TimeType::local_receive_time) < timeEventM_FT(FT_index,grl::TimeType::local_receive_time)){
        kuka_index++;
    }
    while(kuka_index == 0 && FT_index<FT_time_size && timeEventM_Kuka(kuka_index,grl::TimeType::local_receive_time) > timeEventM_FT(FT_index,grl::TimeType::local_receive_time) ){
        FT_index++;
    }
 
    auto initial_local_time = std::min(timeEventM_FT(FT_index,grl::TimeType::local_receive_time), timeEventM_Kuka(kuka_index, grl::TimeType::local_receive_time));
    auto initial_device_time_kuka = timeEventM_Kuka(kuka_index,grl::TimeType::device_time);
    auto initial_device_time_FT = timeEventM_FT(FT_index,grl::TimeType::device_time);
    std::cout<< "Main initial_local_time: " << initial_local_time << std::endl;
    timeEventM_FT.col(grl::TimeType::local_request_time) = timeEventM_FT.col(grl::TimeType::local_request_time) - initial_local_time * grl::VectorXd::Ones(FT_time_size);
    timeEventM_FT.col(grl::TimeType::local_receive_time) = timeEventM_FT.col(grl::TimeType::local_receive_time) - initial_local_time * grl::VectorXd::Ones(FT_time_size);
    timeEventM_FT.col(grl::TimeType::device_time) = timeEventM_FT.col(grl::TimeType::device_time) - initial_device_time_FT * grl::VectorXd::Ones(FT_time_size);
    timeEventM_FT.col(grl::TimeType::time_Y) = timeEventM_FT.col(grl::TimeType::time_Y) - timeEventM_FT(FT_index, grl::TimeType::time_Y) * grl::VectorXd::Ones(FT_time_size);

    timeEventM_Kuka.col(grl::TimeType::local_request_time) = timeEventM_Kuka.col(grl::TimeType::local_request_time) - initial_local_time * grl::VectorXd::Ones(kuka_time_size);
    timeEventM_Kuka.col(grl::TimeType::local_receive_time) = timeEventM_Kuka.col(grl::TimeType::local_receive_time) - initial_local_time * grl::VectorXd::Ones(kuka_time_size);
    timeEventM_Kuka.col(grl::TimeType::device_time) = timeEventM_Kuka.col(grl::TimeType::device_time) - initial_device_time_kuka * grl::VectorXd::Ones(kuka_time_size);
    timeEventM_Kuka.col(grl::TimeType::time_Y) = timeEventM_Kuka.col(grl::TimeType::time_Y) - timeEventM_Kuka(kuka_index, grl::TimeType::time_Y) * grl::VectorXd::Ones(kuka_time_size);
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Write KUKA data to CSV
    ////////////////////////////////////////////////////////////////////////////////////////////////////
  
    grl::writeFRIMessageToCSV(KUKA_FRI_CSVfilename,
        timeEventM_Kuka, 
        sequenceCounterVec,
        reflectedSequenceCounterVec,
        measuredJointPosition, 
        measuredTorque, 
        commandedJointPosition, 
        commandedTorque, 
        externalTorque,
        jointStateInterpolated,
        kuka_index);

        timeEventM_Kuka.col(grl::TimeType::counterIdx) = std::move(sequenceCounterVec.cast<int64_t>());
        grl::writeTimeEventToCSV(KUKA_TimeEvent_CSV, timeEventM_Kuka, kuka_index);
        std::vector<std::string> kuka_labels = grl::Time_Labels;
        kuka_labels.insert(std::end(kuka_labels), std::begin(grl::M_Pos_Joint_Labels), std::end(grl::M_Pos_Joint_Labels));
        // Write the joint angles into csv
        grl::writeMatrixToCSV(KUKA_Joint_CSV, kuka_labels, timeEventM_Kuka, measuredJointPosition);
        std::vector<std::string> PK_Pose_Labels = grl::getLabels(grl::LabelsType::Kuka_Pose);
        // forward kinematic to get the of the end effector
        // markerPose_FT = true;  // Get the marker pose directly, or get the end effector pose.
        // bool markerPose_FT = false;
        std::vector<sva::PTransformd> PoseEE = grl::getPoseEE(measuredJointPosition);
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

        
   
    // Get the pose of the marker 22.
    // Since the bad data is skipped, the timeEventM_FT also needs to be filled out with the orinial time stamp.

    
    std::vector<std::string> FT_Labels_Pose = grl::getLabels(grl::LabelsType::FT_Pose);

    grl::writeFTKUKAMessageToCSV(FTKUKA_TimeEvent_CSV, timeEventM_FT, timeEventM_Kuka, measuredJointPosition, markerPose_FT, FT_index, kuka_index);
    if(validsize_22>2){
        grl::writeTimeEventToCSV(FT_TimeEvent_CSV, timeEventM_FT, FT_index);
        grl::writeMatrixToCSV(FT_Marker22_CSV, FT_Labels_Pose, timeEventM_FT, markerPose_FT, FT_index);
    }
    // Change it back to the original size
    timeEventM_FT.resize(FT_size, grl::col_timeEvent);
    markerPose_FT.resize(FT_size, grl::col_Pose);
    // Get the pose of the bone marker
    int validsize_55 = grl::getMarkerPose(logKUKAiiwaFusionTrackP, markerID_55, timeEventM_FT, markerPose_FT);

    std::cout<<"validsize_55: " << validsize_55 << std::endl;
    if(validsize_55>2) {
        grl::regularizeTimeEvent(timeEventM_FT, initial_local_time, initial_device_time_FT, FT_index);
        grl::writeMatrixToCSV(FT_Marker55_CSV, FT_Labels_Pose, timeEventM_FT, markerPose_FT, FT_index);
    }
     // Change it back to the original size
    timeEventM_FT.resize(FT_size, grl::col_timeEvent);
    markerPose_FT.resize(FT_size, grl::col_Pose);
    // Get the pose of the bone marker
    int validsize_50000 = grl::getMarkerPose(logKUKAiiwaFusionTrackP, markerID_50000, timeEventM_FT, markerPose_FT);

    std::cout<<"validsize_50000: " << validsize_50000 << std::endl;
    if(validsize_50000>2) {
        grl::regularizeTimeEvent(timeEventM_FT, initial_local_time, initial_device_time_FT, FT_index);
        grl::writeMatrixToCSV(FT_Marker50000_CSV, FT_Labels_Pose, timeEventM_FT, markerPose_FT, FT_index);
    }
   
    





    // grl::writeFTKUKAMessageToCSV(FTKUKA_TimeEvent_CSV, logKUKAiiwaFusionTrackP, kukaStatesP, markerID_22);
    return 0;
}







