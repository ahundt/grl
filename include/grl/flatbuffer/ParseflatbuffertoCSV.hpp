#ifndef GRL_READ_DATA_FROM_BINARY
#define GRL_READ_DATA_FROM_BINARY

#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE

// grl
#include "grl/vrep/Vrep.hpp"
#include "grl/vrep/VrepRobotArmDriver.hpp"
#include "grl/vrep/VrepRobotArmJacobian.hpp"
#include "grl/vrep/Eigen.hpp"
#include "grl/vrep/SpaceVecAlg.hpp"

//
#include "grl/flatbuffer/FlatbufferToEigen.hpp"


// FusionTrack Libraries
#include <ftkInterface.h>

#include "grl/vector_ostream.hpp"
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
#include <vector>
#include <set>
#include <iterator>
#include <algorithm>

#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>


#include <string>
#include <tuple>
#include <boost/format.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "camodocal/EigenUtils.h"

#include "grl/time.hpp"

// SpaceVecAlg
// https://github.com/jrl-umi3218/SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>


// RBDyn
// https://github.com/jrl-umi3218/RBDyn
#include <RBDyn/Body.h>
#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/FK.h>

// mc_rbdyn_urdf
// https://github.com/jrl-umi3218/mc_rbdyn_urdf
#include <mc_rbdyn_urdf/urdf.h>
#include "kukaiiwaURDF.h"



namespace grl {


    /// Define some constants.
    const double RadtoDegree =1;// 180/3.14159265359;
    const double MeterToMM = 1;
    /// Define the int64_t vector and matrix, which is used for time data.
    typedef Eigen::Matrix<int64_t , Eigen::Dynamic , 1>  VectorXd;
    typedef Eigen::Matrix<int64_t , Eigen::Dynamic , Eigen::Dynamic>  MatrixXd;

    std::vector<std::string> Time_Labels = {"local_receive_time_X", "local_request_time_offset", "device_time_offset", "time_Y", "counter"};
    // std::vector<std::string> PK_Pose_Labels = {"P_X", "P_Y", "P_Z", "Q_X", "Q_Y", "Q_Z", "Q_W"};
    std::vector<std::string> Joint_Labels = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6", "Joint_7"};
    std::vector<std::string> PK_Pose_Labels  = {"X", "Y", "Z", "A", "B", "C"};
    struct kuka_tag {};
    struct fusiontracker_tag {};
    int col_timeEvent=Time_Labels.size();
    int col_Kuka_Joint = Joint_Labels.size();
    int col_Pose = PK_Pose_Labels.size();
    enum TimeType { local_receive_time = 0, local_request_time = 1, device_time = 2, time_Y = 3, counterIdx=4};
    enum LabelsType { FT_Pose = 0, Joint = 1, Kuka_Pose= 2};


    /// Get CSV labels
    /// @param label indicate the type of the label.
    std::vector<std::string> getLabels( LabelsType label){
        std::vector<std::string> labels;
        labels.insert(labels.end(), Time_Labels.begin(), Time_Labels.end());
        switch(label) {
            case FT_Pose:
                labels.insert(labels.end(), PK_Pose_Labels.begin(), PK_Pose_Labels.end());
                break;
            case Joint:
                labels.insert(labels.end(), Joint_Labels.begin(), Joint_Labels.end());
                break;
            case Kuka_Pose:
                labels.insert(labels.end(), PK_Pose_Labels.begin(), PK_Pose_Labels.end());
                break;
            default:
                std::cout<<"Only return Time_Labels..."<<std::endl;
        }
        return labels;
    }

    /// Check the monotonicity for the time column, which will be used as the time series
    /// @param T &time time series.
    template<typename T>
    bool checkmonotonic( T &time){
        for(int i=1; i<time.size(); ++i){
            if(time(i)<=time(i-1) || time(i)<0) {
                std::cout<<"Receive time NOT VALID: " << time(i) << std::endl;
                return false;
            }

        }
        return true;
    }

    /// Return the original timeEvent of fusiontracker
    /// @param TlogKUKAiiwaFusionTrackP pointer of the root object for fusiontracker.
    /// @param fusiontracker_tag tag dispatch
    grl::MatrixXd  getTimeStamp(const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP, fusiontracker_tag){

        auto states = logKUKAiiwaFusionTrackP->states();
        std::size_t state_size = states->size();
        assert(state_size);
        int row = 0;
        grl::MatrixXd timeEventM(state_size,col_timeEvent);
        for(int i = 0; i<state_size; ++i){
            auto kukaiiwaFusionTrackMessage = states->Get(i);
            auto FT_Message = kukaiiwaFusionTrackMessage->deviceState_as_FusionTrackMessage();
            auto FT_Frame = FT_Message->frame();
            auto counter = FT_Frame->counter();
            auto Makers = FT_Frame->markers();
            if(Makers!=nullptr) {
                auto makerSize = Makers->size();
                for(int markerIndex=0; markerIndex<makerSize; markerIndex++) {
                    auto marker = Makers->Get(markerIndex);
                    auto marker_ID = marker->geometryID();
                    if(marker_ID == 22 || marker_ID == 55){
                        auto timeEvent = kukaiiwaFusionTrackMessage->timeEvent();
                        timeEventM(row,TimeType::local_receive_time) = timeEvent->local_receive_time();
                        timeEventM(row,TimeType::local_request_time) = timeEvent->local_request_time();
                        timeEventM(row,TimeType::device_time) = timeEvent->device_time();
                        timeEventM(row,TimeType::time_Y) = timeEventM(row,TimeType::device_time) - timeEventM(row,TimeType::local_receive_time);
                        timeEventM(row,TimeType::counterIdx) = counter;
                        row++;
                        break;
                    }
                }
            }
        }
        if(row < state_size) {
            timeEventM.conservativeResize(row, Eigen::NoChange_t{});
        }
        grl::VectorXd timeCol = timeEventM.col(TimeType::local_receive_time);
        assert(checkmonotonic(timeCol));
        return timeEventM;
    }
    /// Return the original timeEvent of Kuka
    /// @param kukaStatesP pointer of the root object for kuka.
    /// @param kuka_tag tag dispatch
    /// @return timeEventM timeEvent matrix which should have four columns
    /// local_receive_time_offset = local_receive_time - initial_local_receive_time
    /// local_request_time_offset = local_request_time - initial_local_request_time
    /// device_time_offset = device_time - initial_device_time
    /// time_Y = device_time_offset - local_receive_time_offset
    grl::MatrixXd  getTimeStamp(const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP, kuka_tag){
        auto states = kukaStatesP->states();
        std::size_t state_size = states->size();
        assert(state_size);
        grl::MatrixXd timeEventM(state_size,col_timeEvent);
        for(int i = 0; i<state_size; ++i){
            auto KUKAiiwaState = states->Get(i);
            auto FRIMessage = KUKAiiwaState->FRIMessage();
            auto identifier = FRIMessage->messageIdentifier();
            auto timeEvent = FRIMessage->timeStamp();
            timeEventM(i,TimeType::local_receive_time) = timeEvent->local_receive_time();
            timeEventM(i,TimeType::local_request_time) = timeEvent->local_request_time();
            timeEventM(i,TimeType::device_time) = timeEvent->device_time();
            timeEventM(i,TimeType::time_Y) =  timeEventM(i,TimeType::device_time) - timeEventM(i,TimeType::local_receive_time);
            timeEventM(i,TimeType::counterIdx) = identifier;
        }
        grl::VectorXd timeCol = timeEventM.col(TimeType::local_receive_time);
        assert(checkmonotonic(timeCol));
        return timeEventM;
    }
    /// Process the time data to get the time offset based on the starting time point.
    /// See https://github.com/facontidavide/PlotJuggler/issues/68
    /// @param timeEventM timeEvent matrix which should have four columns
    void regularizeTimeEvent(grl::MatrixXd& timeEventM){
        std::size_t time_size = timeEventM.rows();
        assert(time_size>0);
        auto initial_local_time = timeEventM(0,TimeType::local_receive_time);
        auto initial_device_time = timeEventM(0,TimeType::device_time);
        timeEventM.col(TimeType::local_receive_time) = timeEventM.col(TimeType::local_receive_time) - initial_local_time * grl::VectorXd::Ones(time_size);
        timeEventM.col(TimeType::local_request_time) = timeEventM.col(TimeType::local_request_time) - initial_local_time * grl::VectorXd::Ones(time_size);

        timeEventM.col(TimeType::device_time) = timeEventM.col(TimeType::device_time) - initial_device_time * grl::VectorXd::Ones(time_size);
        timeEventM.col(TimeType::time_Y) = timeEventM.col(TimeType::device_time) - timeEventM.col(TimeType::local_receive_time);
        grl::VectorXd timeCol = timeEventM.col(TimeType::local_receive_time);
        assert(checkmonotonic(timeCol));
    }


    /// Get the maker pose based on the markerID. The bad data, which means the frame doesn't have the indicated marker information, has been filtered out.
    /// As for the bad data, both the marker pose and the corresponding timeEvent is skipped.
    /// @param logKUKAiiwaFusionTrackP, pointer of the root object for fusiontracker.
    /// @param makerID, the indicated marker.
    /// @param timeEventM, timeEvent without bad data, which is filled out.
    /// @param markerPose, the pose matrix, which is filled out.
    /// @return row, the number of valid rows.
    int  getMarkerPose(const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP,
                                    uint32_t &makerID,
                                    grl::MatrixXd& timeEventM,
                                    Eigen::MatrixXd& markerPose,
                                    bool markerFrame = false){
        auto states = logKUKAiiwaFusionTrackP->states();
        std::size_t state_size = states->size();
        assert(state_size>0);
        // The first columne is counter
        int row = 0;
        // Eigen::MatrixXd markerPose(state_size, grl::col_Pose);
        int BadCount = 0;
        for(int i = 0; i<state_size-BadCount; ++i){
            auto kukaiiwaFusionTrackMessage = states->Get(i);
            auto FT_Message = kukaiiwaFusionTrackMessage->deviceState_as_FusionTrackMessage();
            auto FT_Frame = FT_Message->frame();
            auto counter = FT_Frame->counter();
            auto Makers = FT_Frame->markers();
            /// In some frames, there are no markers, where we skip that line. Also we need to remove the corresponding line in TimeEvent.
            if(Makers!=nullptr) {
                auto makerSize = Makers->size();
                for(int markerIndex=0; markerIndex<makerSize; markerIndex++) {
                    auto marker = Makers->Get(markerIndex);
                    auto marker_ID = marker->geometryID();
                    if(marker_ID == makerID){

                        auto timeEvent = kukaiiwaFusionTrackMessage->timeEvent();
                        timeEventM(row,TimeType::local_receive_time) = timeEvent->local_receive_time();
                        timeEventM(row,TimeType::local_request_time) = timeEvent->local_request_time();
                        timeEventM(row,TimeType::device_time) = timeEvent->device_time();
                        timeEventM(row,TimeType::time_Y) = timeEventM(row,TimeType::device_time) - timeEventM(row,TimeType::local_receive_time);
                        timeEventM(row,TimeType::counterIdx) = counter;
                        auto Pose = marker->transform();
                        auto FB_r = Pose->position();
                        auto FB_q = Pose->orientation();
                        // Convert the flatbuffer type to Eigen type
                        Eigen::Vector3d r(FB_r.x(), FB_r.y(), FB_r.z());
                        Eigen::Quaterniond q(FB_q.w(), FB_q.x(), FB_q.y(), FB_q.z());
                        Eigen::Matrix3d E = q.normalized().toRotationMatrix();
                        Eigen::Vector3d eulerAngleEigen = RadtoDegree*E.eulerAngles(0,1,2);
                        Eigen::RowVectorXd pose(grl::col_Pose);
                        pose << r.transpose(), eulerAngleEigen.transpose();


                        if(markerFrame == true) {
                            Eigen::VectorXd onePose(7);
                            onePose << r, q.w(), q.x(), q.y(), q.z();
                            auto markerToCameraTransform = grl::MarkerPoseToAffine3f(onePose);
                            auto cameraTomarkerTransform = markerToCameraTransform.inverse();
                            auto _markerPose = grl::Affine3fToMarkerPose(cameraTomarkerTransform);
                            pose = grl::getPluckerPose(_markerPose);

                        }
                        markerPose.row(row++) = pose;
                    }
                }
            }
        }
        if(row < state_size) {
            markerPose.conservativeResize(row, Eigen::NoChange_t{});
            timeEventM.conservativeResize(row, Eigen::NoChange_t{});
        }
        std::cout <<"State size: " << state_size <<"  markerPose size: " << markerPose.rows() << "  timeEvent: " << timeEventM.rows() << "  makerID: " << makerID <<std::endl;
        return row;
    }



    /// Get the joint angles of a specific joint (joint_index)
    /// @param kukaStatesP pointer of the root object for kuka.
    /// @param joint_index, return the joint angles of the indicated joint.
    /// @return jointPosition, Eigen vector which contains joint position.
    Eigen::VectorXf getJointAnglesFromKUKA(const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP, int joint_index){

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
    /// Get the all the joint angles.
    /// @param kukaStatesP pointer of the root object for kuka.
    /// @return allJointPosition, Eigen matrix which contains the positions of all the joints.
    Eigen::MatrixXd getAllJointAngles(const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP){

        auto states = kukaStatesP->states();
        std::size_t state_size = states->size();
        Eigen::MatrixXd allJointPosition(state_size, 7);
        for(int i = 0; i<state_size; ++i){
            auto KUKAiiwaState = states->Get(i);
            auto FRIMessage = KUKAiiwaState->FRIMessage();
            auto joints_Position = FRIMessage->measuredJointPosition(); // flatbuffers::Vector<double> *
            Eigen::VectorXd oneStateJointPosition(7);
            for(int joint_index=0; joint_index<7; joint_index++){
                oneStateJointPosition(joint_index) = joints_Position->Get(joint_index);
            }
            allJointPosition.row(i) = oneStateJointPosition.transpose();
        }
        return allJointPosition;
    }
    /// Write the data to CSV file
    /// @param CSV_FileName, the file name.
    /// @param labels, the labels for the data to write, which should be the first row of the csv file
    /// @param timeM, by default we write all the data combined with the timestamp, whose type is grl::MatrixXd (int64_t).
    /// @param dataM, the data to write, whose type is Eigen::MatrixXd
    template <class T1, class T2>
    void writeMatrixToCSV(const std::string& CSV_FileName, std::vector<std::string> &labels, T1& timeM, T2& dataM){
        std::size_t labels_size = labels.size();
        std::size_t cols_size = timeM.cols() + dataM.cols();
        std::size_t time_rows_size = timeM.rows();
        std::size_t data_rows_size = dataM.rows();
        assert(labels_size == cols_size && time_rows_size>0 && cols_size>0 && time_rows_size==data_rows_size);
        auto time = timeM.col(TimeType::local_receive_time);
        assert(checkmonotonic(time));
        // create an ofstream for the file output (see the link on streams for more info)
        std::ofstream fs;
        // create a name for the file output
        fs.open(CSV_FileName, std::ofstream::out | std::ofstream::app);
        // write the file labels;
        for(int col=0; col<cols_size-1; col++){
            fs << labels[col] << ",";
        }
        fs << labels[cols_size-1] << std::endl;
        for(int row_index=0; row_index<time_rows_size; ++row_index) {
            // write the data to the output file
            auto timerow = timeM.row(row_index);
            auto matrixrow = dataM.row(row_index);
            for(int col=0; col<timeM.cols(); col++){
                fs << timerow[col] << ",";
            }
            for(int col=0; col<dataM.cols()-1; col++){
                fs << matrixrow[col] << ",";
            }
            fs << matrixrow[dataM.cols()-1] << std::endl;
        }
        fs.close();
        std::cout<< CSV_FileName << ": Rows " << time_rows_size << "   Cols: " << cols_size << std::endl;
    }
    /// Process the timeEvent to get more information, and write the result into a csv file directly.
    /// @param timeEventM
    void writeTimeEventToCSV( std::string & CSV_FileName, grl::MatrixXd& timeEventM) {
        std::size_t time_size = timeEventM.rows();
        grl::VectorXd local_receive_timeV = timeEventM.col(local_receive_time);
        grl::VectorXd local_request_timeV = timeEventM.col(local_request_time);
        grl::VectorXd device_timeV = timeEventM.col(device_time);
        grl::VectorXd counter = timeEventM.col(counterIdx);
        grl::VectorXd receive_request = local_receive_timeV - local_request_timeV;
        //  grl::VectorXd device_time_offset = local_receive_time - local_request_time;
        std::ofstream fs;
        // create a name for the file output
        fs.open( CSV_FileName, std::ofstream::out | std::ofstream::app);
         // write the file headers
        fs << "local_receive_time_X,"
           << "local_request_time_offset,"
           << "device_time_offset,"
           << "Y,"
           << "Receive-Request,"
           << "device_time_step,"
           << "receive_time_step,"
           << "counter"
           << std::endl;
        int64_t device_time_step = 0;
        int64_t receive_time_step = 0;
        for(int i=0; i<time_size; ++i) {
            if(i>0) {
                device_time_step = device_timeV(i) - device_timeV(i-1);
                receive_time_step = local_receive_timeV(i) - local_receive_timeV(i-1);
            }
            // write the data to the output file
            fs << local_receive_timeV(i) << ","         // A
               << local_request_timeV(i)<<","          // B
               << device_timeV(i) <<","                        // C
               << device_timeV(i) - local_receive_timeV(i) << ","
               << receive_request(i) << ","
               << device_time_step << ","
               << receive_time_step << ","
               << counter(i) << std::endl;  //D
        }
        fs.close();
    }
    /// With the same method to process the time data from both Kuka and fusiontracker, combine it and write to a csv file.
    /// @param FTKUKA_TimeEvent_CSVfilename, the csv file name
    void writeFTKUKATimeEventToCSV(std::string& FTKUKA_TimeEvent_CSVfilename,
                                    const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP,
                                    const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP) {

        grl::MatrixXd timeEventM_FT = grl::getTimeStamp(logKUKAiiwaFusionTrackP, grl::fusiontracker_tag());
        grl::MatrixXd timeEventM_Kuka = grl::getTimeStamp(kukaStatesP, grl::kuka_tag());
        std::size_t kuka_state_size = timeEventM_Kuka.rows();
        std::cout<< "------Kuka_state_size: "<< timeEventM_Kuka.rows() << std::endl;

        std::size_t FT_state_size = timeEventM_FT.rows();
        std::cout<< "------FT_state_size: "<< timeEventM_FT.rows() << std::endl;
        int kuka_index = 0;
        int FT_index = 0;

        // Filter out the very beginning data,since the tracker starts to work once the scene is loaded in vrep.
        // But kuka starts to work only after clicking on the start button.
        // To combine the time from two devices, they should share the same starting time point.
        while(kuka_index<kuka_state_size && timeEventM_Kuka(kuka_index,TimeType::local_receive_time) < timeEventM_FT(FT_index,TimeType::local_receive_time)){
            kuka_index++;
        }
        while(FT_index<FT_state_size && timeEventM_Kuka(kuka_index,local_receive_time) > timeEventM_FT(FT_index,local_receive_time) && kuka_index == 0){
            FT_index++;
        }

        auto initial_local_time = std::min(timeEventM_FT(FT_index,local_receive_time), timeEventM_Kuka(kuka_index,local_receive_time));
        auto initial_device_time_kuka = timeEventM_Kuka(kuka_index,device_time);
        auto initial_device_time_FT = timeEventM_FT(FT_index,device_time);
        grl::VectorXd FT_local_request_time = timeEventM_FT.col(local_request_time) - initial_local_time * grl::VectorXd::Ones(FT_state_size);
        grl::VectorXd FT_local_receive_time = timeEventM_FT.col(local_receive_time) - initial_local_time * grl::VectorXd::Ones(FT_state_size);
        grl::VectorXd FT_device_time = timeEventM_FT.col(device_time) - initial_device_time_FT * grl::VectorXd::Ones(FT_state_size);

        grl::VectorXd kuka_local_request_time = timeEventM_Kuka.col(local_request_time) - initial_local_time * grl::VectorXd::Ones(kuka_state_size);
        grl::VectorXd kuka_local_receive_time = timeEventM_Kuka.col(local_receive_time) - initial_local_time * grl::VectorXd::Ones(kuka_state_size);
        grl::VectorXd kuka_device_time = timeEventM_Kuka.col(device_time) - initial_device_time_kuka * grl::VectorXd::Ones(kuka_state_size);


        std::ofstream fs;
        // create a name for the file output
        fs.open( FTKUKA_TimeEvent_CSVfilename, std::ofstream::out | std::ofstream::app);
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
            // If the row value is the kuka time, then the FT cells should be left blank.
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
                // If the row value is the FT time, then the kuka cells should be left blank.
                fs << FT_local_receive_time(FT_index) <<","
                   << FT_local_request_time(FT_index) << ","
                   <<","
                   << FT_device_time(FT_index) << ","
                   << ","
                   << FT_device_time(FT_index) - FT_local_receive_time(FT_index) - FT_diff << ","
                   << std::endl;
                FT_index++;
            } else {
                // In case the time is extactly equivent with each other.
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
    /// Get the transformation matrix from LBR_iiwa_14_R820#0 to OpticalTrackerBase#0.
    /// See the physical relationship in vrep.
    std::vector<sva::PTransformd>  getRobotToTrackerMatrix(std::vector<sva::PTransformd>& FudicialPose){
        Eigen::Vector3d trans;
        trans << 0.20652678608894 , -0.70480275154114 ,1.4510889053345 ;
        Eigen::Matrix3d rot;
        rot << -0.19507896900177, -0.8963907957077, -0.39802974462509  ,
               0.9409122467041, -0.28804504871368, 0.17808490991592,
               -0.28378140926361, -0.33688074350357, 0.89776360988617 ;
        sva::PTransformd relativeTransform(rot, trans);
        std::vector<sva::PTransformd> to(FudicialPose.size());
        for(int i=0; i<FudicialPose.size();++i){
            auto pose = FudicialPose[i];
            to[i] = relativeTransform*pose;
        }
          return std::move(to);
    }
    /// Based on the RBDy and URDF model, get the cartesian pose of the end effector.
    /// BE CAREFUL THAT THE URDF MODEL HAS BEEN MODIFIED, THE MARKER LINK HAS BEEN ADDED.
    ///  <origin rpy="0 0 0" xyz="0.052998 0.090310 0.090627"/>
    /// This is gotten from VREP, the oritation of the marker dummy is identical with the flange ('Fiducial#22' and 'RobotFlangeTip').
    /// SEE kukaiiwaURDF.h
    /// @param jointAngles, the joint angles matrix
    /// @return poseEE, contain the translation and the Euler angle.
    std::vector<sva::PTransformd> getPoseEE(Eigen::MatrixXd& jointAngles, bool markerPose = false){
        using namespace Eigen;
	    using namespace sva;
	    using namespace rbd;

        namespace cst = boost::math::constants;
        auto strRobot = mc_rbdyn_urdf::rbdyn_from_urdf(XYZSarmUrdf);
        rbd::MultiBody mb = strRobot.mb;
        rbd::MultiBodyConfig mbc(mb);
        rbd::MultiBodyGraph mbg(strRobot.mbg);
        std::size_t nrJoints = mb.nrJoints();
        std::size_t nrBodies = strRobot.mb.nrBodies();
        std::vector<std::string> jointNames;
        std::cout<<"Joint Size: "<< nrJoints << std::endl;

        std::size_t row_size = jointAngles.rows();
        std::size_t body_size = mbc.bodyPosW.size();
        /// The translation and Euler angles in world coordinate.
        ///
        std::vector<sva::PTransformd> EEpose;

        for(int rowIdx=0; rowIdx<row_size; rowIdx++){
            Eigen::VectorXd oneStateJointPosition = jointAngles.row(rowIdx);
            int jointIdx = 0;
            for(int jointIndex = 0; jointIndex< nrJoints; jointIndex++) {
                const auto & joint = strRobot.mb.joint(jointIndex);
                jointNames.push_back(joint.name());
                // std::cout << "Joint Name: " << joint.name() << "     Size: " << mbc.q[jointIndex].size() << std::endl;
                if (mbc.q[jointIndex].size() > 0) {
                    mbc.q[jointIndex][0] = oneStateJointPosition[jointIdx];
                    jointIdx++;
                }
            }
            rbd::forwardKinematics(mb, mbc);
            // Pose of the end effector relative to robot base frame.
            if(markerPose){
                EEpose.push_back(mbc.bodyPosW[body_size-1]);
            } else{
                EEpose.push_back(mbc.bodyPosW[body_size-2]);
            }
        }
        // std::cout<< "---------------------" << std::endl;
        // for(int i=0; i<nrBodies;++i){
        //     std::cout<< "---------------------  " << i <<  std::endl;
        //     std::cout<< mbc.bodyPosW[i] << std::endl;
        // }
        return std::move(EEpose);
    }








}
#endif