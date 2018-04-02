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
#include "grl/flatbuffer/CSVIterator.hpp"


// FusionTrack Libraries
// #include <ftkInterface.h>

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
    std::vector<std::string> M_Pos_Joint_Labels = {"M_Pos_J1", "M_Pos_J2", "M_Pos_J3", "M_Pos_J4", "M_Pos_J5", "M_Pos_J6", "M_Pos_J7"};
    std::vector<std::string> M_Tor_Joint_Labels = {"M_Tor_J1", "M_Tor_J2", "M_Tor_J3", "M_Tor_J4", "M_Tor_J5", "M_Tor_J6", "M_Tor_J7"};
    std::vector<std::string> C_Pos_Joint_Labels = {"C_Pos_J1", "C_Pos_J2", "C_Pos_J3", "C_Pos_J4", "C_Pos_J5", "C_Pos_J6", "C_Pos_J7"};
    std::vector<std::string> C_Tor_Joint_Labels = {"C_Tor_J1", "C_Tor_J2", "C_Tor_J3", "C_Tor_J4", "C_Tor_J5", "C_Tor_J6", "C_Tor_J7"};
    std::vector<std::string> E_Tor_Joint_Labels = {"E_Tor_J1", "E_Tor_J2", "E_Tor_J3", "E_Tor_J4", "E_Tor_J5", "E_Tor_J6", "E_Tor_J7"};
    std::vector<std::string> IPO_Joint_Labels = {"IPO_J1", "IPO_J2", "IPO_J3", "IPO_J4", "IPO_J5", "IPO_J6", "IPO_J7"};
    std::vector<std::string> Joint_Labels = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6", "Joint_7"};
    std::vector<std::string> Transform_Labels  = {"X", "Y", "Z", "Q_W", "Q_X", "Q_Y", "Q_Z"};
    std::vector<std::string> PK_Pose_Labels  = {"X", "Y", "Z", "A", "B", "C"};
    enum TimeType { local_receive_time = 0, local_request_time = 1, device_time = 2, time_Y = 3, counterIdx=4};
    enum Joint_Index { joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7};
    enum LabelsType { FT_Pose = 0, Joint = 1, Kuka_Pose= 2};
    
    struct kuka_tag {};
    struct fusiontracker_tag {};
    int col_timeEvent=Time_Labels.size();
    int col_Kuka_Joint = Joint_Labels.size();
    const static int jointNum = 7;
    int col_Pose = PK_Pose_Labels.size();
 
    




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
                std::cout<< i << "   Receive time NOT VALID: " << time(i) << std::endl;
                return false;
            }

        }
        return true;
    }
    template<typename T>
    int getStateSize(const T &binaryObjectP)  {
            auto states = binaryObjectP->states();
            return states->size();

    }

    /// Return the original timeEvent of fusiontracker
    /// @param TlogKUKAiiwaFusionTrackP pointer of the root object for fusiontracker.
    /// @param fusiontracker_tag tag dispatch
    grl::MatrixXd  getTimeStamp(const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP, fusiontracker_tag, int markerID = 0){

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
                    // if(markerID == 0){
                    //     if(marker_ID == 22 || marker_ID == 55){
                    //         auto timeEvent = kukaiiwaFusionTrackMessage->timeEvent();
                    //         timeEventM(row,TimeType::local_receive_time) = timeEvent->local_receive_time();
                    //         timeEventM(row,TimeType::local_request_time) = timeEvent->local_request_time();
                    //         timeEventM(row,TimeType::device_time) = timeEvent->device_time();
                    //         timeEventM(row,TimeType::time_Y) = timeEventM(row,TimeType::device_time) - timeEventM(row,TimeType::local_receive_time);
                    //         timeEventM(row,TimeType::counterIdx) = counter;
                    //         row++;
                    //         break;
                    //     }
                    // } else {
                        if(marker_ID == markerID){
                            auto timeEvent = kukaiiwaFusionTrackMessage->timeEvent();
                            timeEventM(row,TimeType::local_receive_time) = timeEvent->local_receive_time();
                            timeEventM(row,TimeType::local_request_time) = timeEvent->local_request_time();
                            timeEventM(row,TimeType::device_time) = timeEvent->device_time();
                            timeEventM(row,TimeType::time_Y) = timeEventM(row,TimeType::device_time) - timeEventM(row,TimeType::local_receive_time);
                            timeEventM(row,TimeType::counterIdx) = counter;
                            row++;
                            break;
                        }
                    // }

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
    void regularizeTimeEvent(grl::MatrixXd& timeEventM,
        int64_t initial_local_time,
        int64_t initial_device_time,
        int rowIndex){
            std::cout<< "initial_local_time: " << initial_local_time << std::endl;
        std::size_t time_size = timeEventM.rows();
        assert(time_size>0);
        timeEventM.col(grl::TimeType::local_request_time) = timeEventM.col(grl::TimeType::local_request_time) - initial_local_time * grl::VectorXd::Ones(time_size);
        timeEventM.col(grl::TimeType::local_receive_time) = timeEventM.col(grl::TimeType::local_receive_time) - initial_local_time * grl::VectorXd::Ones(time_size);
        timeEventM.col(grl::TimeType::device_time) = timeEventM.col(grl::TimeType::device_time) - initial_device_time * grl::VectorXd::Ones(time_size);
        timeEventM.col(grl::TimeType::time_Y) = timeEventM.col(grl::TimeType::time_Y) - timeEventM(rowIndex, grl::TimeType::time_Y) * grl::VectorXd::Ones(time_size);
    }


    /// Get the maker pose based on the markerID. The bad data, which means the frame doesn't have the indicated marker information, has been filtered out.
    /// As for the bad data, both the marker pose and the corresponding timeEvent is skipped.
    /// @param logKUKAiiwaFusionTrackP, pointer of the root object for fusiontracker.
    /// @param markerID, the indicated marker.
    /// @param timeEventM, timeEvent without bad data, which is filled out.
    /// @param markerPose, the pose matrix, which is filled out.
    /// @return row, the number of valid rows.
    int  getMarkerPose(const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP,
                                    uint32_t &markerID,
                                    grl::MatrixXd& timeEventM,
                                    Eigen::MatrixXd& markerPose,
                                    bool markerFrame = false){
        auto states = logKUKAiiwaFusionTrackP->states();
        std::size_t state_size = states->size();
        assert(state_size>0);
        // The first columne is counter
        int row = 0;
        // Eigen::MatrixXd markerPose(state_size, grl::col_Pose);
        // std::cout <<"State size: " << state_size << "   markerPose rows: " << markerPose.rows() << std::endl;
        int BadCount = 0;
        for(int i = 0; i<state_size; ++i){
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
                    if(marker_ID == markerID){
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
                        Eigen::Vector3d eulerAngleEigen = RadtoDegree*E.eulerAngles(0,1,2); // X Y Z
                        Eigen::RowVectorXd pose(grl::col_Pose);
                        pose << r.transpose(), eulerAngleEigen.transpose();


                        // if(markerFrame == true) {
                        //     Eigen::VectorXd onePose(7);
                        //     onePose << r, q.w(), q.x(), q.y(), q.z();
                        //     auto markerToCameraTransform = grl::MarkerPoseToAffine3f(onePose);
                        //     auto cameraTomarkerTransform = markerToCameraTransform.inverse();
                        //     auto _markerPose = grl::Affine3fToMarkerPose(cameraTomarkerTransform);
                        //     pose = grl::getPluckerPose(_markerPose);

                        // }



                        markerPose.row(row++) = pose;

                        // Once read the specified marker information, skip out of the marker loop.
                        // It can keep from reading duplicate information.
                        // Sometimes in the same frame, there exists two markers with the same geometryID
                        break;
                    }
                }
            } else {
                BadCount++;
            }
        }
        if(row < state_size) {
            int64_t FT_diff = timeEventM(0,TimeType::device_time) - timeEventM(0,TimeType::local_receive_time);
            timeEventM.col(TimeType::time_Y) = timeEventM.col(TimeType::time_Y) - FT_diff * grl::VectorXd::Ones(timeEventM.rows());
            markerPose.conservativeResize(row, Eigen::NoChange_t{});
            timeEventM.conservativeResize(row, Eigen::NoChange_t{});
        }
        double diff = static_cast<double>(state_size-markerPose.rows());
        std::cout <<"State size: " << state_size <<"  markerPose size: " << markerPose.rows()
                  << "  lossing rate " << diff/state_size << "  markerID: " << markerID <<std::endl;
        return row;
    }
     /// Get the maker pose based on the markerID. The bad data, which means the frame doesn't have the indicated marker information, has been filtered out.
    /// As for the bad data, both the marker pose and the corresponding timeEvent is skipped.
    /// @param logKUKAiiwaFusionTrackP, pointer of the root object for fusiontracker.
    /// @param markerID, the indicated marker.
    /// @param timeEventM, timeEvent without bad data, which is filled out.
    /// @param markerPose, the pose matrix, which is filled out.
    /// @return row, the number of valid rows.
    int  getMarkerMessage(const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP,
        uint32_t &markerID,
        grl::MatrixXd& timeEventM_FT, 
        Eigen::MatrixXd&  transform, 
        int startIndex = 0){
            auto states = logKUKAiiwaFusionTrackP->states();
            std::size_t state_size = states->size();
            assert(state_size>0);
            // The first columne is counter
            int row = 0;
            // Eigen::MatrixXd markerPose(state_size, grl::col_Pose);
            // std::cout <<"State size: " << state_size << "   markerPose rows: " << markerPose.rows() << std::endl;
            int BadCount = 0;
            for(int stateIdx = startIndex; startIndex<state_size; ++startIndex){
                auto kukaiiwaFusionTrackMessage = states->Get(stateIdx);
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
                        if(marker_ID == markerID){
                            auto timeEvent = kukaiiwaFusionTrackMessage->timeEvent();
                            timeEventM_FT(row,TimeType::local_receive_time) = timeEvent->local_receive_time();
                            timeEventM_FT(row,TimeType::local_request_time) = timeEvent->local_request_time();
                            timeEventM_FT(row,TimeType::device_time) = timeEvent->device_time();
                            timeEventM_FT(row,TimeType::time_Y) = timeEventM_FT(row,TimeType::device_time) - timeEventM_FT(row,TimeType::local_receive_time);
                            timeEventM_FT(row,TimeType::counterIdx) = counter;
                            auto Pose = marker->transform();
                            auto FB_r = Pose->position();
                            auto FB_q = Pose->orientation();
                            // Convert the flatbuffer type to Eigen type
                            // Eigen::Vector3d r(FB_r.x(), FB_r.y(), FB_r.z());
                            // Eigen::Quaterniond q(FB_q.w(), FB_q.x(), FB_q.y(), FB_q.z());
                            transform.row(row++) << FB_r.x(), FB_r.y(), FB_r.z(), FB_q.w(), FB_q.x(), FB_q.y(), FB_q.z();
                            break;
                        }
                    }
                } else {
                    BadCount++;
                }
            }
            if(row < state_size) {
                int64_t FT_diff = timeEventM_FT(0,TimeType::device_time) - timeEventM_FT(0,TimeType::local_receive_time);
                timeEventM_FT.col(TimeType::time_Y) = timeEventM_FT.col(TimeType::time_Y) - FT_diff * grl::VectorXd::Ones(timeEventM_FT.rows());
                transform.conservativeResize(row, Eigen::NoChange_t{});
                timeEventM_FT.conservativeResize(row, Eigen::NoChange_t{});
            }
            double diff = static_cast<double>(state_size-transform.rows());
            std::cout <<"State size: " << state_size <<"  markerPose size: " << transform.rows()
            << "  lossing rate " << diff/state_size << "  markerID: " << markerID <<std::endl;
            return row;
    }

     /// Get the joint angles of a specific joint (joint_index)
    /// @param kukaStatesP pointer of the root object for kuka.
    /// @param joint_index, return the joint angles of the indicated joint.
    /// @return jointPosition, Eigen vector which contains joint position.
    int getFRIMessage(const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP,
        grl::MatrixXd& timeEventM_Kuka, 
        Eigen::VectorXd&  sequenceCounterVec,
        Eigen::VectorXd&  reflectedSequenceCounterVec,
        Eigen::MatrixXd&  measuredJointPosition, 
        Eigen::MatrixXd&  measuredTorque, 
        Eigen::MatrixXd&  commandedJointPosition, 
        Eigen::MatrixXd&  commandedTorque, 
        Eigen::MatrixXd&  externalTorque,
        Eigen::MatrixXd&  jointStateInterpolated,
        int startIndex = 0){
        
            auto states = kukaStatesP->states();
            std::size_t state_size = states->size();
            Eigen::VectorXf measurdjointPosVec(state_size);
            if(startIndex>state_size){
                return -1;
            }
            for(int stateIdx = startIndex; stateIdx<state_size; ++stateIdx){
                auto KUKAiiwaState = states->Get(stateIdx);
                auto FRIMessage = KUKAiiwaState->FRIMessage();
                auto timeEvent = FRIMessage->timeStamp();
                timeEventM_Kuka(stateIdx,TimeType::local_receive_time) = timeEvent->local_receive_time();
                timeEventM_Kuka(stateIdx,TimeType::local_request_time) = timeEvent->local_request_time();
                timeEventM_Kuka(stateIdx,TimeType::device_time) = timeEvent->device_time();
                timeEventM_Kuka(stateIdx,TimeType::time_Y) =  timeEventM_Kuka(stateIdx,TimeType::device_time) - timeEventM_Kuka(stateIdx,TimeType::local_receive_time);
                // timeEventM_Kuka(stateIdx,TimeType::counterIdx) = identifier;
                sequenceCounterVec(stateIdx) = FRIMessage->sequenceCounter();
                reflectedSequenceCounterVec(stateIdx) = FRIMessage->reflectedSequenceCounter();
                for(int jointIdx=0; jointIdx<7; jointIdx++){
                    measuredJointPosition(stateIdx, jointIdx) = FRIMessage->measuredJointPosition()->size()>0 ? FRIMessage->measuredJointPosition()->Get(jointIdx):0; // flatbuffers::Vector<double> *
                    measuredTorque(stateIdx, jointIdx) = FRIMessage->measuredTorque()->size()>0 ? FRIMessage->measuredTorque()->Get(jointIdx):0; 
                    commandedJointPosition(stateIdx, jointIdx) = FRIMessage->commandedJointPosition()->size()>0 ? FRIMessage->commandedJointPosition()->Get(jointIdx):0;
                    commandedTorque(stateIdx, jointIdx) = FRIMessage->commandedTorque()->size()>0 ? FRIMessage->commandedTorque()->Get(jointIdx):0;
                    externalTorque(stateIdx, jointIdx) = FRIMessage->externalTorque()->size()>0 ? FRIMessage->externalTorque()->Get(jointIdx):0;
                    jointStateInterpolated(stateIdx, jointIdx) = FRIMessage->jointStateInterpolated()->size()>0 ? FRIMessage->jointStateInterpolated()->Get(jointIdx):0;
                }

            }
            return 1;
        }


    /// With the same method to process the time data from both Kuka and fusiontracker, combine it and write to a csv file.
    /// @param FTKUKA_TimeEvent_CSVfilename, the csv file name
    void writeFRIMessageToCSV(std::string& KUKA_FRI_CSVfilename,
        grl::MatrixXd& timeEventM_Kuka, 
        Eigen::VectorXd&  sequenceCounterVec,
        Eigen::VectorXd&  reflectedSequenceCounterVec,
        Eigen::MatrixXd&  measuredJointPosition, 
        Eigen::MatrixXd&  measuredTorque, 
        Eigen::MatrixXd&  commandedJointPosition, 
        Eigen::MatrixXd&  commandedTorque, 
        Eigen::MatrixXd&  externalTorque,
        Eigen::MatrixXd&  jointStateInterpolated,
        int startIndex = 0) {

            int kuka_time_size = timeEventM_Kuka.rows();

            assert(kuka_time_size == sequenceCounterVec.size());
            assert(kuka_time_size == reflectedSequenceCounterVec.size());
            assert(kuka_time_size == measuredJointPosition.rows());
            assert(kuka_time_size == measuredTorque.rows());
            assert(kuka_time_size == commandedJointPosition.rows());
            assert(kuka_time_size == commandedTorque.rows());
            assert(kuka_time_size == externalTorque.rows());
            assert(kuka_time_size == jointStateInterpolated.rows());
            auto receive_time = timeEventM_Kuka.col(grl::TimeType::local_receive_time);
            assert(checkmonotonic(receive_time));
        
            std::ofstream fs;
            // create a name for the file output
            fs.open(KUKA_FRI_CSVfilename, std::ofstream::out | std::ofstream::app);
            // write the file headers
            fs << grl::Time_Labels[grl::TimeType::local_receive_time] << ","
            << grl::Time_Labels[grl::TimeType::local_request_time] << ","
            << grl::Time_Labels[grl::TimeType::device_time] << ","
            << grl::Time_Labels[grl::TimeType::time_Y] << ","
            << "SequenceCounter,"
            << "reflectedSequenceCounter,"
            << grl::M_Pos_Joint_Labels[grl::Joint_Index::joint_1] << ","
            << grl::M_Pos_Joint_Labels[grl::Joint_Index::joint_2] << ","
            << grl::M_Pos_Joint_Labels[grl::Joint_Index::joint_3] << ","
            << grl::M_Pos_Joint_Labels[grl::Joint_Index::joint_4] << ","
            << grl::M_Pos_Joint_Labels[grl::Joint_Index::joint_5] << ","
            << grl::M_Pos_Joint_Labels[grl::Joint_Index::joint_6] << ","
            << grl::M_Pos_Joint_Labels[grl::Joint_Index::joint_7] << ","

            << grl::M_Tor_Joint_Labels[grl::Joint_Index::joint_1] << ","
            << grl::M_Tor_Joint_Labels[grl::Joint_Index::joint_2] << ","
            << grl::M_Tor_Joint_Labels[grl::Joint_Index::joint_3] << ","
            << grl::M_Tor_Joint_Labels[grl::Joint_Index::joint_4] << ","
            << grl::M_Tor_Joint_Labels[grl::Joint_Index::joint_5] << ","
            << grl::M_Tor_Joint_Labels[grl::Joint_Index::joint_6] << ","
            << grl::M_Tor_Joint_Labels[grl::Joint_Index::joint_7] << ","

            << grl::C_Pos_Joint_Labels[grl::Joint_Index::joint_1] << ","
            << grl::C_Pos_Joint_Labels[grl::Joint_Index::joint_2] << ","
            << grl::C_Pos_Joint_Labels[grl::Joint_Index::joint_3] << ","
            << grl::C_Pos_Joint_Labels[grl::Joint_Index::joint_4] << ","
            << grl::C_Pos_Joint_Labels[grl::Joint_Index::joint_5] << ","
            << grl::C_Pos_Joint_Labels[grl::Joint_Index::joint_6] << ","
            << grl::C_Pos_Joint_Labels[grl::Joint_Index::joint_7] << ","

            << grl::C_Tor_Joint_Labels[grl::Joint_Index::joint_1] << ","
            << grl::C_Tor_Joint_Labels[grl::Joint_Index::joint_2] << ","
            << grl::C_Tor_Joint_Labels[grl::Joint_Index::joint_3] << ","
            << grl::C_Tor_Joint_Labels[grl::Joint_Index::joint_4] << ","
            << grl::C_Tor_Joint_Labels[grl::Joint_Index::joint_5] << ","
            << grl::C_Tor_Joint_Labels[grl::Joint_Index::joint_6] << ","
            << grl::C_Tor_Joint_Labels[grl::Joint_Index::joint_7] << ","

            << grl::E_Tor_Joint_Labels[grl::Joint_Index::joint_1] << ","
            << grl::E_Tor_Joint_Labels[grl::Joint_Index::joint_2] << ","
            << grl::E_Tor_Joint_Labels[grl::Joint_Index::joint_3] << ","
            << grl::E_Tor_Joint_Labels[grl::Joint_Index::joint_4] << ","
            << grl::E_Tor_Joint_Labels[grl::Joint_Index::joint_5] << ","
            << grl::E_Tor_Joint_Labels[grl::Joint_Index::joint_6] << ","
            << grl::E_Tor_Joint_Labels[grl::Joint_Index::joint_7] << ","

            << grl::IPO_Joint_Labels[grl::Joint_Index::joint_1] << ","
            << grl::IPO_Joint_Labels[grl::Joint_Index::joint_2] << ","
            << grl::IPO_Joint_Labels[grl::Joint_Index::joint_3] << ","
            << grl::IPO_Joint_Labels[grl::Joint_Index::joint_4] << ","
            << grl::IPO_Joint_Labels[grl::Joint_Index::joint_5] << ","
            << grl::IPO_Joint_Labels[grl::Joint_Index::joint_6] << ","
            << grl::IPO_Joint_Labels[grl::Joint_Index::joint_7] << std::endl;
           
            int kuka_index = startIndex;
            // int64_t diff = timeEventM_Kuka(startIndex, grl::TimeType::time_Y);
            while ( kuka_index < kuka_time_size)
            {
                
                fs << timeEventM_Kuka(kuka_index, grl::TimeType::local_receive_time) <<","
                   << timeEventM_Kuka(kuka_index, grl::TimeType::local_request_time) <<","
                   << timeEventM_Kuka(kuka_index, grl::TimeType::device_time) <<","
                   << timeEventM_Kuka(kuka_index, grl::TimeType::time_Y)<<","
                   << sequenceCounterVec(kuka_index) <<","
                   << reflectedSequenceCounterVec(kuka_index) <<","
                   << measuredJointPosition(kuka_index, grl::Joint_Index::joint_1) <<","
                   << measuredJointPosition(kuka_index, grl::Joint_Index::joint_2) <<","
                   << measuredJointPosition(kuka_index, grl::Joint_Index::joint_3) <<","
                   << measuredJointPosition(kuka_index, grl::Joint_Index::joint_4) <<","
                   << measuredJointPosition(kuka_index, grl::Joint_Index::joint_5) <<","
                   << measuredJointPosition(kuka_index, grl::Joint_Index::joint_6) <<","
                   << measuredJointPosition(kuka_index, grl::Joint_Index::joint_7) <<","
                   
                   << measuredTorque(kuka_index, grl::Joint_Index::joint_1) <<","
                   << measuredTorque(kuka_index, grl::Joint_Index::joint_2) <<","
                   << measuredTorque(kuka_index, grl::Joint_Index::joint_3) <<","
                   << measuredTorque(kuka_index, grl::Joint_Index::joint_4) <<","
                   << measuredTorque(kuka_index, grl::Joint_Index::joint_5) <<","
                   << measuredTorque(kuka_index, grl::Joint_Index::joint_6) <<","
                   << measuredTorque(kuka_index, grl::Joint_Index::joint_7) <<","

                   << commandedJointPosition(kuka_index, grl::Joint_Index::joint_1) <<","
                   << commandedJointPosition(kuka_index, grl::Joint_Index::joint_2) <<","
                   << commandedJointPosition(kuka_index, grl::Joint_Index::joint_3) <<","
                   << commandedJointPosition(kuka_index, grl::Joint_Index::joint_4) <<","
                   << commandedJointPosition(kuka_index, grl::Joint_Index::joint_5) <<","
                   << commandedJointPosition(kuka_index, grl::Joint_Index::joint_6) <<","
                   << commandedJointPosition(kuka_index, grl::Joint_Index::joint_7) <<","

                   << commandedTorque(kuka_index, grl::Joint_Index::joint_1) <<","
                   << commandedTorque(kuka_index, grl::Joint_Index::joint_2) <<","
                   << commandedTorque(kuka_index, grl::Joint_Index::joint_3) <<","
                   << commandedTorque(kuka_index, grl::Joint_Index::joint_4) <<","
                   << commandedTorque(kuka_index, grl::Joint_Index::joint_5) <<","
                   << commandedTorque(kuka_index, grl::Joint_Index::joint_6) <<","
                   << commandedTorque(kuka_index, grl::Joint_Index::joint_7) <<","

                   << externalTorque(kuka_index, grl::Joint_Index::joint_1) <<","
                   << externalTorque(kuka_index, grl::Joint_Index::joint_2) <<","
                   << externalTorque(kuka_index, grl::Joint_Index::joint_3) <<","
                   << externalTorque(kuka_index, grl::Joint_Index::joint_4) <<","
                   << externalTorque(kuka_index, grl::Joint_Index::joint_5) <<","
                   << externalTorque(kuka_index, grl::Joint_Index::joint_6) <<","
                   << externalTorque(kuka_index, grl::Joint_Index::joint_7) <<","

                   << jointStateInterpolated(kuka_index, grl::Joint_Index::joint_1) <<","
                   << jointStateInterpolated(kuka_index, grl::Joint_Index::joint_2) <<","
                   << jointStateInterpolated(kuka_index, grl::Joint_Index::joint_3) <<","
                   << jointStateInterpolated(kuka_index, grl::Joint_Index::joint_4) <<","
                   << jointStateInterpolated(kuka_index, grl::Joint_Index::joint_5) <<","
                   << jointStateInterpolated(kuka_index, grl::Joint_Index::joint_6) <<","
                   << jointStateInterpolated(kuka_index, grl::Joint_Index::joint_7) << std::endl;
                kuka_index++;
            } 
            
            std::cout << "kuka_time_size: " << kuka_time_size << "   kuka_index: " << kuka_index << std::endl;
            fs.close();
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
    /// @param rowIndex, In order to make the data from two devices match each other, we just take down the data from the specific row.
    template <class T1, class T2>
    void writeMatrixToCSV(const std::string& CSV_FileName, std::vector<std::string> &labels, T1& timeM, T2& dataM, int rowIndex=0){
        std::size_t labels_size = labels.size();
        std::size_t cols_size = timeM.cols() + dataM.cols();
        std::size_t time_rows_size = timeM.rows();
        std::size_t data_rows_size = dataM.rows();
        assert(labels_size == cols_size && time_rows_size>0 && cols_size>0 && time_rows_size==data_rows_size);
        auto time = timeM.col(TimeType::local_receive_time);
        // assert(checkmonotonic(time));
        // create an ofstream for the file output (see the link on streams for more info)
        std::ofstream fs;
        // create a name for the file output
        fs.open(CSV_FileName, std::ofstream::out | std::ofstream::app);
        // write the file labels;
        for(int col=0; col<cols_size-1; col++){
            fs << labels[col] << ",";
        }
        fs << labels[cols_size-1] << std::endl;
        for(int row_index=rowIndex; row_index<time_rows_size; ++row_index) {
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
    void writeTimeEventToCSV( std::string & CSV_FileName, grl::MatrixXd& timeEventM, int rowIdx) {
        std::size_t time_size = timeEventM.rows();
        grl::VectorXd local_receive_timeV = timeEventM.col(grl::TimeType::local_receive_time);
        grl::VectorXd local_request_timeV = timeEventM.col(grl::TimeType::local_request_time);
        grl::VectorXd device_timeV = timeEventM.col(grl::TimeType::device_time);
        grl::VectorXd time_YV = timeEventM.col(grl::TimeType::time_Y);
        grl::VectorXd counter = timeEventM.col(grl::TimeType::counterIdx);
        grl::VectorXd receive_request = local_receive_timeV - local_request_timeV;
        //  grl::VectorXd device_time_offset = local_receive_time - local_request_time;
        std::ofstream fs;
        // create a name for the file output
        fs.open( CSV_FileName, std::ofstream::out | std::ofstream::app);
         // write the file headers
        fs << grl::Time_Labels[grl::TimeType::local_receive_time] << ","
           << grl::Time_Labels[grl::TimeType::local_request_time] << ","
           << grl::Time_Labels[grl::TimeType::device_time] << ","
           << grl::Time_Labels[grl::TimeType::time_Y] << ","
           << "Receive-Request,"
           << "device_time_step,"
           << "receive_time_step,"
           << "counter"
           << std::endl;
        int64_t device_time_step = 0;
        int64_t receive_time_step = 0;
        for(int i=rowIdx; i<time_size; ++i) {
            if(i>0) {
                device_time_step = device_timeV(i) - device_timeV(i-1);
                receive_time_step = local_receive_timeV(i) - local_receive_timeV(i-1);
            }
            // write the data to the output file
            fs << local_receive_timeV(i) << ","         // A
               << local_request_timeV(i) <<","          // B
               << device_timeV(i) <<","                         // C
               << time_YV(i) <<","   
               << receive_request(i) << ","
               << device_time_step << ","
               << receive_time_step << ","
               << counter(i) << std::endl;  //D
        }
        fs.close();
    }


    /// With the same method to process the time data from both Kuka and fusiontracker, combine it and write to a csv file.
    /// @param FTKUKA_TimeEvent_CSVfilename, the csv file name
    void writeFTKUKAMessageToCSV(std::string& FTKUKA_TimeEvent_CSVfilename,
        grl::MatrixXd& timeEventM_FT, 
        grl::MatrixXd& timeEventM_Kuka, 
        Eigen::MatrixXd& jointAngles, 
        Eigen::MatrixXd& markerPose,
        int FT_index=0,
        int kuka_index=0) {
            int FT_time_size = timeEventM_FT.rows();
            int kuka_time_size = timeEventM_Kuka.rows();
            grl::VectorXd FT_local_request_time = timeEventM_FT.col(local_request_time);
            grl::VectorXd FT_local_receive_time = timeEventM_FT.col(local_receive_time);
            grl::VectorXd FT_device_time = timeEventM_FT.col(device_time);
            // assert(checkmonotonic(FT_local_receive_time)); // Hit the asserion, should check the time data from FT_index, not from 0
    
            grl::VectorXd kuka_local_request_time = timeEventM_Kuka.col(local_request_time);
            grl::VectorXd kuka_local_receive_time = timeEventM_Kuka.col(local_receive_time);
            grl::VectorXd kuka_device_time = timeEventM_Kuka.col(device_time);   
            // assert(checkmonotonic(kuka_local_receive_time));
            int64_t kuka_diff = kuka_device_time(kuka_index) - kuka_local_receive_time(kuka_index);
            int64_t FT_diff = FT_device_time(FT_index) - FT_local_receive_time(FT_index);
          
        
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
            << "Y_kuka,"
            << "FT_X,"
            << "FT_Y,"
            << "FT_Z,"
            << "FT_A,"
            << "FT_B,"
            << "FT_C,"
            << "K_Joint1,"
            << "K_Joint2,"
            << "K_Joint3,"
            << "K_Joint4,"
            << "K_Joint5,"
            << "K_Joint6,"
            << "K_Joint7"
            << std::endl;
            std::cout << "Start to write ... "<< std::endl <<"FT_index: " << FT_index << "   kuka_index: " << kuka_index << std::endl;
        
            while ( kuka_index < kuka_time_size && FT_index < FT_time_size )
            {
                // If the row value is the kuka time, then the FT cells should be left blank.
                if ( kuka_local_receive_time(kuka_index) < FT_local_receive_time(FT_index) ){
                // write the data to the output file
                auto jointrow = jointAngles.row(kuka_index);
                fs << kuka_local_receive_time(kuka_index) <<","
                <<","
                << kuka_local_request_time(kuka_index) << ","
                << ","
                << kuka_device_time(kuka_index) <<","
                << ","
                << kuka_device_time(kuka_index) - kuka_local_receive_time(kuka_index) - kuka_diff << ","
                << ",,,,,,"
                << jointrow[0] << ","
                << jointrow[1] << ","
                << jointrow[2] << ","
                << jointrow[3] << ","
                << jointrow[4] << ","
                << jointrow[5] << ","
                << jointrow[6]
                << std::endl;
                kuka_index++;
            } else if( kuka_local_receive_time(kuka_index) > FT_local_receive_time(FT_index)) {
                auto matrixrow = markerPose.row(FT_index);
                // If the row value is the FT time, then the kuka cells should be left blank.
                fs << FT_local_receive_time(FT_index) <<","
                << FT_local_request_time(FT_index) << ","
                <<","
                << FT_device_time(FT_index) << ","
                << ","
                << FT_device_time(FT_index) - FT_local_receive_time(FT_index) - FT_diff << ","
                << ","
                << matrixrow[0] << ","
                << matrixrow[1] << ","
                << matrixrow[2] << ","
                << matrixrow[3] << ","
                << matrixrow[4] << ","
                << matrixrow[5] << std::endl;
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
            std::cout << "FT_index: " << FT_index << "   kuka_index: " << kuka_index << "  Sum:  " << FT_index+kuka_index << std::endl;
            fs.close();
}
    /// With the same method to process the time data from both Kuka and fusiontracker, combine it and write to a csv file.
    /// @param FTKUKA_TimeEvent_CSVfilename, the csv file name
    void writeFTKUKAMessageToCSV(std::string& FTKUKA_TimeEvent_CSVfilename,
                                    const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP,
                                    const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP,
                                    uint32_t &markerID) {

        grl::MatrixXd timeEventM_FT = grl::getTimeStamp(logKUKAiiwaFusionTrackP, grl::fusiontracker_tag(), markerID);
        grl::MatrixXd timeEventM_Kuka = grl::getTimeStamp(kukaStatesP, grl::kuka_tag());

        std::size_t FT_size = grl::getStateSize(logKUKAiiwaFusionTrackP);
        std::size_t kuka_size = grl::getStateSize(kukaStatesP);
 
        std::cout<< "------Kuka_size: "<< kuka_size << std::endl;
        std::cout<< "------FT_size: "<< FT_size << std::endl;

        std::size_t FT_time_size = timeEventM_FT.rows();
        std::size_t kuka_time_size = timeEventM_Kuka.rows();
        grl::MatrixXd timeEventM = grl::MatrixXd::Zero(FT_size, timeEventM_FT.cols());
        Eigen::MatrixXd markerPose(FT_size, grl::col_Pose);
        // Get the pose of the marker 22.
        // Since the bad data is skipped, the timeEventM_FT also needs to be filled out with the orinial time stamp.
    
        int validsize_22 = grl::getMarkerPose(logKUKAiiwaFusionTrackP, markerID, timeEventM, markerPose);
        assert(timeEventM.rows() == timeEventM_FT.rows());
        //////////////////////////////////////////////////////////////////////////////////



        int kuka_index = 0;
        int FT_index = 0;

        // Skip the very beginning data,since the tracker starts to work once the scene is loaded in vrep.
        // But kuka starts to work only after clicking on the start button.
        // To combine the time from two devices, they should have the same starting time point.
        while(kuka_index<kuka_time_size && timeEventM_Kuka(kuka_index,TimeType::local_receive_time) < timeEventM_FT(FT_index,TimeType::local_receive_time)){
            kuka_index++;
        }
        while(FT_index<FT_time_size && timeEventM_Kuka(kuka_index,local_receive_time) > timeEventM_FT(FT_index,local_receive_time) && kuka_index == 0){
            FT_index++;
        }

        auto initial_local_time = std::min(timeEventM_FT(FT_index,local_receive_time), timeEventM_Kuka(kuka_index,local_receive_time));
        auto initial_device_time_kuka = timeEventM_Kuka(kuka_index,device_time);
        auto initial_device_time_FT = timeEventM_FT(FT_index,device_time);
        grl::VectorXd FT_local_request_time = timeEventM_FT.col(local_request_time) - initial_local_time * grl::VectorXd::Ones(FT_time_size);
        grl::VectorXd FT_local_receive_time = timeEventM_FT.col(local_receive_time) - initial_local_time * grl::VectorXd::Ones(FT_time_size);
        grl::VectorXd FT_device_time = timeEventM_FT.col(device_time) - initial_device_time_FT * grl::VectorXd::Ones(FT_time_size);

        grl::VectorXd kuka_local_request_time = timeEventM_Kuka.col(local_request_time) - initial_local_time * grl::VectorXd::Ones(kuka_time_size);
        grl::VectorXd kuka_local_receive_time = timeEventM_Kuka.col(local_receive_time) - initial_local_time * grl::VectorXd::Ones(kuka_time_size);
        grl::VectorXd kuka_device_time = timeEventM_Kuka.col(device_time) - initial_device_time_kuka * grl::VectorXd::Ones(kuka_time_size);
        
        Eigen::MatrixXd jointAngles = grl::getAllJointAngles(kukaStatesP);

        
        int64_t kuka_diff = kuka_device_time(kuka_index) - kuka_local_receive_time(kuka_index);
        int64_t FT_diff = FT_device_time(FT_index) - FT_local_receive_time(FT_index);
       
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
           << "Y_kuka,"
           << "FT_X,"
           << "FT_Y,"
           << "FT_Z,"
           << "FT_A,"
           << "FT_B,"
           << "FT_C,"
           << "K_Joint1,"
           << "K_Joint2,"
           << "K_Joint3,"
           << "K_Joint4,"
           << "K_Joint5,"
           << "K_Joint6,"
           << "K_Joint7"
           << std::endl;
        std::cout << "Start to write ... "<< std::endl <<"FT_index: " << FT_index << "   kuka_index: " << kuka_index << std::endl;
       
        while ( kuka_index < kuka_time_size && FT_index < FT_time_size )
        {
            // If the row value is the kuka time, then the FT cells should be left blank.
            if ( kuka_local_receive_time(kuka_index) < FT_local_receive_time(FT_index) ){
                // write the data to the output file
                auto jointrow = jointAngles.row(kuka_index);
                fs << kuka_local_receive_time(kuka_index) <<","
                   <<","
                   << kuka_local_request_time(kuka_index) << ","
                   << ","
                   << kuka_device_time(kuka_index) <<","
                   << ","
                   << kuka_device_time(kuka_index) - kuka_local_receive_time(kuka_index) - kuka_diff << ","
                   << ",,,,,,"
                   << jointrow[0] << ","
                   << jointrow[1] << ","
                   << jointrow[2] << ","
                   << jointrow[3] << ","
                   << jointrow[4] << ","
                   << jointrow[5] << ","
                   << jointrow[6]
                   << std::endl;
                   kuka_index++;
            } else if( kuka_local_receive_time(kuka_index) > FT_local_receive_time(FT_index)) {
                auto matrixrow = markerPose.row(FT_index);
                // If the row value is the FT time, then the kuka cells should be left blank.
                fs << FT_local_receive_time(FT_index) <<","
                   << FT_local_request_time(FT_index) << ","
                   <<","
                   << FT_device_time(FT_index) << ","
                   << ","
                   << FT_device_time(FT_index) - FT_local_receive_time(FT_index) - FT_diff << ","
                   << ","
                   << matrixrow[0] << ","
                   << matrixrow[1] << ","
                   << matrixrow[2] << ","
                   << matrixrow[3] << ","
                   << matrixrow[4] << ","
                   << matrixrow[5] << std::endl;
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
        std::cout << "FT_index: " << FT_index << "   kuka_index: " << kuka_index << "  Sum:  " << FT_index+kuka_index << std::endl;
        fs.close();
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

    mc_rbdyn_urdf::URDFParserResult getURDFModel(){
        using namespace Eigen;
	    using namespace sva;
	    using namespace rbd;

        namespace cst = boost::math::constants;
        auto strRobot = mc_rbdyn_urdf::rbdyn_from_urdf(XYZSarmUrdf);
        return std::move(strRobot);
    }


int getRowsNumber(std::string filename){
     // std::ifstream     file("/home/cjiao1/src/V-REP_PRO_EDU_V3_4_0_Linux/2018_03_20_20_35_20/KUKA_Joint.csv");
     std::ifstream     file(filename);
     std::size_t row_size = 0;
    
     if (!file.is_open()) {
         std::cout << "failed to open " << filename << '\n';
     } else {
         grl::CSVIterator loop(file);
 
         for(grl::CSVIterator loop(file); loop != grl::CSVIterator(); ++loop)
         {  
            row_size++;
         }
     }
     return row_size;

}

    /// Get the joint angles at specific time point (index)
    /// @return jointPosition, Eigen vector which contains joint position of the seven joints.
    int64_t getJointAnglesFromCSV(std::string filename, Eigen::VectorXf &jointPosition, int rowIdx){

        // std::ifstream     file("/home/cjiao1/src/V-REP_PRO_EDU_V3_4_0_Linux/2018_03_20_20_35_20/KUKA_Joint.csv");
        // int rowNum = getRowsNumber(filename);
        std::ifstream     file(filename);
        std::size_t joint_size = 7;
        // Eigen::VectorXf jointPosition(joint_size);
        if (!file.is_open()) {
            std::cout << "failed to open " << filename << '\n';
        } else {
            grl::CSVIterator loop(file);
          
            int row = 0;
    
            for(grl::CSVIterator loop(file); loop != grl::CSVIterator(); ++loop)
            {
                    if(row == rowIdx){
                        
                        for(int joint_index = 0; joint_index<joint_size; ++joint_index){
                          
                            jointPosition(joint_index) = boost::lexical_cast<double>((*loop)[joint_index+5]);
                        }
                        return boost::lexical_cast<uint64_t>((*loop)[0]);
                    }
                    row++;

            }
        }
        return -1;
        
    }

    /// Get the joint angles at specific time point (index)

    /// @return jointPosition, Eigen vector which contains joint position of the seven joints.
    Eigen::VectorXf getMarkerPoseFromCSV(std::string filename, int rowIdx){

        std::ifstream file(filename);
        std::size_t size = 6;
        Eigen::VectorXf markerpose(size);
        if (!file.is_open()) {
            std::cout << "failed to open " << filename << '\n';
        } else {
            grl::CSVIterator loop(file);
           
            int row = 0;
    
            for(grl::CSVIterator loop(file); loop != grl::CSVIterator(); ++loop)
            {
    
                if(row == rowIdx){
                    for(int index = 0; index<size; ++index){
    
                        markerpose(index) = std::stof((*loop)[index+5]);
                    }
                    break;
                }
               
                assert(row <= rowIdx);
                row++;
            }
        }
        return markerpose;
    }





}
#endif