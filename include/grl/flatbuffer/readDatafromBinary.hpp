#ifndef GRL_READ_DATA_FROM_BINARY
#define GRL_READ_DATA_FROM_BINARY

#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE

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
#include <string>
#include <vector>
#include <set>
#include <iterator>
#include <algorithm>


namespace grl {
    typedef Eigen::Matrix<int64_t , Eigen::Dynamic , 1>  VectorXd;
    typedef Eigen::Matrix<int64_t , Eigen::Dynamic , Eigen::Dynamic>  MatrixXd;

    const char *Time_Labels[] = {"local_receive_time_X", "local_request_time_offset", "device_time_offset", "time_Y"};
    const char *FT_Pose_Labels[] = {"Counter", "P_X", "P_Y", "P_Z", "Q_X", "Q_Y", "Q_Z", "Q_W"};
    const char *Joint_Labels[] = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6", "Joint_7"};
    struct kuka_tag {};
    struct fusiontracker_tag {};
    enum TimeType { device_time = 0, local_request_time = 1, local_receive_time = 2 };
    /// Help initialize a vector array of strings.
    /// See https://stackoverflow.com/questions/4268886/initialize-a-vector-array-of-strings
    template<typename T, size_t N>
    T * end(T (&ra)[N]) {
        return ra + N;
    }


    std::vector<std::string> getLabels(fusiontracker_tag){
        std::vector<std::string> Time_Labels_Vec(Time_Labels, grl::end(Time_Labels));
        std::vector<std::string> FT_Pose_Labels_Vec(FT_Pose_Labels, grl::end(FT_Pose_Labels));
        std::vector<std::string> FT_Labels_Pose;
        FT_Labels_Pose.reserve(Time_Labels_Vec.size()+FT_Pose_Labels_Vec.size());
        FT_Labels_Pose.insert(FT_Labels_Pose.end(), Time_Labels_Vec.begin(), Time_Labels_Vec.end());
        FT_Labels_Pose.insert(FT_Labels_Pose.end(), FT_Pose_Labels_Vec.begin(), FT_Pose_Labels_Vec.end());
        return FT_Labels_Pose;
    }

    std::vector<std::string> getLabels(kuka_tag){
        std::vector<std::string> Time_Labels_Vec(Time_Labels, grl::end(Time_Labels));
        std::vector<std::string> Joint_Labels_Vec(Joint_Labels, grl::end(Joint_Labels));
        std::vector<std::string> Kuka_Joint_Labels;
        Kuka_Joint_Labels.reserve(Time_Labels_Vec.size()+Joint_Labels_Vec.size());
        Kuka_Joint_Labels.insert(Kuka_Joint_Labels.end(), Time_Labels_Vec.begin(), Time_Labels_Vec.end());
        Kuka_Joint_Labels.insert(Kuka_Joint_Labels.end(), Joint_Labels_Vec.begin(), Joint_Labels_Vec.end());
        return Kuka_Joint_Labels;
    }

    /// Remove a certain row or colum for the given matrix in Eigen
    /// See https://stackoverflow.com/questions/13290395/how-to-remove-a-certain-row-or-column-while-using-eigen-library-c
    template<typename T>
    void removeRow(T& matrix, unsigned int rowToRemove)
    {
        unsigned int numRows = matrix.rows()-1;
        unsigned int numCols = matrix.cols();

        if( rowToRemove < numRows )
            matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.bottomRows(numRows-rowToRemove);

        matrix.conservativeResize(numRows,numCols);
    }
    template<typename T>
    void removeColumn(T& matrix, unsigned int colToRemove)
    {
        unsigned int numRows = matrix.rows();
        unsigned int numCols = matrix.cols()-1;

        if( colToRemove < numCols )
            matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.rightCols(numCols-colToRemove);

        matrix.conservativeResize(numRows,numCols);
    }

    /// Get the original time from KUKAiiwaStates
    grl::VectorXd getTimeStamp(const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP, kuka_tag, TimeType time_type){
        auto states = kukaStatesP->states();
        std::size_t state_size = states->size();
        grl::VectorXd timeStamp(state_size);
        for(int i = 0; i<state_size; ++i){
            auto KUKAiiwaState = states->Get(i);
            auto FRIMessage = KUKAiiwaState->FRIMessage();
            auto timeEvent = FRIMessage->timeStamp();
            switch(time_type) {
                case device_time:{
                    timeStamp(i) = timeEvent->device_time();
                    break;
                }
                case local_request_time:{
                    timeStamp(i) = timeEvent->local_request_time();
                    break;
                }
                case local_receive_time:{
                    timeStamp(i) = timeEvent->local_receive_time();
                    break;
                }
                default:{
                     std::perror("No time type!");
                }
            }
        }
        return timeStamp;
    }
    /// Get the original time from LogKUKAiiwaFusionTrack
    grl::VectorXd  getTimeStamp(const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP, fusiontracker_tag, TimeType time_type){

        auto states = logKUKAiiwaFusionTrackP->states();
        std::size_t state_size = states->size();
        grl::VectorXd timeStamp(state_size);
        //Eigen::VectoXd time_stamp(state_size);
        for(int i = 0; i<state_size; ++i){
            auto kukaiiwaFusionTrackMessage = states->Get(i);
            auto timeEvent = kukaiiwaFusionTrackMessage->timeEvent();
            switch(time_type) {
                case device_time:{
                    timeStamp(i) = timeEvent->device_time();
                    break;
                }
                case local_request_time:{
                    timeStamp(i) = timeEvent->local_request_time();
                    break;
                }
                case local_receive_time:{
                    timeStamp(i) = timeEvent->local_receive_time();
                    break;
                }
                default:{
                     std::perror("No time type!");
                }
            }
        }
        return timeStamp;
    }
    /// Return the original timeEvent
    grl::MatrixXd  getTimeStamp(const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP, fusiontracker_tag){

        auto states = logKUKAiiwaFusionTrackP->states();
        std::size_t state_size = states->size();
        assert(state_size);
        grl::MatrixXd timeEventM(state_size,4);
        for(int i = 0; i<state_size; ++i){
            auto kukaiiwaFusionTrackMessage = states->Get(i);
            auto timeEvent = kukaiiwaFusionTrackMessage->timeEvent();
            timeEventM(i,0) = timeEvent->local_receive_time();
            timeEventM(i,1) = timeEvent->local_request_time();
            timeEventM(i,2) = timeEvent->device_time();
            timeEventM(i,3) = timeEventM(i,2) - timeEventM(i,0);
        }
        return timeEventM;
    }
    /// Return the original timeEvent
    grl::MatrixXd  getTimeStamp(const fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> &kukaStatesP, kuka_tag){
        auto states = kukaStatesP->states();
        std::size_t state_size = states->size();
        assert(state_size);
        grl::MatrixXd timeEventM(state_size,4);
        for(int i = 0; i<state_size; ++i){
            auto KUKAiiwaState = states->Get(i);
            auto FRIMessage = KUKAiiwaState->FRIMessage();
            auto timeEvent = FRIMessage->timeStamp();
            timeEventM(i,0) = timeEvent->local_receive_time();
            timeEventM(i,1) = timeEvent->local_request_time();
            timeEventM(i,2) = timeEvent->device_time();
            timeEventM(i,3) = timeEventM(i,2) - timeEventM(i,0);
        }
        return timeEventM;
    }

    void regularizeTimeEvent(grl::MatrixXd& timeEventM){
        std::size_t time_size = timeEventM.rows();
        assert(time_size>0);
        auto initial_local_time = timeEventM(0,0);
        auto initial_device_time = timeEventM(0,3);
        timeEventM.col(1) = timeEventM.col(1) - initial_local_time * grl::VectorXd::Ones(time_size);
        timeEventM.col(0) = timeEventM.col(0) - initial_local_time * grl::VectorXd::Ones(time_size);
        timeEventM.col(2) = timeEventM.col(2) - initial_device_time * grl::VectorXd::Ones(time_size);
        timeEventM.col(3) = timeEventM.col(2) - timeEventM.col(0);
    }

    template<typename T>
    bool checkmonotonic( T &time){
        for(int i=1; i<time.size(); ++i){
            if(time(i)-time(i-1)<0 || time(i)<0)
                return false;
        }
        return true;
    }

    Eigen::MatrixXd  getMarkerPose(const fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> &logKUKAiiwaFusionTrackP, uint32_t &makerID, grl::MatrixXd& timeEvent){
        auto states = logKUKAiiwaFusionTrackP->states();
        std::size_t state_size = states->size();
        assert(state_size>0);
        // The first columne is counter
        std::size_t cols = 12;
        int row = 0;
        Eigen::MatrixXd markerPose(state_size, cols);
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
                        // markerPose.conservativeResize(row + 1, cols);
                        auto Pose = marker->transform();
                        auto position = Pose->position();
                        auto orientationQtn = Pose->orientation();
                        Eigen::VectorXd onePose(cols);
                        grl::VectorXd oneTime = timeEvent.row(i);
                        onePose << oneTime.cast<double>(), counter,1000* position.x(), 1000*position.y(), 1000*position.z(), orientationQtn.x(), orientationQtn.y(), orientationQtn.z(), orientationQtn.w();
                        markerPose.row(row++) = onePose.transpose();
                        continue;
                    }
                }
            }
        }
        if(row < state_size) {
            markerPose.conservativeResize(row, Eigen::NoChange_t{});
        }
        std::cout <<"State size: " << state_size <<"  markerPose size: " << markerPose.rows() << "  timeEvent: " << timeEvent.rows() << "  makerID: " << makerID <<std::endl;
        return markerPose;
    }



    /// Get the joint angles of a specific joint (joint_index)
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
    /// Reture all joint angles
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
    template <class T>
    void writeMatrixToCSV(const std::string& CSV_FileName, std::vector<std::string> &labels, T& t){
        std::size_t labels_size = labels.size();
        std::size_t cols_size = t.cols();
        std::size_t rows_size = t.rows();
        assert(labels_size == cols_size && rows_size>0 && cols_size>0);
        // create an ofstream for the file output (see the link on streams for more info)
        std::ofstream fs;
        // create a name for the file output
        fs.open(CSV_FileName, std::ofstream::out | std::ofstream::app);
        // write the file labels
        for(int col=0; col<cols_size-1; col++){
            fs << labels[col] << ",";
        }
        fs << labels[cols_size-1] << std::endl;
        for(int row_index=0; row_index<rows_size; ++row_index) {
            // write the data to the output file
            auto row = t.row(row_index);
            for(int col=0; col<cols_size-1; col++){
                fs << row[col] << ",";
            }
            fs << row[cols_size-1] << std::endl;
        }
        fs.close();
    }

    void writeJointAngToCSV(std::string CSV_FileName,
                              grl::VectorXd device_time,
                              grl::VectorXd local_request_time,
                              grl::VectorXd local_receive_time,
                              Eigen::MatrixXd &jointAngles) {
        std::size_t time_size = device_time.size();
        std::size_t row_size = jointAngles.rows();
        assert(time_size == row_size);

        // auto initial_local_time = local_request_time(0);
        auto initial_local_time = local_receive_time(0);
        auto initial_device_time = device_time(0);
        local_request_time = local_request_time - initial_local_time * grl::VectorXd::Ones(time_size);
        local_receive_time = local_receive_time - initial_local_time * grl::VectorXd::Ones(time_size);
        device_time = device_time - initial_device_time * grl::VectorXd::Ones(time_size);
        grl::VectorXd receive_request = local_receive_time - local_request_time;

        // create an ofstream for the file output (see the link on streams for more info)

        std::ofstream fs;
        // create a name for the file output

        fs.open(CSV_FileName, std::ofstream::out | std::ofstream::app);
        fs << "local_receive_time_X,"
           << "local_request_time_offset,"
           << "device_time_offset,"
           << "time_Y,"
           << "Receive-Request,"
           << "device_time_step,"
           << "receive_time_step,"
           << "Joint_1" << ",Joint_2" << ",Joint_3"
           << ",Joint_4" << ",Joint_5" << ",Joint_6" << ",Joint_7" << std::endl;
        int64_t device_time_step = 0;
        int64_t receive_time_step = 0;
        for(int i=0; i<time_size; ++i) {
            if(i>0) {
                device_time_step = device_time(i) - device_time(i-1);
                receive_time_step = local_receive_time(i) - local_receive_time(i-1);
            }
            Eigen::RowVectorXd jointAngle = jointAngles.row(i);
            // write the data to the output file
            fs << local_receive_time(i) <<","          // B
               << local_request_time(i)<< ","         // A
               << device_time(i) <<","                        // C
               << device_time(i) - local_receive_time(i) << ","
               << receive_request(i) << ","
               << device_time_step << ","
               << receive_time_step << ","
               << jointAngle[0] << ","
               << jointAngle[1] << ","
               << jointAngle[2] << ","
               << jointAngle[3] << ","
               << jointAngle[4] << ","
               << jointAngle[5] << ","
               << jointAngle[6] << std::endl;
        }
        fs.close();
    }



    void writeTimeEventToCSV(

        std::string & CSV_FileName,
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
        fs.open( CSV_FileName, std::ofstream::out | std::ofstream::app);
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



    void writeEEPoseToCSV(
        std::string & CSV_FileName,
        grl::VectorXd device_time,
        grl::VectorXd local_request_time,
        grl::VectorXd local_receive_time,
        Eigen::MatrixXd &poseEE){

        std::size_t time_size = device_time.size();
        std::size_t row_size = poseEE.rows();
        assert(time_size == row_size);

        // auto initial_local_time = local_request_time(0);
        auto initial_local_time = local_receive_time(0);
        auto initial_device_time = device_time(0);
        local_request_time = local_request_time - initial_local_time * grl::VectorXd::Ones(time_size);
        local_receive_time = local_receive_time - initial_local_time * grl::VectorXd::Ones(time_size);
        device_time = device_time - initial_device_time * grl::VectorXd::Ones(time_size);
        grl::VectorXd receive_request = local_receive_time - local_request_time;
        //  grl::VectorXd device_time_offset = local_receive_time - local_request_time;
        std::ofstream fs;
        // create a name for the file output
        fs.open( CSV_FileName, std::ofstream::out | std::ofstream::app);
         // write the file headers
        fs << "local_receive_time_X,"
           << "local_request_time_offset,"
           << "device_time_offset,"
           << "time_Y,"
           << "Receive-Request,"
           << "device_time_step,"
           << "receive_time_step,"
           << "X,"
           << "Y,"
           << "Z,"
           << "A,"
           << "B,"
           << "C,"
           << std::endl;
        int64_t device_time_step = 0;
        int64_t receive_time_step = 0;
        for(int i=0; i<time_size; ++i) {
            if(i>0) {
                device_time_step = device_time(i) - device_time(i-1);
                receive_time_step = local_receive_time(i) - local_receive_time(i-1);
            }
            Eigen::RowVectorXd pose = poseEE.row(i);
            // write the data to the output file
            fs << local_receive_time(i) <<","          // B
               << local_request_time(i)<< ","         // A
               << device_time(i) <<","                        // C
               << device_time(i) - local_receive_time(i) << ","
               << receive_request(i) << ","
               << device_time_step << ","
               << receive_time_step << ","
               << pose(0) << ","
               << pose(1) << ","
               << pose(2) << ","
               << pose(3) << ","
               << pose(4) << ","
               << pose(5) << std::endl;  //D
        }
        fs.close();
    }
    void writeMarkerPoseToCSV(
        std::string & CSV_FileName,
        grl::MatrixXd &timeEventM,
        Eigen::MatrixXd &markerPose){
        std::size_t time_size = timeEventM.rows();
        std::size_t marker_size = markerPose.rows();
        assert(time_size == marker_size);

        std::ofstream fs;
        // create a name for the file output
        fs.open( CSV_FileName, std::ofstream::out | std::ofstream::app);
         // write the file headers
        fs << "local_receive_time_X,"
           << "local_request_time_offset,"
           << "device_time_offset,"
           << "P_X,"
           << "P_Y,"
           << "P_Z,"
           << "Q_X,"
           << "Q_Y,"
           << "Q_Z,"
           << "Q_W"
           << std::endl;
        for(int i=0; i<time_size; ++i){
            grl::VectorXd timeEvent = timeEventM.row(i);
            Eigen::RowVectorXd pose = markerPose.row(i);
            // write the data to the output file
            fs << timeEvent(1)<< ","         // A
               << timeEvent(0) <<","          // B
               << timeEvent(2) <<","                        // C
               << pose(0) << ","
               << pose(1) << ","
               << pose(2) << ","
               << pose(3) << ","
               << pose(4) << ","
               << pose(5) << ","
               << pose(6) << std::endl;
        }
        fs.close();
    }

    void writeFT_KUKATimeEventToCSV(
        std::string &fusiontrackBinaryfile,
        std::string &kukaBinaryfile,
        std::string& FTKUKA_CSVfilename,
        std::string& FT_CSVfilename,
        std::string& FT_PoseCSVfilename,
        std::string& KUKA_CSVfilename) {
    fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> KUKAiiwaStatesRoot =
        fbs_tk::open_root<grl::flatbuffer::KUKAiiwaStates>(kukaBinaryfile);
    auto KUKA_states = KUKAiiwaStatesRoot->states();
    std::size_t kuka_state_size = KUKA_states->size();
    std::cout<< "------Kuka_state_size: "<< kuka_state_size << std::endl;

    grl::VectorXd kuka_device_time = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::device_time);
    grl::VectorXd kuka_local_request_time = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::local_request_time);
    grl::VectorXd kuka_local_receive_time = grl::getTimeStamp(KUKAiiwaStatesRoot, grl::kuka_tag(), grl::TimeType::local_receive_time);
    writeTimeEventToCSV(KUKA_CSVfilename, kuka_device_time, kuka_local_request_time, kuka_local_receive_time);



    fbs_tk::Root<grl::flatbuffer::LogKUKAiiwaFusionTrack> logKUKAiiwaFusionTrackP = fbs_tk::open_root<grl::flatbuffer::LogKUKAiiwaFusionTrack>(fusiontrackBinaryfile);
    auto FT_states = logKUKAiiwaFusionTrackP->states();
    std::size_t FT_state_size = FT_states->size();
    //std::cout<< "------FusionTrack State Size: "<< FT_state_size << std::endl;
    grl::VectorXd FT_device_time = grl:: getTimeStamp(logKUKAiiwaFusionTrackP, grl::fusiontracker_tag(), grl::TimeType::device_time);
    grl::VectorXd FT_local_request_time = grl::getTimeStamp(logKUKAiiwaFusionTrackP, grl::fusiontracker_tag(), grl::TimeType::local_request_time);
    grl::VectorXd FT_local_receive_time = grl::getTimeStamp(logKUKAiiwaFusionTrackP, grl::fusiontracker_tag(), grl::TimeType::local_receive_time);
    writeTimeEventToCSV(FT_CSVfilename, FT_device_time, FT_local_request_time, FT_local_receive_time);
    uint32_t makerID = 22;
    // Eigen::MatrixXd markerPose = grl::getMarkerPose(logKUKAiiwaFusionTrackP, makerID);
    // std::cout <<"Rows: " << markerPose.rows() << std::endl;
    //writeMarkerPoseToCSV(FT_PoseCSVfilename, FT_device_time, FT_local_request_time, FT_local_receive_time, markerPose);


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
    fs.open( FTKUKA_CSVfilename, std::ofstream::out | std::ofstream::app);
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


void AnalysizeFTKUKATimeEvent(grl::MatrixXd& timeEventM_FT, grl::MatrixXd& timeEventM_Kuka)
{

    std::size_t kuka_state_size = timeEventM_Kuka.rows();
    std::cout<< "------Kuka_state_size: "<< timeEventM_Kuka.rows() << std::endl;

    std::size_t FT_state_size = timeEventM_FT.rows();
    std::cout<< "------FT_state_size: "<< timeEventM_FT.rows() << std::endl;
    int kuka_index = 0;
    int FT_index = 0;

    // filter out the very beginning data, which can gurantee to record the data when the two devices work simultaniously.
    while(timeEventM_Kuka(kuka_index,0) < timeEventM_FT(FT_index,0)){
        kuka_index++;
    }
    while(timeEventM_Kuka(kuka_index,0) > timeEventM_FT(FT_index,0) && kuka_index == 0){
        FT_index++;
    }

    auto initial_local_time = std::min(timeEventM_FT(FT_index,0), timeEventM_Kuka(kuka_index,0));
    auto initial_device_time_kuka = timeEventM_Kuka(kuka_index,2);
    auto initial_device_time_FT = timeEventM_FT(FT_index,2);
    grl::VectorXd FT_local_request_time = timeEventM_FT.col(1) - initial_local_time * grl::VectorXd::Ones(FT_state_size);
    grl::VectorXd FT_local_receive_time = timeEventM_FT.col(0) - initial_local_time * grl::VectorXd::Ones(FT_state_size);
    grl::VectorXd FT_device_time = timeEventM_FT.col(2) - initial_device_time_FT * grl::VectorXd::Ones(FT_state_size);

    grl::VectorXd kuka_local_request_time = timeEventM_Kuka.col(1) - initial_local_time * grl::VectorXd::Ones(kuka_state_size);
    grl::VectorXd kuka_local_receive_time = timeEventM_Kuka.col(0) - initial_local_time * grl::VectorXd::Ones(kuka_state_size);
    grl::VectorXd kuka_device_time = timeEventM_Kuka.col(2) - initial_device_time_kuka * grl::VectorXd::Ones(kuka_state_size);


    // std::ofstream fs;
    // // create a name for the file output
    // fs.open( FTKUKA_CSVfilename, std::ofstream::out | std::ofstream::app);
    // // write the file headers
    // fs << "local_receive_time_offset_X,"
    //    << "FT_local_request_time,"
    //    << "KUKA_local_request_time,"
    //    << "FT_device_time_offset,"
    //    << "device_time_offset_kuka,"
    //    << "Y_FT,"
    //    << "Y_kuka"
    //    << std::endl;

    // int64_t kuka_diff = kuka_device_time(kuka_index) - kuka_local_receive_time(kuka_index);
    // int64_t FT_diff = FT_device_time(FT_index) - FT_local_receive_time(FT_index);
    // while ( kuka_index < kuka_state_size && FT_index < FT_state_size )
    // {
    //     if ( kuka_local_receive_time(kuka_index) < FT_local_receive_time(FT_index) ){
    //         // write the data to the output file
    //         fs << kuka_local_receive_time(kuka_index) <<","
    //            <<","
    //            << kuka_local_request_time(kuka_index) << ","
    //            << ","
    //            << kuka_device_time(kuka_index) <<","
    //            << ","
    //            << kuka_device_time(kuka_index) - kuka_local_receive_time(kuka_index) - kuka_diff
    //            << std::endl;
    //            kuka_index++;
    //     } else if( kuka_local_receive_time(kuka_index) > FT_local_receive_time(FT_index)) {
    //         // write the data to the output file
    //         fs << FT_local_receive_time(FT_index) <<","
    //            << FT_local_request_time(FT_index) << ","
    //            <<","
    //            << FT_device_time(FT_index) << ","
    //            << ","
    //            << FT_device_time(FT_index) - FT_local_receive_time(FT_index) - FT_diff << ","
    //            << std::endl;
    //         FT_index++;
    //     } else {
    //         fs << FT_local_receive_time(FT_index) <<","
    //            << FT_local_request_time(FT_index) << ","
    //            << kuka_local_request_time(kuka_index) << ","
    //            << FT_device_time(FT_index) << ","
    //            << kuka_device_time(kuka_index) <<","
    //            << FT_device_time(FT_index) - FT_local_receive_time(FT_index) - FT_diff << ","
    //            << kuka_device_time(kuka_index) - kuka_local_receive_time(kuka_index) - kuka_diff
    //            << std::endl;
    //         FT_index++;
    //         kuka_index++;
    //     }
    // }
    // fs.close();
}

}
#endif