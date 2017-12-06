// system includes
#include <boost/test/unit_test.hpp>
#include <exception>
#include <iostream>
#include <vector>



#include "grl/kuka/KukaToFlatbuffer.hpp"
#include "grl/kuka/KukaJAVAdriver.hpp"
#include "grl/kuka/Kuka.hpp"
#include "grl/kuka/KukaDriver.hpp"

#include "flatbuffers/util.h"


// #include "grl/flatbuffer/flatbuffer.hpp"
/// Make reference to void map_repeatedDouble() in pb_frimessages_callback.c and pb_frimessages_callbacks.h
pb_callback_t makeupJointValues(std::vector<double> &jointvalue) {
     /// Make reference to void map_repeatedDouble() in pb_frimessages_callback.c and pb_frimessages_callbacks.h
    pb_callback_t *values = new pb_callback_t;
    int numDOF = 7;
    values->funcs.encode =  &encode_repeatedDouble;
    values->funcs.decode = &decode_repeatedDouble;
    tRepeatedDoubleArguments *arg = new tRepeatedDoubleArguments;
    // map the local container data to the message data fields
    arg->max_size = numDOF;
    arg->size = 0;
    arg->value = (double*) malloc(numDOF * sizeof(double));

    for(int i=0; i<numDOF; i++) {
        arg->value[i] = jointvalue[i];
    }
    values->arg = arg;
    return *values;
}

int main(int argc, char **argv)
{

 bool debug = true;
    if (debug) std::cout << "starting KukaTest" << std::endl;
    std::shared_ptr<flatbuffers::FlatBufferBuilder> fbbP;
    fbbP = std::make_shared<flatbuffers::FlatBufferBuilder>();
    double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();
    grl::robot::arm::KukaDriver::Params params = std::make_tuple(
            "Robotiiwa"               , // RobotName,
            "KUKA_LBR_IIWA_14_R820"   , // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800)
            "0.0.0.0"                 , // LocalUDPAddress
            "30010"                   , // LocalUDPPort
            "172.31.1.147"            , // RemoteUDPAddress
            "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
            "30200"                   , // LocalHostKukaKoniUDPPort,
            "192.170.10.2"            , // RemoteHostKukaKoniUDPAddress,
            "30200"                   , // RemoteHostKukaKoniUDPPort
            "JAVA"                    , // KukaCommandMode (options are FRI, JAVA)
            "FRI"                       // KukaMonitorMode (options are FRI, JAVA)
                );
    std::string RobotName("Robotiiwa" );
    std::string destination("where this message is going (URI)");
    std::string source("where this message came from (URI)");
    std::string basename = RobotName; //std::get<0>(params);
    bool setArmConfiguration_ = true; // set the arm config first time
    bool max_control_force_stop_ = false;
    double nullspace_stiffness_ = 2.0;
    double nullspace_damping_ = 0.5;

    grl::flatbuffer::ArmState armControlMode_(grl::flatbuffer::ArmState::StartArm);
    grl::flatbuffer::KUKAiiwaInterface commandInterface_ = grl::flatbuffer::KUKAiiwaInterface::SmartServo;// KUKAiiwaInterface::SmartServo;
    grl::flatbuffer::KUKAiiwaInterface monitorInterface_ = grl::flatbuffer::KUKAiiwaInterface::FRI;
    ::ControlMode controlMode = ::ControlMode::ControlMode_POSITION_CONTROLMODE;
    ::ClientCommandMode clientCommandMode = ::ClientCommandMode::ClientCommandMode_POSITION;
    ::OverlayType overlayType = ::OverlayType::OverlayType_NO_OVERLAY;
    ::ConnectionInfo connectionInfo{::FRISessionState::FRISessionState_IDLE,
                                    ::FRIConnectionQuality::FRIConnectionQuality_POOR,
                                    true, 4, false, 0};
    grl::robot::arm::KukaState armState;
    std::shared_ptr<KUKA::FRI::ClientData> friData(std::make_shared<KUKA::FRI::ClientData>(7));
    cartographer::common::Time startTime;

    std::vector<double> joint_stiffness_ = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    std::vector<double> joint_damping_ = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    std::vector<double> joint_AccelerationRel_(7,0.5);
    std::vector<double> joint_VelocityRel_(7,1.0);
    bool updateMinimumTrajectoryExecutionTime = false;
    double minimumTrajectoryExecutionTime = 4;

    std::vector<double> absoluteGoalPos(7,0.2);

    std::size_t goal_position_command_time_duration = 4;
    std::shared_ptr<grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation>> highLevelDriverClassP;
    std::shared_ptr<grl::robot::arm::KukaDriver> kukaDriverP;

    kukaDriverP=std::make_shared<grl::robot::arm::KukaDriver>(params);
    kukaDriverP->set(grl::flatbuffer::ArmState::MoveArmJointServo);
    kukaDriverP->set(goal_position_command_time_duration,grl::time_duration_command_tag());
    std::cout << "KUKA COMMAND MODE: " << std::get<grl::robot::arm::KukaDriver::KukaCommandMode>(params) << "\n";
    int64_t sequenceNumber = 0;
    auto controlState = grl::toFlatBuffer(*fbbP, basename, sequenceNumber++, duration, armState, armControlMode_);

    //Cartesian Impedance Values
    grl::flatbuffer::Vector3d cart_stiffness_trans_ = grl::flatbuffer::Vector3d(500,500,500);
    grl::flatbuffer::EulerRotation cart_stifness_rot_ = grl::flatbuffer::EulerRotation(200,200,200,grl::flatbuffer::EulerOrder::xyz);
    grl::flatbuffer::Vector3d cart_damping_trans_ = grl::flatbuffer::Vector3d(0.3,0.3,0.3);
    grl::flatbuffer::EulerRotation cart_damping_rot_ = grl::flatbuffer::EulerRotation(0.3,0.3,0.3,grl::flatbuffer::EulerOrder::xyz);
    grl::flatbuffer::EulerPose cart_stiffness_ = grl::flatbuffer::EulerPose(cart_stiffness_trans_, cart_stifness_rot_);
    grl::flatbuffer::EulerPose cart_damping_ = grl::flatbuffer::EulerPose(cart_damping_trans_, cart_damping_rot_);

    grl::flatbuffer::EulerPose cart_max_path_deviation_  = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(1000,1000,100), grl::flatbuffer::EulerRotation(5.,5.,5., grl::flatbuffer::EulerOrder::xyz));
    grl::flatbuffer::EulerPose cart_max_ctrl_vel_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(1000,1000,1000), grl::flatbuffer::EulerRotation(6.3,6.3,6.3, grl::flatbuffer::EulerOrder::xyz));
    grl::flatbuffer::EulerPose cart_max_ctrl_force_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(200,200,200), grl::flatbuffer::EulerRotation(200.,200.,200., grl::flatbuffer::EulerOrder::xyz));



    auto setCartesianImpedance = grl::toFlatBuffer(*fbbP, cart_stiffness_, cart_damping_,
            nullspace_stiffness_, nullspace_damping_, cart_max_path_deviation_, cart_max_ctrl_vel_, cart_max_ctrl_force_, max_control_force_stop_);
    auto setJointImpedance = grl::toFlatBuffer(*fbbP, joint_stiffness_, joint_damping_);
    auto setSmartServo = grl::toFlatBuffer(*fbbP, joint_AccelerationRel_, joint_VelocityRel_, updateMinimumTrajectoryExecutionTime, minimumTrajectoryExecutionTime);
    auto FRIConfig = grl::toFlatBuffer(*fbbP, overlayType, connectionInfo, false, 3501, false, 3502);
    std::vector<flatbuffers::Offset<grl::flatbuffer::LinkObject>> tools;
    std::vector<flatbuffers::Offset<grl::flatbuffer::ProcessData>> processData;
    for(int i=0; i<7; i++) {
        std::string linkname = "Link" + std::to_string(i);
        std::string parent = i==0?"Base":("Link" + std::to_string(i-1));
        grl::flatbuffer::Pose pose = grl::flatbuffer::Pose(grl::flatbuffer::Vector3d(i,i,i), grl::flatbuffer::Quaternion(i,i,i,i));
        grl::flatbuffer::Inertia inertia = grl::flatbuffer::Inertia(1, pose, 1, 2, 3, 4, 5, 6);
        flatbuffers::Offset<grl::flatbuffer::LinkObject> linkObject = grl::toFlatBuffer(*fbbP, linkname, parent, pose, inertia);
        tools.push_back(linkObject);
        processData.push_back(
            grl::toFlatBuffer(*fbbP,
            "dataType"+ std::to_string(i),
            "defaultValue"+ std::to_string(i),
            "displayName"+ std::to_string(i),
            "id"+ std::to_string(i),
            "min"+ std::to_string(i),
            "max"+ std::to_string(i),
            "unit"+ std::to_string(i),
            "value"+ std::to_string(i)));

    }
    std::string currentMotionCenter = "currentMotionCenter";
    auto kukaiiwaArmConfiguration = grl::toFlatBuffer(
        *fbbP,
        RobotName,
        commandInterface_,
        monitorInterface_,
        clientCommandMode,
        overlayType,
        controlMode,
        setCartesianImpedance,
        setJointImpedance,
        setSmartServo,
        FRIConfig,
        tools,
        processData,
        "currentMotionCenter",
        true);
    bool setArmControlState = true; // only actually change the arm state when this is true.
    ::MessageHeader messageHeader{1,2,3};
    ::RobotInfo robotInfo{true, 7, true, ::SafetyState::SafetyState_NORMAL_OPERATION};
    ::JointValues strjointValues;
    std::vector<double> jointValues(7, 0.1);



    strjointValues.value = makeupJointValues(jointValues);
    ::MessageMonitorData messageMonitorData{
        true, strjointValues,
        true, strjointValues,
        true, strjointValues,
        true, strjointValues,
        true, strjointValues,
        true, ::TimeStamp{1,2}
    };
    ::MessageIpoData ipoData{
        true, strjointValues,
        true, clientCommandMode,
        true, overlayType,
        true, 4.0
    };
    ::Transformation transformation{ "Transformation", 12, {1,2,3,4,5,6,7,8,9,10,11,12}, true, ::TimeStamp{3,4}};
    /// Transformation requestedTransformations[5] = { transformation, transformation, transformation, transformation, transformation };
    ::MessageEndOf endOfMessageData{true, 32, true, ::Checksum{true, 32}};
    ::FRIMonitoringMessage friMonitoringMessage {messageHeader,
        true, robotInfo,
        true, messageMonitorData,
        true, connectionInfo,
        true, ipoData,
        5,
        { transformation, transformation, transformation, transformation, transformation },
        true, endOfMessageData};
    grl::TimeEvent time_event_stamp;
    auto friMessageLog = grl::toFlatBuffer(
        *fbbP,
        ::FRISessionState::FRISessionState_IDLE,
        ::FRIConnectionQuality::FRIConnectionQuality_POOR,
        controlMode,
        friMonitoringMessage,
        time_event_stamp);
    auto kukaiiwaState = grl::toFlatBuffer(
        *fbbP,
        RobotName,
        destination,
        source,
        duration,
        true, controlState,
        true, kukaiiwaArmConfiguration,
        false, 0,
        false, 0,
        friMessageLog);
    std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState>> kukaiiwaStateVec;
    kukaiiwaStateVec.push_back(kukaiiwaState);
    auto states = grl::toFlatBuffer(*fbbP, kukaiiwaStateVec);
    grl::flatbuffer::FinishKUKAiiwaStatesBuffer(*fbbP, states);
    flatbuffers::Verifier verifier(fbbP->GetBufferPointer(),fbbP->GetSize());
    std::cout<< "VerifyKUKAiiwaStatesBuffer: " << grl::flatbuffer::VerifyKUKAiiwaStatesBuffer(verifier) << std::endl;


    std::string binary_file_path = "Kuka_test_binary.flik";
    std::string json_file_path = "kuka_test_text.json";

    std::string fbs_filename("KUKAiiwa.fbs");
    grl::SaveFlatBufferFile(
        fbbP->GetBufferPointer(),
        fbbP->GetSize(),
        binary_file_path,
        fbs_filename,
        json_file_path);
  std::cout << "End of the program" << std::endl;
  return success;
} // End of main function
