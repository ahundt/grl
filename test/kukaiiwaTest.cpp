#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE grlTest

#define FLATBUFFERS_DEBUG_VERIFICATION_FAILUR
// system includes
#include <boost/test/unit_test.hpp>
#include <exception>
#include <iostream>
#include <vector>

#include "grl/flatbuffer/flatbuffer.hpp"
#include "grl/kuka/KukaToFlatbuffer.hpp"
#include "grl/kuka/KukaJAVAdriver.hpp"

#include "grl/kuka/Kuka.hpp"
#include "grl/kuka/KukaDriver.hpp"

/// Make reference to void map_repeatedDouble() in pb_frimessages_callback.c and pb_frimessages_callbacks.h
std::shared_ptr<pb_callback_t> makeupJointValues(std::vector<double> &jointvalue) {
     /// Make reference to void map_repeatedDouble() in pb_frimessages_callback.c and pb_frimessages_callbacks.h
    std::shared_ptr<pb_callback_t> values(std::make_shared<pb_callback_t>());
    // pb_callback_t *values = new pb_callback_t;
    int numDOF = 7;
    values->funcs.encode = &encode_repeatedDouble;
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
    return values;
}

volatile std::sig_atomic_t signalStatusG = 0;

// called when the user presses ctrl+c
void signal_handler(int signal)
{
  signalStatusG = signal;
}

int charP_size(const char* buf)
{
    int Size = 0;
    while (buf[Size] != '\0') Size++;
    return Size;
}

BOOST_AUTO_TEST_SUITE(KukaTest)

BOOST_AUTO_TEST_CASE(runRepeatedly)
{
    const std::size_t MegaByte = 1024*1024;
    // The maximum size of a single flatbuffer is 2GB - 1, but actually single_buffer_limit_bytes can't be set beyond 2045 MB, otherwise assert might be launched.
    // In 32bits, this evaluates to 2GB - 1, defined in base.h.
    // #define FLATBUFFERS_MAX_BUFFER_SIZE ((1ULL << (sizeof(soffset_t) * 8 - 1)) - 1)
    const std::size_t single_buffer_limit_bytes = 2045*MegaByte;
    // Install a signal handler to catch a signal when CONTROL+C
    std::signal(SIGINT, signal_handler);
    bool OK = true;
    bool debug = true;
    if (debug) std::cout << "starting KukaTest" << std::endl;
    // std::shared_ptr<flatbuffers::FlatBufferBuilder> fbbP;
    flatbuffers::FlatBufferBuilder fbb;
    // fbbP = std::make_shared<flatbuffers::FlatBufferBuilder>();
    // std::shared_ptr<KUKA::FRI::ClientData> friData(std::make_shared<KUKA::FRI::ClientData>(7));
    // std::shared_ptr<grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation>> highLevelDriverClassP;
    double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();


    if (debug) std::cout << "duration: " << duration << std::endl;
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

    grl::flatbuffer::ArmState armControlMode = grl::flatbuffer::ArmState::StartArm;
    grl::flatbuffer::KUKAiiwaInterface commandInterface = grl::flatbuffer::KUKAiiwaInterface::SmartServo;// KUKAiiwaInterface::SmartServo;
    grl::flatbuffer::KUKAiiwaInterface monitorInterface = grl::flatbuffer::KUKAiiwaInterface::FRI;
    ::ControlMode controlMode = ::ControlMode::ControlMode_POSITION_CONTROLMODE;
    ::ClientCommandMode clientCommandMode = ::ClientCommandMode::ClientCommandMode_POSITION;
    ::OverlayType overlayType = ::OverlayType::OverlayType_NO_OVERLAY;
    ::ConnectionInfo connectionInfo{::FRISessionState::FRISessionState_IDLE,
                                    ::FRIConnectionQuality::FRIConnectionQuality_POOR,
                                    true, 4, false, 0};


    std::vector<double> joint_stiffness_(7, 0.2);
    std::vector<double> joint_damping_(7, 0.3);
    std::vector<double> joint_AccelerationRel_(7, 0.5);
    std::vector<double> joint_VelocityRel_(7, 1.0);
    bool updateMinimumTrajectoryExecutionTime = false;
    double minimumTrajectoryExecutionTime = 4;
    std::size_t goal_position_command_time_duration = 4;

    std::cout << "KUKA COMMAND MODE: " << std::get<grl::robot::arm::KukaDriver::KukaCommandMode>(params) << "\n";
    int64_t sequenceNumber = 0;
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
    double nullspace_stiffness_ = 0.1;
    double nullspace_damping_ = 0.1;

    ::MessageHeader messageHeader{1,2,3};
    ::RobotInfo robotInfo{true, 7, true, ::SafetyState::SafetyState_NORMAL_OPERATION};
    ::JointValues strjointValues;
    std::vector<double> jointValues(7, 0.1);
    boost::container::static_vector<double, 7> jointstate;
    for(int i = 0; i<7; i++) {
        jointstate.push_back(jointValues[i]);
    }
      /// Make reference to void map_repeatedDouble() in pb_frimessages_callback.c and pb_frimessages_callbacks.h
    strjointValues.value = *makeupJointValues(jointValues);
    grl::robot::arm::KukaState::joint_state kukajointstate(jointstate);

    ////////////////////////////////////////////////////////////
    // Double check the initialization of this parameter //////
    //////////////////////////////////////////////////////////
    grl::robot::arm::KukaState armState;
    armState.position = kukajointstate;
    armState.torque = kukajointstate;
    armState.externalTorque = kukajointstate;
    armState.commandedPosition = kukajointstate;
    armState.commandedTorque = kukajointstate;
    armState.ipoJointPosition = kukajointstate;
    armState.ipoJointPositionOffsets = kukajointstate;
    armState.commandedCartesianWrenchFeedForward = kukajointstate;
    armState.externalForce = kukajointstate;
    armState.sessionState = grl::toFlatBuffer(connectionInfo.sessionState);
    armState.connectionQuality = grl::toFlatBuffer(connectionInfo.quality);
    armState.safetyState = grl::toFlatBuffer(::SafetyState::SafetyState_NORMAL_OPERATION);
    armState.operationMode = grl::toFlatBuffer(OperationMode::OperationMode_TEST_MODE_1);
    armState.driveState = grl::toFlatBuffer(::DriveState::DriveState_OFF);

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
    ::FRIMonitoringMessage friMonitoringMessage {
        messageHeader,
        true, robotInfo,
        true, messageMonitorData,
        true, connectionInfo,
        true, ipoData,
        5,
        { transformation, transformation, transformation, transformation, transformation },
        true, endOfMessageData};
    grl::TimeEvent time_event_stamp;
    std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState>> kukaiiwaStateVec;
    std::size_t builder_size_bytes = 0;
    std::size_t previous_size = 0;


    while(!signalStatusG && OK && builder_size_bytes < single_buffer_limit_bytes )
    {
        // grl::robot::arm::copy(friMonitoringMessage, armState);
        flatbuffers::Offset<grl::flatbuffer::ArmControlState> controlState = grl::toFlatBuffer(fbb, basename, sequenceNumber++, duration, armState, armControlMode);
        flatbuffers::Offset<grl::flatbuffer::CartesianImpedenceControlMode> setCartesianImpedance = grl::toFlatBuffer(fbb, cart_stiffness_, cart_damping_,
                nullspace_stiffness_, nullspace_damping_, cart_max_path_deviation_, cart_max_ctrl_vel_, cart_max_ctrl_force_, max_control_force_stop_);
        flatbuffers::Offset<grl::flatbuffer::JointImpedenceControlMode> setJointImpedance = grl::toFlatBuffer(fbb, joint_stiffness_, joint_damping_);
        flatbuffers::Offset<grl::flatbuffer::SmartServo> setSmartServo = grl::toFlatBuffer(fbb, joint_AccelerationRel_, joint_VelocityRel_, updateMinimumTrajectoryExecutionTime, minimumTrajectoryExecutionTime);
        flatbuffers::Offset<grl::flatbuffer::FRI> FRIConfig = grl::toFlatBuffer(fbb, overlayType, connectionInfo, false, 3501, false, 3502);

        std::vector<flatbuffers::Offset<grl::flatbuffer::LinkObject>> tools;
        std::vector<flatbuffers::Offset<grl::flatbuffer::ProcessData>> processData;

        for(int i=0; i<7; i++) {
            std::string linkname = "Link" + std::to_string(i);
            std::string parent = i==0?"Base":("Link" + std::to_string(i-1));
            grl::flatbuffer::Pose pose = grl::flatbuffer::Pose(grl::flatbuffer::Vector3d(i,i,i), grl::flatbuffer::Quaternion(i,i,i,i));
            grl::flatbuffer::Inertia inertia = grl::flatbuffer::Inertia(1, pose, 1, 2, 3, 4, 5, 6);
            flatbuffers::Offset<grl::flatbuffer::LinkObject> linkObject = grl::toFlatBuffer(fbb, linkname, parent, pose, inertia);
            tools.push_back(linkObject);
            processData.push_back(
                grl::toFlatBuffer(fbb,
                "dataType"+ std::to_string(i),
                "defaultValue"+ std::to_string(i),
                "displayName"+ std::to_string(i),
                "id"+ std::to_string(i),
                "min"+ std::to_string(i),
                "max"+ std::to_string(i),
                "unit"+ std::to_string(i),
                "value"+ std::to_string(i)));
        }


        // copy the state data into a more accessible object
        /// TODO(ahundt) switch from this copy to a non-deprecated call

        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaArmConfiguration> kukaiiwaArmConfiguration = grl::toFlatBuffer(
            fbb,
            RobotName,
            commandInterface,
            monitorInterface,
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

        flatbuffers::Offset<grl::flatbuffer::FRIMessageLog> friMessageLog = grl::toFlatBuffer(
            fbb,
            ::FRISessionState::FRISessionState_IDLE,
            ::FRIConnectionQuality::FRIConnectionQuality_POOR,
            controlMode,
            friMonitoringMessage,
            time_event_stamp);
        grl::flatbuffer::Wrench cartesianWrench{grl::flatbuffer::Vector3d(1,1,1),grl::flatbuffer::Vector3d(1,1,1),grl::flatbuffer::Vector3d(1,1,1)};


        flatbuffers::Offset<grl::flatbuffer::JointState> jointStatetab = grl::toFlatBuffer(fbb, jointValues, jointValues, jointValues, jointValues);
        grl::flatbuffer::Pose cartesianFlangePose = grl::flatbuffer::Pose(grl::flatbuffer::Vector3d(1,1,1), grl::flatbuffer::Quaternion(2,3,4,5));
        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorState> kukaiiwaMonitorState = grl::toFlatBuffer(
            fbb,
            jointStatetab, // flatbuffers::Offset<grl::flatbuffer::JointState> &measuredState,
            cartesianFlangePose,
            jointStatetab, // flatbuffers::Offset<grl::flatbuffer::JointState> &jointStateReal,
            jointStatetab, // flatbuffers::Offset<grl::flatbuffer::JointState> &jointStateInterpolated,
            jointStatetab, // flatbuffers::Offset<grl::flatbuffer::JointState> &externalState,
            ::FRISessionState::FRISessionState_IDLE,
            ::OperationMode::OperationMode_TEST_MODE_1,
            cartesianWrench);
            std::vector<double> torqueSensorLimits(7,0.5);
        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorConfiguration> monitorConfig = grl::toFlatBuffer(
            fbb,
            "hardvareVersion",
            torqueSensorLimits,
            true,
            false,
            processData);
        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState> kukaiiwaState = grl::toFlatBuffer(
            fbb,
            RobotName,
            destination,
            source,
            duration,
            true, controlState,
            true, kukaiiwaArmConfiguration,
            true, kukaiiwaMonitorState,
            false, monitorConfig,
            friMessageLog);

        kukaiiwaStateVec.push_back(kukaiiwaState);

        builder_size_bytes = fbb.GetSize();
        std::size_t newData = builder_size_bytes - previous_size;
        previous_size = builder_size_bytes;
        std::cout<< "single message data size (bytes): " << newData << "  Buffer size: " << builder_size_bytes/MegaByte <<" MB" << std::endl;
    }

    std::cout<< "kukaiiwaStateVec:" << kukaiiwaStateVec.size() << std::endl;

    flatbuffers::Offset<grl::flatbuffer::KUKAiiwaStates> kukaStates = grl::toFlatBuffer(fbb, kukaiiwaStateVec);

    grl::flatbuffer::FinishKUKAiiwaStatesBuffer(fbb, kukaStates);

    uint8_t *buf = fbb.GetBufferPointer();
    std::size_t bufsize = fbb.GetSize();

    flatbuffers::Verifier verifier(buf, bufsize);
    OK = OK && grl::flatbuffer::VerifyKUKAiiwaStatesBuffer(verifier);
    //assert(OK && "VerifyKUKAiiwaStatesBuffer");

    std::cout << "Buffer size: " << bufsize << std::endl;

    std::string binary_file_path = "Kuka_test_binary.iiwa";
    std::string json_file_path = "kuka_test_text.json";

    // Get the current working directory
    std::string fbs_filename("KUKAiiwa.fbs");

    OK = OK && grl::SaveFlatBufferFile(
        buf,
        fbb.GetSize(),
        binary_file_path,
        fbs_filename,
        json_file_path);
    assert(OK && "SaveFlatBufferFile");
    std::cout << "End of the program" << std::endl;

}

BOOST_AUTO_TEST_SUITE_END()
