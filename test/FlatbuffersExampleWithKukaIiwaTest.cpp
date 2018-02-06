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

using boost::asio::ip::udp;


enum { max_length = 1024 };

enum class HowToMove
{
   remain_stationary,
   absolute_position,
   relative_position,
   absolute_position_with_relative_rotation
};

enum class DriverToUse
{
   low_level_fri_function,
   low_level_fri_class,
   kuka_driver_high_level_class
};

BOOST_AUTO_TEST_SUITE(KukaTest)

BOOST_AUTO_TEST_CASE(runRepeatedly)
{
    const std::size_t MegaByte = 1024*1024;
    // In 32bits, the maximum size of a single flatbuffer is 2GB - 1 (defined in base.h), but actually single_buffer_limit_bytes can't be set beyond 2045 MB, otherwise assert might be launched.
    const std::size_t single_buffer_limit_bytes = 2045*MegaByte;
    // Install a signal handler to catch a signal when CONTROL+C
    std::signal(SIGINT, signal_handler);
    bool OK = true;
    bool debug = true;
    if (debug) std::cout << "starting KukaTest" << std::endl;


    std::string localhost("192.170.10.100");
    std::string localport("30200");
    std::string remotehost("192.170.10.2");
    std::string remoteport("30200");
    // A single class for an I/O service object.
    boost::asio::io_service io_service;
    std::shared_ptr<KUKA::FRI::ClientData> friData(std::make_shared<KUKA::FRI::ClientData>(7));
    cartographer::common::Time startTime;
    BOOST_VERIFY(friData);
    std::vector<double> ipoJointPos(7,0);
    std::vector<double> jointOffset(7,0); // length 7, value 0
    boost::container::static_vector<double, 7> jointStateToCommand(7,0);
    grl::robot::arm::KukaState armState;
    std::unique_ptr<grl::robot::arm::LinearInterpolation> lowLevelStepAlgorithmP;

    // Need to tell the system how long in milliseconds it has to reach the goal
    // or it will never move!
    std::size_t goal_position_command_time_duration = 4;
    lowLevelStepAlgorithmP.reset(new grl::robot::arm::LinearInterpolation());

    std::shared_ptr<grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation>> highLevelDriverClassP;
    DriverToUse driverToUse = DriverToUse::kuka_driver_high_level_class;
    if(driverToUse == DriverToUse::low_level_fri_class)
    {
      /// @todo TODO(ahundt) BUG: Need way to supply time to reach specified goal for position control and eliminate this allocation internally in the kuka driver. See similar comment in KukaFRIDriver.hpp
      /// IDEA: PASS A LOW LEVEL STEP ALGORITHM PARAMS OBJECT ON EACH UPDATE AND ONLY ONE INSTANCE OF THE ALGORITHM OBJECT ITSELF
      highLevelDriverClassP = std::make_shared<grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation>>(io_service,
        std::make_tuple("KUKA_LBR_IIWA_14_R820",
                        localhost,
                        localport,
                        remotehost,
                        remoteport/*,4 ms per tick*/,
                        grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation>::run_automatically));
    }
    std::shared_ptr<boost::asio::ip::udp::socket> socketP;

    std::shared_ptr<grl::robot::arm::KukaDriver> kukaDriverP;
    double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();


    if(driverToUse == DriverToUse::kuka_driver_high_level_class)
    {
        grl::robot::arm::KukaDriver::Params params = std::make_tuple(
                "Robotiiwa"               , // RobotName,
                "KUKA_LBR_IIWA_14_R820"   , // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800)
                "0.0.0.0"                 , // LocalUDPAddress
                "30010"                   , // LocalUDPPort
                "172.31.1.147"            , // RemoteUDPAddress
                "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
                "30200"                   , // LocalHostKukaKoniUDPPort,
                remotehost                , // RemoteHostKukaKoniUDPAddress,
                remoteport                , // RemoteHostKukaKoniUDPPort
                "FRI"                     , // KukaCommandMode (options are FRI, JAVA)
                "FRI"                       // KukaMonitorMode (options are FRI, JAVA)
                );
        /// @todo TODO(ahundt) Currently assumes ip address
        kukaDriverP=std::make_shared<grl::robot::arm::KukaDriver>(params);
        // kukaDriverP->construct();
        // Default to joint servo mode for commanding motion
        kukaDriverP->set(grl::flatbuffer::ArmState::MoveArmJointServo);
        kukaDriverP->set(goal_position_command_time_duration,grl::time_duration_command_tag());
        // std::cout << "KUKA COMMAND MODE: " << std::get<grl::robot::arm::KukaDriver::KukaCommandMode>(params) << "\n";
    }

    unsigned int num_missed = 0;

    for (std::size_t i = 0; !signalStatusG; ++i) {

        /// perform the update step, receiving and sending data to/from the arm
        boost::system::error_code send_ec, recv_ec;
        std::size_t send_bytes_transferred = 0, recv_bytes_transferred = 0;
        bool haveNewData = false;
        grl::TimeEvent time_event_stamp;
        if(driverToUse == DriverToUse::kuka_driver_high_level_class)
        {
            kukaDriverP->set( jointStateToCommand, grl::revolute_joint_angle_open_chain_command_tag());
            kukaDriverP->run_one();
            //kukaDriverP->get();
        }

        /// use the interpolated joint position from the previous update as the base
        /// The interpolated position is where the java side is commanding,
        /// or the fixed starting position with a hold position command on the java side.
        if(i!=0 && friData){
             grl::robot::arm::copy(friData->monitoringMsg,ipoJointPos.begin(),grl::revolute_joint_angle_interpolated_open_chain_state_tag());
        }


    }
    std::shared_ptr<flatbuffers::FlatBufferBuilder> fbbP;
    fbbP = std::make_shared<flatbuffers::FlatBufferBuilder>();
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

    bool logdata = false;
    if (logdata == true) {
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
            flatbuffers::Offset<grl::flatbuffer::ArmControlState> controlState = grl::toFlatBuffer(*fbbP, basename, sequenceNumber++, duration, armState, armControlMode);
            flatbuffers::Offset<grl::flatbuffer::CartesianImpedenceControlMode> setCartesianImpedance = grl::toFlatBuffer(*fbbP, cart_stiffness_, cart_damping_,
                    nullspace_stiffness_, nullspace_damping_, cart_max_path_deviation_, cart_max_ctrl_vel_, cart_max_ctrl_force_, max_control_force_stop_);
            flatbuffers::Offset<grl::flatbuffer::JointImpedenceControlMode> setJointImpedance = grl::toFlatBuffer(*fbbP, joint_stiffness_, joint_damping_);
            flatbuffers::Offset<grl::flatbuffer::SmartServo> setSmartServo = grl::toFlatBuffer(*fbbP, joint_AccelerationRel_, joint_VelocityRel_, updateMinimumTrajectoryExecutionTime, minimumTrajectoryExecutionTime);
            flatbuffers::Offset<grl::flatbuffer::FRI> FRIConfig = grl::toFlatBuffer(*fbbP, overlayType, connectionInfo, false, 3501, false, 3502);

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


            // copy the state data into a more accessible object
            /// TODO(ahundt) switch from this copy to a non-deprecated call

            flatbuffers::Offset<grl::flatbuffer::KUKAiiwaArmConfiguration> kukaiiwaArmConfiguration = grl::toFlatBuffer(
                *fbbP,
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
                *fbbP,
                ::FRISessionState::FRISessionState_IDLE,
                ::FRIConnectionQuality::FRIConnectionQuality_POOR,
                controlMode,
                friMonitoringMessage,
                time_event_stamp);
            grl::flatbuffer::Wrench cartesianWrench{grl::flatbuffer::Vector3d(1,1,1),grl::flatbuffer::Vector3d(1,1,1),grl::flatbuffer::Vector3d(1,1,1)};


            flatbuffers::Offset<grl::flatbuffer::JointState> jointStatetab = grl::toFlatBuffer(*fbbP, jointValues, jointValues, jointValues, jointValues);
            grl::flatbuffer::Pose cartesianFlangePose = grl::flatbuffer::Pose(grl::flatbuffer::Vector3d(1,1,1), grl::flatbuffer::Quaternion(2,3,4,5));
            flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorState> kukaiiwaMonitorState = grl::toFlatBuffer(
                *fbbP,
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
                *fbbP,
                "hardvareVersion",
                torqueSensorLimits,
                true,
                false,
                processData);
            flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState> kukaiiwaState = grl::toFlatBuffer(
                *fbbP,
                RobotName,
                destination,
                source,
                time_event_stamp,
                true, controlState,
                true, kukaiiwaArmConfiguration,
                true, kukaiiwaMonitorState,
                false, monitorConfig,
                friMessageLog);

            kukaiiwaStateVec.push_back(kukaiiwaState);

            builder_size_bytes = fbbP->GetSize();
            std::size_t newData = builder_size_bytes - previous_size;
            previous_size = builder_size_bytes;
            // std::cout<< "single message data size (bytes): " << newData << "  Buffer size: " << builder_size_bytes/MegaByte <<" MB" << std::endl;
        }

        //std::cout<< "kukaiiwaStateVec:" << kukaiiwaStateVec.size() << std::endl;

        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaStates> kukaStates = grl::toFlatBuffer(*fbbP, kukaiiwaStateVec);

        grl::flatbuffer::FinishKUKAiiwaStatesBuffer(*fbbP, kukaStates);

        uint8_t *buf = fbbP->GetBufferPointer();
        std::size_t bufsize = fbbP->GetSize();

        /// To expand the capacity of a single buffer, _max_tables is set to 10000000
        flatbuffers::uoffset_t _max_depth = 64;
        flatbuffers::uoffset_t _max_tables = 10000000;
        flatbuffers::Verifier verifier(buf, bufsize, _max_depth, _max_tables);
        OK = OK && grl::flatbuffer::VerifyKUKAiiwaStatesBuffer(verifier);
        assert(OK && "VerifyKUKAiiwaStatesBuffer");

        //std::cout << "Buffer size: " << bufsize << std::endl;

        std::string binary_file_path = "Kuka_test_binary.iiwa";
        std::string json_file_path = "kuka_test_text.json";

        // Get the current working directory
        std::string fbs_filename("KUKAiiwa.fbs");

        OK = OK && grl::SaveFlatBufferFile(
            buf,
            fbbP->GetSize(),
            binary_file_path,
            fbs_filename,
            json_file_path);
        assert(OK && "SaveFlatBufferFile");
    }
    std::cout << "End of the program" << std::endl;

}

BOOST_AUTO_TEST_SUITE_END()
