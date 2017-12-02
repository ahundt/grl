#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE grlTest

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

volatile std::sig_atomic_t signalStatusG = 0;

// called when the user presses ctrl+c
void signal_handler(int signal)
{
  signalStatusG = signal;
}

BOOST_AUTO_TEST_SUITE(KukaTest)

BOOST_AUTO_TEST_CASE(runRepeatedly)
{
    const std::size_t MegaByte = 1024*1024;
    // If we write too large a flatbuffer
    const std::size_t single_buffer_limit_bytes = 2047*MegaByte;
    // Install a signal handler to catch a signal when CONTROL+C
    std::signal(SIGINT, signal_handler);
    bool OK = false;
    bool debug = true;
    if (debug) std::cout << "starting KukaTest" << std::endl;
    std::shared_ptr<flatbuffers::FlatBufferBuilder> fbbP;
    fbbP = std::make_shared<flatbuffers::FlatBufferBuilder>();
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

    grl::flatbuffer::ArmState armControlMode(grl::flatbuffer::ArmState::StartArm);
    grl::flatbuffer::KUKAiiwaInterface commandInterface = grl::flatbuffer::KUKAiiwaInterface::SmartServo;// KUKAiiwaInterface::SmartServo;
    grl::flatbuffer::KUKAiiwaInterface monitorInterface = grl::flatbuffer::KUKAiiwaInterface::FRI;
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

    std::size_t goal_position_command_time_duration = 4;
    std::shared_ptr<grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation>> highLevelDriverClassP;

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
    std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState>> kukaiiwaStateVec;
    std::size_t builder_size_bytes = 0;
    std::size_t previous_size = 0;


    // while(!signalStatusG && OK && builder_size_bytes < single_buffer_limit_bytes )
    // {
        auto controlState = grl::toFlatBuffer(*fbbP, basename, sequenceNumber++, duration, armState, armControlMode);

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

        auto kukaiiwaArmConfiguration = grl::toFlatBuffer(
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

        kukaiiwaStateVec.push_back(kukaiiwaState);

        builder_size_bytes = fbbP->GetSize();
        std::size_t newData = builder_size_bytes - previous_size;
        previous_size = builder_size_bytes;
        std::cout<< "single message data size (bytes): " << newData << "  Buffer size: " << builder_size_bytes/MegaByte <<" MB" << std::endl;
    // }

    std::cout<< "kukaiiwaStateVec:" << kukaiiwaStateVec.size() << std::endl;

    flatbuffers::Offset<grl::flatbuffer::KUKAiiwaStates> kukaStates = grl::toFlatBuffer(*fbbP, kukaiiwaStateVec);


    grl::flatbuffer::FinishKUKAiiwaStatesBuffer(*fbbP, kukaStates);

    uint8_t *buf = fbbP->GetBufferPointer();
    std::size_t bufsize = fbbP->GetSize();
    flatbuffers::Verifier verifier(buf,fbbP->GetSize());
    std::cout<< "VerifyKUKAiiwaStatesBuffer: " << grl::flatbuffer::VerifyKUKAiiwaStatesBuffer(verifier) << std::endl;
    std::string binary_file_path = "Kuka_test_binary.iiwa";
    std::string json_file_path = "kuka_test_text.json";


    OK = flatbuffers::SaveFile(binary_file_path.c_str(), reinterpret_cast<const char *>(buf), bufsize, true);

    // Get the current working directory
    std::string fbs_filename("KUKAiiwa.fbs");
    flatbuffers::Parser parser;
    std::vector<std::string> includePaths;

    OK = OK && grl::ParseSchemaFile(parser, fbs_filename, includePaths, false);
    std::string jsongen;
      // now generate text from the flatbuffer binary

    OK = OK && GenerateText(parser, buf, &jsongen);
      // Write the data get from flatbuffer binary to json file on disk.
    std::cout << "buffer :" << (unsigned)strlen(reinterpret_cast<const char *>(buf)) << std::endl;
      std::ofstream out(json_file_path);
      out << jsongen.c_str();
      out.close();

      std::cout << jsongen.c_str() << std::endl;
    // OK = OK && grl::SaveFlatBufferFile(
    //     buf,
    //     fbbP->GetSize(),
    //     binary_file_path,
    //     fbs_filename,
    //     json_file_path);
    std::cout << "Save json file correctly? " << OK << "  Buffer size saved to binary file: " << bufsize << std::endl;
    std::cout << "End of the program" << std::endl;



}



BOOST_AUTO_TEST_SUITE_END()
