#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE grlTest

// system includes
#include <boost/test/unit_test.hpp>
#include <exception>
#include <iostream>
#include <vector>

#include "grl/kuka/KukaToFlatbuffer.hpp"
#include "grl/kuka/KukaJAVAdriver.hpp"
#include "grl/kuka/Kuka.hpp"
#include "grl/kuka/KukaDriver.hpp"


BOOST_AUTO_TEST_SUITE(KukaTest)

BOOST_AUTO_TEST_CASE(runRepeatedly)
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
    flatbuffers::Offset<grl::flatbuffer::ArmControlState> controlState;
    grl::flatbuffer::ArmState armControlMode_(grl::flatbuffer::ArmState::StartArm);
    grl::flatbuffer::KUKAiiwaInterface commandInterface_ = grl::flatbuffer::KUKAiiwaInterface::SmartServo;// KUKAiiwaInterface::SmartServo;
    grl::flatbuffer::KUKAiiwaInterface monitorInterface_ = grl::flatbuffer::KUKAiiwaInterface::FRI;
    ::ControlMode controlMode_ = ::ControlMode::ControlMode_POSITION_CONTROLMODE;
    ::ClientCommandMode clientCommandMode = ::ClientCommandMode::ClientCommandMode_POSITION;
    ::OverlayType overlayType = ::OverlayType::OverlayType_NO_OVERLAY;
    ::ConnectionInfo connectionInfo{::FRISessionState::FRISessionState_IDLE,
                                    ::FRIConnectionQuality::FRIConnectionQuality_POOR,
                                    true, (uint32_t)4, false, (uint32_t)0};
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
    controlState = grl::toFlatBuffer(*fbbP, basename, sequenceNumber++, duration, armState, armControlMode_);

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
        controlMode_,
        setCartesianImpedance,
        setJointImpedance,
        setSmartServo,
        FRIConfig,
        tools,
        processData,
        "currentMotionCenter",
        true);
    bool setArmControlState = true; // only actually change the arm state when this is true.
    // ::MessageHeader messageHeader = ::MessageHeader(1,2,3);
    // ::RobotInfo robotInfo = ::RobotInfo(true, 7, true, ::SafetyState::SafetyState_NORMAL_OPERATION)
    // ::FRIMonitoringMessage friMonitoringMessage = ::FRIMonitoringMessage


    // auto friMessageLog = grl::toFlatBuffer(
    //     *fbbP,
    //     connectionInfo.sessionState,
    //     connectionInfo.quality,
    //     controlMode_,
    // );

    // auto kukaiiwastate = flatbuffer::CreateKUKAiiwaState(*fbbP, RobotName, destination, source, duration, controlState, setArmConfiguration_,kukaiiwaArmConfiguration);
    // auto kukaiiwaStateVec = fbbP->CreateVector(&kukaiiwastate, 1);
    // auto states = flatbuffer::CreateKUKAiiwaStates(*fbbP,kukaiiwaStateVec);
    // grl::flatbuffer::FinishKUKAiiwaStatesBuffer(*fbbP, states);

}

BOOST_AUTO_TEST_SUITE_END()
