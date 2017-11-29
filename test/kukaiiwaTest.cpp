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
    // grl::robot::arm::KukaJAVAdriver::Parms javaDriverP = grl::robot::arm::KukaJAVAdriver::defaultParams();
    // rl::robot::arm::KukaJAVAdriver javaDriver(javaDriverP);
    // javaDriver.construct();
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
    std::string basename = RobotName; //std::get<0>(params);

    flatbuffers::Offset<grl::flatbuffer::ArmControlState> controlState;
    grl::flatbuffer::ArmState armControlMode_(grl::flatbuffer::ArmState::StartArm);

    grl::robot::arm::KukaState armState;
    std::shared_ptr<KUKA::FRI::ClientData> friData(std::make_shared<KUKA::FRI::ClientData>(7));
    cartographer::common::Time startTime;
    double delta = -0.0005;
    double delta_sum = 0;
    /// consider moving joint angles based on time
    int joint_to_move = 6;

    std::vector<double> ipoJointPos(7,0);
    std::vector<double> jointOffset(7,0); // length 7, value 0
    boost::container::static_vector<double, 7> jointStateToCommand(7,0);

    // Absolute goal position to travel to in some modes of HowToMove
    // Set all 7 joints to go to a position 1 radian from the center
    std::vector<double> absoluteGoalPos(7,0.2);
    std::unique_ptr<grl::robot::arm::LinearInterpolation> lowLevelStepAlgorithmP;

    // Need to tell the system how long in milliseconds it has to reach the goal
    // or it will never move!
    std::size_t goal_position_command_time_duration = 4;
    lowLevelStepAlgorithmP.reset(new grl::robot::arm::LinearInterpolation());
    // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800) see grl::robot::arm::KukaState::KUKA_LBR_IIWA_14_R820

    std::shared_ptr<grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation>> highLevelDriverClassP;
    std::shared_ptr<grl::robot::arm::KukaDriver> kukaDriverP;

    kukaDriverP=std::make_shared<grl::robot::arm::KukaDriver>(params);
    kukaDriverP->set(grl::flatbuffer::ArmState::MoveArmJointServo);
    kukaDriverP->set(goal_position_command_time_duration,grl::time_duration_command_tag());
    std::cout << "KUKA COMMAND MODE: " << std::get<grl::robot::arm::KukaDriver::KukaCommandMode>(params) << "\n";
    int64_t sequenceNumber = 0;
    controlState = grl::toFlatBuffer(*fbbP, basename, sequenceNumber++, duration, armState, armControlMode_);

    auto clientCommandMode = grl::flatbuffer::EClientCommandMode::POSITION;
    auto overlayType =  grl::flatbuffer::EOverlayType::NO_OVERLAY;
 //TODO: Custom flatbuffer type. Load defaults from params/config
    //Cartesian Impedance Values
    // grl::flatbuffer::EulerOrder eulerOrder = grl::flatbuffer::EulerOrder::xyz;
    // grl::flatbuffer::Vector3d cart_stiffness_trans_ = grl::flatbuffer::Vector3d(500,500,500);
    // grl::flatbuffer::EulerRotation cart_stifness_rot_ = grl::toFlatBuffer(cart_stiffness_trans_, eulerOrder);

    // grl::flatbuffer::Vector3d cart_damping_trans_ = grl::flatbuffer::Vector3d(0.3,0.3,0.3);
    // grl::flatbuffer::EulerRotation cart_damping_rot_ = grl::toFlatBuffer(cart_damping_trans_, eulerOrder);

    // grl::flatbuffer::EulerPose cart_stiffness_ = grl::toFlatBuffer(cart_stiffness_trans_, cart_stifness_rot_);
    // grl::flatbuffer::EulerPose cart_damping_ = grl::toFlatBuffer(cart_damping_trans_, cart_damping_rot_);

    // grl::flatbuffer::Vector3d cart_max_path_deviation_trans_ = grl::flatbuffer::Vector3d(1000,1000,1000);
    // grl::flatbuffer::Vector3d cart_max_path_deviation_rot_ = grl::flatbuffer::Vector3d(5., 5., 5.);
    // grl::flatbuffer::Vector3d cart_max_ctrl_force_trans_ = grl::flatbuffer::Vector3d(200,200,200);
    // grl::flatbuffer::Vector3d cart_max_ctrl_force_rot_ = grl::flatbuffer::Vector3d(6.3,6.3,6.3);
    // grl::flatbuffer::EulerPose cart_max_path_deviation_  = grl::flatbuffer::EulerPose(cart_max_path_deviation_trans_, grl::toFlatBuffer(cart_max_path_deviation_rot_, eulerOrder));
    // grl::flatbuffer::EulerPose cart_max_ctrl_vel_ = grl::toFlatBuffer(cart_max_path_deviation_trans_, grl::toFlatBuffer(cart_max_ctrl_force_rot_, eulerOrder));
    // grl::flatbuffer::EulerPose cart_max_ctrl_force_ = grl::toFlatBuffer(cart_max_ctrl_force_trans_, grl::toFlatBuffer(cart_max_ctrl_force_trans_, eulerOrder));
    // double nullspace_stiffness_ = 2.0;
    // double nullspace_damping_ = 0.5;




    //auto stiffnessPose  = flatbuffer::CreateEulerPoseParams(*fbbP,&cart_stiffness_trans_,&cart_stiffness_rot_);
    //auto dampingPose  = flatbuffer::CreateEulerPoseParams(*fbbP,&cart_damping_trans_,&cart_damping_rot_)
    // auto setCartesianImpedance = grl::toFlatBuffer(*fbbP,)

    // grl::flatbuffer::CreateCartesianImpedenceControlMode(*fbbP, &cart_stiffness_, &cart_damping_,
    //       nullspace_stiffness_, nullspace_damping_, &cart_max_path_deviation_, &cart_max_ctrl_vel_, &cart_max_ctrl_force_, max_control_force_stop_)
    // auto jointStiffnessBuffer = fbbP->CreateVector(joint_stiffness_.data(),joint_stiffness_.size());
    // auto jointDampingBuffer = fbbP->CreateVector(joint_damping_.data(),joint_damping_.size())
    // auto setJointImpedance = grl::flatbuffer::CreateJointImpedenceControlMode(*fbbP, jointStiffnessBuffer, jointDampingBuffer)
    // auto kukaiiwaArmConfiguration = flatbuffer::CreateKUKAiiwaArmConfiguration(*fbbP,name,commandInterface_,monitorInterface_, clientCommandMode, overlayType,
    //                   controlMode_, setCartesianImpedance, setJointImpedance);



}

BOOST_AUTO_TEST_SUITE_END()
