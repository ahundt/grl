#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE grlTest

// system includes
#include <boost/test/unit_test.hpp>

#include <exception>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <vector>
#include <boost/log/trivial.hpp>
//// local includes
#include "grl/vrep/KukaLBRiiwaVrepPlugin.hpp"

BOOST_AUTO_TEST_SUITE(KukaLBRiiwaVrepPluginTest)

BOOST_AUTO_TEST_CASE(initialization){
    auto plugin = std::make_shared<grl::vrep::KukaVrepPlugin>();
    BOOST_CHECK_THROW (std::make_shared<grl::vrep::KukaVrepPlugin>(),boost::exception);
    plugin->construct(); // shouldn't work, because vrep isn't around to get handles
    // uncomment to test error output
    try {
        // BOOST_LOG_TRIVIAL(info) << "Starting KUKA LBR iiwa plugin connection to Kuka iiwa\n";
        auto kukaVrepPluginPG = std::make_shared<grl::vrep::KukaVrepPlugin>();
        kukaVrepPluginPG->construct();
    } catch (boost::exception& e){
        // log the error and print it to the screen, don't release the exception
        // BOOST_LOG_TRIVIAL(error) <<  boost::diagnostic_information(e);
    }

}

BOOST_AUTO_TEST_CASE(connectToFake){
        std::vector<std::string> jointNames;

        jointNames.push_back("LBR_iiwa_14_R820_joint1"); // Joint1Handle,
        jointNames.push_back("LBR_iiwa_14_R820_joint2"); // Joint2Handle,
        jointNames.push_back("LBR_iiwa_14_R820_joint3"); // Joint3Handle,
        jointNames.push_back("LBR_iiwa_14_R820_joint4"); // Joint4Handle,
        jointNames.push_back("LBR_iiwa_14_R820_joint5"); // Joint5Handle,
        jointNames.push_back("LBR_iiwa_14_R820_joint6"); // Joint6Handle,
        jointNames.push_back("LBR_iiwa_14_R820_joint7"); // Joint7Handle,

        auto config = std::make_tuple(
                    jointNames              , // JointHandles,
                    "RobotFlangeTip"          , // RobotFlangeTipHandle,
                    "RobotMillTip"            , // RobotTipHandle,
                    "RobotMillTipTarget"      , // RobotTargetHandle,
                    "Robotiiwa"               , // RobotTargetBaseHandle,
                    "KUKA_LBR_IIWA_14_R820"   , // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800)
                    "tcp://0.0.0.0"           , // LocalUDPAddress
                    "30010"                   , // LocalUDPPort
                    "172.31.1.147"            , // RemoteUDPAddress
                    "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
                    "30200"                   , // LocalHostKukaKoniUDPPort,
                    "192.170.10.2"            , // RemoteHostKukaKoniUDPAddress,
                    "30200"                   , // RemoteHostKukaKoniUDPPort
                    "JAVA"                    , // KukaCommandMode (options are FRI, JAVA)
                    "FRI"                     , // KukaMonitorMode (options are FRI, JAVA)
                    "IK_Group1_iiwa"            // IKGroupName
                );
        auto plugin = std::make_shared<grl::vrep::KukaVrepPlugin>(config);
        std::shared_ptr<grl::vrep::VrepRobotArmDriver> VrepRobotArmDriverSimulatedP_;
         // Get the arm that will be used to generate simulated results to command robot
        // the "base" of this ik is Robotiiwa
        VrepRobotArmDriverSimulatedP_ = std::make_shared<grl::vrep::VrepRobotArmDriver>();
        VrepRobotArmDriverSimulatedP_->construct();
}

BOOST_AUTO_TEST_SUITE_END()
