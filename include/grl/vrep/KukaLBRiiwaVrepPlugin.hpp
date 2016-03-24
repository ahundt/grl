#ifndef KUKA_VREP_PLUGIN_HPP_
#define KUKA_VREP_PLUGIN_HPP_

#include <iostream>
#include <memory>
#include <array>

#include <boost/log/trivial.hpp>
#include <boost/exception/all.hpp>
#include <boost/algorithm/string.hpp>

#include "grl/kuka/KukaDriver.hpp"
#include "grl/vrep/Vrep.hpp"
#include "grl/vrep/VrepRobotArmDriver.hpp"
#include <iterator>
#include "v_repLib.h"

/// @todo move elsewhere, because it will conflict with others' implementations of outputting vectors
template<typename T>
inline boost::log::formatting_ostream& operator<<(boost::log::formatting_ostream& out,  std::vector<T>& v)
{
    out << "[";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last) 
            out << ", ";
    }
    out << "]";
    return out;
}

namespace grl { namespace vrep {




/// Creates a complete vrep plugin object
/// usage:
/// @code
///    auto kukaPluginPG = std::make_shared<grl::KukaVrepPlugin>();
///    kukaPluginPG->construct();
///    while(true) kukaPluginPG->run_one();
/// @endcode
///
/// @todo this implementation is a bit hacky, redesign it
/// @todo separate out grl specific code from general kuka control code
/// @todo Template on robot driver and create a driver that just reads/writes to/from the simulation, then pass the two templates so the simulation and the real driver can be selected.
///
class KukaVrepPlugin : public std::enable_shared_from_this<KukaVrepPlugin> {
public:

    enum ParamIndex {
        JointNames,
        RobotTipName,
        RobotTargetName,
        RobotTargetBaseName,
        LocalZMQAddress,
        RemoteZMQAddress,
        LocalHostKukaKoniUDPAddress,
        LocalHostKukaKoniUDPPort,
        RemoteHostKukaKoniUDPAddress,
        RemoteHostKukaKoniUDPPort,
        KukaCommandMode,
        KukaMonitorMode,
        IKGroupName
    };
    
    typedef std::tuple<
        std::vector<std::string>,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string
        > Params;
    
    
    static const Params defaultParams(){
        std::vector<std::string> jointHandles;
        
        jointHandles.push_back("LBR_iiwa_14_R820_joint1"); // Joint1Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint2"); // Joint2Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint3"); // Joint3Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint4"); // Joint4Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint5"); // Joint5Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint6"); // Joint6Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint7"); // Joint7Handle,
        return std::make_tuple(
                    jointHandles              , // JointHandles,
                    "RobotMillTip"            , // RobotTipHandle,
                    "RobotMillTipTarget"      , // RobotTargetHandle,
                    "Robotiiwa"               , // RobotTargetBaseHandle,
                    "tcp://0.0.0.0:30010"     , // LocalZMQAddress
                    "tcp://172.31.1.147:30010", // RemoteZMQAddress
                    "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
                    "30200"                   , // LocalHostKukaKoniUDPPort,
                    "192.170.10.2"            , // RemoteHostKukaKoniUDPAddress,
                    "30200"                   , // RemoteHostKukaKoniUDPPort
                    "JAVA"                    , // KukaCommandMode (options are FRI, JAVA)
                    "FRI"                     , // KukaMonitorMode (options are FRI, JAVA)
                    "IK_Group1_iiwa"            // IKGroupName
                );
    }
    
    // parameters for measured arm
    static const Params measuredArmParams(){
        std::vector<std::string> jointHandles;
        
        jointHandles.push_back("LBR_iiwa_14_R820_joint1#0"); // Joint1Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint2#0"); // Joint2Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint3#0"); // Joint3Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint4#0"); // Joint4Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint5#0"); // Joint5Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint6#0"); // Joint6Handle,
        jointHandles.push_back("LBR_iiwa_14_R820_joint7#0"); // Joint7Handle,
        return std::make_tuple(
                    jointHandles              , // JointHandles,
                    "RobotMillTip#0"            , // RobotTipHandle,
                    "RobotMillTipTarget#0"      , // RobotTargetHandle,
                    "Robotiiwa#0"               , // RobotTargetBaseHandle,
                    "tcp://0.0.0.0:30010"     , // LocalZMQAddress
                    "tcp://172.31.1.147:30010", // RemoteZMQAddress
                    "tcp://0.0.0.0:30011"     , // LocalConfigZMQAddress
                    "tcp://172.31.1.147:30011", // RemoteConfigZMQAddress
                    "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
                    "30200"                   , // LocalHostKukaKoniUDPPort,
                    "192.170.10.2"            , // RemoteHostKukaKoniUDPAddress,
                    "30200"                   , // RemoteHostKukaKoniUDPPort
                    "JAVA"                    , // KukaCommandMode (options are FRI, JAVA)
                    "FRI"                     , // KukaMonitorMode (options are FRI, JAVA)
                    "IK_Group1_iiwa#0"            // IKGroupName
                );
    }






/// @todo allow KukaFRIThreadSeparator parameters to be updated
/// @param params a tuple containing all of the parameter strings needed to configure the device.
///
/// The KukaCommandMode parameters supports the options "FRI" and "JAVA". This configures how commands will
/// be sent to the arm itself. "FRI" mode is via a direct "Fast Robot Interface" "KUKA KONI"
/// ethernet connection which provides substantially higher performance and response time,
///  but is extremely sensitive to delays, and any delay will halt the robot and require a manual reset.
///  "JAVA" mode sends the command to the Java application installed on the KUKA robot, which then submits
///  it to the arm itself to execute. This is a much more forgiving mode of communication, but it is subject to delays.
///
/// @todo read ms_per_tick from the java interface
KukaVrepPlugin (Params params = defaultParams())
      :
      params_(params)
{
}

void construct(){construct(params_);}

/// construct() function completes initialization of the plugin
/// @todo move this into the actual constructor, but need to correctly handle or attach vrep shared libraries for unit tests.
void construct(Params params){
  params_ = params;
  // keep driver threads from exiting immediately after creation, because they have work to do!
  device_driver_workP_.reset(new boost::asio::io_service::work(device_driver_io_service));
  kukaDriverP_=std::make_shared<robot::arm::KukaDriver>(std::make_tuple(
      
        std::get<RobotTargetBaseName>(params),
        std::get<LocalZMQAddress>(params),
        std::get<RemoteZMQAddress>(params),
        std::get<LocalHostKukaKoniUDPAddress>(params),
        std::get<LocalHostKukaKoniUDPPort>(params),
        std::get<RemoteHostKukaKoniUDPAddress>(params),
        std::get<RemoteHostKukaKoniUDPPort>(params),
        std::get<KukaCommandMode>(params),
        std::get<KukaMonitorMode>(params)

        
  ));
  kukaDriverP_->construct();
  initHandles();
}


void run_one(){

  if(!allHandlesSet) BOOST_THROW_EXCEPTION(std::runtime_error("KukaVrepPlugin: Handles have not been initialized, cannot run updates."));
  getRealKukaAngles();
  bool isError = getStateFromVrep(); // true if there is an error
  allHandlesSet = !isError;
  /// @todo re-enable simulation feedback based on actual kuka state
  syncVrepAndKuka();
}


~KukaVrepPlugin(){
	device_driver_workP_.reset();
    
    if(driver_threadP){
      device_driver_io_service.stop();
      driver_threadP->join();
    }
}

private:



void initHandles() {
    
    vrepRobotArmDriverP_ = std::make_shared<VrepRobotArmDriver>
    (
        std::make_tuple(
            std::get<JointNames>             (params_),
            std::get<RobotTipName>           (params_),
            std::get<RobotTargetName>        (params_),
            std::get<RobotTargetBaseName>    (params_),
            std::get<IKGroupName>            (params_)
        )
    );
    
    
    vrepRobotArmDriverP_->construct();
    
    measuredParams_ = measuredArmParams();
    vrepMeasuredRobotArmDriverP_ = std::make_shared<VrepRobotArmDriver>
    (
        std::make_tuple(
            std::get<JointNames>             (measuredParams_),
            std::get<RobotTipName>           (measuredParams_),
            std::get<RobotTargetName>        (measuredParams_),
            std::get<RobotTargetBaseName>    (measuredParams_),
            std::get<IKGroupName>            (measuredParams_)
        )
    );
    vrepMeasuredRobotArmDriverP_->construct();
    /// @todo remove this assumption
	allHandlesSet  = true;
}

void getRealKukaAngles() {
      /// @todo m_haveReceivedRealData = true is a DANGEROUS HACK!!!! REMOVE ME AFTER DEBUGGING
      m_haveReceivedRealData = true;



}

void syncVrepAndKuka(){
    
        if(!allHandlesSet || !m_haveReceivedRealData) return;
    
        /// @todo make this handled by template driver implementations/extensions
        kukaDriverP_->set( simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());
        if(0) kukaDriverP_->set( simJointForce   , grl::revolute_joint_torque_open_chain_command_tag());        
    
        kukaDriverP_->run_one();
        // We have the real kuka state read from the device now
        // update real joint angle data
        realJointPosition.clear();
        kukaDriverP_->get(std::back_inserter(realJointPosition), grl::revolute_joint_angle_open_chain_state_tag());
        
        realJointForce.clear();
        kukaDriverP_->get(std::back_inserter(realJointForce), grl::revolute_joint_torque_open_chain_state_tag());
    
        realExternalJointTorque.clear();
        kukaDriverP_->get(std::back_inserter(realExternalJointTorque), grl::revolute_joint_torque_external_open_chain_state_tag());
    
        realExternalForce.clear();
        kukaDriverP_->get(std::back_inserter(realExternalForce), grl::cartesian_external_force_tag());
    
    std::cout << "Measured Torque: ";
    std::cout << std::setw(6);
    for (float t:realJointForce) {
        std::cout << t << " ";
    }
    std::cout << '\n';
    
    std::cout << "External Torque: ";
    std::cout << std::setw(6);
    for (float t:realExternalJointTorque) {
        std::cout << t << " ";
    }
    std::cout << '\n';
    
    std::cout << "External Force: ";
    for (float t:realExternalForce) {
        std::cout << t << " ";
    }
    std::cout << '\n';
    
    if(!allHandlesSet || !vrepMeasuredRobotArmDriverP_) return;
    
    VrepRobotArmDriver::State measuredArmState;
    std::get<VrepRobotArmDriver::JointPosition>(measuredArmState) = realJointPosition;
    std::get<VrepRobotArmDriver::JointForce>(measuredArmState) = realJointForce;
    
    vrepMeasuredRobotArmDriverP_->setState(measuredArmState);
    
}

/// @todo if there aren't real limits set via the kuka model already then implement me
void setArmLimits(){
			//simSetJointInterval(simInt objectHandle,simBool cyclic,const simFloat* interval); //Sets the interval parameters of a joint (i.e. range values)
			//simSetJointMode(simInt jointHandle,simInt jointMode,simInt options); //Sets the operation mode of a joint. Might have as side-effect the change of additional properties of the joint

}

/// @return isError - true if there is an error, false if there is no Error
bool getStateFromVrep(){
            if(!allHandlesSet || !vrepRobotArmDriverP_) return false;
			
            VrepRobotArmDriver::State armState;
    
            bool isError = vrepRobotArmDriverP_->getState(armState);
    
            if(isError) return isError;
    
            simJointPosition             = std::get<VrepRobotArmDriver::JointPosition>      (armState);
            simJointForce                = std::get<VrepRobotArmDriver::JointForce>         (armState);
            simJointTargetPosition       = std::get<VrepRobotArmDriver::JointTargetPosition>(armState);
            simJointTransformationMatrix = std::get<VrepRobotArmDriver::JointMatrix>        (armState);
    
//			for (int i=0 ; i < KUKA::LBRState::NUM_DOF ; i++)
//			{	
//				simGetJointPosition(jointHandle[i],&simJointPosition[i]);  //retrieves the intrinsic position of a joint (Angle for revolute joint)
//				simGetJointForce(jointHandle[i],&simJointForce[i]);	//retrieves the force or torque applied to a joint along/about its active axis. This function retrieves meaningful information only if the joint is prismatic or revolute, and is dynamically enabled. 
//				simGetJointTargetPosition(jointHandle[i],&simJointTargetPosition[i]);  //retrieves the target position of a joint
//				simGetJointMatrix(jointHandle[i],&simJointTransformationMatrix[i]);   //retrieves the intrinsic transformation matrix of a joint (the transformation caused by the joint movement)
//
//			}
//            
//			BOOST_LOG_TRIVIAL(info) << "simJointPostition = " << simJointPosition << std::endl;
//			BOOST_LOG_TRIVIAL(info) << "simJointForce = " << simJointForce << std::endl;
//			BOOST_LOG_TRIVIAL(info) << "simJointTargetPostition = " << simJointTargetPosition << std::endl;
//			BOOST_LOG_TRIVIAL(info) << "simJointTransformationMatrix = " << simJointTransformationMatrix << std::endl;
//			
//			float simTipPosition[3];
//			float simTipOrientation[3];
//
//			simGetObjectPosition(target, targetBase, simTipPosition);
//			simGetObjectOrientation(target, targetBase, simTipOrientation);
//			
//			for (int i = 0 ; i < 3 ; i++)
//			{
//				BOOST_LOG_TRIVIAL(info) << "simTipPosition[" << i << "] = " << simTipPosition[i] << std::endl;
//				BOOST_LOG_TRIVIAL(info) << "simTipOrientation[" << i <<  "] = " << simTipOrientation[i] << std::endl;
//
//			}
			// Send updated position to the real arm based on simulation
            return false;
}

/// @todo reenable this component
bool updateVrepFromKuka() {
			// Step 1
			////////////////////////////////////////////////////
			// call the functions here and just print joint angles out
			// or display something on the screens

			///////////////////
			// call our object to get the latest real kuka state
			// then use the functions below to set the simulation state
			// to match

			/////////////////// assuming given real joint position (angles), forces, target position and target velocity 


			// setting the simulation variables to data from real robot (here they have been assumed given)

			for (int i=0 ; i < 7 ; i++)
			{
				//simSetJointPosition(jointHandle[i],realJointPosition[i]); //Sets the intrinsic position of a joint. May have no effect depending on the joint mode
            
            
				//simSetJointTargetPosition(jointHandle[i],realJointTargetPosition[i]);  //Sets the target position of a joint if the joint is in torque/force mode (also make sure that the joint's motor and position control are enabled
            
				//simSetJointForce(jointHandle[i],realJointForce[i]);  //Sets the maximum force or torque that a joint can exert. This function has no effect when the joint is not dynamically enabled
            
            
				//simSetJointTargetVelocity(jointHandle[i],realJointTargetVelocity[i]);  //Sets the intrinsic target velocity of a non-spherical joint. This command makes only sense when the joint mode is: (a) motion mode: the joint's motion handling feature must be enabled (simHandleJoint must be called (is called by default in the main script), and the joint motion properties must be set in the joint settings dialog), (b) torque/force mode: the dynamics functionality and the joint motor have to be enabled (position control should however be disabled)
			}
            return false;
}

volatile bool allHandlesSet = false;
volatile bool m_haveReceivedRealData = false;

boost::asio::io_service device_driver_io_service;
std::unique_ptr<boost::asio::io_service::work> device_driver_workP_;
std::unique_ptr<std::thread> driver_threadP;
std::shared_ptr<grl::robot::arm::KukaDriver> kukaDriverP_;
std::shared_ptr<VrepRobotArmDriver> vrepRobotArmDriverP_;
std::shared_ptr<VrepRobotArmDriver> vrepMeasuredRobotArmDriverP_;
//boost::asio::deadline_timer sendToJavaDelay;

/// @todo replace all these simJoint elements with simple VrepRobotArmDriver::State
std::vector<float> simJointPosition = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
// std::vector<float> simJointVelocity = {0.0,0.0,0.0,0.0,0.0,0.0,0.0}; no velocity yet
std::vector<float> simJointForce = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<float> simJointTargetPosition = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
VrepRobotArmDriver::TransformationMatrices simJointTransformationMatrix;

/// @note loss of precision! kuka sends double values, if you write custom code don't use these float values. Vrep uses floats internally which is why they are used here.
std::vector<float> realJointPosition        = { 0, 0, 0, 0, 0, 0, 0 };
// does not exist
// std::vector<float> realJointTargetPosition  = { 0, 0, 0, 0, 0, 0, 0 };
std::vector<float> realJointForce           = { 0, 0, 0, 0, 0, 0, 0 };
// does not exist yet
//std::vector<float> realJointTargetVelocity  = { 0, 0, 0, 0, 0, 0, 0 };
std::vector<float> realExternalJointTorque  = { 0, 0, 0, 0, 0, 0, 0};
std::vector<float> realExternalForce = { 0, 0, 0, 0, 0, 0};
private:
Params params_;
Params measuredParams_;
/// for use in FRI clientdata mode
std::shared_ptr<KUKA::FRI::ClientData> friData_;
};
  
}} // grl::vrep

#endif
