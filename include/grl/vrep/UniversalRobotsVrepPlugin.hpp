#ifndef UniversalRobots_VREP_PLUGIN_HPP_
#define UniversalRobots_VREP_PLUGIN_HPP_

#include <iostream>
#include <memory>
#include <array>

#include <boost/range/algorithm/copy.hpp>
#include <boost/asio.hpp>
#include <boost/log/trivial.hpp>
#include <boost/exception/all.hpp>
#include <boost/algorithm/string.hpp>
#include <ur_modern_driver/ur_hardware_interface_standalone.h>

#include "grl/vrep/Vrep.hpp"
#include "grl/vrep/VrepRobotArmDriver.hpp"

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
///    auto UniversalRobotsPluginPG = std::make_shared<grl::KukaVrepPlugin>();
///    kukaPluginPG->construct();
///    while(true) kukaPluginPG->run_one();
/// @endcode
///
/// @todo this implementation is a bit hacky, redesign it
/// @todo separate out grl specific code from general kuka control code
/// @todo Template on robot driver and create a driver that just reads/writes to/from the simulation, then pass the two templates so the simulation and the real driver can be selected.
///
class UniversalRobotsVrepPlugin : public std::enable_shared_from_this<UniversalRobotsVrepPlugin> {
public:

    enum ParamIndex {
        Joint1Name, 
        Joint2Name, 
        Joint3Name, 
        Joint4Name, 
        Joint5Name, 
        Joint6Name, 
        Joint7Name,
        RobotFlangeTipName,
        RobotTipName,
        RobotTargetName,
        RobotTargetBaseName,
        LocalZMQAddress,
        RemoteZMQAddress,
        LocalHostUniversalRobotsKoniUDPAddress,
        LocalHostUniversalRobotsKoniUDPPort,
        RemoteHostUniversalRobotsKoniUDPAddress,
        RemoteHostUniversalRobotsKoniUDPPort,
        UniversalRobotsCommandMode,
        RobotIKGroup
    };
    
    /// @todo allow default params
    typedef std::tuple<
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
        return std::make_tuple(
                    "LBR_iiwa_14_R820_joint1" , // Joint1Handle, 
                    "LBR_iiwa_14_R820_joint2" , // Joint2Handle, 
                    "LBR_iiwa_14_R820_joint3" , // Joint3Handle, 
                    "LBR_iiwa_14_R820_joint4" , // Joint4Handle, 
                    "LBR_iiwa_14_R820_joint5" , // Joint5Handle, 
                    "LBR_iiwa_14_R820_joint6" , // Joint6Handle, 
                    "LBR_iiwa_14_R820_joint7" , // Joint7Handle,
                    "RobotFlangeTip"          , // RobotFlangeTipHandle,
                    "RobotMillTip"            , // RobotTipHandle,
                    "RobotMillTipTarget"      , // RobotTargetHandle,
                    "Robotiiwa"               , // RobotTargetBaseHandle,
                    "tcp://0.0.0.0:30010"     , // LocalZMQAddress
                    "tcp://172.31.1.147:30010", // RemoteZMQAddress
                    "192.170.10.100"          , // LocalHostUniversalRobotsKoniUDPAddress,
                    "30200"                   , // LocalHostUniversalRobotsKoniUDPPort,
                    "192.170.10.2"            , // RemoteHostUniversalRobotsKoniUDPAddress,
                    "30200"                   , // RemoteHostUniversalRobotsKoniUDPPort
                    "JAVA"                    , // UniversalRobotsCommandMode (options are FRI, JAVA)
                    "IK_Group1_iiwa"
                );
    }





/// @todo allow UniversalRobotsFRIThreadSeparator parameters to be updated
/// @param params a tuple containing all of the parameter strings needed to configure the device.
///
/// The UniversalRobotsCommandMode parameters supports the options "FRI" and "JAVA". This configures how commands will
/// be sent to the arm itself. "FRI" mode is via a direct "Fast Robot Interface" "UniversalRobots KONI"
/// ethernet connection which provides substantially higher performance and response time,
//  but is extremely sensitive to delays, and any delay will halt the robot and require a manual reset.
//  "JAVA" mode sends the command to the Java application installed on the UniversalRobots robot, which then submits
//  it to the arm itself to execute. This is a much more forgiving mode of communication, but it is subject to delays.
UniversalRobotsVrepPlugin (Params params = defaultParams())
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
  
  
  UniversalRobotsDriverP_.reset(new ur::UrHardwareInterfaceStandalone(std::string(std::get<LocalHostUniversalRobotsKoniUDPAddress >        (params))));
  initHandles();
}


void run_one(){

  if(!allHandlesSet) BOOST_THROW_EXCEPTION(std::runtime_error("UniversalRobotsVrepPlugin: Handles have not been initialized, cannot run updates."));
  getRealUniversalRobotsAngles();
  bool isError = getStateFromVrep(); // true if there is an error
  allHandlesSet = !isError;
  /// @todo re-enable simulation feedback based on actual UniversalRobots state
  //updateVrepFromUniversalRobots();
  sendSimulatedJointAnglesToUniversalRobots();

}


~UniversalRobotsVrepPlugin(){
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
            std::vector<std::string>
            {
                std::get<Joint1Name>             (params_),
                std::get<Joint2Name>             (params_),
                std::get<Joint3Name>             (params_),
                std::get<Joint4Name>             (params_),
                std::get<Joint5Name>             (params_),
                std::get<Joint6Name>             (params_),
                std::get<Joint7Name>             (params_)
            },
            std::get<RobotFlangeTipName>     (params_),
            std::get<RobotTipName>           (params_),
            std::get<RobotTargetName>        (params_),
            std::get<RobotTargetBaseName>    (params_),
            std::get<RobotIKGroup>           (params_)
        )
    );
    vrepRobotArmDriverP_->construct();
    /// @todo remove this assumption
	allHandlesSet  = true;
}

void getRealUniversalRobotsAngles() {
      /// @todo m_haveReceivedRealData = true is a DANGEROUS HACK!!!! REMOVE ME AFTER DEBUGGING
      m_haveReceivedRealData = true;
    if(UniversalRobotsDriverP_)
    {
      std::vector<double> vals;
      UniversalRobotsDriverP_->read_joint_position(vals);
      realJointPosition.clear();
      boost::copy(vals,std::back_inserter(realJointPosition));
      
      UniversalRobotsDriverP_->read_joint_velocity(vals);
      realJointPosition.clear();
      boost::copy(vals,std::back_inserter(realJointVelocity));
    }
        
#if BOOST_VERSION < 105900
        // here we expect the simulation to be slightly ahead of the arm
        // so we get the simulation based joint angles and update the arm
       BOOST_LOG_TRIVIAL(trace) << "Real joint angles from FRI: " << realJointPosition << "\n";
#endif

}

void sendSimulatedJointAnglesToUniversalRobots(){
    
        if(!allHandlesSet || !m_haveReceivedRealData) return;
    
/// @todo make this handled by template driver implementations/extensions

        if( UniversalRobotsDriverP_)
        {
            std::vector<double> dpos;
            boost::copy(simJointPosition,std::back_inserter(dpos));
            UniversalRobotsDriverP_->write_joint_position(dpos);
        }
    
}

/// @todo if there aren't real limits set via the UniversalRobots model already then implement me
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
    
            return false;
}

/// @todo reenable this component
bool updateVrepFromUniversalRobots() {
			// Step 1
			////////////////////////////////////////////////////
			// call the functions here and just print joint angles out
			// or display something on the screens

			///////////////////
			// call our object to get the latest real UniversalRobots state
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
std::unique_ptr<ur::UrHardwareInterfaceStandalone> UniversalRobotsDriverP_;
std::shared_ptr<VrepRobotArmDriver> vrepRobotArmDriverP_;
//boost::asio::deadline_timer sendToJavaDelay;

/// @todo replace all these simJoint elements with simple VrepRobotArmDriver::State
std::vector<float> simJointPosition = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
// std::vector<float> simJointVelocity = {0.0,0.0,0.0,0.0,0.0,0.0,0.0}; no velocity yet
std::vector<float> simJointForce = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<float> simJointTargetPosition = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
VrepRobotArmDriver::TransformationMatrices simJointTransformationMatrix;

/// @note loss of precision! UniversalRobots sends double values, if you write custom code don't use these float values. Vrep uses floats internally which is why they are used here.
std::vector<float> realJointPosition        = { 0, 0, 0, 0, 0, 0, 0 };
// does not exist
// std::vector<float> realJointTargetPosition  = { 0, 0, 0, 0, 0, 0, 0 };
std::vector<float> realJointForce           = { 0, 0, 0, 0, 0, 0, 0 };
std::vector<float> realJointVelocity        = { 0, 0, 0, 0, 0, 0, 0 };
// does not exist yet
//std::vector<float> realJointTargetVelocity  = { 0, 0, 0, 0, 0, 0, 0 };

private:
Params params_;
};
  
}} // grl::vrep

#endif