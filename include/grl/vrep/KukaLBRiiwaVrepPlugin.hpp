#ifndef KUKA_VREP_PLUGIN_HPP_
#define KUKA_VREP_PLUGIN_HPP_

#include <iostream>
#include <memory>
#include <array>

#include <boost/log/trivial.hpp>
#include <boost/exception/all.hpp>
#include <boost/algorithm/string.hpp>

#include "grl/KukaFRIThreadSeparator.hpp"
#include "grl/KukaFriClientData.hpp"
#include "grl/AzmqFlatbuffer.hpp"
#include "grl/flatbuffer/JointState_generated.h"
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
        Joint1Name, 
        Joint2Name, 
        Joint3Name, 
        Joint4Name, 
        Joint5Name, 
        Joint6Name, 
        Joint7Name,
        RobotTipName,
        RobotTargetName,
        RobotTargetBaseName,
        LocalZMQAddress,
        RemoteZMQAddress,
        LocalHostKukaKoniUDPAddress,
        LocalHostKukaKoniUDPPort,
        RemoteHostKukaKoniUDPAddress,
        RemoteHostKukaKoniUDPPort,
        KukaCommandMode
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
                    "RobotMillTip"            , // RobotTipHandle,
                    "RobotMillTipTarget"      , // RobotTargetHandle,
                    "Robotiiwa"               , // RobotTargetBaseHandle,
                    "tcp://0.0.0.0:30010"     , // LocalZMQAddress
                    "tcp://172.31.1.147:30010", // RemoteZMQAddress
                    "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
                    "30200"                   , // LocalHostKukaKoniUDPPort,
                    "192.170.10.2"            , // RemoteHostKukaKoniUDPAddress,
                    "30200"                   , // RemoteHostKukaKoniUDPPort
                    "JAVA"                      // KukaCommandMode (options are FRI, JAVA)
                );
    }





/// @todo allow KukaFRIThreadSeparator parameters to be updated
/// @param params a tuple containing all of the parameter strings needed to configure the device.
///
/// The KukaCommandMode parameters supports the options "FRI" and "JAVA". This configures how commands will
/// be sent to the arm itself. "FRI" mode is via a direct "Fast Robot Interface" "KUKA KONI"
/// ethernet connection which provides substantially higher performance and response time,
//  but is extremely sensitive to delays, and any delay will halt the robot and require a manual reset.
//  "JAVA" mode sends the command to the Java application installed on the KUKA robot, which then submits
//  it to the arm itself to execute. This is a much more forgiving mode of communication, but it is subject to delays.
KukaVrepPlugin (Params params = defaultParams())
      :
      params_(params)
{
/// @todo figure out how to re-enable when .so isn't loaded
 // initHandles();
 if( boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI_ASYNC")))
 {
     kukaFRIThreadSeparatorP.reset(
        new grl::KukaFRIThreadSeparator(
              device_driver_io_service,
              std::make_tuple(
                  std::string(std::get<LocalHostKukaKoniUDPAddress >        (params)),
                  std::string(std::get<LocalHostKukaKoniUDPPort    >        (params)),
                  std::string(std::get<RemoteHostKukaKoniUDPAddress>        (params)),
                  std::string(std::get<RemoteHostKukaKoniUDPPort   >        (params)),
                  grl::KukaFRIThreadSeparator::run_automatically
              )
          ));
  }
  else if( boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI")))
  {
     kukaFRIClientDataDriverP_.reset(
        new grl::robot::arm::KukaFRIClientDataDriver(
              device_driver_io_service,
              std::make_tuple(
                  std::string(std::get<LocalHostKukaKoniUDPAddress >        (params)),
                  std::string(std::get<LocalHostKukaKoniUDPPort    >        (params)),
                  std::string(std::get<RemoteHostKukaKoniUDPAddress>        (params)),
                  std::string(std::get<RemoteHostKukaKoniUDPPort   >        (params)),
                  grl::robot::arm::KukaFRIClientDataDriver::run_automatically
              )
          )
  
     );
  }
  
  	try {
        BOOST_LOG_TRIVIAL(trace) << "connecting zmq\n";
		boost::system::error_code ec;
		azmq::socket socket(device_driver_io_service, ZMQ_DEALER);
		socket.bind(   std::get<LocalZMQAddress>             (params_).c_str()   );
		socket.connect(std::get<RemoteZMQAddress>            (params_).c_str()   );
		kukaJavaDriverP = std::make_shared<AzmqFlatbuffer>(std::move(socket));
    
        // start up the driver thread
        /// @todo perhaps allow user to control this?
        driver_threadP.reset(new std::thread([&]{ device_driver_io_service.run(); }));
	} catch( boost::exception &e) {
            e << errmsg_info("KukaLBRiiwaVrepPlugin: Unable to connect to ZeroMQ Socket from " + 
                               std::get<LocalZMQAddress>             (params_) + " to " + 
                               std::get<RemoteZMQAddress>            (params_));
        throw;
    }
}

/// construct() function completes initialization of the plugin
/// @todo move this into the actual constructor, but need to correctly handle or attach vrep shared libraries for unit tests.
void construct(){
  initHandles();
}


void run_one(){

  if(!allHandlesSet) BOOST_THROW_EXCEPTION(std::runtime_error("KukaVrepPlugin: Handles have not been initialized, cannot run updates."));
  getRealKukaAngles();
  bool isError = getStateFromVrep(); // true if there is an error
  allHandlesSet = !isError;
  /// @todo re-enable simulation feedback based on actual kuka state
  //updateVrepFromKuka();
  sendSimulatedJointAnglesToKuka();

}


~KukaVrepPlugin(){
    if(driver_threadP){
	  //workP.reset();
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
            std::get<RobotTipName>           (params_),
            std::get<RobotTargetName>        (params_),
            std::get<RobotTargetBaseName>    (params_)
        )
    );
    /// @todo remove this assumption
	allHandlesSet  = true;
}

void getRealKukaAngles() {
      /// @todo DANGEROUS HACK!!!! REMOVE ME AFTER DEBUGGING
      m_haveReceivedRealData = true;
    
    if(kukaFRIThreadSeparatorP)
    {
        boost::system::error_code send_ec,recv_ec;
        std::size_t send_bytes, recv_bytes;
        BOOST_VERIFY(kukaFRIThreadSeparatorP);
        std::shared_ptr<grl::robot::arm::kuka::iiwa::MonitorState> updatedState;
        kukaFRIThreadSeparatorP->async_getLatestState(updatedState,recv_ec,recv_bytes,send_ec,send_bytes);
        
        if (updatedState && !recv_ec && !send_ec) {
            this->m_haveReceivedRealData = true;
        } else {
            /// @todo should the results of getlatest state even be possible to call without receiving real data? should the library change?
            // if we didn't actually get anything don't try and update
            return;
        }
        // We have the real kuka state read from the device now
        // update real joint angle data
        realJointPosition.clear();
        grl::robot::arm::copy(updatedState->get(), std::back_inserter(realJointPosition), grl::revolute_joint_angle_open_chain_state_tag());
        
        
        realJointForce.clear();
        grl::robot::arm::copy(updatedState->get(), std::back_inserter(realJointForce), grl::revolute_joint_torque_open_chain_state_tag());
        
        realJointPosition.clear();
        grl::robot::arm::copy(updatedState->get(), std::back_inserter(realJointPosition), grl::revolute_joint_angle_open_chain_state_tag());
        
        
        // here we expect the simulation to be slightly ahead of the arm
        // so we get the simulation based joint angles and update the arm
       
       BOOST_LOG_TRIVIAL(trace) << "Real joint angles from FRI: " << realJointPosition << "\n";
    } else if (kukaFRIClientDataDriverP_)
    {
      /// @todo fix this hack, real data hasn't actually been received because send & receive happen simultaneously
      m_haveReceivedRealData = true;
    }



}

void sendSimulatedJointAnglesToKuka(){
    
        if(!allHandlesSet || !m_haveReceivedRealData) return;
    
/// @todo make this handled by template driver implementations/extensions

        if(kukaJavaDriverP && boost::iequals(std::get<KukaCommandMode>(params_),std::string("JAVA")))
        {
            /////////////////////////////////////////
            // Client sends to server asynchronously!
            
           /// @todo if allocation is a performance problem use boost::container::static_vector<double,7>
           std::vector<double> joints;
           
           auto fbbP = kukaJavaDriverP->GetUnusedBufferBuilder();
           
           /// @todo should we use simJointTargetPosition here?
           joints.clear();
           boost::copy(simJointPosition, std::back_inserter(joints));
           auto jointPos = fbbP->CreateVector(&joints[0], joints.size());
           
           BOOST_LOG_TRIVIAL(info) << "sending joint angles: " << joints << " from local zmq: " << std::get<LocalZMQAddress>            (params_) << " to remote zmq: " << std::get<RemoteZMQAddress>            (params_);
           
           /// @note we don't have a velocity right now, sending empty!
           joints.clear();
           //boost::copy(simJointVelocity, std::back_inserter(joints));
           auto jointVel = fbbP->CreateVector(&joints[0], joints.size());
           joints.clear();
           boost::copy(simJointForce, std::back_inserter(joints));
           auto jointAccel = fbbP->CreateVector(&joints[0], joints.size());
           auto jointState = grl::flatbuffer::CreateJointState(*fbbP,jointPos,jointVel,jointAccel);
           grl::flatbuffer::FinishJointStateBuffer(*fbbP, jointState);
           kukaJavaDriverP->async_send_flatbuffer(fbbP);
            
        }
        else if(boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI")))
        {
            // note: this one sends *and* receives the joint data!
            BOOST_VERIFY(kukaFRIClientDataDriverP_.get()!=nullptr);
            std::shared_ptr<KUKA::FRI::ClientData> friData;
            
            // Set the FRI to the simulated joint positions
            grl::robot::arm::set(friData->commandMsg, simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());
            grl::robot::arm::set(friData->commandMsg, simJointForce   , grl::revolute_joint_torque_open_chain_command_tag());
        
        
            boost::system::error_code send_ec,recv_ec;
            std::size_t send_bytes, recv_bytes;
            bool error = kukaFRIClientDataDriverP_->update_state(friData,recv_ec,recv_bytes,send_ec,send_bytes);
        
            if(!error)
            {
              this->m_haveReceivedRealData = true;
            }
            else
            {
               /// @todo should the results of getlatest state even be possible to call without receiving real data? should the library change?
              return;
            }
        
            
            // We have the real kuka state read from the device now
            // update real joint angle data
            realJointPosition.clear();
            grl::robot::arm::copy(friData->monitoringMsg, std::back_inserter(realJointPosition), grl::revolute_joint_angle_open_chain_state_tag());
            
            
            realJointForce.clear();
            grl::robot::arm::copy(friData->monitoringMsg, std::back_inserter(realJointForce), grl::revolute_joint_torque_open_chain_state_tag());
            
            realJointPosition.clear();
            grl::robot::arm::copy(friData->monitoringMsg, std::back_inserter(realJointPosition), grl::revolute_joint_angle_open_chain_state_tag());
        
        
        }
        else if( boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI_ASYNC")))
        {
            BOOST_VERIFY(kukaFRIThreadSeparatorP);
            // create the command for the FRI
            auto commandP = std::make_shared<grl::robot::arm::kuka::iiwa::CommandState>();
            // Set the FRI to the simulated joint positions
            grl::robot::arm::set(*commandP, simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());
            grl::robot::arm::set(*commandP, simJointForce   , grl::revolute_joint_torque_open_chain_command_tag());
            // send the command
            this->kukaFRIThreadSeparatorP->async_sendCommand(commandP);
        } else {
            BOOST_THROW_EXCEPTION(std::runtime_error(std::string("KukaVrepPlugin: Selected KukaCommandMode ")+std::get<KukaCommandMode>(params_)+" does not exist! Options are JAVA, FRI, and FRI_ASYNC"));
        }
    
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
std::unique_ptr<std::thread> driver_threadP;
std::shared_ptr<grl::robot::arm::KukaFRIClientDataDriver> kukaFRIClientDataDriverP_;
std::shared_ptr<grl::KukaFRIThreadSeparator> kukaFRIThreadSeparatorP;
std::shared_ptr<AzmqFlatbuffer> kukaJavaDriverP;
std::shared_ptr<VrepRobotArmDriver> vrepRobotArmDriverP_;
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

private:
Params params_;

};
  
}} // grl::vrep

#endif