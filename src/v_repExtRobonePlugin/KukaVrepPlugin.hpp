#ifndef KUKA_VREP_PLUGIN_HPP_
#define KUKA_VREP_PLUGIN_HPP_

#include <iostream>
#include <memory>

#include <boost/log/trivial.hpp>

#include "robone/KukaFRIThreadSeparator.hpp"
#include "robone/AzmqFlatbuffer.hpp"
#include "robone/flatbuffer/JointState_generated.h"

#include "v_repLib.h"

/// @todo separate out robone specific code from general kuka control code
/// @todo Template on robot driver and create a driver that just reads/writes to/from the simulation, then pass the two templates so the simulation and the real driver can be selected.
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
        ImplantCutPathName,
        RemoveBallJointPathName,
        FemurBoneName,
        LocalZMQAddress,
        RemoteZMQAddress
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
        std::string
        > Params;
    
    
    static const Params defaultParams(){
        return std::make_tuple(
                    "LBR_iiwa_14_R820_joint1", // Joint1Handle, 
                    "LBR_iiwa_14_R820_joint2", // Joint2Handle, 
                    "LBR_iiwa_14_R820_joint3", // Joint3Handle, 
                    "LBR_iiwa_14_R820_joint4", // Joint4Handle, 
                    "LBR_iiwa_14_R820_joint5", // Joint5Handle, 
                    "LBR_iiwa_14_R820_joint6", // Joint6Handle, 
                    "LBR_iiwa_14_R820_joint7", // Joint7Handle,
                    "RobotTip#0"             , // RobotTipHandle,
                    "RobotTarget#0"          , // RobotTargetHandle,
                    "Robotiiwa"              , // RobotTargetBaseHandle,
                    "ImplantCutPath"         , // ImplantCutPathHandle,
                    "RemoveBallJoint"        , // RemoveBallJointPathHandle,
                    "FemurBone"              , // FemurBoneHandle
                    "tcp://0.0.0.0:5563"     , // LocalZMQAddress
                    "tcp://172.31.1.147:5563"  // RemoteZMQAddress
                );
    }

/// @todo allow KukaFRIThreadSeparator parameters to be updated
KukaVrepPlugin (Params params = defaultParams())
      :
      kukaFRIThreadSeparatorP(new robone::KukaFRIThreadSeparator(device_driver_io_service)),
      params_(params)
{
  initHandles();
  
  	{
		boost::system::error_code ec;
		azmq::socket socket(device_driver_io_service, ZMQ_DEALER);
		socket.bind(   std::get<LocalZMQAddress>             (params_).c_str()   );
		socket.connect(std::get<RemoteZMQAddress>            (params_).c_str()   );
		kukaJavaDriverP = std::make_shared<AzmqFlatbuffer>(std::move(socket));
	}
    
        // start up the driver thread
        /// @todo perhaps allow user to control this?
        driver_threadP.reset(new std::thread([&]{ device_driver_io_service.run(); }));
}


void run_one(){

  if(!allHandlesSet) BOOST_THROW_EXCEPTION(std::runtime_error("KukaVrepPlugin: Handles have not been initialized, cannot run updates."));
  getRealKukaAngles();
  getStateFromVrep();
  updateVrepFromKuka();
  sendSimulatedJointAnglesToKuka();

}

private:



/// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
/// @todo throw an exception if any of the handles is -1
void initHandles() {

	jointHandle[0] = simGetObjectHandle(std::get<Joint1Name>             (params_).c_str()   );	//Obtain Joint Handles
	jointHandle[1] = simGetObjectHandle(std::get<Joint2Name>             (params_).c_str()   );
	jointHandle[2] = simGetObjectHandle(std::get<Joint3Name>             (params_).c_str()   );
	jointHandle[3] = simGetObjectHandle(std::get<Joint4Name>             (params_).c_str()   );
	jointHandle[4] = simGetObjectHandle(std::get<Joint5Name>             (params_).c_str()   );
	jointHandle[5] = simGetObjectHandle(std::get<Joint6Name>             (params_).c_str()   );
	jointHandle[6] = simGetObjectHandle(std::get<Joint7Name>             (params_).c_str()   );
	robotTip       = simGetObjectHandle(std::get<RobotTipName>           (params_).c_str()   );					//Obtain RobotTip handle
	target         = simGetObjectHandle(std::get<RobotTargetName>        (params_).c_str()   );
	targetBase     = simGetObjectHandle(std::get<RobotTargetBaseName>    (params_).c_str()   );
	ImplantCutPath = simGetObjectHandle(std::get<ImplantCutPathName>     (params_).c_str()   );
	BallJointPath  = simGetObjectHandle(std::get<RemoveBallJointPathName>(params_).c_str()   );
	bone           = simGetObjectHandle(std::get<FemurBoneName>          (params_).c_str()   );
	allHandlesSet  = true;
}

void getRealKukaAngles() {
        BOOST_VERIFY(kukaFRIThreadSeparatorP);

        kukaFRIThreadSeparatorP->async_getLatestState([this](std::shared_ptr<robone::robot::arm::kuka::iiwa::MonitorState> updatedState){
        
            // We have the real kuka state read from the device now
            // update real joint angle data
            realJointPosition.clear();
            robone::robot::arm::copy(updatedState->get(), std::back_inserter(realJointPosition), robone::revolute_joint_angle_open_chain_state_tag());
            
            
            realJointForce.clear();
            robone::robot::arm::copy(updatedState->get(), std::back_inserter(realJointForce), robone::revolute_joint_torque_open_chain_state_tag());
            
            realJointPosition.clear();
            robone::robot::arm::copy(updatedState->get(), std::back_inserter(realJointPosition), robone::revolute_joint_angle_open_chain_state_tag());
            
            
            // here we expect the simulation to be slightly ahead of the arm
            // so we get the simulation based joint angles and update the arm
            
       });

       // run the async calls and update the state
       kukaFRIThreadSeparatorP->run_user();


}

void sendSimulatedJointAnglesToKuka(){

        BOOST_VERIFY(kukaFRIThreadSeparatorP);
    
/// @todo make this handled by template driver implementations/extensions

        if(kukaJavaDriverP){
            /////////////////////////////////////////
            // Client sends to server asynchronously!
            
            /// @todo if allocation is a performance problem use boost::container::static_vector<double,7>
            std::vector<double> joints;
            
			auto fbbP = kukaJavaDriverP->GetUnusedBufferBuilder();
			
            /// @todo should we use simJointTargetPosition here?
            joints.clear();
            boost::copy(simJointPosition, std::back_inserter(joints));
            auto jointPos = fbbP->CreateVector(&joints[0], joints.size());
            /// @note we don't have a velocity right now, sending empty!
            joints.clear();
            //boost::copy(simJointVelocity, std::back_inserter(joints));
            auto jointVel = fbbP->CreateVector(&joints[0], joints.size());
            joints.clear();
            boost::copy(simJointForce, std::back_inserter(joints));
            auto jointAccel = fbbP->CreateVector(&joints[0], joints.size());
			auto jointState = robone::CreateJointState(*fbbP,jointPos,jointVel,jointAccel);
			robone::FinishJointStateBuffer(*fbbP, jointState);
			kukaJavaDriverP->async_send_flatbuffer(fbbP);
            
        } else {
            // create the command for the FRI
            auto commandP = std::make_shared<robone::robot::arm::kuka::iiwa::CommandState>();
            // Set the FRI to the simulated joint positions
            robone::robot::arm::set(*commandP, simJointPosition, robone::revolute_joint_angle_open_chain_command_tag());
            // send the command
            this->kukaFRIThreadSeparatorP->async_sendCommand(commandP);
        }
    
        kukaFRIThreadSeparatorP->run_user();
}

/// @todo if there aren't real limits set via the kuka model already then implement me
void setArmLimits(){
			//simSetJointInterval(simInt objectHandle,simBool cyclic,const simFloat* interval); //Sets the interval parameters of a joint (i.e. range values)
			//simSetJointMode(simInt jointHandle,simInt jointMode,simInt options); //Sets the operation mode of a joint. Might have as side-effect the change of additional properties of the joint

}

bool getStateFromVrep(){
            if(!allHandlesSet) return false;
			
			for (int i=0 ; i < KUKA::LBRState::NUM_DOF ; i++)
			{	
				simGetJointPosition(jointHandle[i],&simJointPosition[i]);  //retrieves the intrinsic position of a joint (Angle for revolute joint)
				simGetJointForce(jointHandle[i],&simJointForce[i]);	//retrieves the force or torque applied to a joint along/about its active axis. This function retrieves meaningful information only if the joint is prismatic or revolute, and is dynamically enabled. 
				simGetJointTargetPosition(jointHandle[i],&simJointTargetPosition[i]);  //retrieves the target position of a joint
				simGetJointMatrix(jointHandle[i],&simJointTransformationMatrix[i]);   //retrieves the intrinsic transformation matrix of a joint (the transformation caused by the joint movement)
				BOOST_LOG_TRIVIAL(info) << "simJointPostition[" << i << "] = " << simJointPosition[i] << std::endl;
				BOOST_LOG_TRIVIAL(info) << "simJointForce[" << i << "] = " << simJointForce[i] << std::endl;
				BOOST_LOG_TRIVIAL(info) << "simJointTargetPostition[" << i << "] = " << simJointTargetPosition[i] << std::endl;
				BOOST_LOG_TRIVIAL(info) << "simJointTransformationMatrix[" << i << "] = " << simJointTransformationMatrix[i] << std::endl;
				BOOST_LOG_TRIVIAL(info) << std::endl;

			}
			
			float simTipPosition[3];
			float simTipOrientation[3];

			simGetObjectPosition(target, targetBase, simTipPosition);
			simGetObjectOrientation(target, targetBase, simTipOrientation);
			
			for (int i = 0 ; i < 3 ; i++)
			{
				BOOST_LOG_TRIVIAL(info) << "simTipPosition[" << i << "] = " << simTipPosition[i] << std::endl;
				BOOST_LOG_TRIVIAL(info) << "simTipOrientation[" << i <<  "] = " << simTipOrientation[i] << std::endl;

			}
			// Send updated position to the real arm based on simulation
            return true;
}

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
				simSetJointPosition(jointHandle[i],realJointPosition[i]); //Sets the intrinsic position of a joint. May have no effect depending on the joint mode
            
            
				//simSetJointTargetPosition(jointHandle[i],realJointTargetPosition[i]);  //Sets the target position of a joint if the joint is in torque/force mode (also make sure that the joint's motor and position control are enabled
            
				simSetJointForce(jointHandle[i],realJointForce[i]);  //Sets the maximum force or torque that a joint can exert. This function has no effect when the joint is not dynamically enabled
            
            
				//simSetJointTargetVelocity(jointHandle[i],realJointTargetVelocity[i]);  //Sets the intrinsic target velocity of a non-spherical joint. This command makes only sense when the joint mode is: (a) motion mode: the joint's motion handling feature must be enabled (simHandleJoint must be called (is called by default in the main script), and the joint motion properties must be set in the joint settings dialog), (b) torque/force mode: the dynamics functionality and the joint motor have to be enabled (position control should however be disabled)
			}
    
            return true;
}


std::vector<int> jointHandle = {-1,-1,-1,-1,-1,-1,-1};	//global variables defined
int robotTip = -1;					//Obtain RobotTip handle
int target = -1;
int targetBase = -1;
int ImplantCutPath = -1;
int BallJointPath = -1;
int bone = -1;

bool allHandlesSet = false;

boost::asio::io_service vrep_owned_io_service;
boost::asio::io_service device_driver_io_service;
std::unique_ptr<std::thread> driver_threadP;
std::shared_ptr<robone::KukaFRIThreadSeparator> kukaFRIThreadSeparatorP;
std::shared_ptr<AzmqFlatbuffer> kukaJavaDriverP;

std::vector<float> simJointPosition = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
// std::vector<float> simJointVelocity = {0.0,0.0,0.0,0.0,0.0,0.0,0.0}; no velocity yet
std::vector<float> simJointForce = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<float> simJointTargetPosition = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<float> simJointTransformationMatrix = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};

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

#endif