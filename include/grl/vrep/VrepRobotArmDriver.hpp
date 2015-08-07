#ifndef _GRL_VREP_ROBOT_ARM_DRIVER_HPP_
#define _GRL_VREP_ROBOT_ARM_DRIVER_HPP_

#include <iostream>
#include <memory>
#include <array>

#include <boost/log/trivial.hpp>
#include <boost/exception/all.hpp>
#include <boost/algorithm/string.hpp>

#include "grl/vrep/Vrep.hpp"

#include "v_repLib.h"


namespace grl { namespace vrep {


class VrepRobotArmDriver : public std::enable_shared_from_this<VrepRobotArmDriver> {
public:

    enum ParamIndex {
        JointNames,
        RobotTipName,
        RobotTargetName,
        RobotTargetBaseName,
        RobotIkGroup
    };
    
    /// @todo allow default params
    typedef std::tuple<
        std::vector<std::string>,
        std::string,
        std::string,
        std::string
        > Params;
    
    typedef std::tuple<
        std::vector<int>,
        int,
        int,
        int
        > VrepHandleParams;
    
    static const Params defaultParams()
    {
        std::vector<std::string> jointNames{
                    "LBR_iiwa_14_R820_joint1" , // Joint1Handle,
                    "LBR_iiwa_14_R820_joint2" , // Joint2Handle, 
                    "LBR_iiwa_14_R820_joint3" , // Joint3Handle, 
                    "LBR_iiwa_14_R820_joint4" , // Joint4Handle, 
                    "LBR_iiwa_14_R820_joint5" , // Joint5Handle, 
                    "LBR_iiwa_14_R820_joint6" , // Joint6Handle, 
                    "LBR_iiwa_14_R820_joint7"   // Joint7Handle,
                    };
        
        return std::make_tuple(
                    jointNames                , // JointNames
                    "RobotMillTip"            , // RobotTipName,
                    "RobotMillTipTarget"      , // RobotTargetName,
                    "Robotiiwa"                 // RobotTargetBaseName,
                );
    }
    
    /// unique tag type so State never
    /// conflicts with a similar tuple
    struct JointStateTag{};
    
    enum JointStateIndex {
        JointPosition,
        JointForce,
        JointTargetPosition,
        JointLowerPositionLimit,
        JointUpperPositionLimit,
        JointMatrix,
        JointStateTagIndex
    };
    
    
    typedef std::vector<float>               JointScalar;
    
    /// @see http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm#simGetJointMatrix for data layout information
    typedef std::array<float,12> TransformationMatrix;
    typedef std::vector<TransformationMatrix> TransformationMatrices;
    
    typedef std::tuple<
        JointScalar,            // jointPosition
    //  JointScalar             // JointVelocity  // no velocity yet
        JointScalar,            // jointForce
        JointScalar,            // jointTargetPosition
        JointScalar,            // JointLowerPositionLimit
        JointScalar,            // JointUpperPositionLimit
        TransformationMatrices, // jointTransformation
        JointStateTag           // JointStateTag unique identifying type so tuple doesn't conflict
    > State;
    
    
    VrepRobotArmDriver(Params params = defaultParams())
    : params_(params)
    {
    }
    
/// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
/// @todo throw an exception if any of the handles is -1
void construct() {
    std::vector<int> jointHandle;
    getHandleFromParam<JointNames>(params_,std::back_inserter(jointHandle));
    handleParams_ =
    std::make_tuple(
         std::move(jointHandle)                                 //Obtain Joint Handles
	    ,getHandleFromParam<RobotTipName>           (params_)	//Obtain RobotTip handle
	    ,getHandleFromParam<RobotTargetName>        (params_)
	    ,getHandleFromParam<RobotTargetBaseName>    (params_)
    );

	allHandlesSet  = true;
}


/// @return 0 if ok 1 if problem
/// @todo handle cyclic joints (see isCyclic below & simGetJointInterval)
bool getState(State& state){
            if(!allHandlesSet) return false;
            const std::vector<int>& jointHandle = std::get<JointNames>(handleParams_);
    
            std::get<JointPosition>             (state).resize(jointHandle.size());
            std::get<JointForce>                (state).resize(jointHandle.size());
            std::get<JointTargetPosition>       (state).resize(jointHandle.size());
            std::get<JointMatrix>               (state).resize(jointHandle.size());
            std::get<JointLowerPositionLimit>   (state).resize(jointHandle.size());
            std::get<JointUpperPositionLimit>   (state).resize(jointHandle.size());
    
            enum limit {
              lower
              ,upper
              ,numLimits
              };
              
            simBool isCyclic;
            float jointAngleInterval[2]; // min,max
			
			for (int i=0 ; i < jointHandle.size() ; i++)
			{	
                int currentJointHandle = jointHandle[i];
				simGetJointPosition(currentJointHandle,&std::get<JointPosition>(state)[i]);  //retrieves the intrinsic position of a joint (Angle for revolute joint)
				simGetJointForce(currentJointHandle,&std::get<JointForce>(state)[i]);	//retrieves the force or torque applied to a joint along/about its active axis. This function retrieves meaningful information only if the joint is prismatic or revolute, and is dynamically enabled.
				simGetJointTargetPosition(currentJointHandle,&std::get<JointTargetPosition>(state)[i]);  //retrieves the target position of a joint
				simGetJointMatrix(currentJointHandle,&std::get<JointMatrix>(state)[i][0]);   //retrieves the intrinsic transformation matrix of a joint (the transformation caused by the joint movement)
                simGetJointInterval(currentJointHandle,&isCyclic,jointAngleInterval);
                std::get<JointLowerPositionLimit>(state)[i] = jointAngleInterval[lower];
                std::get<JointUpperPositionLimit>(state)[i] = jointAngleInterval[upper];

			}
            
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



bool setState(State& state) {
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
    
            return true;
}

/// @todo deal with !allHandlesSet()
const std::vector<int>& getJointHandles()
{
  return std::get<JointNames>(handleParams_);
}

const Params & getParams(){
   return params_;
}

const VrepHandleParams & getVrepHandleParams(){
   return handleParams_;
}

private:

Params params_;
VrepHandleParams handleParams_;

volatile bool allHandlesSet = false;



};




}} // grl::vrep

#endif // _GRL_VREP_ROBOT_ARM_DRIVER_HPP_