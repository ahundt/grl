#ifndef _GRL_VREP_ROBOT_ARM_DRIVER_HPP_
#define _GRL_VREP_ROBOT_ARM_DRIVER_HPP_

#include <iostream>
#include <memory>
#include <array>

#include <boost/range.hpp>
#include <boost/log/trivial.hpp>
#include <boost/exception/all.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "grl/vrep/Vrep.hpp"

#include "v_repLib.h"


namespace grl { namespace vrep {


    /// @todo TODO(ahundt)  joint to link and joint to respondable only works for v-rep provided scene heirarchy names, modify full class so these workarounds aren't needed
    /// in other words this assumes everything is named like the following:
    /// joint1:            LBR_iiwa_14_R820_joint1
    /// link1:             LBR_iiwa_14_R820_link1
    /// link1_respondable: LBR_iiwa_14_R820_link1_resp
    ///
    /// That assumption is definitely not true and names can be arbitrary so they must be provided
    /// through the v-rep lua API.
    std::vector<std::string> jointToLink(const std::vector<std::string>& jointNames)
    {
        std::vector<std::string> linkNames;
        for(std::string jointName : jointNames)
        {
            boost::algorithm::replace_last(jointName,"joint","link");
            linkNames.push_back(jointName);
        }
        /// @todo TODO(ahundt) FIX HACK! Manually adding last link
        linkNames.push_back("LBR_iiwa_14_R820_link8");
        return linkNames;
    }
    
    /// @todo TODO(ahundt) HACK joint to link and joint to respondable only works for v-rep provided scene heirarchy names, modify full class so these workarounds aren't needed.
    /// @see jointToLink
    std::vector<std::string> jointToLinkRespondable(std::vector<std::string> jointNames)
    {
        auto linkNames = jointToLink(jointNames);
    
        std::vector<std::string> linkNames_resp;
    
        int i = 1;
        for(std::string linkName : linkNames)
        {
            /// @todo TODO(ahundt) link 1 isn't respondable because it is anchored to the ground, but should there be an empty string so the indexes are consistent or skip it entirely? (currently skipping) 
            if(i!=1)
            {
                boost::algorithm::replace_last(linkName,"link" + boost::lexical_cast<std::string>(i),"link" + boost::lexical_cast<std::string>(i) + "_resp");
                linkNames_resp.push_back(linkName);
            }
            ++i;
        }
    
        return linkNames_resp;
    }

/// @brief C++ interface for any open chain V-REP robot arm
///
/// VrepRobotArmDriver makes it easy to specify and interact with the 
/// joints in a V-REP defined robot arm in a consistent manner. 
///
/// @todo add support for links, particularly the respondable aspects, see LBR_iiwa_14_R820_joint1_resp in RoboneSimulation.ttt
/// @todo write generic getters and setters for this object like in KukaFRIalgorithm.hpp and the member functions of KukaFRIdriver, KukaJAVAdriver
class VrepRobotArmDriver : public std::enable_shared_from_this<VrepRobotArmDriver> {
public:

    enum ParamIndex {
        JointNames,
        RobotFlangeTipName, // the tip of the base robot model without tools, where tools get attached
        RobotTipName,       // the tip of the robot tool or end effector, where stuff interacts
        RobotTargetName,
        RobotTargetBaseName,
        RobotIkGroup
    };
    
    typedef std::tuple<
        std::vector<std::string>,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string
        > Params;
    
    typedef std::tuple<
        std::vector<int>,
        int,
        int,
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
                    "RobotFlangeTip"          , // RobotFlangeTipName,
                    "RobotMillTip"            , // RobotTipName,
                    "RobotMillTipTarget"      , // RobotTargetName,
                    "Robotiiwa"               , // RobotTargetBaseName,
                    "IK_Group1_iiwa"            // RobotIkGroup
                );
    }
    
    static const Params measuredArmParams()
    {
        std::vector<std::string> jointNames{
                    "LBR_iiwa_14_R820_joint1#0" , // Joint1Handle,
                    "LBR_iiwa_14_R820_joint2#0" , // Joint2Handle,
                    "LBR_iiwa_14_R820_joint3#0" , // Joint3Handle,
                    "LBR_iiwa_14_R820_joint4#0" , // Joint4Handle,
                    "LBR_iiwa_14_R820_joint5#0" , // Joint5Handle,
                    "LBR_iiwa_14_R820_joint6#0" , // Joint6Handle,
                    "LBR_iiwa_14_R820_joint7#0"   // Joint7Handle,
                    };
        
        return std::make_tuple(
                    jointNames                , // JointNames
                    "RobotFlangeTip#0"          , // RobotFlangeTipName,
                    "RobotMillTip#0"            , // RobotTipName,
                    "RobotMillTipTarget#0"      , // RobotTargetName,
                    "Robotiiwa#0"               , // RobotTargetBaseName,
                    "IK_Group1_iiwa#0"            // RobotIkGroup
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
        JointStateTagIndex,
        ExternalTorque
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
        JointStateTag,          // JointStateTag unique identifying type so tuple doesn't conflict
        JointScalar             // externalTorque
    > State;
    
    
    VrepRobotArmDriver(Params params = defaultParams())
    : params_(params)
    {
    }
    
/// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
/// @warning getting the ik group is optional, so it does not throw an exception
void construct() {
    std::vector<int> jointHandle;
    getHandleFromParam<JointNames>(params_,std::back_inserter(jointHandle));
    handleParams_ = 
    std::make_tuple(
         std::move(jointHandle)                                 //Obtain Joint Handles
	    ,getHandleFromParam<RobotTipName>           (params_)	//Obtain RobotTip handle
	    ,getHandleFromParam<RobotFlangeTipName>     (params_)	//Obtain RobotTip handle
	    ,getHandleFromParam<RobotTargetName>        (params_)
	    ,getHandleFromParam<RobotTargetBaseName>    (params_)
        ,simGetIkGroupHandle(std::get<RobotIkGroup> (params_).c_str())
    );
    
    /// @todo TODO(ahundt) move these functions/member variables into params_ object, take as parameters from lua!
    linkNames = jointToLink(std::get<JointNames>(params_));
    getHandles(linkNames, std::back_inserter(linkHandles));
    linkRespondableNames = jointToLinkRespondable(std::get<JointNames>(params_));
    // First link is fixed to the ground so can't be respondable
    linkHandles.push_back(-1);
    getHandles(boost::make_iterator_range(linkRespondableNames.begin()+1,linkRespondableNames.end()), std::back_inserter(linkHandles));

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
            double inf = std::numeric_limits<double>::infinity();
			
			for (std::size_t i=0 ; i < jointHandle.size() ; i++)
			{	
                int currentJointHandle = jointHandle[i];
				simGetJointPosition(currentJointHandle,&std::get<JointPosition>(state)[i]);  //retrieves the intrinsic position of a joint (Angle for revolute joint)
				simGetJointForce(currentJointHandle,&std::get<JointForce>(state)[i]);	//retrieves the force or torque applied to a joint along/about its active axis. This function retrieves meaningful information only if the joint is prismatic or revolute, and is dynamically enabled.
				simGetJointTargetPosition(currentJointHandle,&std::get<JointTargetPosition>(state)[i]);  //retrieves the target position of a joint
				simGetJointMatrix(currentJointHandle,&std::get<JointMatrix>(state)[i][0]);   //retrieves the intrinsic transformation matrix of a joint (the transformation caused by the joint movement)
                simGetJointInterval(currentJointHandle,&isCyclic,jointAngleInterval);
                
                /// @todo TODO(ahundt) is always setting infinity if it is cyclic the right thing to do?
                if(isCyclic)
                {
                    std::get<JointLowerPositionLimit>(state)[i] = -inf;
                    std::get<JointUpperPositionLimit>(state)[i] = inf;
                }
                else
                {
                    std::get<JointLowerPositionLimit>(state)[i] = jointAngleInterval[lower];
                    std::get<JointUpperPositionLimit>(state)[i] = jointAngleInterval[upper];
                }

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
            if(!allHandlesSet) return false;
            const std::vector<int>& jointHandle = std::get<JointNames>(handleParams_);
            std::vector<float> realJointPosition = std::get<JointPosition>(state);
            std::vector<float> realJointForce = std::get<JointForce>(state);
            std::vector<float> externalJointForce = std::get<ExternalTorque>(state);

			// setting the simulation variables to data from real robot (here they have been assumed given)

			for (int i=0 ; i < 7 ; i++)
			{
				simSetJointPosition(jointHandle[i],realJointPosition[i]); //Sets the intrinsic position of a joint. May have no effect depending on the joint mode
            
            
				//simSetJointTargetPosition(jointHandle[i],realJointTargetPosition[i]);  //Sets the target position of a joint if the joint is in torque/force mode (also make sure that the joint's motor and position control are enabled
            
				//simSetJointForce(jointHandle[i],realJointForce[i]);  //Sets the maximum force or torque that a joint can exert. This function has no effect when the joint is not dynamically enabled
            
            
				//simSetJointTargetVelocity(jointHandle[i],realJointTargetVelocity[i]);  //Sets the intrinsic target velocity of a non-spherical joint. This command makes only sense when the joint mode is: (a) motion mode: the joint's motion handling feature must be enabled (simHandleJoint must be called (is called by default in the main script), and the joint motion properties must be set in the joint settings dialog), (b) torque/force mode: the dynamics functionality and the joint motor have to be enabled (position control should however be disabled)
			}
    
            if (externalHandlesSet) {
                
            }
    
            else {
                
                for (int i=0 ; i < 7 ; i++) {
                    std::string torqueString = boost::lexical_cast<std::string>(externalJointForce[i]);
                    char * externalTorqueBytes = new char[torqueString.length()+1];
                    std::strcpy(externalTorqueBytes, torqueString.c_str());
                    
                    simAddObjectCustomData(jointHandle[i], externalTorqueHandle, externalTorqueBytes, torqueString.length()+1);
                }
            }
    
            return true;
}



/// @todo deal with !allHandlesSet()
const std::vector<int>& getJointHandles()
{
  return std::get<JointNames>(handleParams_);
}

const std::vector<std::string>& getJointNames()
{
  return std::get<JointNames>(params_);
}


/// @todo deal with !allHandlesSet()
const std::vector<int>& getLinkHandles()
{
  return linkHandles;
}

const std::vector<std::string>& getLinkNames()
{
  return linkNames;
}


/// @todo deal with !allHandlesSet()
const std::vector<int>& getLinkRespondableHandles()
{
  return linkRespondableHandles;
}

const std::vector<std::string>& getLinkRespondableNames()
{
  return linkRespondableNames;
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

/// @todo TODO(ahundt) put these into params or handleparams as appropriate
std::vector<std::string> linkNames;
/// @todo TODO(ahundt) put these into params or handleparams as appropriate
std::vector<std::string> linkRespondableNames;

/// @todo TODO(ahundt) put these into params or handleparams as appropriate
std::vector<int> linkHandles;
/// @todo TODO(ahundt) put these into params or handleparams as appropriate
std::vector<int> linkRespondableHandles;

/// This is a unique identifying handle for external torque values
/// since they are set as V-REP custom data.
/// @see simAddObjectCustomData
int externalTorqueHandle = 310832412;

volatile bool allHandlesSet = false;
volatile bool externalHandlesSet = false;



};




}} // grl::vrep

#endif // _GRL_VREP_ROBOT_ARM_DRIVER_HPP_