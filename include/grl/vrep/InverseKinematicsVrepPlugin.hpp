/// @author Andrew Hundt <athundt@gmail.com>
#ifndef _INVERSE_KINEMATICS_VREP_PLUGIN_
#define _INVERSE_KINEMATICS_VREP_PLUGIN_

/// @todo remove IGNORE_ROTATION or make it a runtime configurable parameter
// #define IGNORE_ROTATION



#include <string>
#include <tuple>
#include <boost/format.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

// SpaceVecAlg
// https://github.com/jrl-umi3218/SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
// https://github.com/jrl-umi3218/RBDyn
#include <RBDyn/Body.h>
#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/IK.h>
#include <RBDyn/FV.h>
#include <RBDyn/ID.h>

// Tasks
// https://github.com/jrl-umi3218/Tasks
#include <Tasks/Tasks.h>
#include <Tasks/Bounds.h>
#include <Tasks/QPConstr.h>
#include <Tasks/QPContactConstr.h>
#include <Tasks/QPMotionConstr.h>
#include <Tasks/QPSolver.h>
#include <Tasks/QPTasks.h>

// grl
#include "grl/vrep/Vrep.hpp"
#include "grl/vrep/VrepRobotArmDriver.hpp"
#include "grl/vrep/VrepRobotArmJacobian.hpp"
#include "grl/vrep/Eigen.hpp"
#include "grl/vrep/SpaceVecAlg.hpp"

#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm/copy.hpp>

namespace grl {
namespace vrep {

/// Sets the VREP simulation joint positions from the RBDyn configuration
///
/// @param vrepJointNames the name of each joint, order must match vrepJointHandles
/// @param vrepJointHandles the V-REP simulation object handle for each joint
/// @param rbdJointNames the names of all the simArmMultiBody joints, which typically has more joints than the VREP list.
/// @param simArmMultiBody defines the structure of the robot arm for the RBDyn library, see RBDyn documentation
/// @param simArmConfig defines the current position of the robot arm for the RBDyn library, see RBDyn documentation
/// @param debug if empty string, no effect, if any other string a trace of the joint angles will be printed to std::cout
void SetVRepArmFromRBDyn(
    std::vector<std::string>& vrepJointNames,
    std::vector<int>& vrepJointHandles,
    std::vector<std::string>& rbdJointNames,
    const rbd::MultiBody& simArmMultiBody,
    const rbd::MultiBodyConfig& simArmConfig,
    std::string debug = "")
{
    std::string str;
    for (std::size_t i = 0; i < vrepJointHandles.size(); ++ i)
    {
        /// @todo TODO(ahundt) FIXME JOINT INDICES ARE OFF BY 1, NOT SETTING FIRST JOINT
        // modify parameters as follows https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257466822
        std::string jointName = vrepJointNames[i];
        std::size_t jointIdx = simArmMultiBody.jointIndexByName(jointName);
        float futureAngle = simArmConfig.q[jointIdx][0];
        std::size_t setIndex = i+1;
        if(setIndex<vrepJointHandles.size()) simSetJointPosition(vrepJointHandles[setIndex],futureAngle);
        /// @todo TODO(ahundt) add torque information
        // simSetJointTargetVelocity(jointHandles_[i],jointAngles_dt[i]/simulationTimeStep);
        // simSetJointTargetPosition(jointHandles_[i],jointAngles_dt[i]);
        // simSetJointTargetPosition(jointHandles_[i],futureAngle);
    
        if(!debug.empty())
        {
             str+=boost::lexical_cast<std::string>(futureAngle);
             if (i<vrepJointHandles.size()-1) str+=", ";
        }
    }
    std::string jointName = rbdJointNames[0]; /// @todo TODO(ahundt) HACK: This gets one joint after what's expected
    std::size_t jointIdx = simArmMultiBody.jointIndexByName(jointName);
    float futureAngle = simArmConfig.q[jointIdx][0];
    simSetJointPosition(vrepJointHandles[0],futureAngle);

    if(!debug.empty())
    {
        str+=boost::lexical_cast<std::string>(futureAngle);
        BOOST_LOG_TRIVIAL(trace) << debug << " jointAngles: " << str;
    }
}

/// This object handles taking data from a V-REP based arm simulation
/// using it to configure an arm constrained optimization algorithm,
/// runs the algorithm, then updates the simulation accordingly.
///
/// The types of problems this can solve include reaching a goal position,
/// applying the desired levels of force to a system, and avoiding obstacles
/// at each time step.
///
/// @see the Tasks libraryhttps://github.com/jrl-umi3218/Tasks for a full set of possible capabilities.
///
/// @todo verify Robotiiwa is the correct base, and RobotMillTipTarget is the right target, because if the transform doesn't match the one in the jacobian the algorithm will break
//template<typename DesiredKinematics = DesiredKinematicsObject>
class InverseKinematicsVrepPlugin
{
public:

    //typedef grl::InverseKinematicsController parent_type;
    
    //using parent_type::currentKinematicsStateP_;
    //using parent_type::parent_type::InitializeKinematicsState;
  
    enum  VrepVFControllerParamsIndex {
        DesiredKinematicsObjectName,
        IKGroupName
    };
    
    typedef std::tuple<
             std::string // DesiredKinematicsObjectName
            ,std::string // IKGroupName
        > Params;
    
    static Params defaultParams()
    {
       return std::make_tuple("RobotMillTipTarget","IK_Group1_iiwa");
    }
    
    /// @todo need to call parent constructor:
        /*! Constructor
    */
    InverseKinematicsVrepPlugin()
    {
    }
    
    /// @todo TODO(ahundt) accept params for both simulated and measured arms
    void construct(Params params = defaultParams()){
        // get kinematics group name
        // get number of joints
        
        // Get the arm that will be used to generate simulated results to command robot
        // the "base" of this ik is Robotiiwa
        VrepRobotArmDriverSimulatedP_ = std::make_shared<vrep::VrepRobotArmDriver>();
        VrepRobotArmDriverSimulatedP_->construct();
        // Get the "Measured" arm that will be set based off of real arm sensor data, from sources like KukaVrepPlugin or ROS
        // in example simulation this ends in #0
        VrepRobotArmDriverMeasuredP_ = std::make_shared<vrep::VrepRobotArmDriver>(vrep::VrepRobotArmDriver::measuredArmParams());
        VrepRobotArmDriverMeasuredP_->construct();
        
        ikGroupHandle_ = simGetIkGroupHandle(std::get<IKGroupName>(params).c_str());
        simSetExplicitHandling(ikGroupHandle_,1); // enable explicit handling for the ik
        
        auto armDriverSimulatedParams = VrepRobotArmDriverSimulatedP_->getParams();
        // the base frame of the arm
        ikGroupBaseName_ = (std::get<VrepRobotArmDriver::RobotTargetBaseName>(armDriverSimulatedParams));
        // the tip of the arm
        ikGroupTipName_ = (std::get<VrepRobotArmDriver::RobotTipName>(armDriverSimulatedParams));
        // the target, or where the tip of the arm should go
        ikGroupTargetName_ = (std::get<VrepRobotArmDriver::RobotTargetName>(armDriverSimulatedParams));
        
        ikGroupBaseHandle_ = grl::vrep::getHandle(ikGroupBaseName_);
        ikGroupTipHandle_ = grl::vrep::getHandle(ikGroupTipName_);
        ikGroupTargetHandle_ = grl::vrep::getHandle(ikGroupTargetName_);
        /// for why this is named as it is see: https://github.com/jrl-umi3218/Tasks/blob/master/tests/arms.h#L34
        sva::PTransform<double> X_base = getObjectPTransform(ikGroupBaseHandle_);
        //X_base = sva::PTransformd(X_base.rotation().inverse(), X_base.translation());
        // start by assuming the base is fixed
        bool isFixed = true;
        bool isForwardJoint = true;
        
        {
        
            jointHandles_ = VrepRobotArmDriverSimulatedP_->getJointHandles();
            jointNames_ = VrepRobotArmDriverSimulatedP_->getJointNames();
            
            linkNames_ =VrepRobotArmDriverSimulatedP_->getLinkNames();
            linkHandles_ =VrepRobotArmDriverSimulatedP_->getLinkHandles();
            linkRespondableNames_ = VrepRobotArmDriverSimulatedP_->getLinkRespondableNames();
            linkRespondableHandles_ = VrepRobotArmDriverSimulatedP_->getLinkRespondableHandles();
        
            // save the initial joint angles for later
            // and set the arm to the 0 position for initialization of the
            // optimizer
            std::vector<float> initialJointAngles;
            for (int handle : jointHandles_) {
               float angle;
               simGetJointPosition(handle,&angle);
               simSetJointPosition(handle,0);
               initialJointAngles.push_back(angle);
            }
        
            rbd_bodyNames_.push_back(ikGroupBaseName_);
            rbd_jointNames_.push_back(ikGroupBaseName_);
            // note: bodyNames are 1 longer than link names, and start with the base!
            /// @todo TODO(ahundt) should 1st parameter be linkNames instead of linkRespondableNames_?
            boost::copy(linkNames_, std::back_inserter(rbd_bodyNames_));
            jointNames_.push_back(ikGroupTipName_);
            boost::copy(jointNames_, std::back_inserter(rbd_jointNames_));
        
            rbd_bodyNames_.push_back(ikGroupTipName_);
            getHandles(rbd_bodyNames_,std::back_inserter(bodyHandles_));
            getHandles(rbd_jointNames_,std::back_inserter(rbd_jointHandles_));
            
            // Note that V-REP specifies full transforms to place objects that rotate joints around the Z axis
            /// @todo TODO(ahundt) this is really fixed
            rbd::Joint j_b_0(rbd::Joint::Rev, Eigen::Vector3d::UnitZ(), isForwardJoint, ikGroupBaseName_);
            //rbd::Joint j_b_0(rbd::Joint::Fixed, isForwardJoint, ikGroupBaseName_);
            rbd_mbg_.addJoint(j_b_0);
        
            // based on: https://github.com/jrl-umi3218/Tasks/blob/master/tests/arms.h#L34
            // and https://github.com/jrl-umi3218/RBDyn/issues/18#issuecomment-257214536
            for(std::size_t i = 1; i < rbd_jointNames_.size(); i++)
            {
            
                // Note that V-REP specifies full transforms to place objects that rotate joints around the Z axis
                std::string thisJointName = rbd_jointNames_[i];
                rbd::Joint j_i(rbd::Joint::Rev, Eigen::Vector3d::UnitZ(), isForwardJoint, thisJointName);
                rbd_mbg_.addJoint(j_i);
            }
            
            // Note that V-REP specifies full transforms to place objects that rotate joints around the Z axis
            /// @todo TODO(ahundt) last "joint" RobotMillTip is really fixed...
            //rbd::Joint j_i(rbd::Joint::Fixed, Eigen::Vector3d::UnitZ(), isForwardJoint, ikGroupTipName_);
            //rbd::Joint j_i(rbd::Joint::Fixed, Eigen::Vector3d::UnitZ(), isForwardJoint, ikGroupTipName_);
            //rbd_mbg_.addJoint(j_i);
            
            
            for(std::size_t i = 0; i < rbd_bodyNames_.size(); i++)
            {
                /// @todo TODO(ahundt) extract real inertia and center of mass from V-REP with http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetShapeMassAndInertia.htm
                
                double mass = 1.;
                auto I = Eigen::Matrix3d::Identity();
                auto h = Eigen::Vector3d::Zero();
                /// @todo TODO(ahundt) consider the origin of the inertia! https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257198604
                sva::RBInertiad rbi_i(mass, h, I);
                /// @todo TODO(ahundt) add real support for links, particularly the respondable aspects, see LBR_iiwa_14_R820_joint1_resp in RoboneSimulation.ttt
                //  @todo TODO(ahundt) REMEMBER: The first joint is NOT respondable!
            
                // remember, bodyNames[0] is the ikGroupBaseName, so entity order is
                // bodyNames[i], joint[i], bodyNames[i+1]
                std::string bodyName(rbd_bodyNames_[i]);
                rbd::Body b_i(rbi_i,bodyName);
            
                rbd_mbg_.addBody(b_i);
            }
            
              std::string dummyName0(("Dummy"+ boost::lexical_cast<std::string>(0+10)));
              int currentDummy0 = simGetObjectHandle(dummyName0.c_str());
              Eigen::Affine3d eto0 = getObjectTransform(rbd_jointHandles_[0],-1);
              BOOST_LOG_TRIVIAL(trace) << dummyName0 << " \n" << eto0.matrix();
              setObjectTransform(currentDummy0,-1,eto0);
        
            for(std::size_t i = 0; i < rbd_bodyNames_.size()-1; i++)
            {
                // note: Tasks takes transforms in the successor (child link) frame, so it is the inverse of v-rep
                //       thus we are getting the current joint in the frame of the next joint
                sva::PTransformd to(getObjectPTransform(rbd_jointHandles_[i+1],rbd_jointHandles_[i]));
                // this should be the identity matrix because we set the joints to 0!
                //sva::PTransformd from(getJointPTransform(rbd_jointHandles_[i+1]));
                sva::PTransformd from(sva::PTransformd::Identity());
            
                // Link the PREVIOUSLY created body to the currently created body with the PREVIOUSLY created joint
                // remember, bodyNames[0] is the ikGroupBaseName, so entity order is
                // bodyNames[i], joint[i], bodyNames[i+1]
                std::string prevBody = rbd_bodyNames_[i];
                std::string curJoint = rbd_jointNames_[i];
                std::string nextBody = rbd_bodyNames_[i+1];
                
                rbd_mbg_.linkBodies(prevBody, to, nextBody, from, curJoint);
            
            }
        
        
            // note: Tasks takes transforms in the successor (child link) frame, so it is the inverse of v-rep
            rbd_mbs_.push_back(rbd_mbg_.makeMultiBody(ikGroupBaseName_,isFixed,X_base));
            rbd_mbcs_.push_back(rbd::MultiBodyConfig(rbd_mbs_[0]));
            rbd_mbcs_[0].zero(rbd_mbs_[0]);
        
            // we only have one robot for the moment so the index of it is 0
            const std::size_t simulatedRobotIndex = 0;
            auto& simArmMultiBody = rbd_mbs_[simulatedRobotIndex];
            auto& simArmConfig = rbd_mbcs_[simulatedRobotIndex];
            
            rbd::forwardKinematics(simArmMultiBody, simArmConfig);
            rbd::forwardVelocity(simArmMultiBody, simArmConfig);
        
            // update the simulated arm position
            SetVRepArmFromRBDyn(jointNames_,jointHandles_,rbd_jointNames_,simArmMultiBody,simArmConfig);
        
            debugFrames();
       
           // may need to invert?
            Eigen::Affine3d tipTf = PTranformToEigenAffine(simArmConfig.bodyPosW[simArmMultiBody.bodyIndexByName(ikGroupTipName_)]);
           setObjectTransform(simGetObjectHandle("Dummy"),-1,tipTf);
        
        }
        
        /// @todo verify object lifetimes
        
        // add virtual fixtures? need names and number of rows
        
        
        /// @todo read objective rows from vrep
        
        /// @todo set vrep explicit handling of IK here, plus unset in destructor of this object
        
    }
    
    
    
    /// Set dummy frames named Dummy0 to Dummy19 to the current state of the
    /// RBDyn and V-REP representations of joints for debugging.
    void debugFrames(bool print = false)
    {
        int prevDummy = -1;
        // we only have one robot for the moment so the index of it is 0
        const std::size_t simulatedRobotIndex = 0;
        auto& simArmMultiBody = rbd_mbs_[simulatedRobotIndex];
        auto& simArmConfig = rbd_mbcs_[simulatedRobotIndex];

        // Debug output
       for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
       {
          Eigen::Affine3d eto;
          std::string dummyName2(("Dummy"+ boost::lexical_cast<std::string>(i+11)));
          int currentDummy2 = simGetObjectHandle(dummyName2.c_str());
          eto = getObjectTransform(jointHandles_[i],-1);
          setObjectTransform(currentDummy2,-1,eto);
          if(print) BOOST_LOG_TRIVIAL(trace) << dummyName2 << " V-REP World\n" << eto.matrix();
          
          Eigen::Affine3d NextJointinPrevFrame(getObjectTransform(jointHandles_[i],jointHandles_[i-1]));
          if(print) BOOST_LOG_TRIVIAL(trace) << dummyName2 << " V-REP JointInPrevFrame\n" << NextJointinPrevFrame.matrix();
       
          bool dummy_world_frame = true;
          if( dummy_world_frame )
          {
              // visualize each joint position
              sva::PTransform<double>     plinkWorld = simArmConfig.bodyPosW[simArmMultiBody.bodyIndexByName(linkNames_[i])];
              Eigen::Affine3d linkWorld = PTranformToEigenAffine(plinkWorld);
              std::string dummyName(("Dummy0"+ boost::lexical_cast<std::string>(i+1)));
              int currentDummy = simGetObjectHandle(dummyName.c_str());
              if(print) BOOST_LOG_TRIVIAL(trace) << dummyName << " RBDyn World\n" << linkWorld.matrix();
              setObjectTransform(currentDummy,-1,linkWorld);
              prevDummy=currentDummy;
              sva::PTransform<double>     plinkToSon = simArmConfig.parentToSon[simArmMultiBody.bodyIndexByName(linkNames_[i])];
              Eigen::Affine3d linkToSon = PTranformToEigenAffine(plinkToSon);
              if(print) BOOST_LOG_TRIVIAL(trace) << dummyName << " RBDyn ParentLinkToSon\n" << linkToSon.matrix();
          
          }
          else
          {
              // visualize each joint position
              sva::PTransform<double>     plinkWorld = simArmConfig.parentToSon[simArmMultiBody.bodyIndexByName(linkNames_[i])];
              Eigen::Affine3d linkWorld = PTranformToEigenAffine(plinkWorld);
              std::string dummyName(("Dummy0"+ boost::lexical_cast<std::string>(i+1)));
              int currentDummy = simGetObjectHandle(dummyName.c_str());
              if(print) BOOST_LOG_TRIVIAL(trace) << dummyName << " RBDyn\n" << linkWorld.matrix();
              setObjectTransform(currentDummy,prevDummy,linkWorld);
              prevDummy=currentDummy;
          
          }
       }
    }
    
    
    
    
    
    
    
    
    
    /// Runs at every simulation time step
    void updateKinematics(){
        
        // Only update kinematics once for debugging purposes. Comment this next line when not debugging
        if(ranOnce_) return;
        ranOnce_ = true;
    
        jointHandles_ = VrepRobotArmDriverSimulatedP_->getJointHandles();
        auto eigentestJacobian=::grl::vrep::getJacobian(*VrepRobotArmDriverSimulatedP_);
        
        
        ///////////////////////////////////////////////////////////
        // Copy Joint Interval, the range of motion for each joint
        
        /// @todo TODO(ahundt) change source of current state based on if physical arm is running
        VrepRobotArmDriverSimulatedP_->getState(currentArmState_);
        
        // lower limits
        auto & llim = std::get<vrep::VrepRobotArmDriver::JointLowerPositionLimit>(currentArmState_);
        std::vector<double> llimits(llim.begin(),llim.end());
        
        // upper limits
        auto & ulim = std::get<vrep::VrepRobotArmDriver::JointUpperPositionLimit>(currentArmState_);
        std::vector<double> ulimits(ulim.begin(),ulim.end());
        
        // current position
        auto & currentJointPos = std::get<vrep::VrepRobotArmDriver::JointPosition>(currentArmState_);
        std::vector<double> currentJointPosVec(currentJointPos.begin(),currentJointPos.end());
        
        
        const auto& handleParams = VrepRobotArmDriverSimulatedP_->getVrepHandleParams();
        Eigen::Affine3d currentEndEffectorPose =
        getObjectTransform(
                             std::get<vrep::VrepRobotArmDriver::RobotTipName>(handleParams)
                            ,std::get<vrep::VrepRobotArmDriver::RobotTargetBaseName>(handleParams)
                          );
        auto  currentEigenT = currentEndEffectorPose.translation();
        
        
        Eigen::Affine3d desiredEndEffectorPose =
        getObjectTransform(
                             std::get<vrep::VrepRobotArmDriver::RobotTargetName>(handleParams)
                            ,std::get<vrep::VrepRobotArmDriver::RobotTargetBaseName>(handleParams)
                          );
        auto  desiredEigenT = desiredEndEffectorPose.translation();
        
        //////////////////////
        /// @todo TODO(ahundt) move code below here back into separate independent setup and solve functions, move some steps like limits to construct()
        
       /// simulation tick time step in float seconds from vrep API call
       float simulationTimeStep = simGetSimulationTimeStep();
       
        // we only have one robot for the moment so the index of it is 0
        const std::size_t simulatedRobotIndex = 0;
        auto& simArmMultiBody = rbd_mbs_[simulatedRobotIndex];
        auto& simArmConfig = rbd_mbcs_[simulatedRobotIndex];
       
    
    
       /// @todo TODO(ahundt) rethink where/when/how to send command for the joint angles. Return to LUA? Set Directly? Distribute via vrep send message command?
       
        ////////////////////////////////////////////////////
        // Set joints to current arm position in simulation
        {
            for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
            {
              /// @todo TODO(ahundt) modify parameters as follows https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257466822

              /// @todo TODO(ahundt) FIXME JOINT INDICES ARE OFF BY 1
              std::string jointName = jointNames_[i];
              std::size_t jointIdx = simArmMultiBody.jointIndexByName(jointName);
              /// @todo TODO(ahundt) GOING ONE PAST THE END HERE?!?!
              if(i<jointHandles_.size()-1) simArmConfig.q[jointIdx][0] = currentJointPosVec[i+1];

            //   float futureAngle = currentAngle + jointAngles_dt[i];
              //simSetJointTargetVelocity(jointHandles_[i],jointAngles_dt[i]/simulationTimeStep);
              //simSetJointTargetPosition(jointHandles_[i],jointAngles_dt[i]);
              //simSetJointTargetPosition(jointHandles_[i],futureAngle);
            //   simSetJointPosition(jointHandles_[i],futureAngle);
            //         str+=boost::lexical_cast<std::string>(jointAngles_dt[i]);
            //         if (i<jointHandles_.size()-1)
            //             str+=", ";
            }
            std::string jointName = rbd_jointNames_[0]; /// @todo TODO(ahundt) HACK: This gets one joint after what's expected
            std::size_t jointIdx = simArmMultiBody.jointIndexByName(jointName);
            simArmConfig.q[jointIdx][0] = currentJointPosVec[0];
        }
        
        rbd::forwardKinematics(simArmMultiBody, simArmConfig);
        rbd::forwardVelocity(simArmMultiBody, simArmConfig);
        

        /// @todo TODO(ahundt) make solver object a member variable if possible, initialize in constructor
        tasks::qp::QPSolver solver;

        int bodyI = simArmMultiBody.bodyIndexByName(ikGroupTipName_);
        
        ////////////////////////////////////////////////////
        // Set position goal of the arm
#if 0
        // go to the real target position
        auto targetWorldTransform = getObjectPTransform(ikGroupTargetHandle_);
        BOOST_LOG_TRIVIAL(trace) << "target translation (vrep format):\n"<< targetWorldTransform.translation();
        tasks::qp::PositionTask posTask(rbd_mbs_, simulatedRobotIndex, ikGroupTipName_,targetWorldTransform.translation());
#else
        // go to a debugging target position
        auto targetWorldTransform = simArmConfig.bodyPosW[simArmMultiBody.bodyIndexByName(ikGroupTipName_)].translation();
        //targetWorldTransform.z() += 0.0001;
        BOOST_LOG_TRIVIAL(trace) << "target translation (rbdyn format):\n"<< targetWorldTransform;
        tasks::qp::PositionTask posTask(rbd_mbs_, simulatedRobotIndex, ikGroupTipName_,targetWorldTransform);

#endif
        tasks::qp::SetPointTask posTaskSp(rbd_mbs_, simulatedRobotIndex, &posTask, 1., 0.1);

        double inf = std::numeric_limits<double>::infinity();
        
       
        ////////////////////////////////////
        // Set joint limits
       
        // joint limit objects, initialize to current q so entries
        // will be reasonable, such as empty entries for fixed joints
        std::vector<std::vector<double> > lBound = simArmConfig.q;
        std::vector<std::vector<double> > uBound = simArmConfig.q;
        
        
        // for all joints
        for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
        {
            /// limits must be organized as described in https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257793242
            std::string jointName = jointNames_[i];
            std::size_t jointIdx = simArmMultiBody.jointIndexByName(jointName);
            lBound[jointIdx][0] = llimits[i];
            uBound[jointIdx][0] = ulimits[i];
        }
        
        tasks::qp::JointLimitsConstr jointConstr(rbd_mbs_, simulatedRobotIndex, {lBound, uBound}, simulationTimeStep);

        // Test add*Constraint
        solver.addBoundConstraint(&jointConstr);
        BOOST_VERIFY(solver.nrBoundConstraints() == 1);
        solver.addConstraint(&jointConstr);
        BOOST_VERIFY(solver.nrConstraints() == 1);

        solver.nrVars(rbd_mbs_, {}, {});
        solver.updateConstrSize();

        solver.addTask(&posTaskSp);
        BOOST_VERIFY(solver.nrTasks() == 1);

        ////////////////////////////////////
        // Run constrained optimization
#if 1
        // use basic inverse kinematics to solve for the position
        rbd::InverseKinematics ik(simArmMultiBody,simArmMultiBody.jointIndexByName(ikGroupTipName_));
        ik.inverseKinematics(simArmMultiBody,simArmConfig,getObjectPTransform(ikGroupTargetHandle_));
        // update the simulated arm position
        SetVRepArmFromRBDyn(jointNames_,jointHandles_,rbd_jointNames_,simArmMultiBody,simArmConfig);
        
        
#elif 0
        // multiple iteration version of solving
        // Test JointLimitsConstr
        /// @todo TODO(ahundt) was this commented correctly?
        //simArmConfig = mbcInit;
        int numSolverIterations = 100;
        double timeStepDividedIntoIterations = simulationTimeStep/numSolverIterations;
        // This actually runs every time step, so only one iteration here, unless we want to subdivide
        // a v-rep time step into smaller rbdyn time steps.
        for(int i = 0; i < numSolverIterations; ++i)
        {
            //BOOST_REQUIRE(solver.solve(rbd_mbs_, rbd_mbcs_));
            solver.solve(rbd_mbs_, rbd_mbcs_);
            // This should be handled by the simulator or physical robot, "forward simulation of dynamics"
            rbd::eulerIntegration(simArmMultiBody, simArmConfig, timeStepDividedIntoIterations);

            rbd::forwardKinematics(simArmMultiBody, simArmConfig);
            rbd::forwardVelocity(simArmMultiBody, simArmConfig);
            //BOOST_REQUIRE_GT(simArmConfig.q[1][simulatedRobotIndex], -cst::pi<double>()/4. - 0.01);
        }
#else
        // single iteration version of solving
        {
            solver.solve(rbd_mbs_, rbd_mbcs_);
            rbd::eulerIntegration(simArmMultiBody, simArmConfig, 0.0001);
            rbd::forwardKinematics(simArmMultiBody, simArmConfig);
            rbd::forwardVelocity(simArmMultiBody, simArmConfig);
            // update the simulated arm position
            SetVRepArmFromRBDyn(jointNames_,jointHandles_,rbd_jointNames_,simArmMultiBody,simArmConfig);
        }
#endif
       debugFrames();
       // may need to invert?
        Eigen::Affine3d tipTf = PTranformToEigenAffine(simArmConfig.bodyPosW[simArmMultiBody.bodyIndexByName(ikGroupTipName_)]);
       setObjectTransform(simGetObjectHandle("Dummy"),-1,tipTf);
    } // end updateKinematics()
    
    /// may not need this it is in the base class
    /// blocking call, call in separate thread, just allocates memory
    void run_one(){
       updateKinematics();
       
    }
    
    
    /// may not need this it is in the base class
    /// this will have output
    /// blocking call, call in separate thread, just allocates memory
    /// this runs the actual optimization algorithm
    void solve(){
        
    }
    
    rbd::MultiBodyGraph                 rbd_mbg_;
    std::vector<rbd::MultiBody>         rbd_mbs_;
    std::vector<rbd::MultiBodyConfig>   rbd_mbcs_;
    std::vector<rbd::Body>              rbd_bodies_;
    std::vector<sva::RBInertia<double>> rbd_inertias_;
    std::vector<rbd::Joint>             rbd_joints_;
    std::vector<std::string>            rbd_bodyNames_;
    std::vector<int>                    bodyHandles_;
    /// rbd "joints" include fixed joints that bridge
    /// various v-rep objects like the world origin and the ik group base.
    /// This is organized a bit differently from the v-rep joint names
    /// because we need additional fixed "joints" so we have transforms
    /// that match the V-REP scene
    std::vector<std::string>            rbd_jointNames_;
    /// rbd "joints" include fixed joints that bridge
    /// various v-rep objects like the world origin and the ik group base.
    /// This is organized a bit differently from the v-rep joint names
    /// because we need additional fixed "joints" so we have transforms
    /// that match the V-REP scene
    std::vector<int>                    rbd_jointHandles_;
    
    
    //tasks::qp::QPSolver qp_solver_;
    
    std::vector<int> jointHandles_; ///< @todo move this item back into VrepRobotArmDriver
    std::vector<int> linkHandles_;
    std::vector<int> linkRespondableHandles_;
    std::vector<std::string> jointNames_;
    std::vector<std::string> linkNames_;
    std::vector<std::string> linkRespondableNames_;
    
    int ikGroupHandle_;
    int ikGroupBaseHandle_;
    int ikGroupTipHandle_;
    int ikGroupTargetHandle_;
    
    std::string ikGroupBaseName_;
    std::string ikGroupTipName_;
    std::string ikGroupTargetName_;
    
    std::shared_ptr<vrep::VrepRobotArmDriver> VrepRobotArmDriverSimulatedP_;
    std::shared_ptr<vrep::VrepRobotArmDriver> VrepRobotArmDriverMeasuredP_;
    vrep::VrepRobotArmDriver::State currentArmState_;
    
    bool ranOnce_ = false;
};

} // namespace vrep
} // namespace grl

#endif