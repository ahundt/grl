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
            std::size_t numJoints = jointHandles_.size();
        
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
        
            /// @todo TODO(ahundt) does any value for mass make sense? it is fixed to the ground
            double mass = 1.;
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            Eigen::Vector3d h = Eigen::Vector3d::Zero();

            sva::RBInertiad rbi_base(mass, h, I);
            rbd::Body baseBody(rbi_base,ikGroupBaseName_);
        
            rbd_mbg_.addBody(baseBody);
            
            // Note that V-REP specifies full transforms to place objects that rotate joints around the Z axis
            rbd::Joint j_b_0(rbd::Joint::Fixed, isForwardJoint, ikGroupBaseName_);
        
            rbd_mbg_.addJoint(j_b_0);
        
            /// @todo TODO(ahundt) consider the origin of the inertia! https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257198604
            sva::RBInertiad rbi_1(mass, h, I);
            // add one extra body so the loop works out cleanly
            std::string bodyName1(rbd_bodyNames_[1]);
            rbd::Body b_1(rbi_1,bodyName1);
            rbd_mbg_.addBody(b_1);
        
            // based on: https://github.com/jrl-umi3218/Tasks/blob/master/tests/arms.h#L34
            // and https://github.com/jrl-umi3218/RBDyn/issues/18#issuecomment-257214536
            for(std::size_t i = 1; i <= numJoints; i++)
            {
            
                // Note that V-REP specifies full transforms to place objects that rotate joints around the Z axis
                rbd::Joint j_i(rbd::Joint::Rev, Eigen::Vector3d::UnitZ(), isForwardJoint, rbd_jointNames_[i]);
                rbd_mbg_.addJoint(j_i);
            
            
                /// @todo TODO(ahundt) extract real inertia and center of mass from V-REP with http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetShapeMassAndInertia.htm
                
                double mass = 1.;
                I = Eigen::Matrix3d::Identity();
                h = Eigen::Vector3d::Zero();
                /// @todo TODO(ahundt) consider the origin of the inertia! https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257198604
                sva::RBInertiad rbi_i(mass, h, I);
                /// @todo TODO(ahundt) add real support for links, particularly the respondable aspects, see LBR_iiwa_14_R820_joint1_resp in RoboneSimulation.ttt
                //  @todo TODO(ahundt) REMEMBER: The first joint is NOT respondable!
            
                // remember, bodyNames[0] is the ikGroupBaseName, so entity order is
                // bodyNames[i], joint[i], bodyNames[i+1]
                std::string bodyName(rbd_bodyNames_[i+1]);
                rbd::Body b_i(rbi_i,bodyName);
            
                rbd_mbg_.addBody(b_i);
            }
            
        
            // add in the tip
            mass = 0;
            I = Eigen::Matrix3d::Identity();
            h = Eigen::Vector3d::Zero();
            /// @todo TODO(ahundt) consider the origin of the inertia! https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257198604
            sva::RBInertiad rbi_tip(mass, h, I);
            // remember, bodyNames[0] is the ikGroupBaseName, so entity order is
            // bodyNames[i], joint[i], bodyNames[i+1]
            rbd::Body b_tip(rbi_tip,ikGroupTipName_);
        
            rbd_mbg_.addBody(b_tip);
            
            // Note that V-REP specifies full transforms to place objects that rotate joints around the Z axis
            rbd::Joint j_i(rbd::Joint::Fixed, Eigen::Vector3d::UnitZ(), isForwardJoint, ikGroupTipName_);
            rbd_mbg_.addJoint(j_i);
            
              std::string dummyName0(("Dummy"+ boost::lexical_cast<std::string>(0+10)));
              int currentDummy0 = simGetObjectHandle(dummyName0.c_str());
              Eigen::Affine3d eto0 = getObjectTransform(rbd_jointHandles_[0],-1);
              BOOST_LOG_TRIVIAL(trace) << dummyName0 << " \n" << eto0.matrix();
              setObjectTransform(currentDummy0,-1,eto0);
        
            for(std::size_t i = 1; i < rbd_bodyNames_.size(); i++)
            {
                // note: Tasks takes transforms in the successor (child link) frame, so it is the inverse of v-rep
                //       thus we are getting the current joint in the frame of the next joint
                sva::PTransformd to(getObjectPTransform(rbd_jointHandles_[i],rbd_jointHandles_[i-1]));
                // this should be the identity matrix because we set the joints to 0!
                sva::PTransformd from(getJointPTransform(rbd_jointHandles_[i]));
            
                // Link the PREVIOUSLY created body to the currently created body with the PREVIOUSLY created joint
                // remember, bodyNames[0] is the ikGroupBaseName, so entity order is
                // bodyNames[i], joint[i], bodyNames[i+1]
                std::string prevBody = rbd_bodyNames_[i-1];
                std::string curBody =  rbd_bodyNames_[i];
                std::string prevJoint = rbd_jointNames_[i-1];
                
                rbd_mbg_.linkBodies(prevBody, to, curBody, from, prevJoint);
            
            
                Eigen::Affine3d eto (PTranformToEigenAffine(to));
//                std::string dummyName((prevBody + prevJoint + curBody + boost::lexical_cast<std::string>(i)));
//                BOOST_LOG_TRIVIAL(trace) << dummyName << " \n" << eto.matrix();
//                BOOST_LOG_TRIVIAL(trace) << dummyName << " eigen only: \n" << getObjectTransform(rbd_jointHandles_[i-1],rbd_jointHandles_[i]).matrix();
            
              std::string dummyName2(("Dummy"+ boost::lexical_cast<std::string>(i+10)));
              int currentDummy2 = simGetObjectHandle(dummyName2.c_str());
              eto = getObjectTransform(rbd_jointHandles_[i],-1);
              BOOST_LOG_TRIVIAL(trace) << dummyName2 << " V-REP\n" << eto.matrix();
              setObjectTransform(currentDummy2,-1,eto);
            
            }
        
        
            // note: Tasks takes transforms in the successor (child link) frame, so it is the inverse of v-rep
            rbd_mbs_.push_back(rbd_mbg_.makeMultiBody(ikGroupBaseName_,isFixed,X_base));
            rbd_mbcs_.push_back(rbd::MultiBodyConfig(rbd_mbs_[0]));
            rbd_mbcs_[0].zero(rbd_mbs_[0]);
        
        
        
        
        
        int prevDummy = -1;
        std::string str;
        std::size_t simulatedRobotIndex = 0;
        
        rbd::forwardKinematics(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);
        rbd::forwardVelocity(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);
        
       for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
       {
          float currentAngle;
          auto ret = simGetJointPosition(jointHandles_[i],&currentAngle);
          BOOST_VERIFY(ret!=-1);
          /// @todo TODO(ahundt) modify parameters as follows https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257466822
          //rbd_mbcs_[0].q[i]={currentAngle};
       
          /// @todo TODO(ahundt) add torque information
          float futureAngle = rbd_mbcs_[simulatedRobotIndex].q[rbd_mbs_[simulatedRobotIndex].jointIndexByName(jointNames_[i])][0];
          //simSetJointTargetVelocity(jointHandles_[i],jointAngles_dt[i]/simulationTimeStep);
          //simSetJointTargetPosition(jointHandles_[i],jointAngles_dt[i]);
          //simSetJointTargetPosition(jointHandles_[i],futureAngle);
           //simSetJointPosition(jointHandles_[i],futureAngle);
                 str+=boost::lexical_cast<std::string>(futureAngle);
                 if (i<jointHandles_.size()-1)
                     str+=", ";
       
          bool dummy_world_frame = true;
          if( dummy_world_frame )
          {
              // visualize each joint position
              sva::PTransform<double>     plinkWorld = rbd_mbcs_[simulatedRobotIndex].bodyPosW[rbd_mbs_[simulatedRobotIndex].bodyIndexByName(linkNames_[i])];
              Eigen::Affine3d linkWorld = PTranformToEigenAffine(plinkWorld);
              std::string dummyName(("Dummy0"+ boost::lexical_cast<std::string>(i+1)));
              int currentDummy = simGetObjectHandle(dummyName.c_str());
              BOOST_LOG_TRIVIAL(trace) << dummyName << " RBDyn\n" << linkWorld.matrix();
              setObjectTransform(currentDummy,-1,linkWorld);
              prevDummy=currentDummy;
          }
          else
          {
              // visualize each joint position
              sva::PTransform<double>     plinkWorld = rbd_mbcs_[simulatedRobotIndex].parentToSon[rbd_mbs_[simulatedRobotIndex].bodyIndexByName(linkNames_[i])];
              Eigen::Affine3d linkWorld = PTranformToEigenAffine(plinkWorld);
              std::string dummyName(("Dummy0"+ boost::lexical_cast<std::string>(i+1)));
              int currentDummy = simGetObjectHandle(dummyName.c_str());
              BOOST_LOG_TRIVIAL(trace) << dummyName << " RBDyn\n" << linkWorld.matrix();
              setObjectTransform(currentDummy,prevDummy,linkWorld);
              prevDummy=currentDummy;
          
          }
       }
       
       
       
       
           /////////////////////////////////////////
           // Put stuff back
       #if 0
           for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
           {
              float currentAngle;
              auto ret = simGetJointPosition(jointHandles_[i],&currentAngle);
              BOOST_VERIFY(ret!=-1);
              /// @todo TODO(ahundt) modify parameters as follows https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257466822
              //rbd_mbcs_[0].q[i]={currentAngle};
              rbd_mbcs_[simulatedRobotIndex].q[rbd_mbs_[simulatedRobotIndex].jointIndexByName(jointNames_[i])][0] = initialJointAngles[i];
            
              /// @todo TODO(ahundt) remove #if 0 after debugging is done
              // set the joints back where they were
              simSetJointPosition(jointHandles_[i],initialJointAngles[i]);
            }
        #endif
        rbd::forwardKinematics(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);
        rbd::forwardVelocity(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);

       
       
       
       // may need to invert?
        Eigen::Affine3d tipTf = PTranformToEigenAffine(rbd_mbcs_[simulatedRobotIndex].bodyPosW[rbd_mbs_[simulatedRobotIndex].bodyIndexByName(ikGroupTipName_)]);
       setObjectTransform(simGetObjectHandle("Dummy"),-1,tipTf);
        BOOST_LOG_TRIVIAL(trace) << "jointAngles: "<< str;
        
        
        
        
        
        
            /// @todo TODO(ahundt) remove #if 0 after debugging is done
            // set the joints back where they were
            #if 0
            for(std::size_t i = 0; i < jointHandles_.size(); ++i)
            {
               simSetJointPosition(jointHandles_[i],initialJointAngles[i]);
            }
            #endif
        }
        
        positionLimitsName = std::get<IKGroupName>(params)+"_PositionLimits";
        
        velocityLimitsName = std::get<IKGroupName>(params)+"_VelocityLimits";
        
        /// @todo verify object lifetimes
        
        // add virtual fixtures? need names and number of rows
        
        
        /// @todo read objective rows from vrep
        
        /// @todo set vrep explicit handling of IK here, plus unset in destructor of this object
        
    }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    /// check out sawConstraintController
    void updateKinematics(){
    
        jointHandles_ = VrepRobotArmDriverSimulatedP_->getJointHandles();
        auto eigentestJacobian=::grl::vrep::getJacobian(*VrepRobotArmDriverSimulatedP_);
        
        
        ///////////////////////////////////////////////////////////
        // Copy Joint Interval, the range of motion for each joint
        
        /// @todo TODO(ahundt) change source of current state based on if physical arm is running
        VrepRobotArmDriverSimulatedP_->getState(currentArmState_);
        
        // lower limits
        auto & llim = std::get<vrep::VrepRobotArmDriver::JointLowerPositionLimit>(currentArmState_);
        std::vector<double> llimits(llim.begin(),llim.end());
        // jointPositionLimitsVFP_->LowerLimits = vctDoubleVec(llimits.size(),&llimits[0]);
        
        // upper limits
        auto & ulim = std::get<vrep::VrepRobotArmDriver::JointUpperPositionLimit>(currentArmState_);
        std::vector<double> ulimits(ulim.begin(),ulim.end());
        // jointPositionLimitsVFP_->UpperLimits = vctDoubleVec(ulimits.size(),&ulimits[0]);
        
        // current position
        auto & currentJointPos = std::get<vrep::VrepRobotArmDriver::JointPosition>(currentArmState_);
        std::vector<double> currentJointPosVec(currentJointPos.begin(),currentJointPos.end());
        // vctDoubleVec vctDoubleVecCurrentJoints(currentJointPosVec.size(),&currentJointPosVec[0]);
        
        // update limits
        /// @todo does this leak memory when called every time around?
        // UpdateJointPosLimitsVF(positionLimitsName,jointPositionLimitsVFP_->UpperLimits,jointPositionLimitsVFP_->LowerLimits,vctDoubleVecCurrentJoints);
        
        
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
        /// @todo move code below here back under run_one updateKinematics() call
        
       /// @todo need to provide tick time in double seconds and get from vrep API call
       float simulationTimeStep = simGetSimulationTimeStep();
       
       
    
        // we only have one robot so the index of it is 0
        const std::size_t simulatedRobotIndex = 0;
       /// @todo: rethink where/when/how to send command for the joint angles. Return to LUA? Set Directly? Distribute via vrep send message command?
       std::vector<std::vector<double>> q_forward_kinematics;
        std::string str;
       // str = "";
       for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
       {
          /// @todo TODO(ahundt) modify parameters as follows https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257466822
          rbd_mbcs_[simulatedRobotIndex].q[rbd_mbs_[simulatedRobotIndex].jointIndexByName(jointNames_[i])] = {currentJointPosVec[i]};
          /// @todo TODO(ahundt) add torque information
       
        //   float futureAngle = currentAngle + jointAngles_dt[i];
          //simSetJointTargetVelocity(jointHandles_[i],jointAngles_dt[i]/simulationTimeStep);
          //simSetJointTargetPosition(jointHandles_[i],jointAngles_dt[i]);
          //simSetJointTargetPosition(jointHandles_[i],futureAngle);
        //   simSetJointPosition(jointHandles_[i],futureAngle);
        //         str+=boost::lexical_cast<std::string>(jointAngles_dt[i]);
        //         if (i<jointHandles_.size()-1)
        //             str+=", ";
       }
        
        rbd::forwardKinematics(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);
        rbd::forwardVelocity(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);
        

        /// @todo TODO(ahundt) make solver object a member variable if possible, initialize in constructor
        tasks::qp::QPSolver solver;

        int bodyI = rbd_mbs_[simulatedRobotIndex].bodyIndexByName(ikGroupTipName_);
        //tasks::qp::PositionTask posTask(rbd_mbs_, simulatedRobotIndex, ikGroupTipName_,desiredEigenT);
        // note: Tasks takes transforms in the successor (child link) frame, so it is the inverse of v-rep
        tasks::qp::PositionTask posTask(rbd_mbs_, simulatedRobotIndex, ikGroupTipName_,getObjectPTransform(ikGroupTargetHandle_).translation());
        tasks::qp::SetPointTask posTaskSp(rbd_mbs_, simulatedRobotIndex, &posTask, 10., 1.);
        BOOST_LOG_TRIVIAL(trace) << "target translation (vrep format):\n"<< getObjectTransform(ikGroupTargetHandle_).translation();

        double inf = std::numeric_limits<double>::infinity();
        
        // joint limit objects
        std::vector<std::vector<double> > lBound;
        std::vector<std::vector<double> > uBound;
        
        /// @todo TODO(ahundt) need base limits?
        //lBound.push_back({llimits[i]});
        //uBound.push_back({ulimits[i]});
        
        // for all joints
        for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
        {
            /// @todo TODO(ahundt) limits must be organized as described in https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257793242
            lBound.push_back({llimits[i]});
            uBound.push_back({ulimits[i]});
        }
        
        /// @todo TODO(ahundt) need static tip limits?
        //lBound.push_back({0});
        //uBound.push_back({0});
        
        //tasks::qp::JointLimitsConstr jointConstr(rbd_mbs_, simulatedRobotIndex, {lBound, uBound}, 0.001);

        // Test add*Constraint
        //solver.addBoundConstraint(&jointConstr);
        //BOOST_VERIFY(solver.nrBoundConstraints() == 1);
        //solver.addConstraint(&jointConstr);
        //BOOST_VERIFY(solver.nrConstraints() == 1);

        solver.nrVars(rbd_mbs_, {}, {});
        solver.updateConstrSize();

        solver.addTask(&posTaskSp);
        BOOST_VERIFY(solver.nrTasks() == 1);

#if 0
        // Test JointLimitsConstr
        /// @todo TODO(ahundt) was this commented correctly?
        //rbd_mbcs_[simulatedRobotIndex] = mbcInit;
        
        // This actually runs every time step, so only one iteration here, unless we want to subdivide
        // a v-rep time step into smaller rbdyn time steps.
        for(int i = 0; i < 1000; ++i)
        {
            //BOOST_REQUIRE(solver.solve(rbd_mbs_, rbd_mbcs_));
            solver.solve(rbd_mbs_, rbd_mbcs_);
            // This should be handled by the simulator or physical robot, "forward simulation of dynamics"
            //rbd::eulerIntegration(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex], simulationTimeStep);

            rbd::forwardKinematics(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);
            rbd::forwardVelocity(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);
            //BOOST_REQUIRE_GT(rbd_mbcs_[simulatedRobotIndex].q[1][simulatedRobotIndex], -cst::pi<double>()/4. - 0.01);
        }
        
        
        int prevDummy = -1;
        
       for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
       {
          float currentAngle;
          auto ret = simGetJointPosition(jointHandles_[i],&currentAngle);
          BOOST_VERIFY(ret!=-1);
          /// @todo TODO(ahundt) modify parameters as follows https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257466822
          //rbd_mbcs_[0].q[i]={currentAngle};
       
          /// @todo TODO(ahundt) add torque information
          float futureAngle = rbd_mbcs_[simulatedRobotIndex].q[rbd_mbs_[simulatedRobotIndex].jointIndexByName(jointNames_[i])][0];
          //simSetJointTargetVelocity(jointHandles_[i],jointAngles_dt[i]/simulationTimeStep);
          //simSetJointTargetPosition(jointHandles_[i],jointAngles_dt[i]);
          //simSetJointTargetPosition(jointHandles_[i],futureAngle);
           simSetJointPosition(jointHandles_[i],futureAngle);
                 str+=boost::lexical_cast<std::string>(futureAngle);
                 if (i<jointHandles_.size()-1)
                     str+=", ";
       
          bool dummy_world_frame = true;
          if( dummy_world_frame )
          {
              // visualize each joint position
              sva::PTransform<double>     plinkWorld = rbd_mbcs_[simulatedRobotIndex].bodyPosW[rbd_mbs_[simulatedRobotIndex].bodyIndexByName(linkNames_[i])];
              Eigen::Affine3d linkWorld = PTranformToEigenAffine(plinkWorld);
              std::string dummyName(("Dummy"+ boost::lexical_cast<std::string>(i)));
              int currentDummy = simGetObjectHandle(dummyName.c_str());
              BOOST_LOG_TRIVIAL(trace) << dummyName << " \n" << linkWorld.matrix();
              setObjectTransform(currentDummy,-1,linkWorld);
              prevDummy=currentDummy;
          }
          else
          {
              // visualize each joint position
              sva::PTransform<double>     plinkWorld = rbd_mbcs_[simulatedRobotIndex].parentToSon[rbd_mbs_[simulatedRobotIndex].bodyIndexByName(linkNames_[i])];
              Eigen::Affine3d linkWorld = PTranformToEigenAffine(plinkWorld);
              std::string dummyName(("Dummy"+ boost::lexical_cast<std::string>(i)));
              int currentDummy = simGetObjectHandle(dummyName.c_str());
              BOOST_LOG_TRIVIAL(trace) << dummyName << " \n" << linkWorld.matrix();
              setObjectTransform(currentDummy,prevDummy,linkWorld);
              prevDummy=currentDummy;
          
          }
       } // end jointHandles for loop
       #endif
       // may need to invert?
        Eigen::Affine3d tipTf = PTranformToEigenAffine(rbd_mbcs_[simulatedRobotIndex].bodyPosW[rbd_mbs_[simulatedRobotIndex].bodyIndexByName(ikGroupTipName_)]);
       setObjectTransform(simGetObjectHandle("Dummy"),-1,tipTf);
        BOOST_LOG_TRIVIAL(trace) << "jointAngles: "<< str;
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
    std::vector<std::string>            rbd_jointNames_;
    /// rbd "joints" include fixed joints that bridge
    /// various v-rep objects like the world origin and the ik group base.
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
    std::string positionLimitsName;
    std::string velocityLimitsName;
};

} // namespace vrep
} // namespace grl

#endif