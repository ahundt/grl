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
        
            bodyNames_.push_back(ikGroupBaseName_);
            // note: bodyNames are 1 longer than link names, and start with the base!
            /// @todo TODO(ahundt) should 1st parameter be linkNames instead of linkRespondableNames_?
            boost::copy(linkRespondableNames_, std::back_inserter(bodyNames_));
        
            bodyNames_.push_back(ikGroupTipName_);
        
            /// @todo TODO(ahundt) does any value for mass make sense? it is fixed to the ground
            double mass = 1.;
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            Eigen::Vector3d h = Eigen::Vector3d::Zero();

            sva::RBInertiad rbi_base(mass, h, I);
            rbd::Body baseBody(rbi_base,ikGroupBaseName_);
            rbd_mbg_.addBody(baseBody);
        
        
            // based on: https://github.com/jrl-umi3218/Tasks/blob/master/tests/arms.h#L34
            // and https://github.com/jrl-umi3218/RBDyn/issues/18#issuecomment-257214536
            for(std::size_t i = 0; i < numJoints; i++)
            {
                /// @todo TODO(ahundt) extract real inertia and center of mass from V-REP with http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetShapeMassAndInertia.htm
                
                double mass = 1.;
                Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
                Eigen::Vector3d h = Eigen::Vector3d::Zero();
                /// @todo TODO(ahundt) consider the origin of the inertia! https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257198604
                sva::RBInertiad rbi_i(mass, h, I);
                /// @todo TODO(ahundt) add real support for links, particularly the respondable aspects, see LBR_iiwa_14_R820_joint1_resp in RoboneSimulation.ttt
                //  @todo TODO(ahundt) REMEMBER: The first joint is NOT respondable!
            
                // remember, bodyNames[0] is the ikGroupBaseName, so entity order is
                // bodyNames[i], joint[i], bodyNames[i+1]
                rbd::Body b_i(rbi_i,bodyNames_[i+1]);
            
                rbd_mbg_.addBody(b_i);
            
                // Note that V-REP specifies full transforms to place objects that rotate joints around the Z axis
                rbd::Joint j_i(rbd::Joint::Rev, Eigen::Vector3d::UnitZ(), isForwardJoint, jointNames_[i]);
            
                rbd_mbg_.addJoint(j_i);
            
                sva::PTransformd to(getJointPTransform(jointHandles_[i]));
                sva::PTransformd from(sva::PTransformd::Identity());
            
                // remember, bodyNames[0] is the ikGroupBaseName, so entity order is
                // bodyNames[i], joint[i], bodyNames[i+1]
                rbd_mbg_.linkBodies(bodyNames_[i], to, bodyNames_[i+1], from, jointNames_[i]);
            
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
        
            sva::PTransformd to(getObjectPTransform(ikGroupTipHandle_,jointHandles_[numJoints-1]));
            sva::PTransformd from(sva::PTransformd::Identity());
        
            // remember, bodyNames[0] is the ikGroupBaseName, so entity order is
            // bodyNames[i], joint[i], bodyNames[i+1]
            rbd_mbg_.linkBodies(bodyNames_[numJoints], to, bodyNames_[numJoints+1], from, ikGroupTipName_);
        
        
            rbd_mbs_.push_back(rbd_mbg_.makeMultiBody(ikGroupBaseName_,isFixed,X_base));
            rbd_mbcs_.push_back(rbd::MultiBodyConfig(rbd_mbs_[0]));
            rbd_mbcs_[0].zero(rbd_mbs_[0]);
        
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

        /// The row/column major order is swapped between cisst and VREP!
        // this->currentKinematicsStateP_->Jacobian.SetSize(eigentestJacobian.cols(),eigentestJacobian.rows());
        //Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor> > mckp2(this->currentKinematicsStateP_->Jacobian.Pointer(),this->currentKinematicsStateP_->Jacobian.cols(),this->currentKinematicsStateP_->Jacobian.rows());
        //mckp2 = eigentestJacobian.cast<double>();
        
        
        //Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor> > mf(eigentestJacobian,eigentestJacobian.cols(),eigentestJacobian.rows());
        //Eigen::MatrixXf eigenJacobian = mf;
        //Eigen::MatrixXf eigenJacobian = eigentestJacobian;
        
        
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
        // auto& currentCisstT = currentKinematicsStateP_->Frame.Translation();
        // currentCisstT[0] = currentEigenT(0);
        // currentCisstT[1] = currentEigenT(1);
        // currentCisstT[2] = currentEigenT(2);
#ifndef IGNORE_ROTATION
        // Eigen::Map<Eigen::Matrix<double,3,3,Eigen::ColMajor>> ccr(currentKinematicsStateP_->Frame.Rotation().Pointer());
        // ccr = currentEndEffectorPose.rotation();
#endif // IGNORE_ROTATION
        /// @todo set rotation component of current position
        
        
        Eigen::Affine3d desiredEndEffectorPose =
        getObjectTransform(
                             std::get<vrep::VrepRobotArmDriver::RobotTargetName>(handleParams)
                            ,std::get<vrep::VrepRobotArmDriver::RobotTargetBaseName>(handleParams)
                          );
        auto  desiredEigenT = desiredEndEffectorPose.translation();
        // auto& desiredCisstT = desiredKinematicsStateP_->Frame.Translation();
        // desiredCisstT[0] = desiredEigenT(0);
        // desiredCisstT[1] = desiredEigenT(1);
        // desiredCisstT[2] = desiredEigenT(2);
#ifndef IGNORE_ROTATION
        // Eigen::Map<Eigen::Matrix<double,3,3,Eigen::ColMajor>> dcr(desiredKinematicsStateP_->Frame.Rotation().Pointer());
        // dcr = desiredEndEffectorPose.rotation();
#endif // IGNORE_ROTATION
        /// @todo set rotation component of desired position
        
        // for debugging, the translation between the current and desired position in cartesian coordinates
        // auto inputDesired_dx = desiredCisstT - currentCisstT;
        
        // vct3 dx_translation, dx_rotation;
        
        // // Rotation part
        // vctAxAnRot3 dxRot;
        // vct3 dxRotVec;
        // dxRot.FromNormalized((currentKinematicsStateP_->Frame.Inverse() * desiredKinematicsStateP_->Frame).Rotation());
        // dxRotVec = dxRot.Axis() * dxRot.Angle();
        // dx_rotation[0] = dxRotVec[0];
        // dx_rotation[1] = dxRotVec[1];
        // dx_rotation[2] = dxRotVec[2];
        // //dx_rotation.SetAll(0.0);
        // dx_rotation = currentKinematicsStateP_->Frame.Rotation() * dx_rotation;
        
        // Eigen::AngleAxis<float> tipToTarget_cisstToEigen;
        
        // Eigen::Matrix3f rotmat;
        // double theta = std::sqrt(dx_rotation[0]*dx_rotation[0]+dx_rotation[1]*dx_rotation[1]+dx_rotation[2]*dx_rotation[2]);
        // rotmat= Eigen::AngleAxisf(theta,Eigen::Vector3f(dx_rotation[0]/theta,dx_rotation[1]/theta,dx_rotation[2]/theta));
        
//        std::cout << "\ntiptotarget     \n" << tipToTarget.matrix() << "\n";
//        std::cout << "\ntiptotargetcisst\n" << rotmat.matrix() << "\n";
        
        
        //BOOST_LOG_TRIVIAL(trace) << "\n   test         desired dx: " << inputDesired_dx << " " << dx_rotation << "\noptimizer Calculated dx: " << optimizerCalculated_dx;
        // SetKinematics(*currentKinematicsStateP_);  // replaced by name of object
        // fill these out in the desiredKinematicsStateP_
        //RotationType RotationMember; // vcRot3
        //TranslationType TranslationMember; // vct3
    
        // SetKinematics(*desiredKinematicsStateP_); // replaced by name of object
        // call setKinematics with the new kinematics
        // sawconstraintcontroller has kinematicsState
        // set the jacobian here
        
        //////////////////////
        /// @todo move code below here back under run_one updateKinematics() call
        
       /// @todo need to provide tick time in double seconds and get from vrep API call
       float simulationTimeStep = simGetSimulationTimeStep();
    //    UpdateOptimizer(simulationTimeStep);
       
    //    vctDoubleVec jointAngles_dt;
    //    auto returncode = Solve(jointAngles_dt);
       
       
       /// @todo check the return code, if it doesn't have a result, use the VREP version as a fallback and report an error.
       // if(returncode != nmrConstraintOptimizer::NMR_OK) BOOST_THROW_EXCEPTION(std::runtime_error("InverseKinematicsVrepPlugin: constrained optimization error, please investigate"));
       
       
       /// @todo: rethink where/when/how to send command for the joint angles. Return to LUA? Set Directly? Distribute via vrep send message command?
       std::vector<std::vector<double>> q_forward_kinematics;
        std::string str;
       // str = "";
       for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
       {
          /// @todo TODO(ahundt) modify parameters as follows https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257466822
          rbd_mbcs_[0].q[i]={currentJointPosVec[i]};
       
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
    
        // we only have one robot so the index of it is 0
        const std::size_t simulatedRobotIndex = 0;
        
        rbd::forwardKinematics(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);
        rbd::forwardVelocity(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);
        

        /// @todo TODO(ahundt) make solver object a member variable if possible
        tasks::qp::QPSolver solver;

        int bodyI = rbd_mbs_[simulatedRobotIndex].bodyIndexByName(ikGroupTipName_);
        tasks::qp::PositionTask posTask(rbd_mbs_, simulatedRobotIndex, ikGroupTipName_,desiredEigenT);
        tasks::qp::SetPointTask posTaskSp(rbd_mbs_, simulatedRobotIndex, &posTask, 10., 1.);

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


        // Test JointLimitsConstr
        /// @todo TODO(ahundt) was this commented correctly?
        //rbd_mbcs_[simulatedRobotIndex] = mbcInit;
        for(int i = 0; i < 10; ++i)
        {
            //BOOST_REQUIRE(solver.solve(rbd_mbs_, rbd_mbcs_));
            solver.solve(rbd_mbs_, rbd_mbcs_);
            rbd::eulerIntegration(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex], 0.001);

            rbd::forwardKinematics(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);
            rbd::forwardVelocity(rbd_mbs_[simulatedRobotIndex], rbd_mbcs_[simulatedRobotIndex]);
            //BOOST_REQUIRE_GT(rbd_mbcs_[simulatedRobotIndex].q[1][simulatedRobotIndex], -cst::pi<double>()/4. - 0.01);
        }
        
        
       for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
       {
          float currentAngle;
          auto ret = simGetJointPosition(jointHandles_[i],&currentAngle);
          BOOST_VERIFY(ret!=-1);
          /// @todo TODO(ahundt) modify parameters as follows https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257466822
          //rbd_mbcs_[0].q[i]={currentAngle};
       
          /// @todo TODO(ahundt) add torque information
       
           float futureAngle = rbd_mbcs_[simulatedRobotIndex].q[0][i];
          //simSetJointTargetVelocity(jointHandles_[i],jointAngles_dt[i]/simulationTimeStep);
          //simSetJointTargetPosition(jointHandles_[i],jointAngles_dt[i]);
          //simSetJointTargetPosition(jointHandles_[i],futureAngle);
           simSetJointPosition(jointHandles_[i],futureAngle);
                 str+=boost::lexical_cast<std::string>(futureAngle);
                 if (i<jointHandles_.size()-1)
                     str+=", ";
       }
        BOOST_LOG_TRIVIAL(trace) << "jointAngles_dt: "<< str;
        /// @todo TODO(ahundt) extract results
        
        //auto optimizerCalculated_dx = this->currentKinematicsStateP_->Jacobian * jointAngles_dt;
       
        // BOOST_LOG_TRIVIAL(trace) << "\n            desired dx: " << inputDesired_dx << " " << dx_rotation << "\noptimizer Calculated dx: " << optimizerCalculated_dx;
    }
    
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
    std::vector<std::string>            bodyNames_;
    
    
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