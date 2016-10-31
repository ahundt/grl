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

// Tasks
// https://github.com/jrl-umi3218/Tasks
#include <Tasks/Tasks.h>

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
        
        /// @todo TODO(ahundt) does not generally get the transform to the base, hardcoded for the Kuka to "Robotiiwa".
        std::string ikGroupBaseName("Robotiiwa");
        int ikGroupBaseHandle = simGetObjectHandle(ikGroupBaseName.c_str());
        /// for why this is named as it is see: https://github.com/jrl-umi3218/Tasks/blob/master/tests/arms.h#L34
        sva::PTransform<double> X_base = getObjectPTransform(ikGroupBaseHandle);
        // start by assuming the base is fixed
        bool isFixed = true;
        bool isForwardJoint = true;
        
        {
        
            auto jointHandles = VrepRobotArmDriverSimulatedP_->getJointHandles();
            auto jointNames = VrepRobotArmDriverSimulatedP_->getJointNames();
            
            auto linkNames =VrepRobotArmDriverSimulatedP_->getLinkNames();
            auto linkNameHandles =VrepRobotArmDriverSimulatedP_->getLinkHandles();
            auto linkRespondableNames = VrepRobotArmDriverSimulatedP_->getLinkRespondableNames();
            auto linkRespondableNameHandles =VrepRobotArmDriverSimulatedP_->getLinkRespondableHandles();
            std::size_t numJoints = jointHandles.size();
        
            std::vector<std::string> bodyNames;
            bodyNames.push_back(ikGroupBaseName);
            // note: bodyNames are 1 longer than link names, and start with the base!
            /// @todo TODO(ahundt) should 1st parameter be linkNames instead of linkRespondableNames?
            boost::copy(linkRespondableNames, std::back_inserter(bodyNames));
        
                            
            double mass = 1.;
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            Eigen::Vector3d h = Eigen::Vector3d::Zero();

            sva::RBInertiad rbi_base(mass, h, I);
            rbd::Body baseBody(rbi_base,ikGroupBaseName.c_str());
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
                rbd::Body b_i(rbi_i,bodyNames[i+1].c_str());
            
                rbd_mbg_.addBody(b_i);
            
                // Note that V-REP specifies full transforms to place objects that rotate joints around the Z axis
                rbd::Joint j_i(rbd::Joint::Rev, Eigen::Vector3d::UnitZ(), isForwardJoint, jointNames[i].c_str());
            
                rbd_mbg_.addJoint(j_i);
            
                sva::PTransformd to(getJointPTransform(jointHandles[i]));
                sva::PTransformd from(sva::PTransformd::Identity());
            
                // remember, bodyNames[0] is the ikGroupBaseName, so entity order is
                // bodyNames[i], joint[i], bodyNames[i+1]
                rbd_mbg_.linkBodies(bodyNames[i], to, bodyNames[i+1], from, jointNames[i]);
            
            }
        
        
            rbd_mb_ = rbd_mbg_.makeMultiBody(ikGroupBaseName,isFixed,X_base);
            rbd_mbc_ = rbd::MultiBodyConfig(rbd_mb_);
            rbd_mbc_.zero(rbd_mb_);
        
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
        std::string str;
       // str = "";
       for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
       {
          float currentAngle;
          auto ret = simGetJointPosition(jointHandles_[i],&currentAngle);
          BOOST_VERIFY(ret!=-1);
        //   float futureAngle = currentAngle + jointAngles_dt[i];
          //simSetJointTargetVelocity(jointHandles_[i],jointAngles_dt[i]/simulationTimeStep);
          //simSetJointTargetPosition(jointHandles_[i],jointAngles_dt[i]);
          //simSetJointTargetPosition(jointHandles_[i],futureAngle);
        //   simSetJointPosition(jointHandles_[i],futureAngle);
        //         str+=boost::lexical_cast<std::string>(jointAngles_dt[i]);
        //         if (i<jointHandles_.size()-1)
        //             str+=", ";
       }
        BOOST_LOG_TRIVIAL(trace) << "jointAngles_dt: "<< str;
        
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
    
    rbd::MultiBodyGraph          rbd_mbg_;
    rbd::MultiBody               rbd_mb_;
    rbd::MultiBodyConfig         rbd_mbc_;
    std::vector<rbd::Body>       rbd_bodies_;
    std::vector<sva::RBInertia<double>> rbd_inertias_;
    std::vector<rbd::Joint>      rbd_joints_;
    
    
    std::vector<int> jointHandles_; ///< @todo move this item back into VrepRobotArmDriver
    int ikGroupHandle_;
    std::shared_ptr<vrep::VrepRobotArmDriver> VrepRobotArmDriverSimulatedP_;
    std::shared_ptr<vrep::VrepRobotArmDriver> VrepRobotArmDriverMeasuredP_;
    vrep::VrepRobotArmDriver::State currentArmState_;
    std::string positionLimitsName;
    std::string velocityLimitsName;
};

} // namespace vrep
} // namespace grl

#endif