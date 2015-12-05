/// @author Andrew Hundt <athundt@gmail.com>
#ifndef _VREP_VF_CONTROLLER_
#define _VREP_VF_CONTROLLER_

/// @todo remove IGNORE_ROTATION or make it a runtime configurable parameter
// #define IGNORE_ROTATION



#include <string>
#include <tuple>
#include <boost/format.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sawConstraintController/mtsVFController.h>

#include "grl/cisst/GrlVFController.hpp"
#include "grl/vrep/Vrep.hpp"
#include "grl/vrep/VrepRobotArmDriver.hpp"
#include "grl/vrep/VrepRobotArmJacobian.hpp"
#include "grl/vrep/Eigen.hpp"

#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm/copy.hpp>

namespace grl {

/// @todo verify Robotiiwa is the correct base, and RobotMillTipTarget is the right target, because if the transform doesn't match the one in the jacobian the algorithm will break
//template<typename DesiredKinematics = DesiredKinematicsObject>
class VrepInverseKinematicsController : public grl::InverseKinematicsController
{
public:
    typedef InverseKinematicsController parent_type;
    
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
    VrepInverseKinematicsController(size_t num_joints = 7, mtsVFBase::CONTROLLERMODE cm = mtsVFBase::CONTROLLERMODE::JPOS):
     InverseKinematicsController(num_joints,cm)
    {
    }
    
    void construct(Params params = defaultParams()){
        // get kinematics group name
        // get number of joints
        VrepRobotArmDriverP_ = std::make_shared<vrep::VrepRobotArmDriver>();
        VrepRobotArmDriverP_->construct();
        
        ikGroupHandle_ = simGetIkGroupHandle(std::get<IKGroupName>(params).c_str());
        simSetExplicitHandling(ikGroupHandle_,1); // enable explicit handling for the ik

        this->currentKinematicsStateP_->Name = std::get<IKGroupName>(params);
        
        this->desiredKinematicsStateP_->Name = std::get<DesiredKinematicsObjectName>(params);
        
        /// @todo set desiredKinematicsStateP name, INITIALIZE ALL MEMBER OBJECTS, & SET NAMES
        //
        positionLimitsName = std::get<IKGroupName>(params)+"_PositionLimits";
        this->jointPositionLimitsVFP_->Name = positionLimitsName;
        
        velocityLimitsName = std::get<IKGroupName>(params)+"_VelocityLimits";
        this->jointVelocityLimitsVFP_->Name = velocityLimitsName;
        
//            /// This will hold the Jacobian
//    std::unique_ptr<prmKinematicsState> currentKinematicsStateP_;
//    
//    /// This will hold the xyz position and the rotation of where I want to go
//    std::unique_ptr<prmKinematicsState> desiredKinematicsStateP_;
//    
//    // want to follow a goal position
//    /// @todo will want to use an addVFFollow member function once it is added in
//    std::unique_ptr<mtsVFDataBase> followVFP_;
//    
//    /// need velocity limits for the joints
//    std::unique_ptr<mtsVFDataJointLimits> jointVelocityLimitsVFP_;
//    /// joints cannot go to certain position
//    std::unique_ptr<mtsVFDataJointLimits> jointPositionLimitsVFP_;
        
        /// @todo verify object lifetimes
        //parent_type::parent_type::InitializeKinematicsState(this->currentKinematicsStateP_)
        
            // for each virtual fixture need names and number of rows
        
        
        /// @todo read objective rows from vrep
        
        // Initialize Variables for update
        //jointHandles_ = VrepRobotArmDriverP_->getJointHandles();
        //int numJoints =jointHandles_.size();
        
#ifndef IGNORE_ROTATION
        /// @todo objectiveRows seems to currently be a 3 vector, may eventually want a rotation and translation, perhaps with quaternion rotation
        followVFP_->ObjectiveRows = 6;
#else
        followVFP_->ObjectiveRows = 3;
#endif
        // set the names once for each object, only once
        followVFP_->KinNames.push_back(currentKinematicsStateP_->Name);
        followVFP_->KinNames.push_back(desiredKinematicsStateP_->Name);
        
        AddVFFollowPath(*followVFP_);
        
        /// @todo set vrep explicit handling of IK here, plus unset in destructor of this object
    }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    /// check out sawConstraintController
    void updateKinematics(){
    
        jointHandles_ = VrepRobotArmDriverP_->getJointHandles();
        auto eigentestJacobian=::grl::vrep::getJacobian(*VrepRobotArmDriverP_);

        /// The row/column major order is swapped between cisst and VREP!
        this->currentKinematicsStateP_->Jacobian.SetSize(eigentestJacobian.cols(),eigentestJacobian.rows());
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor> > mckp2(this->currentKinematicsStateP_->Jacobian.Pointer(),this->currentKinematicsStateP_->Jacobian.cols(),this->currentKinematicsStateP_->Jacobian.rows());
        mckp2 = eigentestJacobian.cast<double>();
        
        
        //Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor> > mf(eigentestJacobian,eigentestJacobian.cols(),eigentestJacobian.rows());
        //Eigen::MatrixXf eigenJacobian = mf;
        Eigen::MatrixXf eigenJacobian = eigentestJacobian;
        
        
        ///////////////////////////////////////////////////////////
        // Copy Joint Interval, the range of motion for each joint
        
        
        // lower limits
        auto & llim = std::get<vrep::VrepRobotArmDriver::JointLowerPositionLimit>(currentArmState_);
        std::vector<double> llimits(llim.begin(),llim.end());
        jointPositionLimitsVFP_->LowerLimits = vctDoubleVec(llimits.size(),&llimits[0]);
        
        // upper limits
        auto & ulim = std::get<vrep::VrepRobotArmDriver::JointUpperPositionLimit>(currentArmState_);
        std::vector<double> ulimits(ulim.begin(),ulim.end());
        jointPositionLimitsVFP_->UpperLimits = vctDoubleVec(ulimits.size(),&ulimits[0]);
        
        
        const auto& handleParams = VrepRobotArmDriverP_->getVrepHandleParams();
        Eigen::Affine3d currentEndEffectorPose =
        getObjectTransform(
                             std::get<vrep::VrepRobotArmDriver::RobotTipName>(handleParams)
                            ,std::get<vrep::VrepRobotArmDriver::RobotTargetBaseName>(handleParams)
                          );
        auto  currentEigenT = currentEndEffectorPose.translation();
        auto& currentCisstT = currentKinematicsStateP_->Frame.Translation();
        currentCisstT[0] = currentEigenT(0);
        currentCisstT[1] = currentEigenT(1);
        currentCisstT[2] = currentEigenT(2);
#ifndef IGNORE_ROTATION
        Eigen::Map<Eigen::Matrix<double,3,3,Eigen::ColMajor>> ccr(currentKinematicsStateP_->Frame.Rotation().Pointer());
        ccr = currentEndEffectorPose.rotation();
#endif // IGNORE_ROTATION
        /// @todo set rotation component of current position
        
        
        Eigen::Affine3d desiredEndEffectorPose =
        getObjectTransform(
                             std::get<vrep::VrepRobotArmDriver::RobotTargetName>(handleParams)
                            ,std::get<vrep::VrepRobotArmDriver::RobotTargetBaseName>(handleParams)
                          );
        auto  desiredEigenT = desiredEndEffectorPose.translation();
        auto& desiredCisstT = desiredKinematicsStateP_->Frame.Translation();
        desiredCisstT[0] = desiredEigenT(0);
        desiredCisstT[1] = desiredEigenT(1);
        desiredCisstT[2] = desiredEigenT(2);
#ifndef IGNORE_ROTATION
        Eigen::Map<Eigen::Matrix<double,3,3,Eigen::ColMajor>> dcr(desiredKinematicsStateP_->Frame.Rotation().Pointer());
        dcr = desiredEndEffectorPose.rotation();
#endif // IGNORE_ROTATION
        /// @todo set rotation component of desired position
        
        // for debugging, the translation between the current and desired position in cartesian coordinates
        auto inputDesired_dx = desiredCisstT - currentCisstT;
        
        vct3 dx_translation, dx_rotation;
        
        // Rotation part
        vctAxAnRot3 dxRot;
        vct3 dxRotVec;
        dxRot.FromNormalized((currentKinematicsStateP_->Frame.Inverse() * desiredKinematicsStateP_->Frame).Rotation());
        dxRotVec = dxRot.Axis() * dxRot.Angle();
        dx_rotation[0] = dxRotVec[0];
        dx_rotation[1] = dxRotVec[1];
        dx_rotation[2] = dxRotVec[2];
        //dx_rotation.SetAll(0.0);
        dx_rotation = currentKinematicsStateP_->Frame.Rotation() * dx_rotation;
        
        Eigen::AngleAxis<float> tipToTarget_cisstToEigen;
        
        Eigen::Matrix3f rotmat;
        double theta = std::sqrt(dx_rotation[0]*dx_rotation[0]+dx_rotation[1]*dx_rotation[1]+dx_rotation[2]*dx_rotation[2]);
        rotmat= Eigen::AngleAxisf(theta,Eigen::Vector3f(dx_rotation[0]/theta,dx_rotation[1]/theta,dx_rotation[2]/theta));
        
//        std::cout << "\ntiptotarget     \n" << tipToTarget.matrix() << "\n";
//        std::cout << "\ntiptotargetcisst\n" << rotmat.matrix() << "\n";
        
        
        //BOOST_LOG_TRIVIAL(trace) << "\n   test         desired dx: " << inputDesired_dx << " " << dx_rotation << "\noptimizer Calculated dx: " << optimizerCalculated_dx;
        SetKinematics(*currentKinematicsStateP_);  // replaced by name of object
        // fill these out in the desiredKinematicsStateP_
        //RotationType RotationMember; // vcRot3
        //TranslationType TranslationMember; // vct3
    
        SetKinematics(*desiredKinematicsStateP_); // replaced by name of object
        // call setKinematics with the new kinematics
        // sawconstraintcontroller has kinematicsState
        // set the jacobian here
        
        //////////////////////
        /// @todo move code below here back under run_one updateKinematics() call
        
       /// @todo need to provide tick time in double seconds and get from vrep API call
       UpdateOptimizer(0.01);
       
       vctDoubleVec jointAngles_dt;
       auto returncode = Solve(jointAngles_dt);
       
       
       /// @todo check the return code, if it doesn't have a result, use the VREP version as a fallback and report an error.
       if(returncode != nmrConstraintOptimizer::NMR_OK) BOOST_THROW_EXCEPTION(std::runtime_error("VrepInverseKinematicsController: constrained optimization error, please investigate"));
       
       
       /// @todo: rethink where/when/how to send command for the joint angles. Return to LUA? Set Directly? Distribute via vrep send message command?
        std::string str;
       // str = "";
       for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
       {
          float currentAngle;
          auto ret = simGetJointPosition(jointHandles_[i],&currentAngle);
          BOOST_VERIFY(ret!=-1);
          float futureAngle = currentAngle + jointAngles_dt[i];
          //simSetJointTargetPosition(jointHandles_[i],jointAngles_dt[i]);
          //simSetJointTargetPosition(jointHandles_[i],futureAngle);
          simSetJointPosition(jointHandles_[i],futureAngle);
                str+=boost::lexical_cast<std::string>(jointAngles_dt[i]);
                if (i<jointHandles_.size()-1)
                    str+=", ";
       }
        BOOST_LOG_TRIVIAL(trace) << "jointAngles_dt: "<< str;
        
        auto optimizerCalculated_dx = this->currentKinematicsStateP_->Jacobian * jointAngles_dt;
       
        BOOST_LOG_TRIVIAL(trace) << "\n            desired dx: " << inputDesired_dx << " " << dx_rotation << "\noptimizer Calculated dx: " << optimizerCalculated_dx;
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
    
    std::vector<int> jointHandles_; ///< @todo move this item back into VrepRobotArmDriver
    int ikGroupHandle_;
    std::shared_ptr<vrep::VrepRobotArmDriver> VrepRobotArmDriverP_;
    vrep::VrepRobotArmDriver::State currentArmState_;
    std::string positionLimitsName;
    std::string velocityLimitsName;
};

} // namespace grl

#endif