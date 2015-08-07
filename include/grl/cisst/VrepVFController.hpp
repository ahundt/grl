/// @author Andrew Hundt <athundt@gmail.com>

#ifndef _VREP_VF_CONTROLLER_
#define _VREP_VF_CONTROLLER_

#include <string>
#include <tuple>
#include <boost/format.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sawConstraintController/mtsVFController.h>

#include "grl/cisst/GrlVFController.hpp"
#include "grl/vrep/Vrep.hpp"
#include "grl/vrep/VrepRobotArmDriver.hpp"
#include "grl/vrep/Eigen.hpp"

namespace grl {

/// This handles a whole vrep path object
class DesiredKinematicsPath {
  
    enum  VrepVFControllerParamsIndex {
        DesiredKinematicsObjectName
    };
    
    typedef std::tuple<
            std::string // DesiredKinematicsObjectName
        > VrepVFControllerParams;
    
    
    void construct()
    {
        
    }
    
    Eigen::Affine3d getDesiredPose(){
        
    }
};

/// This handles a specific vrep pose
class DesiredKinematicsObject {
  
    enum  VrepVFControllerParamsIndex {
        DesiredKinematicsObjectName
    };
    
    std::tuple<
            std::string // DesiredKinematicsObjectName
        > Params;
    
    void construct()
    {
        
    }
    
    
    Eigen::Affine3d getDesiredPose(){
        
    }
};

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
        
        /// @todo objectiveRows seems to currently be a 3 vector, may eventually want a rotation and translation, perhaps with quaternion rotation
        followVFP_->ObjectiveRows = 3;
        // set the names once for each object, only once
        followVFP_->KinNames.push_back(currentKinematicsStateP_->Name);
        followVFP_->KinNames.push_back(desiredKinematicsStateP_->Name);
        
        AddVFFollowPath(*followVFP_);
        
        /// @todo set vrep explicit handling of IK here, plus unset in destructor of this object
    }
    
    /// check out sawConstraintController
    void updateKinematics(){
    
        // Initialize Variables for update
        jointHandles_ = VrepRobotArmDriverP_->getJointHandles();
        int numJoints =jointHandles_.size();
        std::vector<float> ikCalculatedJointValues(numJoints,0);
        VrepRobotArmDriverP_->getState(currentArmState_);
        
        /// Run inverse kinematics, but all we really want is the jacobian
        /// @todo find version that only returns jacobian
        /// @see http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm#simCheckIkGroup
        auto ikcalcResult =
        simCheckIkGroup(ikGroupHandle_
                       ,numJoints
                       ,&jointHandles_[0]
                       ,&ikCalculatedJointValues[0]
                       ,NULL /// @todo do we need to use these options?
                       );
        /// @see http://www.coppeliarobotics.com/helpFiles/en/apiConstants.htm#ikCalculationResults
        if(ikcalcResult!=sim_ikresult_success)
        {
            BOOST_THROW_EXCEPTION(std::runtime_error("VrepInverseKinematicsController: didn't run inverse kinematics"));
        }
        
        // Get the Jacobian
        int jacobianSize[2];
        float* jacobian=simGetIkGroupMatrix(ikGroupHandle_,0,jacobianSize);
        
        this->currentKinematicsStateP_->Jacobian.SetSize(jacobianSize[0],jacobianSize[1]);
        
        // Transfer the Jacobian to cisst

        // jacobianSize[0] represents the row count of the Jacobian 
        // (i.e. the number of equations or the number of joints involved
        // in the IK resolution of the given kinematic chain)
        // Joints appear from tip to base.

        // jacobianSize[1] represents the column count of the Jacobian 
        // (i.e. the number of constraints, e.g. x,y,z,alpha,beta,gamma,jointDependency)

        // The Jacobian data is ordered row-wise, i.e.:
        // [row0,col0],[row1,col0],..,[rowN,col0],[row0,col1],[row1,col1],etc.

        std::string str;
        for (int i=0;i<jacobianSize[0];i++)
        {
            for (int j=0;j<jacobianSize[1];j++)
            {
                str+=boost::str(boost::format("%.1e") % jacobian[static_cast<int>(j*jacobianSize[0]+i)]);
                if (j<jacobianSize[1]-1)
                    str+=", ";
                float currentValue = jacobian[static_cast<int>(j*jacobianSize[0]+i)];
                this->currentKinematicsStateP_->Jacobian[i][j] = currentValue;
            }
        }
        BOOST_LOG_TRIVIAL(trace) << str;
        
        
        ///////////////////////////////////////////////////////////
        /// @todo Set Current Joint State
        
        // currentKinematicsStateP_->JointState = ...
        // prmJointState* JointState;
        
        
        ///////////////////////////////////////////////////////////
        // Copy Joint Interval, the range of motion for each joint
        
        
        // lower limits
        auto & llim = std::get<vrep::VrepRobotArmDriver::JointLowerPositionLimit>(currentArmState_);
        std::vector<double> llimits(llim.begin(),llim.end());
        jointPositionLimitsVFP_->LowerLimits = vctDoubleVec(llimits.size(),&llimits[0]);
        
        // upper limits
        auto & ulim = std::get<vrep::VrepRobotArmDriver::JointLowerPositionLimit>(currentArmState_);
        std::vector<double> ulimits(ulim.begin(),ulim.end());
        jointPositionLimitsVFP_->UpperLimits = vctDoubleVec(ulimits.size(),&ulimits[0]);
        
        /// @todo get target position, probably relative to base
        const auto& handleParams = VrepRobotArmDriverP_->getVrepHandleParams();
        
        Eigen::Affine3d desiredEndEffectorPose =
        getObjectTransform(
                             std::get<vrep::VrepRobotArmDriver::RobotTargetName>(handleParams)
                            ,std::get<vrep::VrepRobotArmDriver::RobotTargetBaseName>(handleParams)
                          );
        auto  eigenT = desiredEndEffectorPose.translation();
        auto& cisstT = desiredKinematicsStateP_->Frame.Translation();
        cisstT[0] = eigenT(0);
        cisstT[1] = eigenT(1);
        cisstT[2] = eigenT(2);
        
        SetKinematics(*currentKinematicsStateP_);  // replaced by name of object
        // fill these out in the desiredKinematicsStateP_
        //RotationType RotationMember; // vcRot3
        //TranslationType TranslationMember; // vct3
    
        SetKinematics(*desiredKinematicsStateP_); // replaced by name of object
        // call setKinematics with the new kinematics
        // sawconstraintcontroller has kinematicsState
        // set the jacobian here
        
    }
    
    /// may not need this it is in the base class
    /// blocking call, call in separate thread, just allocates memory
    void run_one(){
       updateKinematics();
       /// @todo need to provide tick time in double seconds
       UpdateOptimizer(0.05);
       
       vctDoubleVec jointAngles;
       auto returncode = Solve(jointAngles);
       
       
       /// @todo check the return code, if it doesn't have a result, use the VREP version as a fallback and report an error.
       if(returncode != nmrConstraintOptimizer::NMR_OK) BOOST_THROW_EXCEPTION(std::runtime_error("VrepInverseKinematicsController: constrained optimization error, please investigate"));
       
       
       /// @todo: rethink where/when/how to send command for the joint angles. Return to LUA? Set Directly? Distribute via vrep send message command?
       for (int i=0 ; i < jointHandles_.size() ; i++)
       {
          simSetJointTargetPosition(jointHandles_[i],jointAngles[i]);
       }
       
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