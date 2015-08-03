#ifndef _VREP_VF_CONTROLLER_
#define _VREP_VF_CONTROLLER_

#include <string>
#include <tuple>
#include <boost/format.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sawConstraintController/mtsVFController.h>

#include <grl/cisst/GrlVFController.hpp>

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

template<typename DesiredKinematics = DesiredKinematicsObject>
class VrepVFController : GrlVFController {
    
    typedef GrlVFController parent_type;
  
    enum  VrepVFControllerParamsIndex {
        DesiredKinematicsObjectName,
        IKGroupName
    };
    
    typedef std::tuple<
            std::string // IKGroupName
            ,std::string // DesiredKinematicsObjectName
        > Params;
    
    Params defaultParams();
    
    void construct(Params params){
        // get kinematics group name
        // get number of joints
        ikGroupHandle_ = simGetIkGroupHandle(std::get<IKGroupName>(params));
        simSetExplicitHandling(ikGroupHandle_,1); // enable explicit handling for the ik
        std::size_t numJoints;

        currentKinematicsStateP_.reset(new prmKinematicsState());
        currentKinematicsStateP_->Name = std::get<IKGroupName>(params);
        
        
        /// @todo verify object lifetimes
        parent_t::paren_t::InitializeKinematicsState(currentKinematicsStateP_)
            
            // for each virtual fixture need names and number of rows
    }
    
    /// check out sawConstraintController
    void updateKinematics(){
        std::string str;
        float jacobianSize[2];
        float* jacobian=simGetIkGroupMatrix(ikGroupHandle_,0,jacobianSize);

        // jacobianSize[0] represents the row count of the Jacobian 
        // (i.e. the number of equations or the number of joints involved
        // in the IK resolution of the given kinematic chain)
        // Joints appear from tip to base.

        // jacobianSize[1] represents the column count of the Jacobian 
        // (i.e. the number of constraints, e.g. x,y,z,alpha,beta,gamma,jointDependency)

        // The Jacobian data is ordered row-wise, i.e.:
        // [row0,col0],[row1,col0],..,[rowN,col0],[row0,col1],[row1,col1],etc.

        for (int i=0;i<jacobianSize[0];i++)
        {
            std::string str;
            for (int j=0;j<jacobianSize[1];j++)
            {
                if (str.size()==0)
                    str+=", ";
                str+=boost::str(boost::format("%.1e") % jacobian[static_cast<int>(j*jacobianSize[0]+i)]);
            }
            printf(str.c_str());
        }
        
        // call setKinematics with the new kinematics
        // sawconstraintcontroller has kinematicsState
        // set the jacobian here
    }
    
    
    /// may not need this it is in the base class
    /// blocking call, call in separate thread, just allocates memory
    void updateOptimizer(){
        // this 
    }
    
    
    /// may not need this it is in the base class
    /// this will have output
    /// blocking call, call in separate thread, just allocates memory
    /// this runs the actual optimization algorithm
    void solve(){
        
    }
    
    int ikGroupHandle_;
};

#endif