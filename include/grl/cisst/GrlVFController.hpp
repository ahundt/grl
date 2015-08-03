#ifndef _GRL_VF_CONTROLLER_
#define _GRL_VF_CONTROLLER_

#include "sawConstraintController/mtsVFController.h"


/// create the data object and put the data inside
/// @todo move to own header with no vrep dependencies
/// @todo figure out how the interface for following the motion will work
struct GrlVFController : mtsVFController {
    
    typedef mtsVFController parent_type;
  
    enum  GrlVFControllerParamsIndex {
        currentKinematicsName,
        desiredKinematicsName,
        followVFName,
        followVFNumRows,
        jointVelocityLimitsVFName,
        JointVelocityLimitsVFNumRows,
        JointPositionVFName,
        JointPositionNumRows
    };
    
    std::tuple<
        std::string // currentKinematicsName
        ,std::string // desiredKinematicsName
        ,std::string // followVFName
        ,std::size_t // followVFNumRows
        ,std::string // jointVelocityLimitsVFName
        ,std::size_t // JointVelocityLimitsVFNumRows
        ,std::string // JointPositionVFName
        ,std::size_t  // JointPositionNumRows
        > GrlVFControllerParams;
    
    
    /// @todo maybe parameterize this somehow
    static const std::size_t totalRows = 6;
    
    defaultParams (){
        
    }
  
    grlVFController():
    
    {
        
    }
    (
    construct(){
    }
    
    /// This will hold the Jacobian
    std::unique_ptr<prmKinematicsState> currentKinematicsStateP_;
    
    /// This will hold the xyz position and the rotation of where I want to go
    std::unique_ptr<prmKinematicsState> desiredKinematicsStateP_;
    
    // want to follow a goal position
    /// @todo will want to use an addVFFollow member function once it is added in
    std::unique_ptr<mtsVFDataBase> followVFP_;
    
    /// need velocity limits for the joints
    std::unique_ptr<mtsVfDataJointLimits> jointVelocityLimitsVFP_;
    /// joints cannot go to certain position
    std::unique_ptr<mtsVfDataJointLimits> jointPositionLimitsVFP_;
    
};



#endif