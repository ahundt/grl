/// @author Andrew Hundt <athundt@gmail.com>

#ifndef _GRL_VF_CONTROLLER_
#define _GRL_VF_CONTROLLER_

#include <tuple>
#include <string>
#include <sawConstraintController/mtsVFController.h>
#include <sawConstraintController/mtsVFDataJointLimits.h>

namespace grl {

/// create the data object and put the data inside
/// @todo move to own header with no vrep dependencies
/// @todo figure out how the interface for following the motion will work
/// VFControlller stands for "virtual fixtures" controller
struct InverseKinematicsController : public mtsVFController {
public:
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
    
    typedef std::tuple<
        std::string // currentKinematicsName
        ,std::string // desiredKinematicsName
        ,std::string // followVFName
        ,std::size_t // followVFNumRows
        ,std::string // jointVelocityLimitsVFName
        ,std::size_t // JointVelocityLimitsVFNumRows
        ,std::string // JointPositionVFName
        ,std::size_t  // JointPositionNumRows
        > Params;
    
    
    /// @todo maybe parameterize this somehow
    static const std::size_t totalRows = 7;
    
    /// @todo this is just done quickly, make it and the constructor be done right
    InverseKinematicsController(size_t num_joints, mtsVFBase::CONTROLLERMODE cm):
         mtsVFController(num_joints,cm) // parent
        ,currentKinematicsStateP_(new prmKinematicsState())
        ,desiredKinematicsStateP_(new prmKinematicsState())
        ,followVFP_(new mtsVFDataBase())
        ,jointVelocityLimitsVFP_(new mtsVFDataJointLimits())
        ,jointPositionLimitsVFP_(new mtsVFDataJointLimits())
    {
    
    }
    
    Params defaultParams()
    {
        
    }
  
    InverseKinematicsController()
    {
        
    }
    
    void construct(){
    }
    
    /// This will hold the Jacobian
    std::unique_ptr<prmKinematicsState> currentKinematicsStateP_;
    
    /// This will hold the xyz position and the rotation of where I want to go
    std::unique_ptr<prmKinematicsState> desiredKinematicsStateP_;
    
    // want to follow a goal position
    /// @todo will want to use an addVFFollow member function once it is added in
    std::unique_ptr<mtsVFDataBase> followVFP_;
    
    /// need velocity limits for the joints
    std::unique_ptr<mtsVFDataJointLimits> jointVelocityLimitsVFP_;
    /// joints cannot go to certain position
    std::unique_ptr<mtsVFDataJointLimits> jointPositionLimitsVFP_;
    
    
};

} // namespace grl

#endif