#ifndef _GRL_VREP_ROBOT_ARM_DRIVER_HPP_
#define _GRL_VREP_ROBOT_ARM_DRIVER_HPP_

#include <iostream>
#include <memory>
#include <array>

#include <boost/log/trivial.hpp>
#include <boost/exception/all.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "grl/vrep/Vrep.hpp"
#include "grl/vrep/VrepRobotArmDriver.hpp"
#include "grl/vrep/Eigen.hpp"

#include "v_repLib.h"


namespace grl { namespace vrep {


/// @todo verify that the ordering of the jacobian returned is accurate against VrepVFController.hpp
Eigen::MatrixXf getJacobian(VrepRobotArmDriver& driver)
{

        // Initialize Variables for update
        auto jointHandles_ = driver.getJointHandles();
        int numJoints =jointHandles_.size();
        std::vector<float> ikCalculatedJointValues(numJoints,0);
        VrepRobotArmDriver::State currentArmState_;
        driver.getState(currentArmState_);
        
        /// @todo get target position, probably relative to base
        const auto& handleParams = getVrepHandleParams();
        
        auto target = std::get<vrep::VrepRobotArmDriver::RobotTargetName>(handleParams);
        auto tip = std::get<vrep::VrepRobotArmDriver::RobotTipName>(handleParams);
        
        // save the current tip to target transform
        Eigen::Affine3d tipToTarget =getObjectTransform( target,tip);
        
        // set the current transform to the identity so simCheckIkGroup won't fail
        // @see http://www.forum.coppeliarobotics.com/viewtopic.php?f=9&t=3967
        // for details about this issue.
        setObjectTransform( target,tip,Eigen::Affine3d::Identity());
        
        // debug:
        // std::cout << "TipToTargetTransform:\n" << tipToTarget.matrix() << "\n";
        
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
        if(ikcalcResult!=sim_ikresult_success && ikcalcResult != sim_ikresult_not_performed)
        {
            BOOST_LOG_TRIVIAL(error) << "VrepInverseKinematicsController: didn't run inverse kinematics";
            return;
        }
        
        setObjectTransform( target,tip,tipToTarget);
        
        // Get the Jacobian
        int jacobianSize[2];
        float* jacobian=simGetIkGroupMatrix(ikGroupHandle_,0,jacobianSize);
    
        Eigen::MatrixXf eigenJacobian(jacobianSize[0],jacobianSize[1]);
        
        
        // Transfer the Jacobian to cisst

        // jacobianSize[0] represents the row count of the Jacobian 
        // (i.e. the number of equations or the number of joints involved
        // in the IK resolution of the given kinematic chain)
        // Joints appear from tip to base.

        // jacobianSize[1] represents the column count of the Jacobian 
        // (i.e. the number of constraints, e.g. x,y,z,alpha,beta,gamma,jointDependency)

        // The Jacobian data is ordered row-wise, i.e.:
        // [row0,col0],[row1,col0],..,[rowN,col0],[row0,col1],[row1,col1],etc.

        // http://eigen.tuxfamily.org/dox/group__TutorialMapClass.html
        Eigen::Map<Eigen::MatrixXf> mf(jacobian,jacobianSize[0],jacobianSize[1]);
        eigenJacobian = mf;
    
        return eigenJacobian;
}




}} // grl::vrep

#endif // _GRL_VREP_ROBOT_ARM_DRIVER_HPP_