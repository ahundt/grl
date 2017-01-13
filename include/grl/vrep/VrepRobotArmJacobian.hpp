#ifndef _GRL_VREP_ROBOT_ARM_JACOBIAN_HPP_
#define _GRL_VREP_ROBOT_ARM_JACOBIAN_HPP_

#include <iostream>
#include <memory>
#include <array>

#include <boost/exception/all.hpp>
#include <boost/algorithm/string.hpp>

#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "grl/vrep/Vrep.hpp"
#include "grl/vrep/VrepRobotArmDriver.hpp"
#include "grl/vrep/Eigen.hpp"

#include "v_repLib.h"


namespace grl { namespace vrep {

/// @brief get the Jacobian as calculated by vrep in an Eigen::MatrixXf in column major format
/// @param driver the vrep arm driver object that provides access to vrep simulation state for a specific arm
/// @param jacobianOnly true calls simComputeJacobian, false uses simCheckIKGroup which incidentally calculates the Jacobian
///
/// The jacobianOnly option is provided because some irregularities were seen with
/// simComputeJacobian (jacobianOnly == true). Thus (jacobianOnly == false) falls
/// back on using the full vrep provided inverse kinematics calculation which is
/// slower but didn't have the previously seen irregularities. The jacobianOnly option
/// should be eliminated in favor of the simComputeJacobian (jacobianOnly == true) case
/// once all issues are fully resolved.
///
/// @return jacobian in ColumnMajor format where each row is a joint from base to tip, first 3 columns are translation component, last 3 columns are rotation component
Eigen::MatrixXf getJacobian(vrep::VrepRobotArmDriver& driver, bool jacobianOnly = false)
{

        // Initialize Variables for update
        auto jointHandles_ = driver.getJointHandles();
        int numJoints =jointHandles_.size();
        std::vector<float> ikCalculatedJointValues(numJoints,0);
        vrep::VrepRobotArmDriver::State currentArmState_;
        driver.getState(currentArmState_);
        
        /// @todo get target position, probably relative to base
        const auto& handleParams = driver.getVrepHandleParams();
        auto ikGroupHandle_ = std::get<vrep::VrepRobotArmDriver::RobotIkGroup>(handleParams);
    
        if (jacobianOnly)
        {
            // http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm#simComputeJacobian
            auto jacobianResult = simComputeJacobian(ikGroupHandle_,NULL,NULL);
            
            if(jacobianResult == -1)
            {
                spdlog::get("console")->error("VrepInverseKinematicsController: couldn't compute Jacobian");
                return Eigen::MatrixXf();
            }
            
        }
        else
        {
        
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
                spdlog::get("console")->error("VrepInverseKinematicsController: didn't run inverse kinematics");
                return Eigen::MatrixXf();
            }
        
            
            setObjectTransform( target,tip,tipToTarget);
        }
   
        // debug verifying that get and set object transform don't corrupt underlying data
//        Eigen::Affine3d tipToTarget2 =getObjectTransform( target,tip);
//        std::cout << "\ntiptotarget\n" << tipToTarget.matrix();
//        std::cout << "\ntiptotarget2\n" << tipToTarget2.matrix();
        
        // Get the Jacobian
        int jacobianSize[2];
        float* jacobian=simGetIkGroupMatrix(ikGroupHandle_,0,jacobianSize);
        /// @todo FIX HACK jacobianSize include orientation component, should be 7x6 instead of 7x3
        
#ifdef IGNORE_ROTATION
        jacobianSize[1] = 3;
#endif
    
        
        
        // Transfer the Jacobian to cisst

        // jacobianSize[1] represents the row count of the Jacobian
        // (i.e. the number of equations or the number of joints involved
        // in the IK resolution of the given kinematic chain)
        // Joints appear from tip to base.

        // jacobianSize[0] represents the column count of the Jacobian
        // (i.e. the number of constraints, e.g. x,y,z,alpha,beta,gamma,jointDependency)

        // The Jacobian data is in RowMajor order, i.e.:
        // https://en.wikipedia.org/wiki/Row-major_order

        std::string str;
        
        // jacobianSize[1] == eigen ColMajor "rows" , jacobianSize[0] == eigen ColMajor "cols"
        
        Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > mtestjacobian(jacobian,jacobianSize[1],jacobianSize[0]);
        Eigen::MatrixXf eigentestJacobian = mtestjacobian.rowwise().reverse().transpose();
    
        return eigentestJacobian;
}






}} // grl::vrep

#endif // _GRL_VREP_ROBOT_ARM_JACOBIAN_HPP_
