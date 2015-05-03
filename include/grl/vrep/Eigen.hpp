#ifndef _VREP_EIGEN_HPP_
#define _VREP_EIGEN_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include "v_repLib.h"

template<typename InputIterator>
Eigen::Quaterniond vrepToEigenQuaternion(InputIterator vrepQuat){
    // vrep is ordered xyzw, eigen is ordered wxyz
    Eigen::Quaterniond quat(vrepQuat[1],vrepQuat[2],vrepQuat[3],vrepQuat[0]);
    return quat;
}

template<typename InputIterator>
Eigen::Vector3d vrepQuatToEigenVector3dAngleAxis(InputIterator vrepQuat){
    Eigen::AngleAxisd ax3d(vrepToEigenQuaternion(vrepQuat));
    return ax3d.axis()*ax3d.angle();
}


template<typename InputIterator>
Eigen::Vector3d vrepToEigenVector3d(InputIterator vrepVec){
    // vrep is ordered xyzw, eigen is ordered wxyz
    Eigen::Vector3d vec(vrepVec[0],vrepVec[1],vrepVec[2]);
    return vec;
}

std::pair<Eigen::Vector3d,Eigen::Vector3d> getAxisAngleAndTranslation(int ObjectHandle, int BaseFrameObjectHandle){
	
	std::array<float,3> simTipPosition;
	std::array<float,3> simTipOrientation;
    std::array<float,4> simTipQuaternion;

	int ret = simGetObjectPosition(ObjectHandle, BaseFrameObjectHandle, simTipPosition.begin());
    if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("HandEyeCalibrationVrepPlugin: Could not get position"));
	ret = simGetObjectOrientation(ObjectHandle, BaseFrameObjectHandle, simTipOrientation.begin());
    if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("HandEyeCalibrationVrepPlugin: Could not get orientation"));
    ret = simGetObjectQuaternion(ObjectHandle, BaseFrameObjectHandle, simTipQuaternion.begin());
    if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("HandEyeCalibrationVrepPlugin: Could not get quaternion"));
	
    auto simTipAngleAxis = vrepQuatToEigenVector3dAngleAxis(simTipQuaternion.begin());
    auto simTipVec =       vrepToEigenVector3d(simTipPosition.begin());
    
    
    
    return std::make_pair(simTipAngleAxis,simTipVec);
}


#endif