/// @file Eigen.hpp
/// @todo convert to use template metaprgramming to define simple functions that simplify and automate the necessary conversions
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

template<typename Q>
std::array<float,4> EigenToVrepQuaternion(const Q& q){
  std::array<float,4> qa;
  // vrep is ordered xyzw, eigen is ordered wxyz
  qa[0] = q.x();
  qa[1] = q.y();
  qa[2] = q.z();
  qa[3] = q.w();
  
  return qa;
}

template<typename P>
std::array<float,3> EigenToVrepPosition(const P& p){
   std::array<float,3> qv;
   qv[0] = p.x();
   qv[1] = p.y();
   qv[2] = p.z();
   
   return qv;
}

template<typename Input>
Eigen::Vector3d vrepQuatToEigenVector3dAngleAxis(Input vrepQuat){
    Eigen::AngleAxisd ax3d(vrepToEigenQuaternion(vrepQuat));
    return ax3d.axis()*ax3d.angle();
}

template<typename Input>
Eigen::Vector3d eigenRotToEigenVector3dAngleAxis(Input eigenQuat){
    Eigen::AngleAxisd ax3d(eigenQuat);
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
	//std::array<float,3> simTipOrientation;
    std::array<float,4> simTipQuaternion;

	int ret = simGetObjectPosition(ObjectHandle, BaseFrameObjectHandle, simTipPosition.begin());
    if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("HandEyeCalibrationVrepPlugin: Could not get position"));
	//ret = simGetObjectOrientation(ObjectHandle, BaseFrameObjectHandle, simTipOrientation.begin());
    //if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("HandEyeCalibrationVrepPlugin: Could not get orientation"));
    ret = simGetObjectQuaternion(ObjectHandle, BaseFrameObjectHandle, simTipQuaternion.begin());
    if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("HandEyeCalibrationVrepPlugin: Could not get quaternion"));
	
    auto simTipAngleAxis = vrepQuatToEigenVector3dAngleAxis(simTipQuaternion.begin());
    auto simTipVec =       vrepToEigenVector3d(simTipPosition.begin());
    
    
    
    return std::make_pair(simTipAngleAxis,simTipVec);
}

template<typename T>
void setObjectTransform(int objectHandle, int relativeToObjectHandle, T& transform){

   // get quaternion between end effector and
   Eigen::Quaternion<typename T::Scalar> eigenQuat(transform.rotation());
   std::array<float,4> vrepQuat = EigenToVrepQuaternion(eigenQuat);
   simSetObjectQuaternion(objectHandle,relativeToObjectHandle,vrepQuat.begin());
   
   std::array<float,3> vrepPos = EigenToVrepPosition(transform.translation());
   simSetObjectPosition(objectHandle,relativeToObjectHandle,vrepPos.begin());
   
}

Eigen::Affine3d getObjectTransform(int objectHandle, int relativeToObjectHandle){

   std::array<float,4> vrepQuat;
   simGetObjectQuaternion(objectHandle,relativeToObjectHandle,vrepQuat.begin());
   Eigen::Quaterniond eigenQuat(vrepToEigenQuaternion(vrepQuat));
   
   std::array<float,3> vrepPos;
   simGetObjectPosition(objectHandle,relativeToObjectHandle,vrepPos.begin());
   Eigen::Vector3d eigenPos(vrepToEigenVector3d(vrepPos));
   
   Eigen::Affine3d transform;
   
   transform = eigenQuat;
   transform.translation() = eigenPos;
   
   return transform;
}

template<typename T>
static std::string poseString(const Eigen::Transform<T,3,Eigen::Affine>& pose, const std::string& pfx = "")
{
  std::stringstream ss;
  ss.precision(3);
  for (int y=0;y<4;y++)
  {
    ss << pfx;
    for (int x=0;x<4;x++)
    {
      ss << std::setw(8) << pose(y,x) << " ";
    }
    ss << std::endl;
  }
  return ss.str();
}


//---------
//posString
//---------
static std::string posString(const Eigen::Vector3d& pos, const std::string& pfx = "(", const std::string& sfx = ")")
{
  std::stringstream ss;
  ss.precision(3);
  ss << pfx;
  for (int x=0;x<3;x++)
  {
    if (x)
      ss << ", ";
    ss << std::setw(8) << pos(x);
  }
  ss << sfx;
  return ss.str();
}


#endif