/// @file Eigen.hpp
/// @todo convert to use template metaprgramming to define simple functions that simplify and automate the necessary conversions
#ifndef _VREP_EIGEN_HPP_
#define _VREP_EIGEN_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include "v_repLib.h"

namespace vrepquat {
    enum vrep_quat {
       x,
       y,
       z,
       w
    };
}

template<typename InputIterator>
Eigen::Quaterniond vrepToEigenQuaternion(InputIterator vrepQuat){

  //                  0123
  //  vrep is ordered xyzw,
  // eigen is ordered wxyz
  
    Eigen::Quaterniond quat(vrepQuat[vrepquat::w],vrepQuat[vrepquat::x],vrepQuat[vrepquat::y],vrepQuat[vrepquat::z]);
    return quat;
}

template<typename Q>
std::array<float,4> EigenToVrepQuaternion(const Q& q){
  std::array<float,4> qa;
  
  //                  0123
  //  vrep is ordered xyzw,
  // eigen is ordered wxyz
  qa[vrepquat::x] = q.x();
  qa[vrepquat::y] = q.y();
  qa[vrepquat::z] = q.z();
  qa[vrepquat::w] = q.w();
  
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
    return ax3d.angle()*ax3d.axis();
}

template<typename Input>
Eigen::Vector3d eigenRotToEigenVector3dAngleAxis(Input eigenQuat){
    Eigen::AngleAxisd ax3d(eigenQuat);
    return ax3d.angle()*ax3d.axis();
}


template<typename InputIterator>
Eigen::Vector3d vrepToEigenVector3d(InputIterator vrepVec){
    // vrep is ordered xyzw, eigen is ordered wxyz
    Eigen::Vector3d vec(vrepVec[0],vrepVec[1],vrepVec[2]);
    return vec;
}

/// @see http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetJointMatrix.htm
Eigen::Affine3d vrepToEigenTransform(const std::array<float,12>& vrepTransform)
{

  // based on comparing each of the following:
  // http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetJointMatrix.htm
  // https://en.wikipedia.org/wiki/Row-major_order
  // https://eigen.tuxfamily.org/dox/group__TopicStorageOrders.html

  // V-REP is by default Row-major order
  // Eigen is by default Column Major order
  Eigen::Map<const Eigen::Matrix<float,3,4,Eigen::RowMajor>> vmap(vrepTransform.cbegin());
  Eigen::Affine3d eigenTransform;
  eigenTransform.translation() = vmap.block<3,1>(0,3).cast<double>();
  eigenTransform.matrix().block<3,3>(0,0) = vmap.block<3,3>(0,0).cast<double>();

  return eigenTransform;
}


std::pair<Eigen::Vector3d,Eigen::Vector3d> getAxisAngleAndTranslation(int ObjectHandle, int BaseFrameObjectHandle){
	
	std::array<float,3> simTipPosition;
	//std::array<float,3> simTipOrientation;
    std::array<float,4> simTipQuaternion;

	int ret = simGetObjectPosition(ObjectHandle, BaseFrameObjectHandle, simTipPosition.begin());
    if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("HandEyeCalibrationVrepPlugin: Could not get position"));
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
   int ret = simSetObjectQuaternion(objectHandle,relativeToObjectHandle,vrepQuat.begin());
   if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("setObjectTransform: Could not set Quaternion"));
   
   std::array<float,3> vrepPos = EigenToVrepPosition(transform.translation());
   ret = simSetObjectPosition(objectHandle,relativeToObjectHandle,vrepPos.begin());
   if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("setObjectTransform: Could not set Quaternion"));
   
}

/// @param objectHandle The V-Rep object handle for which the transform is needed
/// @param relativeToObjectHandle The frame of reference in which to get the object handle, -1 (the default) gets the absolute position aka world frame
/// @see http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetObjectPosition.htm
/// @see http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetObjectQuaternion.htm
Eigen::Affine3d getObjectTransform(int objectHandle, int relativeToObjectHandle = -1){

   std::array<float,4> vrepQuat;
   int ret = simGetObjectQuaternion(objectHandle,relativeToObjectHandle,vrepQuat.begin());
   Eigen::Quaterniond eigenQuat(vrepToEigenQuaternion(vrepQuat));
   if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("getObjectTransform: Could not get Quaternion"));
   
   std::array<float,3> vrepPos;
   ret = simGetObjectPosition(objectHandle,relativeToObjectHandle,vrepPos.begin());
   if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("getObjectTransform: Could not get Position"));
   Eigen::Vector3d eigenPos(vrepToEigenVector3d(vrepPos));
   
   Eigen::Affine3d transform;
   
   transform = eigenQuat;
   transform.translation() = eigenPos;
   
   return transform;
}

/// @param objectHandle The V-Rep object handle for which the transform is needed
/// @param relativeToObjectHandle The frame of reference in which to get the object handle, -1 (the default) gets the absolute position aka world frame
/// @see http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetObjectPosition.htm
/// @see http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetObjectQuaternion.htm
/// @todo TODO(ahundt) handle errors/return codes from simGetObjectPosition
std::pair<Eigen::Quaterniond,Eigen::Vector3d> getObjectTransformQuaternionTranslationPair(int objectHandle, int relativeToObjectHandle = -1){

   std::array<float,4> vrepQuat;
   int ret = simGetObjectQuaternion(objectHandle,relativeToObjectHandle,vrepQuat.begin());
   if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("getObjectTransformQuaternionTranslationPair: Could not get Quaternion"));
   Eigen::Quaterniond eigenQuat(vrepToEigenQuaternion(vrepQuat));
   
   std::array<float,3> vrepPos;
   ret = simGetObjectPosition(objectHandle,relativeToObjectHandle,vrepPos.begin());
   if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("getObjectTransformQuaternionTranslationPair: Could not get Position"));
   Eigen::Vector3d eigenPos(vrepToEigenVector3d(vrepPos));
   
   return std::make_pair(eigenQuat,eigenPos);
}

/// Eigen version of simGetJointMatrix
Eigen::Affine3d getJointTransform(int objectHandle)
{
  // based on comparing each of the following:
  // http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetJointMatrix.htm
  // https://en.wikipedia.org/wiki/Row-major_order
  // https://eigen.tuxfamily.org/dox/group__TopicStorageOrders.html
  std::array<float,12> vrepTransform;
  int ret = simGetJointMatrix(objectHandle,vrepTransform.begin());
  if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("getJointTransform: Could not get Joint Matrix"));
  return vrepToEigenTransform(vrepTransform);
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