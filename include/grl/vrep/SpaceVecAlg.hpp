#ifndef _VREP_SPACEVECALG_HPP_
#define _VREP_SPACEVECALG_HPP_

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// Vrep Eigen conversions
#include "grl/vrep/Eigen.hpp"

/// Get the plucker transform from the object to the base. 
///
/// @param objectHandle The V-Rep object handle for which the transform is needed
/// @param relativeToObjectHandle The frame of reference in which to get the object handle, -1 (the default) gets the absolute position aka world frame
/// for constructor see: https://github.com/jrl-umi3218/SpaceVecAlg/blob/master/src/PTransform.h
/// @todo TODO(ahundt) FIXME Be careful! the quaternion component is inverted from V-REP, perhaps move out or create a separate version?
sva::PTransform<double> getObjectPTransform(int objectHandle, int relativeToObjectHandle = -1)
{

/// @todo TODO(ahundt) FIXME Be careful! the quaternion component is inverted from V-REP, perhaps move out or create a separate version?
  std::pair<Eigen::Quaterniond,Eigen::Vector3d> baseQuatTransformPair = getObjectTransformQuaternionTranslationPair(objectHandle, relativeToObjectHandle);
  sva::PTransform<double> ptransform(baseQuatTransformPair.first.inverse(),baseQuatTransformPair.second);
  return ptransform;
}


/// @todo TODO(ahundt) FIXME Be careful! the quaternion component is inverted from V-REP, perhaps move out or create a separate version?
sva::PTransform<double> eigenAffineToPtransform(const Eigen::Affine3d& eigenTransform)
{
/// @todo TODO(ahundt) FIXME Be careful! the quaternion component is inverted from V-REP, perhaps move out or create a separate version?
    return sva::PTransform<double>(eigenTransform.rotation().inverse(),eigenTransform.translation());
}

/// @todo TODO(ahundt) FIXME Be careful! the quaternion component is inverted from V-REP, perhaps move out or create a separate version?
Eigen::Affine3d PTranformToEigenAffine(sva::PTransform<double> & ptransform)
{
/// @todo TODO(ahundt) FIXME Be careful! the quaternion component is inverted from V-REP, perhaps move out or create a separate version?
    Eigen::Affine3d eigenTransform;
    eigenTransform.translation() = ptransform.translation();
    eigenTransform.matrix().block<3,3>(0,0) = ptransform.rotation().inverse();
    return eigenTransform;
}

sva::PTransform<double> getJointPTransform(int objectHandle)
{
    return eigenAffineToPtransform(getJointTransform(objectHandle));
}

#endif // _VREP_SPACEVECALG_HPP_