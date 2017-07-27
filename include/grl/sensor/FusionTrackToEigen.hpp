#ifndef GRL_ATRACSYS_FUSION_TRACK_TO_EIGEN_HPP
#define GRL_ATRACSYS_FUSION_TRACK_TO_EIGEN_HPP

// FusionTrack Libraries
#include <ftkInterface.h>
#include <geometryHelper.hpp> // I would like to get rid of this and use only ftk functions + remove atracsys_DIR/bin from include directories

#include <spdlog/spdlog.h>

#include <boost/container/static_vector.hpp>
#include <tuple>
#include <string>
#include <vector>
#include <system_error>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace grl { namespace sensor {

Eigen::Affine3f ftkMarkerToAffine3f(const ftkMarker& marker){

         Eigen::Affine3f cameraToMarkerTransform;
         cameraToMarkerTransform.matrix().block<3,3>(0,0) = Eigen::Map<const Eigen::Matrix<float,3,3,Eigen::RowMajor> >(&marker.rotation[0][0]);
         cameraToMarkerTransform.translation().x() = marker.translationMM[0]/1000;
         cameraToMarkerTransform.translation().y() = marker.translationMM[1]/1000;
         cameraToMarkerTransform.translation().z() = marker.translationMM[2]/1000;
         return cameraToMarkerTransform;
}

ftkMarker Affine3fToftkMarker(const Eigen::Affine3f& cameraToMarkerTransform){
         ftkMarker marker;
         Eigen::Map<Eigen::Matrix<float,3,3,Eigen::RowMajor> > markerRotation(&marker.rotation[0][0]);
         markerRotation = cameraToMarkerTransform.matrix().block<3,3>(0,0);
         marker.translationMM[0] = cameraToMarkerTransform.translation().x()*1000;
         marker.translationMM[1] = cameraToMarkerTransform.translation().y()*1000;
         marker.translationMM[2] = cameraToMarkerTransform.translation().z()*1000;
         return marker;
}

Eigen::Vector3f ftk3DPointToVector3f(const ftk3DPoint& point)
{
    Eigen::Vector3f v;
    v.x() = point.x/1000;
    v.y() = point.y/1000;
    v.z() = point.z/1000;
    return v;
}

ftk3DPoint Vector3fToftk3DPoint(const Eigen::Vector3f& point)
{
    ftk3DPoint v;
    v.x = point.x()*1000;
    v.y = point.y()*1000;
    v.z = point.z()*1000;
    return v;
}

}}
#endif // _TO_EIGEN_HPP
