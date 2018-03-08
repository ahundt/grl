#ifndef GRL_FLATBUFFER_TO_EIGEN
#define GRL_FLATBUFFER_TO_EIGEN

#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE
// // grl

/// Can't include the vrep header files.
// #include "grl/vrep/Vrep.hpp"
// #include "grl/vrep/VrepRobotArmDriver.hpp"
// #include "grl/vrep/VrepRobotArmJacobian.hpp"

// #include "grl/vrep/Eigen.hpp"
// #include "grl/vrep/Vrep.hpp"
// #include "camodocal/calib/HandEyeCalibration.h"

// #include "v_repLib.h"
// FusionTrack Libraries

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "camodocal/EigenUtils.h"




namespace grl {

    Eigen::Affine3f MarkerPoseToAffine3f(const Eigen::VectorXd& markerPose){

        assert(markerPose.size() ==  7);
        Eigen::Affine3f markerToCameraTransform;
        markerToCameraTransform.translation().x() = markerPose[0];
        markerToCameraTransform.translation().y() = markerPose[1];
        markerToCameraTransform.translation().z() = markerPose[2];
        Eigen::Quaterniond q(markerPose[3], markerPose[4], markerPose[5], markerPose[6]);
        Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        markerToCameraTransform.matrix().block<3,3>(0,0) = R.cast<float>();
        return markerToCameraTransform;
    }
    Eigen::VectorXd Affine3fToMarkerPose(const Eigen::Affine3f& markerToCameraTransform){
            Eigen::VectorXd markerPose(7);

            markerPose[0] = markerToCameraTransform.translation().x();
            markerPose[1] = markerToCameraTransform.translation().y();
            markerPose[2] = markerToCameraTransform.translation().z();
            Eigen::Matrix3f R = markerToCameraTransform.matrix().block<3,3>(0,0);
            Eigen::Quaterniond q( R.cast<double>());
            markerPose[3] = q.w();
            markerPose[4] = q.x();
            markerPose[5] = q.y();
            markerPose[6] = q.z();

            return markerPose;
    }

    Eigen::MatrixXd getPluckerPose(std::vector<sva::PTransformd>& Pose ) {
        std::size_t pose_size = Pose.size();
        Eigen::MatrixXd PKPose(pose_size,6);
        for(int i=0; i<pose_size; ++i) {
            Eigen::Matrix3d E  = Pose[i].rotation();    // rotation
            Eigen::Vector3d r = Pose[i].translation();    // translation
            Eigen::Vector3d eulerAngleEigen = E.eulerAngles(0,1,2);
            // Eigen::Quaterniond q(E);
            Eigen::RowVectorXd pose(6);
            pose << r.transpose(), eulerAngleEigen.transpose();
            PKPose.row(i) = pose;
        }
        return PKPose;
    }

    Eigen::RowVectorXd getPluckerPose(const Eigen::VectorXd& markerPose){
        assert(markerPose.size() ==  7);
        Eigen::Vector3d r;
        r[0] = markerPose[0];
        r[1] = markerPose[1];
        r[2] = markerPose[2];
        Eigen::Quaterniond q(markerPose[3], markerPose[4], markerPose[5], markerPose[6]);
        Eigen::Matrix3d E = q.normalized().toRotationMatrix();
        Eigen::Vector3d eulerAngleEigen = E.eulerAngles(0,1,2);
        Eigen::RowVectorXd pose(6);
        pose << r.transpose(), eulerAngleEigen.transpose();
        return pose;
    }

    std::vector<sva::PTransformd> invertPose(const std::vector<sva::PTransformd>& poseTransformd){
        std::size_t size = poseTransformd.size();
        std::vector<sva::PTransformd> inversePose(size);

        for(int i=0; i<size;++i){
            auto pose = poseTransformd[i];
            auto markerToRobotTransform = PTranformToEigenAffine(pose);
            auto robotTomarkerTransform = markerToRobotTransform.inverse();
            auto markerPose = eigenAffineToPtransform(robotTomarkerTransform);
            inversePose[i] = markerPose;

        }
        return std::move(inversePose);
    }



}
#endif