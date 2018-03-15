#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE KinematicsCalibration_test
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include <exception>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <vector>
#include <boost/log/trivial.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigen>

// SpaceVecAlg
// https://github.com/jrl-umi3218/SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>


// RBDyn
// https://github.com/jrl-umi3218/RBDyn
#include <RBDyn/Body.h>
#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/FK.h>

// mc_rbdyn_urdf
// https://github.com/jrl-umi3218/mc_rbdyn_urdf
#include <mc_rbdyn_urdf/urdf.h>

//// local includes
// #include "grl/vrep/KukaLBRiiwaVrepPlugin.hpp"

#include "grl/flatbuffer/FlatbufferToEigen.hpp"
#include "grl/vrep/SpaceVecAlg.hpp"


#include "kukaiiwaURDF.h"

const double degreesToRadians = M_PI/180.0;
const double radiansTodegrees = 180/M_PI;

BOOST_AUTO_TEST_SUITE(KinematicsCalibration_test)


namespace cst = boost::math::constants;
auto strRobot = mc_rbdyn_urdf::rbdyn_from_urdf(XYZSarmUrdf);
rbd::MultiBody mb = strRobot.mb;
rbd::MultiBodyConfig mbc(mb);
rbd::MultiBodyGraph mbg(strRobot.mbg);
std::size_t nrJoints = mb.nrJoints();
std::size_t nrBodies = strRobot.mb.nrBodies();


BOOST_AUTO_TEST_CASE(ForwardKinematics)
{
    Eigen::MatrixXd testJointAngles(5,7);  // Degree
    testJointAngles << -0.06,	-0.03,	0.01,	-0.06,	-0.04,	-0.02,	0.1,
    -0.06,  90.09,	0.01,	-0.07,	-0.04,	-0.01,	0.1,
    -0.06,	-0.06,	90.06,	-0.06,	-0.04,	-0.02,	0.1,
    90.03,	-0.06,	0.06,	-0.06,	-0.04,	-0.02,	0.1,
    0.05,	-0.06,	0.06,	-91.87,	0,	45.48,	0.1;

    testJointAngles = degreesToRadians*testJointAngles;

    Eigen::MatrixXd cartesianPositions(5,6);  // Degree and mm
    cartesianPositions << 0.02,	   0,	   1306,	0,	     0.01,	 0,
                          946,     -1.04,  357.97,  154.15,  89.84,  154.22,
                          -1,      0.5,	   1306,	90.06,   0.04,	 -0.06,
                          0,	   -0.5,   1306,	90.14,	 -0.02,	 0.06,
                          484.81,  0.85,   674.72,	179.97,	 42.7,	 179.91;

    std::size_t testSize = testJointAngles.rows();
    std::size_t jointNum = testJointAngles.cols();
    std::size_t poseDim = cartesianPositions.cols();
    std::vector<std::string> jointNames;
    std::vector<sva::PTransformd> EEpose;

    for(int testIdx=0; testIdx<testSize; testIdx++){
        Eigen::VectorXd oneStateJointPosition = testJointAngles.row(testIdx);
        int jointIdx = 0;

        auto transfroms = mb.transforms();
        for(int jointIndex = 0; jointIndex< nrJoints; jointIndex++) {
            const auto & joint = strRobot.mb.joint(jointIndex);
            jointNames.push_back(joint.name());
            std::cout << "Joint Name: " << joint.name() << "     Size: " << mbc.q[jointIndex].size() << std::endl;
            if (mbc.q[jointIndex].size() > 0) {
                mbc.q[jointIndex][0] = oneStateJointPosition[jointIdx];
                jointIdx++;
            }
            std::cout << "Rotation: " << std::endl << transfroms[jointIndex].rotation() << std::endl;
            std::cout << "Translation: " << std::endl << transfroms[jointIndex].translation() << std::endl;
        }
        rbd::forwardKinematics(mb, mbc);
        // Pose of the end effector relative to robot base frame.
        // body_size-1: end-effector with marker
        // body_size-2: Flange, here this should be used.
        EEpose.push_back(mbc.bodyPosW[nrBodies-2]);


        for(int i=0; i<nrBodies; i++){
            const auto & link = strRobot.mb.body(i);
            std::cout << link.name() << std::endl;
            std::cout << mbc.bodyPosW[i].rotation()<< std::endl;
            std::cout << mbc.bodyPosW[i].translation()<< std::endl;
        }
    }
    Eigen::MatrixXd pluckerpose = grl::getPluckerPose(EEpose );
    pluckerpose.block<5,3>(0,0) = 1000*pluckerpose.block<5,3>(0,0);
    pluckerpose.block<5,3>(0,3) = radiansTodegrees*pluckerpose.block<5,3>(0,3);
    std::cout << pluckerpose << std::endl;
    std::cout << "---------------------------------" << std::endl;


/////////////////////////////////////////////////////////////////////////////////////////
cartesianPositions.block<5,3>(0,3) = degreesToRadians*cartesianPositions.block<5,3>(0,3);
std::vector<sva::PTransformd> PTransform;
for(int testIdx=0; testIdx<testSize; testIdx++){
    Eigen::VectorXd oneCartesianPos = cartesianPositions.row(testIdx);
    Eigen::Vector3d trans;
    trans << oneCartesianPos[0], oneCartesianPos[1], oneCartesianPos[2];
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(oneCartesianPos[5], Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(oneCartesianPos[4], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(oneCartesianPos[3], Eigen::Vector3d::UnitZ());

    sva::PTransformd onePTransform(m, trans);
    PTransform.push_back(onePTransform);


    // std::cout << "original rotation:" << std::endl;
    // std::cout << m << std::endl << std::endl;

    // Eigen::Vector3d ea = m.eulerAngles(0, 1, 2);
    // std::cout << "to Euler angles:" << std::endl;Pack
    // std::cout << radiansTodegrees*ea << std::endl << std::endl;

    // Eigen::Matrix3d n;
    // n = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX())
    //    *Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY())
    //    *Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ());
    // std::cout << "recalc original rotation:" << std::endl;
    // std::cout << n << std::endl;

}



}
}

// BOOST_AUTO_TEST_CASE(PlanarMotion)
// {

// }

// BOOST_AUTO_TEST_CASE(PlanarMotionWithNoise)
// {


// }



