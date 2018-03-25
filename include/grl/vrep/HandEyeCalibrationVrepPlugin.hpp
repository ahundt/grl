#ifndef _HAND_EYE_CALIBRATION_VREP_PLUGIN_HPP_
#define _HAND_EYE_CALIBRATION_VREP_PLUGIN_HPP_

#include <iostream>
#include <memory>

#include <boost/exception/all.hpp>
#include <spdlog/spdlog.h>

#include "grl/vrep/Eigen.hpp"
#include "grl/vrep/Vrep.hpp"
#include "grl/time.hpp"
#include "camodocal/calib/HandEyeCalibration.h"

#include "v_repLib.h"

#include "grl/vector_ostream.hpp"
// boost::filesystem
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>


namespace grl {



/// Creates a complete vrep plugin object
/// usage:
/// @code
///    auto handEyeCalib = std::make_shared<grl::HandEyeCalibrationVrepPlugin>();
///    handEyeCalib->construct();
///    // move objects to new position
/// @endcode
///
/// @todo this implementation is a bit hacky, redesign it
/// @todo separate out grl specific code from general kuka control code
/// @todo Template on robot driver and create a driver that just reads/writes to/from the simulation, then pass the two templates so the simulation and the real driver can be selected.
class HandEyeCalibrationVrepPlugin : public std::enable_shared_from_this<HandEyeCalibrationVrepPlugin> {
public:

  static const bool debug = false;

    enum ParamIndex {
        RobotBaseName,             ///<  V-REP handle string for the base of the first known transform, often a robot base
        RobotTipName,              ///<  V-REP handle string for the tip of the first known transform, often a robot tip
        OpticalTrackerBaseName,    ///<  V-REP handle string for the tip of the first known transform, often a robot tip
        OpticalTrackerDetectedObjectName
    };

    /// @todo allow default params
    typedef std::tuple<
        std::string,
        std::string,
        std::string,
        std::string
        > Params;


    static const Params defaultParams(){
        return std::make_tuple(
                    "Robotiiwa"               , // RobotBaseHandle,
                    "RobotFlangeTip"          , // RobotTipHandle,
                    "OpticalTrackerBase#0"    , // OpticalTrackerBaseName,
                    "Fiducial#22"               // FiducialArmTip
                );
    }

/// @todo allow KukaFRIThreadSeparator parameters to be updated
HandEyeCalibrationVrepPlugin (Params params = defaultParams())
      :
      isFirstFrame(true),
      frameCount(0),
      params_(params)
{
/// @todo figure out how to re-enable when .so isn't loaded
 // initHandles();
}

/// construct() function completes initialization of the plugin
/// @todo move this into the actual constructor, but need to correctly handle or attach vrep shared libraries for unit tests.
void construct(){
  logger_ = spdlog::get("console");
  initHandles();

  // set the current transformEstimate to the initial estimate already in vrep
  if(allHandlesSet) transformEstimate = getObjectTransform(opticalTrackerDetectedObjectName,robotTip);
}


void addFrame() {
   logger_->info( "Adding hand eye calibration frame {}", ++frameCount);

    auto robotTipInRobotBase    = getObjectTransform(robotTip,robotBase);
    auto fiducialInOpticalTrackerBase = getObjectTransform(opticalTrackerDetectedObjectName,opticalTrackerBase);

    if(isFirstFrame){
      firstRobotTipInRobotBaseInverse             = robotTipInRobotBase.inverse();
      firstFiducialInOpticalTrackerBaseInverse    = fiducialInOpticalTrackerBase.inverse();
      isFirstFrame = false;
    }

    auto robotTipInFirstTipBase      = firstRobotTipInRobotBaseInverse * robotTipInRobotBase;        // B_0_Inv * B_i
    auto fiducialInFirstFiducialBase = firstFiducialInOpticalTrackerBaseInverse * fiducialInOpticalTrackerBase;  // A_0_Inv * A_i

    auto rvec1 = eigenRotToEigenVector3dAngleAxis(robotTipInFirstTipBase.rotation());
    auto tvec1 = robotTipInFirstTipBase.translation();
    auto rvec2 = eigenRotToEigenVector3dAngleAxis(fiducialInFirstFiducialBase.rotation());
    auto tvec2 = fiducialInFirstFiducialBase.translation();

    if (rvec1.norm() != 0 && rvec2.norm() != 0) {

        rvecsArm.push_back(rvec1);
        tvecsArm.push_back(tvec1);

        rvecsFiducial.push_back(rvec2);
        tvecsFiducial.push_back(tvec2);
    } else {
      std::cout << "Empty Vector" << std::endl;
      std::cout << rvec1 << std::endl;
      std::cout << rvec2 << std::endl;
    }


   if(debug){

       std::cout << "rvec1: " << std::endl << rvec1 << std::endl;
       std::cout << "tvec1: " << std::endl << tvec1 << std::endl;
       std::cout << "rvec2: " << std::endl << rvec2 << std::endl;
       std::cout << "tvec2: " << std::endl << tvec2 << std::endl;

       logger_->info( "\nrobotTipInRobotBase: \n{}", poseString(robotTipInRobotBase));
       logger_->info( "\nfiducialInOpticalTrackerBase: \n{}", poseString(fiducialInOpticalTrackerBase));

       logger_->info( "\nrobotTipInFirstTipBase: \n{}", poseString(robotTipInFirstTipBase));
       logger_->info( "\nfiducialInFirstFiducialBase: \n{}", poseString(fiducialInFirstFiducialBase));

       // print simulation transfrom from tip to fiducial
       Eigen::Affine3d RobotTipToFiducial = getObjectTransform(opticalTrackerDetectedObjectName,robotTip);
       logger_->info( poseString(RobotTipToFiducial,"expected RobotTipToFiducial (simulation only): "));

       //BOOST_VERIFY(robotTipInFirstTipBase.translation().norm() - fiducialInFirstFiducialBase.translation().norm() < 0.1);
   }
}

/// @brief run solver to estimate the unknown transform and set the simulation to the estimated value
///
/// after calling addFrame a number of times in different positions and orientations
///
/// @todo probably want to run at least part of this in a separate thread.
/// @todo evaluate if applyEstimate should not be called by this
void estimateHandEyeScrew(){

   logger_->info(  "Running Hand Eye Screw Estimate with the following numbers of entries in each category:  rvecsFiducial {} , tvecsFiducial: {} , rvecsArm: {}  , tvecsArm: {}"
   ,rvecsFiducial.size(), tvecsFiducial.size(), rvecsArm.size(), tvecsArm.size());

   BOOST_VERIFY(allHandlesSet);


  handEyeCalib.estimateHandEyeScrew(
      rvecsArm,
      tvecsArm,
      rvecsFiducial,
      tvecsFiducial,
      transformEstimate.matrix(),
      false  //PlanarMotion
  );


    // get fiducial in optical tracker base frame
    int ret = simGetObjectPosition(opticalTrackerDetectedObjectName, opticalTrackerBase, detectedObjectPosition.begin());
    if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("HandEyeCalibrationVrepPlugin: Could not get position"));
    ret = simGetObjectQuaternion(opticalTrackerDetectedObjectName, opticalTrackerBase, detectedObjectQuaternion.begin());
    if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("HandEyeCalibrationVrepPlugin: Could not get quaternion"));

   if(debug){
     // print simulation transfrom from tip to fiducial
     Eigen::Affine3d RobotTipToFiducial = getObjectTransform(opticalTrackerDetectedObjectName,robotTip);
     logger_->info(  "\n{}", poseString(RobotTipToFiducial,"expected RobotTipToFiducial (simulation only): "));
   }

   


   logger_->info( "\n{}", poseString(transformEstimate,"estimated RobotTipToFiducial:"));

   applyEstimate();

   // print results
   
   Eigen::Quaterniond eigenQuat(transformEstimate.rotation());
   logger_->info( "Hand Eye Screw Estimate quat wxyz\n: {} , {} , {} ,  {} \n  translation xyz: {}  {}  {}",
                  eigenQuat.w(), eigenQuat.x(), eigenQuat.y(),eigenQuat.z(), transformEstimate.translation().x(), transformEstimate.translation().y(), transformEstimate.translation().z());

   logger_->info( "Optical Tracker Base Measured quat wxyz\n: {} ,  {} , {} , {} \n translation xyz: {} , {}, {}",
     detectedObjectQuaternion[0], detectedObjectQuaternion[1], detectedObjectQuaternion[2], detectedObjectQuaternion[3],
     detectedObjectPosition[0], detectedObjectPosition[1], detectedObjectPosition[2]);


     // Write the hand eye calibration result into a file
   auto myFile = boost::filesystem::current_path() /"HandEyeCalibration_Result.txt";
   boost::filesystem::ofstream ofs(myFile/*.native()*/);
   // boost::archive::text_oarchive ta(ofs);
   ofs <<"\n\n=========== " + current_date_and_time_string() + " =======================================\n";
   ofs << "Hand Eye Screw Estimate quat wxyz:  " << eigenQuat.w() << "  "<< eigenQuat.x() << "  " << eigenQuat.y() << "  " << eigenQuat.z() 
      << "\ntranslation xyz: " << transformEstimate.translation().x() << "  " 
      << transformEstimate.translation().y() << "  " << transformEstimate.translation().z();

}

/// @brief  Will apply the stored estimate to the v-rep simulation value
///
/// A default transform is saved when construct() was called
/// so if no estimate has been found that will be used.
void applyEstimate(){

   // set transform between end effector and fiducial
   setObjectTransform(opticalTrackerDetectedObjectName, robotTip, transformEstimate);
}

Eigen::Affine3d getEstimate(){
   return transformEstimate;
}


/// @brief Correct modified transform for measured optical tracker data
///
/// If real time transform measurements are happening applyEstimate will move the measured object
/// relative to the sensor and this puts the transform measurement back so the object
/// appears in its real position. However, if running in simulation only
/// the sensor will be in the correct position, so you don't want to call this.
void restoreSensorPosition(){
   // set real optical tracker position and orientation relative to the robot fiducial
   simSetObjectPosition  (opticalTrackerBase, opticalTrackerDetectedObjectName, detectedObjectPosition.begin());
   simSetObjectQuaternion(opticalTrackerBase, opticalTrackerDetectedObjectName, detectedObjectQuaternion.begin());
}

private:



/// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
/// @todo throw an exception if any of the handles is -1
void initHandles() {

	robotTip                         = grl::vrep::getHandleFromParam<RobotTipName>                    (params_);					//Obtain RobotTip handle
	robotBase                        = grl::vrep::getHandleFromParam<RobotBaseName>                   (params_);
	opticalTrackerBase               = grl::vrep::getHandleFromParam<OpticalTrackerBaseName>          (params_);
	opticalTrackerDetectedObjectName = grl::vrep::getHandleFromParam<OpticalTrackerDetectedObjectName>(params_);

	allHandlesSet  = true;
}


camodocal::HandEyeCalibration handEyeCalib;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rvecsArm;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tvecsArm;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rvecsFiducial;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tvecsFiducial;

bool isFirstFrame;
std::size_t frameCount;
Eigen::Affine3d firstRobotTipInRobotBaseInverse;
Eigen::Affine3d firstFiducialInOpticalTrackerBaseInverse;

///
std::array<float,3> detectedObjectPosition;
std::array<float,4> detectedObjectQuaternion;
/// estimate transform between end effector and fiducial
Eigen::Affine3d transformEstimate;

int robotTip = -1;					//Obtain RobotTip handle
int robotBase = -1;
int opticalTrackerDetectedObjectName = -1;
int opticalTrackerBase = -1;
//int ImplantCutPath = -1;
//int BallJointPath = -1;
//int bone = -1;

bool allHandlesSet = false;


private:
Params params_;
std::shared_ptr<spdlog::logger> logger_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif
