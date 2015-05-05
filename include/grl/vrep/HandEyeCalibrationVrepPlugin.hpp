#ifndef _HAND_EYE_CALIBRATION_VREP_PLUGIN_HPP_
#define _HAND_EYE_CALIBRATION_VREP_PLUGIN_HPP_

#include <iostream>
#include <memory>

#include <boost/log/trivial.hpp>
#include <boost/exception/all.hpp>

#include "grl/vrep/Eigen.hpp"
#include "grl/vrep/Vrep.hpp"
#include "camodocal/calib/HandEyeCalibration.h"

#include "v_repLib.h"

/// @todo move elsewhere, because it will conflict with others' implementations of outputting vectors
template<typename T>
inline boost::log::formatting_ostream& operator<<(boost::log::formatting_ostream& out,  std::vector<T>& v)
{
    out << "[";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last) 
            out << ", ";
    }
    out << "]";
    return out;
}

namespace grl {



/// Creates a complete vrep plugin object
/// usage:
/// @code
///    auto kukaPluginPG = std::make_shared<grl::KukaVrepPlugin>();
///    kukaPluginPG->construct();
///    while(true) kukaPluginPG->run_one();
/// @endcode
///
/// @todo this implementation is a bit hacky, redesign it
/// @todo separate out grl specific code from general kuka control code
/// @todo Template on robot driver and create a driver that just reads/writes to/from the simulation, then pass the two templates so the simulation and the real driver can be selected.
class HandEyeCalibrationVrepPlugin : public std::enable_shared_from_this<HandEyeCalibrationVrepPlugin> {
public:

    enum ParamIndex {
        RobotTipName,
        RobotTargetName,
        RobotTargetBaseName,
        OpticalTrackerBaseName,
        HandEyeCalibrationFiducial
    };
    
    /// @todo allow default params
    typedef std::tuple<
        std::string,
        std::string,
        std::string,
        std::string,
        std::string
        > Params;
    
    
    static const Params defaultParams(){
        return std::make_tuple(
                    "RobotFlangeTip"              , // RobotTipHandle,
                    "RobotFlangeTipTarget"        , // RobotTargetHandle,
                    "Robotiiwa"               , // RobotTargetBaseHandle,
                    "OpticalTrackerBase#0"    , // RobotTargetBaseHandle,
                    "Fiducial#22"               // FiducialArmTip
                );
    }

/// @todo allow KukaFRIThreadSeparator parameters to be updated
HandEyeCalibrationVrepPlugin (Params params = defaultParams())
      :
      params_(params)
{
/// @todo figure out how to re-enable when .so isn't loaded
 // initHandles();
  
}

/// construct() function completes initialization of the plugin
/// @todo move this into the actual constructor, but need to correctly handle or attach vrep shared libraries for unit tests.
void construct(){
  initHandles();
}


void addFrame() {
    
    auto robotTipInRobotTargetBaseRotAndTrans = getAxisAngleAndTranslation(robotTip,robotTargetBase);
    rvecsArm.push_back(robotTipInRobotTargetBaseRotAndTrans.first);
    tvecsArm.push_back(robotTipInRobotTargetBaseRotAndTrans.second);
    
    auto fiducialInOpticalTrackerBaseRotAndTrans = getAxisAngleAndTranslation(handEyeCalibFiducial,robotTargetBase);
    rvecsFiducial.push_back(fiducialInOpticalTrackerBaseRotAndTrans.first);
    tvecsFiducial.push_back(fiducialInOpticalTrackerBaseRotAndTrans.second);
}

/// @todo probably want to run at least part of this in a separate thread.
void estimateHandEyeScrew(){
  
  static const bool debug = true;
  
  // estimate transform between end effector and fiducial
  Eigen::Affine3d transformEstimate;
  handEyeCalib.estimateHandEyeScrew(
      rvecsFiducial,
      tvecsFiducial,
      rvecsArm,
      tvecsArm,
      transformEstimate.matrix()
      );
    
    std::array<float,3> simTipPosition;
    std::array<float,4> simTipQuaternion;
    
    // get fiducial in optical tracker base frame
    int ret = simGetObjectPosition(opticalTrackerBase, handEyeCalibFiducial, simTipPosition.begin());
    if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("HandEyeCalibrationVrepPlugin: Could not get position"));
    ret = simGetObjectQuaternion(opticalTrackerBase, handEyeCalibFiducial, simTipQuaternion.begin());
    if(ret==-1) BOOST_THROW_EXCEPTION(std::runtime_error("HandEyeCalibrationVrepPlugin: Could not get quaternion"));

   if(debug){
     // print simulation transfrom from tip to fiducial
     Eigen::Affine3d RobotTipToFiducial = getObjectTransform(handEyeCalibFiducial,robotTip);
     BOOST_LOG_TRIVIAL(info) << poseString(RobotTipToFiducial,"expected RobotTipToFiducial (simulation only):") << std::endl;
   }

   BOOST_LOG_TRIVIAL(info) << poseString(transformEstimate,"estimated RobotTipToFiducial:") << std::endl;
   // set transform between end effector and fiducial
   setObjectTransform(handEyeCalibFiducial, robotTip, transformEstimate);
   
    
   // set real optical tracker position and orientation relative to the robot fiducial
   simSetObjectPosition(opticalTrackerBase, handEyeCalibFiducial, simTipPosition.begin());
   simSetObjectQuaternion(opticalTrackerBase, handEyeCalibFiducial, simTipQuaternion.begin());
   
   // print results
   Eigen::Quaterniond eigenQuat(transformEstimate.rotation());
   BOOST_LOG_TRIVIAL(info) << "Hand Eye Screw Estimate quat wxyz: " << eigenQuat.w() << " " << eigenQuat.x() << " " << eigenQuat.y() << " " << eigenQuat.z() << " " << " translation xyz: " << transformEstimate.translation().x() << " " << transformEstimate.translation().y() << " " << transformEstimate.translation().z() << " " << std::endl;
    
    BOOST_LOG_TRIVIAL(info) << "Optical Tracker Base Estimate quat wxyz: " << simTipQuaternion[0] << " " << simTipQuaternion[1] << " " << simTipQuaternion[2] << " " << simTipQuaternion[3] << " " << " translation xyz: " << simTipPosition[0] << " " << simTipPosition[1] << " " << simTipPosition[2] << " " << std::endl;
  
}
private:



/// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
/// @todo throw an exception if any of the handles is -1
void initHandles() {

	robotTip             = getHandleFromParam<RobotTipName>              (params_);					//Obtain RobotTip handle
	target               = getHandleFromParam<RobotTargetName>           (params_);
	robotTargetBase      = getHandleFromParam<RobotTargetBaseName>       (params_);
	opticalTrackerBase   = getHandleFromParam<OpticalTrackerBaseName>    (params_);
	handEyeCalibFiducial = getHandleFromParam<HandEyeCalibrationFiducial>(params_);
    
	allHandlesSet  = true;
}


camodocal::HandEyeCalibration handEyeCalib;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rvecsArm;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tvecsArm;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rvecsFiducial;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tvecsFiducial;


int robotTip = -1;					//Obtain RobotTip handle
int target = -1;
int robotTargetBase = -1;
int handEyeCalibFiducial = -1;
int opticalTrackerBase = -1;
//int ImplantCutPath = -1;
//int BallJointPath = -1;
//int bone = -1;

bool allHandlesSet = false;


private:
Params params_;

};
  
}

#endif