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
                    "RobotTip#0"              , // RobotTipHandle,
                    "RobotTarget#0"           , // RobotTargetHandle,
                    "Robotiiwa"               , // RobotTargetBaseHandle,
                    "OpticalTrackerBase#0"    , // RobotTargetBaseHandle,
                    "Fiducial#55"               // FiducialArmTip
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


//void run_one(){
//
//  if(!allHandlesSet) BOOST_THROW_EXCEPTION(std::runtime_error("KukaVrepPlugin: Handles have not been initialized, cannot run updates."));
//  getRealKukaAngles();
//  getStateFromVrep();
//  /// @todo re-enable simulation feedback based on actual kuka state
//  //updateVrepFromKuka();
//  sendSimulatedJointAnglesToKuka();
//
//}


//~HandEyeCalibrationVrepPlugin(){
//    if(driver_threadP){
//	  //workP.reset();
//      device_driver_io_service.stop();
//      driver_threadP->join();
//    }
//}

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

void getFrame() {
    if(!allHandlesSet) return false;
    
    auto robotTipInBaseRotAndTrans = getAxisAngleAndTranslation(robotTip,robotTargetBase);
    rvecsArm.push_back(robotTipInBaseRotAndTrans.first);
    tvecsArm.push_back(robotTipInBaseRotAndTrans.second);
    
    auto fiducialInCameraBase      = getAxisAngleAndTranslation(handEyeCalibFiducial,opticalTrackerBase);

}

/// @todo probably want to run at least part of this in a separate thread.
void estimateHandEyeScrew(){
  
  Eigen::Matrix4d transformEstimate;
  handEyeCalib.estimateHandEyeScrew(
      rvecsArm,
      tvecsArm,
      rvecsFiducial,
      tvecsFiducial,
      transformEstimate
      );

      /// @todo set camera position based on this, print out estimate,
  
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