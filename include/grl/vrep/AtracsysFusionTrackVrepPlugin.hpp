#ifndef _ATRACSYS_FUSION_TRACK_VREP_PLUGIN_HPP_
#define _ATRACSYS_FUSION_TRACK_VREP_PLUGIN_HPP_

#include <iostream>
#include <memory>
#include <exception>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <atomic>

#include <spdlog/spdlog.h>

#include <boost/exception/all.hpp>
#include <boost/lexical_cast.hpp>

#include "grl/vrep/Eigen.hpp"
#include "grl/vrep/Vrep.hpp"

#include "grl/sensor/FusionTrack.hpp"
#include "grl/sensor/FusionTrackToEigen.hpp"

#include "v_repLib.h"

#include "grl/vector_ostream.hpp"

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
/// @todo separate out grl specific code from general atracsys control code
/// @todo Template on robot driver and create a driver that just reads/writes to/from the simulation, then pass the two templates so the simulation and the real driver can be selected.
///
/// Conceptually, this class loads up the vrep string identifiers for the objects whose position you want to modify
/// using the AtracsysFusionTrack data. This consists of the Object whose position you wish to modify, the object whose frame
/// it should be modified in, and a bool stating if the optical tracker's measurement should be inverted before applying the
/// position. This class will then constantly collect measurments, then set the object positions from the frames specified
/// for every object detected.
///
/// @note Skips geometries found based on the ini file that aren't actively configured silently.
class AtracsysFusionTrackVrepPlugin : public std::enable_shared_from_this<AtracsysFusionTrackVrepPlugin> {
public:

    /// @see MotionConfigParams
    /// @see VrepMotionConfigTuple
    enum MotionConfigParamsIndex {
        ObjectToMove,
        FrameInWhichToMoveObject,
        ObjectBeingMeasured,
        GeometryID ///< @note GeometryID does not apply to VrepMotionConfigTuple
    };

    typedef std::tuple<
        std::string, // ObjectToMove
        std::string, // FrameInWhichToMoveObject
        std::string, // ObjectBeingMeasured
        std::string  // GeometryID
      > MotionConfigParams;


    struct Params {
       /// the params for the underlying fusiontrack device
       grl::sensor::FusionTrack::Params FusionTrackParams;
       /// optical tracker base (frame of transform measurement)
       std::string OpticalTrackerBase;
        /// params for the objects, frames, and transform inversion to update object positions
       std::vector<MotionConfigParams> MotionConfigParamsVector;
    };


    static const Params defaultParams(){
      return moveBoneParams();
    }

    static const Params emptyDefaultParams(){
        std::vector<MotionConfigParams> motionConfigParams;

        return {grl::sensor::FusionTrack::emptyDefaultParams(),
                "OpticalTrackerBase#0",
                motionConfigParams};
    };

    static const Params moveTrackerParams(){
        std::vector<MotionConfigParams> motionConfigParams;
        motionConfigParams.push_back(std::make_tuple("Fiducial#22","OpticalTrackerBase#0","Fiducial#22","22"));

        return {grl::sensor::FusionTrack::defaultParams(),
                "OpticalTrackerBase#0",
                motionConfigParams};
    }

    static const Params moveBoneParams(){
        std::vector<MotionConfigParams> motionConfigParams;
        motionConfigParams.push_back(std::make_tuple("OpticalTrackerBase#0","Fiducial#55","Fiducial#55","55"));

        return {grl::sensor::FusionTrack::defaultParams(),
                "OpticalTrackerBase#0",
                motionConfigParams};
    }

/// @todo allow parameters to be updated
AtracsysFusionTrackVrepPlugin (Params params = defaultParams())
      :
      params_(params),
      isConnectionEstablished_(false),
      m_shouldStop(false)
{
/// @todo figure out how to re-enable when .so isn't loaded
 // initHandles();
}

/// construct() function completes initialization of the plugin
/// @todo move this into the actual constructor, but need to correctly handle or attach vrep shared libraries for unit tests.
void construct(){
  initHandles();
  m_driverThread.reset(new std::thread(&AtracsysFusionTrackVrepPlugin::update,this));
}

void destruct(){
   m_shouldStop = true;
   if(m_driverThread){
     m_driverThread->join();
   }
}

/// adds an object to active tracking, replacing existing objects with the same GeometryID
void add_object(const MotionConfigParams mcp){
   std::lock_guard<std::mutex> lock(m_frameAccess);
   MotionConfigParamsAddConfig(mcp,m_geometryIDToVrepMotionConfigMap);
}

/// @brief clears all actively tracked objects
/// Does not modify fusiontrack params, such as any loaded geometry ini config files.
void clear_objects(){
   std::lock_guard<std::mutex> lock(m_frameAccess);
   m_geometryIDToVrepMotionConfigMap.clear();
}

/// @brief Remove a geometry and the corresponding objects so they no longer receive tracking updates.
/// Does not modify fusiontrack params, such as any loaded geometry ini config files.
void remove_geometry(int GeometryID){
   std::lock_guard<std::mutex> lock(m_frameAccess);
   m_geometryIDToVrepMotionConfigMap.erase(GeometryID);
}

/// @brief Remove a geometry and the corresponding objects so they no longer receive tracking updates.
/// Does not modify fusiontrack params, such as any loaded geometry ini config files.
void remove_geometry(std::string GeometryID){
   remove_geometry( boost::lexical_cast<int>(GeometryID));
}

/// Is everything ok?
/// @return true if the optical tracker is actively running without any issues
/// @todo consider expanding to support real error codes
bool is_active()
{
  return allHandlesSet && !exceptionPtr && opticalTrackerP && isConnectionEstablished_;
}

void run_one(){

   // rethrow an exception if it occured in the other thread.
   if(exceptionPtr) {
     // note: this exception most likely came from the update() call initializing opticalTrackerP
     std::rethrow_exception(exceptionPtr);
   }

   // don't try to lock or start sending the tracker data
   // until the device has established a connection
   if (!isConnectionEstablished_ || !allHandlesSet) return;

   std::lock_guard<std::mutex> lock(m_frameAccess);

   // if any of the components haven't finished initializing, halt the program with an error
   BOOST_VERIFY(m_receivedFrame && m_nextState && opticalTrackerP);

   Eigen::Affine3f cameraToMarkerTransform;

   for(auto& marker : m_receivedFrame->Markers){

         cameraToMarkerTransform = sensor::ftkMarkerToAffine3f(marker);
         auto configIterator = m_geometryIDToVrepMotionConfigMap.find(marker.geometryId);
         if(configIterator==m_geometryIDToVrepMotionConfigMap.end()) continue; // no configuration for this item
         auto config = configIterator->second;

         // invert the transform from the tracker to the object if needed
         if(m_opticalTrackerBase == std::get<ObjectToMove>(config) &&
            std::get<FrameInWhichToMoveObject>(config) == std::get<ObjectBeingMeasured>(config)) {
           cameraToMarkerTransform = cameraToMarkerTransform.inverse();
         } else if( std::get<FrameInWhichToMoveObject>(config) != m_opticalTrackerBase ){
           BOOST_THROW_EXCEPTION(std::runtime_error("AtracsysFusionTrackVrepPlugin: moving objects other than those being measured and the base itself are not yet supported."));
         }

         setObjectTransform(std::get<ObjectToMove>(config), std::get<FrameInWhichToMoveObject>(config), cameraToMarkerTransform);
   }
}

~AtracsysFusionTrackVrepPlugin(){
  destruct();
}

private:


/// @todo support boost::asio
/// Reads data off of the real optical tracker device in a separate thread
void update() {
   try {
      // initialize all of the real device states
      std::lock_guard<std::mutex> lock(m_frameAccess);
      opticalTrackerP.reset(new grl::sensor::FusionTrack(params_.FusionTrackParams));
      m_receivedFrame.reset(new grl::sensor::FusionTrack::Frame());
      m_nextState.reset(new grl::sensor::FusionTrack::Frame());
      isConnectionEstablished_ = true;
   } catch(...) {
      // transport the exception to the main thread in a safe manner
      exceptionPtr = std::current_exception();
      m_shouldStop = true;
   }

   // run the primary update loop in a separate thread
   while (!m_shouldStop) {
    opticalTrackerP->receive(*m_nextState);
    std::lock_guard<std::mutex> lock(m_frameAccess);
    std::swap(m_receivedFrame,m_nextState);
   }

}


void initHandles() {
    m_opticalTrackerBase = grl::vrep::getHandle(params_.OpticalTrackerBase);
    m_geometryIDToVrepMotionConfigMap = MotionConfigParamsToVrepHandleConfigMap(params_.MotionConfigParamsVector);
	allHandlesSet  = true;
}


private:

Params params_;


int m_opticalTrackerBase = -1;
std::unique_ptr<grl::sensor::FusionTrack> opticalTrackerP;

bool allHandlesSet = false;
/// simple conditional for starting actually setting positions
/// @see update() run_one()
std::atomic<bool> isConnectionEstablished_;





std::mutex m_frameAccess;

std::unique_ptr<grl::sensor::FusionTrack::Frame> m_receivedFrame;
std::unique_ptr<grl::sensor::FusionTrack::Frame> m_nextState;
std::atomic<bool> m_shouldStop;
std::exception_ptr exceptionPtr;

std::unique_ptr<std::thread> m_driverThread;

// note: elements match up with MotionConfigParams and MotionConfigParamsIndex, except here there are 3 and the main int is the geometryID.
// The first int is the object that is being moved
// The second int is the frame the object is being moved within
// The third int is the object being measured by the optical tracker.
typedef std::tuple<int,int,int> VrepMotionConfigTuple;
typedef std::map<int, VrepMotionConfigTuple> GeometryIDToVrepMotionConfigMap;

GeometryIDToVrepMotionConfigMap m_geometryIDToVrepMotionConfigMap;

/// Adds a configuration to to a config map
static void MotionConfigParamsAddConfig(const MotionConfigParams& motionConfig, GeometryIDToVrepMotionConfigMap& IDToHandleConfig)
{
       IDToHandleConfig[boost::lexical_cast<int>(std::get<GeometryID>(motionConfig))] =
         std::make_tuple(
             grl::vrep::getHandleFromParam<ObjectToMove            >(motionConfig),
             grl::vrep::getHandleFromParam<FrameInWhichToMoveObject>(motionConfig),
             grl::vrep::getHandleFromParam<ObjectBeingMeasured     >(motionConfig)
          );
}

// converts the string identifiers for objects to integer handle identifiers
// for use in updating the position of objects.
template<typename InputIterator>
GeometryIDToVrepMotionConfigMap
MotionConfigParamsToVrepHandleConfigMap(const InputIterator& configurations)
{
    GeometryIDToVrepMotionConfigMap IDToHandleConfig;
    for(auto&& motionConfig : configurations){
      MotionConfigParamsAddConfig(motionConfig,IDToHandleConfig);
    }
    return IDToHandleConfig;
}

};

}

#endif // _ATRACSYS_FUSION_TRACK_VREP_PLUGIN_HPP_
