#ifndef _ATRACSYS_FUSION_TRACK_VREP_PLUGIN_HPP_
#define _ATRACSYS_FUSION_TRACK_VREP_PLUGIN_HPP_

#include <iostream>
#include <memory>
#include <exception>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <atomic>
#include <ctime>

#include <spdlog/spdlog.h>

#include <boost/exception/all.hpp>
#include <boost/lexical_cast.hpp>

#include "grl/sensor/FusionTrack.hpp"
#include "grl/sensor/FusionTrackToEigen.hpp"
#include "grl/sensor/FusionTrackToFlatbuffer.hpp"

#include "grl/vrep/Eigen.hpp"
#include "grl/vrep/Vrep.hpp"
#include "grl/time.hpp"

#include "v_repLib.h"

#include "grl/vector_ostream.hpp"

#include <flatbuffers/flatbuffers.h>
#include <flatbuffers/util.h>
#include <flatbuffers/idl.h>

// FusionTrackLogAndTrack is the primary class used by
// AtracsysFusionTrackVrepPlugin
#include "grl/sensor/FusionTrackLogAndTrack.hpp"

namespace grl
{

  /// Enumerates which objects exist and how they will be moved when
  /// the tracker gets an update from a marker.
  /// These strings are converted to integer id handles by vrep.
  ///
  /// @see FusionTrackLogAndTrack::MotionConfigParamsIndex
  /// @see FusionTrackLogAndTrack::MotionConfigParams
  typedef std::tuple<
  std::string, // FusionTrackLogAndTrack::ObjectToMove, example "Fiducial#22"
  std::string, // FusionTrackLogAndTrack::FrameInWhichToMoveObject, example "OpticalTrackerBase#0"
  std::string, // FusionTrackLogAndTrack::ObjectBeingMeasured, example "Fiducial#22"
  std::string  // FusionTrackLogAndTrack::GeometryID, example "22" or "55", convert directly from string to int (not a vrep object name)
  > VrepStringMotionConfigParams;

  /// convert vrep string names to vrep handle integer ids
  /// for the FusionTrackLogAndTrack class's MotionConfigParams data.
  FusionTrackLogAndTrack::MotionConfigParams VrepStringToLogAndTrackMotionConfigParams(const VrepStringMotionConfigParams& strings)
  {
    /// boost::lexical_cast, that can convert numbers from strings to numeric types like int or double and vice versa.
    int geometry_id = boost::lexical_cast<int>(std::get<FusionTrackLogAndTrack::GeometryID>(strings));

    return std::make_tuple(
      grl::vrep::getHandleFromParam<FusionTrackLogAndTrack::MotionConfigParamsIndex::ObjectToMove>(strings),
      grl::vrep::getHandleFromParam<FusionTrackLogAndTrack::MotionConfigParamsIndex::FrameInWhichToMoveObject>(strings),
      grl::vrep::getHandleFromParam<FusionTrackLogAndTrack::MotionConfigParamsIndex::ObjectBeingMeasured>(strings),
      geometry_id
    );
  }

//   /// @brief Convert vrep strings to integer ids that can be used by FusionTrackLogAndTrack
//   /// Helps control which objects are being moved around by the optical tracker in the vrep simulation.
//   /// @see FusionTrackLogAndTrack
//   static void AddMotionConfigUsingVrepStrings(FusionTrackLogAndTrack::GeometryIDToMotionConfigParams &geometryConfigMap, const VrepStringMotionConfigParams &motionConfig)
//   {

//    auto params_vrep_ints = VrepStringToLogAndTrackMotionConfigParams(motionConfig);
//    // get the geometry id string
//    int geometry_id = std::get<FusionTrackLogAndTrack::MotionConfigParamsIndex::GeometryID>(params_vrep_ints);
//    // update the map accordingly
//     geometryConfigMap[geometry_id] = params_vrep_ints;
//   }

//   /// Turn vrep object strings into vrep integer ids and atracsys geometry ids in the format needed by FusionTrackLogAndTrack.
//   ///
//   /// Convert vrep strings to a map for the integer ids that can be used by FusionTrackLogAndTrack.
//   /// Helps control which objects are being moved around by the optical tracker in the vrep simulation.
//   /// converts the string identifiers for objects to integer handle identifiers
//   /// for use in updating the position of objects.
//   /// @see FusionTrackLogAndTrack
//   ///
//   template <typename InputIterator>
//   FusionTrackLogAndTrack::GeometryIDToMotionConfigParams
//   CreateMotionConfigParamsUsingVrepStrings(const InputIterator &configurations)
//   {
//     FusionTrackLogAndTrack::GeometryIDToMotionConfigParams geometryConfigMap;
//     for(auto &&motionConfig : configurations)
//     {
//       AddMotionConfigUsingVrepStrings(geometryConfigMap, motionConfig);
//     }
//     return geometryConfigMap;
//   }

} // namespace grl

#endif // _ATRACSYS_FUSION_TRACK_VREP_PLUGIN_HPP_
