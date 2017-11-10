#ifndef GRL_FUSIONTRACK_LOG_AND_TRACK
#define GRL_FUSIONTRACK_LOG_AND_TRACK

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
#include "grl/time.hpp"

namespace grl
{


/// Atracsys FusionTrack high level api with data logging, object tracking, and frame updates.
///
/// Optical Tracker Controller multi-threaded high level class.
/// See atracsys.com for details about this device.
///
/// Keeps track of the changing position of geometries over time, plus makes it easy to start, stop, and save logs.
/// usage:
/// @code
///    auto ftLogAndTrackPG = std::make_shared<grl::FusionTrackLogAndTrack>();
///    ftLogAndTrackPG->construct();
///    while(true) ftLogAndTrackPG->start_recording();
/// @endcode
///
/// This class runs a fusiontrack driver in the background, plus makes it easy to log data and get frame updates.
/// Additionally, it provides utility functions to manage updates to the position of various objects the tracker may see,
/// by taking care of the details of which objects are unique.
///
/// For example, this class is used loads up the vrep string identifiers for the objects whose position you want to modify
/// using the AtracsysFusionTrack data.
///
/// This consists of the Object whose position you wish to modify, the object whose frame
/// it should be modified in, and a bool stating if the optical tracker's measurement should be inverted before applying the
/// position. This class will then constantly collect measurments, then set the object positions from the frames specified
/// for every object detected.
///
/// It is important to know that log files have a limit of ~512MB which is enough for several
/// minutes of recording. If the limit is hit, a log file is saved automatically with a default name
/// YYYY_MM_DD_HH_MM_SS_FusionTrack.flik in the current working directory.
/// Saving two logs in the same second with the default name means they have the same name and
/// the first log will be overwritten by the second.
///
/// @note Skips geometries found based on the ini file that aren't actively configured silently.
class FusionTrackLogAndTrack : public std::enable_shared_from_this<FusionTrackLogAndTrack>
{
public:
  /// @see MotionConfigParams
  enum MotionConfigParamsIndex
  {
    ObjectToMove,
    FrameInWhichToMoveObject,
    ObjectBeingMeasured,
    GeometryID
  };

  /// Enumerates which objects exist and how they will be moved when
  /// the tracker gets an update from a marker.
  /// @see MotionConfigParams
  typedef std::tuple<
      int, // ObjectToMove, an object identifier such as a vrep handle
      int, // FrameInWhichToMoveObject, an object identifier such as a vrep handle
      int, // ObjectBeingMeasured, an object identifier such as a vrep handle
      int  // GeometryID, geometry file marker id, like 22 or 55. Might differr from the object ids above.
      > MotionConfigParams;

  /// This map is how geometry and pose updates are stored.
  ///
  /// note: elements match up with MotionConfigParams and MotionConfigParamsIndex, except here there are 3 and the main int is the geometryID.
  /// The first int is the object that is being moved
  /// The second int is the frame the object is being moved within
  /// The third int is the object being measured by the optical tracker.
  /// The fourth is the geometry ID that will be detected, this is repeated to make the API more convenient.
  typedef std::map<int, MotionConfigParams> GeometryIDToMotionConfigParams;
  /// TODO(ahundt) line below disabled when simplifying API, once finished remove entirely.
  //   typedef std::tuple<int, int, int> VrepMotionConfigTuple;

  struct Params
  {
    /// the params for the underlying fusiontrack device
    grl::sensor::FusionTrack::Params FusionTrackParams;
    /// optical tracker base object id (frame of transform measurement)
    int OpticalTrackerBase = -1;
    /// params for the objects, frames, and transform inversion to update object positions
    std::vector<MotionConfigParams> MotionConfigParamsVector;
  };

  static const Params defaultParams()
  {
    return emptyDefaultParams();
  }

  static const Params emptyDefaultParams()
  {
    std::vector<MotionConfigParams> motionConfigParams;
    Params p;
    p.FusionTrackParams = grl::sensor::FusionTrack::emptyDefaultParams();
    p.OpticalTrackerBase = -1;
    p.MotionConfigParamsVector = motionConfigParams;
    return p;
  };

//   static const Params moveTrackerParams()
//   {
//     std::vector<MotionConfigParams> motionConfigParams;
//     motionConfigParams.push_back(std::make_tuple("Fiducial#22", "OpticalTrackerBase#0", "Fiducial#22", "22"));

//     return {grl::sensor::FusionTrack::defaultParams(),
//             "OpticalTrackerBase#0",
//             motionConfigParams};
//   }

//   static const Params moveBoneParams()
//   {
//     std::vector<MotionConfigParams> motionConfigParams;
//     motionConfigParams.push_back(std::make_tuple("OpticalTrackerBase#0", "Fiducial#55", "Fiducial#55", "55"));

//     return {grl::sensor::FusionTrack::defaultParams(),
//             "OpticalTrackerBase#0",
//             motionConfigParams};
//   }

  /// @todo allow parameters to be updated
  FusionTrackLogAndTrack(Params params = defaultParams())
      : params_(params),
        isConnectionEstablished_(false),
        m_shouldStop(false)
  {
    /// @todo figure out how to re-enable when .so isn't loaded
    // initHandles();
  }

  /// construct() function completes initialization of the plugin
  /// @todo move this into the actual constructor, but need to correctly handle or attach vrep shared libraries for unit tests.
  void construct()
  {
    m_opticalTrackerBase = params_.OpticalTrackerBase;
    for(const auto & motionParam: params_.MotionConfigParamsVector)
    {
        add_object(motionParam);
    }

    // set up the logger
    try
    {
        loggerP = spdlog::stdout_logger_mt(params_.FusionTrackParams.loggerName);
    }
    catch (spdlog::spdlog_ex ex)
    {
        loggerP = spdlog::get(params_.FusionTrackParams.loggerName);
    }

    allHandlesSet = true;
    m_driverThread.reset(new std::thread(&FusionTrackLogAndTrack::update, this));
  }

  /// Saves any log files and shuts down the driver
  /// Called by the destructor, but since this may take a while
  /// a separate funtion is provided so the process can be started in parallel.
  void destruct()
  {
    m_shouldStop = true;
    if (m_driverThread)
    {
      m_driverThread->join();
    }

    for(auto &saveThreadP : m_saveRecordingThreads)
    {
      m_driverThread->join();
    }
  }

  /// adds an object to active tracking, replacing existing objects with the same GeometryID
  void add_object(const MotionConfigParams mcp)
  {
    std::lock_guard<std::mutex> lock(m_frameAccess);
    int geometry_id =std::get<GeometryID>(mcp);
    // std::cout << "added geometry_id:" <<geometry_id;
    m_geometryIDToMotionConfigParams[geometry_id] = mcp;
  }


  /// replace all configured objects with a new params map.
  void replace_all_objects(const GeometryIDToMotionConfigParams& replacement_objects)
  {
    std::lock_guard<std::mutex> lock(m_frameAccess);
    m_geometryIDToMotionConfigParams = replacement_objects;

  }

  /// @brief clears all actively tracked objects
  /// Does not modify fusiontrack params, such as any loaded geometry ini config files.
  void clear_objects()
  {
    std::lock_guard<std::mutex> lock(m_frameAccess);
    m_geometryIDToMotionConfigParams.clear();
  }

  /// @brief Remove a geometry and the corresponding objects so they no longer receive tracking updates.
  /// Does not modify fusiontrack params, such as any loaded geometry ini config files.
  void remove_geometry(int GeometryID)
  {
    std::lock_guard<std::mutex> lock(m_frameAccess);
    m_geometryIDToMotionConfigParams.erase(GeometryID);
  }

  /// @brief Remove a geometry and the corresponding objects so they no longer receive tracking updates.
  /// Does not modify fusiontrack params, such as any loaded geometry ini config files.
  void remove_geometry(std::string GeometryID)
  {
    remove_geometry(boost::lexical_cast<int>(GeometryID));
  }

  /// Is everything ok?
  /// @return true if the optical tracker is actively running without any issues
  /// @todo consider expanding to support real error codes
  bool is_active()
  {
    return allHandlesSet && !exceptionPtr && opticalTrackerP && isConnectionEstablished_;
  }

  /// Is the optical tracker plugin currently recording log data?
  bool is_recording()
  {
    return is_active() && m_isRecording;
  }

  // Get all the latest transform state measured by the FusionTrack.
  // This is an update of the coordinate frames for the objects being tracked.
  //
  // The tuple defines the frames of a transform.
  // Entry 0 is the object being moved,
  // Entry 1 is the object defining entry 0 moves within,
  // and entry 3 is the transform between the frames.
  std::vector<std::tuple<int, int, Eigen::Affine3f>> get_poses()
  {
    std::vector<std::tuple<int, int, Eigen::Affine3f>> poses;
    // std::cout << "get_poses called" << std::endl;

    // rethrow an exception if it occured in the other thread.
    if (exceptionPtr)
    {
      /// note: this exception most likely came from the update() call initializing opticalTrackerP
      std::rethrow_exception(exceptionPtr);
    }

    // std::cout <<" connection established: " << isConnectionEstablished_ << " all handles set: " << allHandlesSet << std::endl;
    // don't try to lock or start sending the tracker data
    // until the device has established a connection
    if (!isConnectionEstablished_ || !allHandlesSet) {
        // std::cout <<"skipping" << std::endl;
      return std::vector<std::tuple<int, int, Eigen::Affine3f>>();
    }

    // std::cout << "test1" << std::endl;
    std::lock_guard<std::mutex> lock(m_frameAccess);
    // std::cout << "test2" << std::endl;

    // if any of the components haven't finished initializing, halt the program with an error
    BOOST_VERIFY(m_receivedFrame && m_nextState && opticalTrackerP);

    // std::cout << "test3" << std::endl;
    Eigen::Affine3f cameraToMarkerTransform; /// Relative distance between camera and marker?
    // std::cout << "markers: " << m_receivedFrame->Markers.size() << std::endl;
    for(const auto &marker : m_receivedFrame->Markers)
    {

    //   std::cout << "marker geometryid: " << marker.geometryId << std::endl;
      cameraToMarkerTransform = sensor::ftkMarkerToAffine3f(marker);
      auto configIterator = m_geometryIDToMotionConfigParams.find(marker.geometryId);
      if (configIterator == m_geometryIDToMotionConfigParams.end()) continue; // no configuration for this item
      auto config = configIterator->second;
    //   std::cout << "<<<<<<<<<<<<<<<< found config!" << std::endl;

      // invert the transform from the tracker to the object if needed
      if (m_opticalTrackerBase == std::get<ObjectToMove>(config) &&
          std::get<FrameInWhichToMoveObject>(config) == std::get<ObjectBeingMeasured>(config))
      {
        cameraToMarkerTransform = cameraToMarkerTransform.inverse();
      }
      else if (std::get<FrameInWhichToMoveObject>(config) != m_opticalTrackerBase)
      {
        BOOST_THROW_EXCEPTION(std::runtime_error("FusionTrackLogAndTrack: moving objects other than those being measured and the base itself are not yet supported."));
      }

      poses.push_back(std::make_tuple(std::get<ObjectToMove>(config), std::get<FrameInWhichToMoveObject>(config), cameraToMarkerTransform));
    }
    return poses;
  }

  ~FusionTrackLogAndTrack()
  {
    destruct();
  }

  /// start recording the fusiontrack frame data in memory
  /// return true on success, false on failure
  bool start_recording()
  {

    m_isRecording = true;
    return m_isRecording;
  }
  /// stop recording the fusiontrack frame data in memory
  /// return true on success, false on failure
  bool stop_recording()
  {
    m_isRecording = false;
    return !m_isRecording;
  }

  /// save the currently recorded fusiontrack frame data, this also clears the recording
  bool save_recording(std::string filename = std::string())
  {
    if(filename.empty())
    {
      /// TODO(ahundt) Saving the file twice in one second will overwrite!!!!
      filename = current_date_and_time_string() + "_FusionTrack.flik";
    }
    loggerP->info("Save Recording as: ", filename);

    /// lock mutex before accessing file
    std::lock_guard<std::mutex> lock(m_frameAccess);

    // std::move std::move is used to indicate that an object (m_logFileBufferBuilderP) may be "moved from",
    // i.e. allowing the efficient transfer of resources from m_logFileBufferBuilderP to another objec.
    // then save_fbbP = m_logFileBufferBuilderP, and m_logFileBufferBuilderP is nullptr
    // Lambda function: [capture](parameters)->return-type {body}
    // [ = ]: captures all variables used in the lambda by value
    // Lambda functions are just syntactic sugar for inline and anonymous functors.
    // https://stackoverflow.com/questions/7627098/what-is-a-lambda-expression-in-c11

    auto saveLambdaFunction = [
      save_fbbP = std::move(m_logFileBufferBuilderP),
      save_KUKAiiwaFusionTrackMessageBufferP = std::move(m_KUKAiiwaFusionTrackMessageBufferP),
      filename,
      lambdaLoggerP = loggerP
    ]() mutable
    {
      bool success = grl::FinishAndVerifyBuffer(*save_fbbP, *save_KUKAiiwaFusionTrackMessageBufferP);
      bool write_binary_stream = true;
      success = success && flatbuffers::SaveFile(filename.c_str(), reinterpret_cast<const char*>(save_fbbP->GetBufferPointer()), save_fbbP->GetSize(), write_binary_stream);
      /// TODO(ahundt) replace cout with proper spdlog and vrep banner notification
      lambdaLoggerP->info("filename: ", filename, " verifier success: ", success);
    };
    // save the recording to a file in a separate thread, memory will be freed up when file finishes saving
    std::shared_ptr<std::thread> saveLogThread(std::make_shared<std::thread>(saveLambdaFunction));
    m_saveRecordingThreads.push_back(saveLogThread);
    // flatbuffersbuilder does not yet exist
    m_logFileBufferBuilderP = std::make_shared<flatbuffers::FlatBufferBuilder>();
    m_KUKAiiwaFusionTrackMessageBufferP =
        std::make_shared<std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>>();
  }

  // clear the recording buffer from memory immediately to start fresh
  void clear_recording()
  {
    std::lock_guard<std::mutex> lock(m_frameAccess);
    m_logFileBufferBuilderP.reset();
    m_KUKAiiwaFusionTrackMessageBufferP.reset();
  }

  std::unique_ptr<grl::sensor::FusionTrack::Frame> get_frame()
  {
    std::lock_guard<std::mutex> lock(m_frameAccess);
    auto frame = std::move(opticalTrackerP->makeFramePtr());
    std::swap(frame, m_receivedFrame);

    return std::move(frame);
  }

  void setOpticalTrackerBase(int id)
  {
      m_opticalTrackerBase = id;
      params_.OpticalTrackerBase = id;
      allHandlesSet = true;
  }

private:
  /// @todo support boost::asio
  /// Reads data off of the real optical tracker device in a separate thread
  void update()
  {
    const std::size_t MegaByte = 1024*1024;
    // If we write too large a flatbuffer
    const std::size_t single_buffer_limit_bytes = 512*MegaByte;
    try
    {
    //   std::cout << "<<<<<<< geometry filenames: " << params_.FusionTrackParams.geometryFilenames << std::endl;
      // initialize all of the real device states
      std::lock_guard<std::mutex> lock(m_frameAccess);
      opticalTrackerP.reset(new grl::sensor::FusionTrack(params_.FusionTrackParams));
      // std::move is used to indicate that an object t may be "moved from",
      // i.e. allowing the efficient transfer of resources from t to another object.
      m_receivedFrame = std::move(opticalTrackerP->makeFramePtr());
      m_nextState = std::move(opticalTrackerP->makeFramePtr());
      isConnectionEstablished_ = true;
    }
    catch (...)
    {
      // transport the exception to the main thread in a safe manner
      exceptionPtr = std::current_exception();
      m_shouldStop = true;
    }

    auto serialNumbers = opticalTrackerP->getDeviceSerialNumbers();
    // std::cout << "<<<<<<<<<<<<<<<<<  serialNumbers: " << serialNumbers << std::endl;

    // run the primary update loop in a separate thread
    int counter = 0;
    bool saveFileNow = false;
    while (!m_shouldStop)
    {

     // loop through all connected devices
     for(auto serialNumber : serialNumbers)
     {
        m_nextState->SerialNumber = serialNumber;
        opticalTrackerP->receive(*m_nextState);
        {
            std::lock_guard<std::mutex> lock(m_frameAccess);
            if (m_isRecording)
            {
              // convert the buffer into a flatbuffer for recording and add it to the in memory buffer
              // @todo TODO(ahundt) if there haven't been problems, delete this todo, but if recording in the driver thread is time consuming move the code to another thread
              if (!m_logFileBufferBuilderP)
              {
                // flatbuffersbuilder does not yet exist
                m_logFileBufferBuilderP = std::make_shared<flatbuffers::FlatBufferBuilder>();
                m_KUKAiiwaFusionTrackMessageBufferP =
                    std::make_shared<std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>>();
              }
              BOOST_VERIFY(m_logFileBufferBuilderP != nullptr);
              BOOST_VERIFY(opticalTrackerP != nullptr);
              BOOST_VERIFY(m_nextState != nullptr);

              if(m_nextState->Error != FTK_WAR_NO_FRAME)
              {
                  // only collect frames actually containing new data.
                  flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage> oneKUKAiiwaFusionTrackMessage =
                      grl::toFlatBuffer(*m_logFileBufferBuilderP, *opticalTrackerP, *m_nextState);
                  m_KUKAiiwaFusionTrackMessageBufferP->push_back(oneKUKAiiwaFusionTrackMessage);
              }

              // There is a flatbuffers file size limit of 2GB, but we use a conservative 512MB
              if(m_logFileBufferBuilderP->GetSize() > single_buffer_limit_bytes)
              {
                // save the file if we are over the limit
                saveFileNow = true;
              }
          } // end recording steps

          if(m_nextState->Error == FTK_OK)
          {
            // Swaps the values.
            std::swap(m_receivedFrame, m_nextState);
            // std::cout << "thread markers: " << m_receivedFrame->Markers.size() << std::endl;
          }
        } // end m_frameAccess lock

        /// TODO(ahundt) Let the user specify the filenames, or provide a way to check the flatbuffer size and know single_buffer_limit_bytes.
        if(saveFileNow)
        {
          save_recording();
          saveFileNow = false;
        }

        // TODO(ahundt) get atracsys to fix this flaw in their library design
        // the fusionTrack library cannot handle being slammed
        // with update calls, so yield processor time with the
        // shortest possible sleep. If you call as fast as is possible
        // they will write to their .log file, causing all sorts of
        // slowdowns and writing huge files to disk very fast.
        std::this_thread::yield();
      } // end loop through serial numbers
    } // end while loop that keeps driver alive
  }

private:
  Params params_;

  int m_opticalTrackerBase = -1;
  std::unique_ptr<grl::sensor::FusionTrack> opticalTrackerP;

  bool allHandlesSet = false;
  /// simple conditional for starting actually setting positions
  /// @see update() run_one()
  std::atomic<bool> isConnectionEstablished_;

  /// mutex that protects access of the main driver thread in update() from the separate vrep plugin messages thread
  /// it also protects the recording buffer when recording is being started, stopped, or cleared
  std::mutex m_frameAccess;

  /// the current frame available to the user, always acces after locking m_frameAccess
  std::unique_ptr<grl::sensor::FusionTrack::Frame> m_receivedFrame;
  /// the next frame state to access, always acces after locking m_frameAccess
  std::unique_ptr<grl::sensor::FusionTrack::Frame> m_nextState;
  /// builds up the file log in memory as data is received
  /// @todo TODO(ahundt) once using C++14 use unique_ptr https://stackoverflow.com/questions/8640393/move-capture-in-lambda
  std::shared_ptr<flatbuffers::FlatBufferBuilder> m_logFileBufferBuilderP;
  /// this is the current log data stored in memory
  /// @todo TODO(ahundt) once using C++14 use unique_ptr https://stackoverflow.com/questions/8640393/move-capture-in-lambda
  std::shared_ptr<std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> m_KUKAiiwaFusionTrackMessageBufferP;
  /// should the driver stop collecting data from the atracsys devices
  std::atomic<bool> m_shouldStop;
  /// is data currently being recorded
  std::atomic<bool> m_isRecording;
  std::exception_ptr exceptionPtr;

  /// thread that polls the driver for new data and puts the data into the recording
  std::unique_ptr<std::thread> m_driverThread;
  /// @todo TODO(ahundt) the threads that saved files will build up forever, figure out how they can clear themselves out
  std::vector<std::shared_ptr<std::thread>> m_saveRecordingThreads;

  // fully integer identifiers that for a given object geometry id seen by the tracker tells you:
  // 1. which object's pose should move
  // 2. the object whose frame should be used to move the object
  // 3. the object being measured by the optical tracker.
  //
  // this is because sometimes you want to move the optical tracker itself.
  GeometryIDToMotionConfigParams m_geometryIDToMotionConfigParams;
  std::shared_ptr<spdlog::logger> loggerP;
};  /// End of class FusionTrackLogAndTrack

} // namespace grl

#endif // GRL_FUSIONTRACK_LOG_AND_TRACK