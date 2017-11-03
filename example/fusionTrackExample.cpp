// system includes
#include <boost/test/unit_test.hpp>
#include <exception>
#include <iostream>
#include <vector>

//// local includes
#include "grl/sensor/FusionTrack.hpp"
#include "grl/sensor/FusionTrackToEigen.hpp"
#include "grl/sensor/FusionTrackToFlatbuffer.hpp"
#include <ftkInterface.h>
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/util.h"
#include "flatbuffers/idl.h"

#include "grl/time.hpp"

#include <boost/exception/all.hpp>
#include <boost/lexical_cast.hpp>




class FusionTrackFlatBuffer {

public:

  // FusionTrackFlatBuffer(std::string filename, FusionTrack& ft) {}

   /// @see MotionConfigParams
  /// @see VrepMotionConfigTuple
  enum MotionConfigParamsIndex
  {
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

  struct Params
  {
    /// the params for the underlying fusiontrack device
    grl::sensor::FusionTrack::Params FusionTrackParams;
    /// optical tracker base (frame of transform measurement)
    std::string OpticalTrackerBase;
    /// params for the objects, frames, and transform inversion to update object positions
    std::vector<MotionConfigParams> MotionConfigParamsVector;
  };

  static const Params defaultParams()
  {
    return moveBoneParams();
  }

  static const Params emptyDefaultParams()
  {
    std::vector<MotionConfigParams> motionConfigParams;

    return {grl::sensor::FusionTrack::emptyDefaultParams(),
            "OpticalTrackerBase#0",
            motionConfigParams};
  };

  static const Params moveTrackerParams()
  {
    std::vector<MotionConfigParams> motionConfigParams;
    motionConfigParams.push_back(std::make_tuple("Fiducial#22", "OpticalTrackerBase#0", "Fiducial#22", "22"));

    return {grl::sensor::FusionTrack::defaultParams(),
            "OpticalTrackerBase#0",
            motionConfigParams};
  }

  static const Params moveBoneParams()
  {
    std::vector<MotionConfigParams> motionConfigParams;
    motionConfigParams.push_back(std::make_tuple("OpticalTrackerBase#0", "Fiducial#55", "Fiducial#55", "55"));

    return {grl::sensor::FusionTrack::defaultParams(),
            "OpticalTrackerBase#0",
            motionConfigParams};
  }

  Params params_;

      int m_opticalTrackerBase = -1;
      std::unique_ptr<grl::sensor::FusionTrack> opticalTrackerP;
      std::unique_ptr<grl::sensor::FusionTrack> ft;

      /// simple conditional for starting actually setting positions
      /// @see update() run_one()
      std::atomic<bool> isConnectionEstablished_;

      // mutex that protects access of the main driver thread in update() from the separate vrep plugin messages thread
      // it also protects the recording buffer when recording is being started, stopped, or cleared
      std::mutex m_frameAccess;

      /// the current frame available to the user, always acces after locking m_frameAccess
      std::unique_ptr<grl::sensor::FusionTrack::Frame> m_receivedFrame;
      /// the next frame state to access, always acces after locking m_frameAccess
      std::unique_ptr<grl::sensor::FusionTrack::Frame> m_nextState;
      /// builds up the file log in memory as data is received
      /// @todo TODO(ahundt) once using C++14 use unique_ptr https://stackoverflow.com/questions/8640393/move-capture-in-lambda
      // std::shared_ptr<flatbuffers::FlatBufferBuilder> fbb;
      /// this is the current log data stored in memory
      /// @todo TODO(ahundt) once using C++14 use unique_ptr https://stackoverflow.com/questions/8640393/move-capture-in-lambda
      // std::shared_ptr<std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> m_KUKAiiwaFusionTrackMessageBufferP;
      //std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>> m_KUKAiiwaFusionTrackMessageBufferP;
      // should the driver stop collecting data from the atracsys devices
      std::atomic<bool> m_shouldStop;
      // is data currently being recorded
      /// std::atomic<bool> m_isRecording;
      bool m_isRecording;
      std::exception_ptr exceptionPtr;

      // thread that polls the driver for new data and puts the data into the recording
      std::unique_ptr<std::thread> m_driverThread;
      /// @todo TODO(ahundt) the threads that saved files will build up forever, figure out how they can clear themselves out
      std::vector<std::shared_ptr<std::thread>> m_saveRecordingThreads;

      // note: elements match up with MotionConfigParams and MotionConfigParamsIndex, except here there are 3 and the main int is the geometryID.
      // The first int is the object that is being moved
      // The second int is the frame the object is being moved within
      // The third int is the object being measured by the optical tracker??.
      typedef std::tuple<int, int, int> VrepMotionConfigTuple;
      typedef std::map<int, VrepMotionConfigTuple> GeometryIDToVrepMotionConfigMap;

      GeometryIDToVrepMotionConfigMap m_geometryIDToVrepMotionConfigMap;




/// @todo allow parameters to be updated
FusionTrackFlatBuffer(Params params = defaultParams())
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
 /// The class thread represents a single thread of execution.
 /// Threads allow multiple functions to execute concurrently.
 /// Threads begin execution immediately upon construction of the associated
 /// thread object (pending any OS scheduling delays), starting at the
 /// top-level function provided as a constructor argument.
 update();
// m_driverThread.reset(new std::thread(&fusionTrackFlatbuffer::update, this));
}

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




/// private:






  /// Reads data off of the real optical tracker device in a separate thread
  void update()
  {
    try
    {
      // initialize all of the real device states
      // std::lock_guard<std::mutex> lock(m_frameAccess);
      std::cout << "Step into update()..." << std::endl;
      opticalTrackerP.reset(new grl::sensor::FusionTrack(params_.FusionTrackParams));

      if (!opticalTrackerP)
      {
        std::cout << "opticalTrackerP is NULL" << std::endl;
      }
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

    int counter = 0;
    m_isRecording = true;
    flatbuffers::FlatBufferBuilder m_logFileBufferBuilder;
    flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage> oneKUKAiiwaFusionTrackMessage = 0;
    std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>> KUKAiiwaFusionTrackMessageBuffer;


    // run the primary update loop in a separate thread
    while (!m_shouldStop && counter<2)
    {
      opticalTrackerP->receive(*m_nextState);
      /// std::lock_guard<std::mutex> lock(m_frameAccess);
      if (m_isRecording)
      {
        // std::cout << "Is it m_logFileBufferBuilderP? " << m_logFileBufferBuilderP << std::endl;
        // convert the buffer into a flatbuffer for recording and add it to the in memory buffer
        /// @todo TODO(ahundt) if there haven't been problems, delete this todo, but if recording in the driver thread is time consuming move the code to another thread
        // if (!m_KUKAiiwaFusionTrackMessageBufferP)
        // {
        //   m_KUKAiiwaFusionTrackMessageBufferP =
        //       std::make_shared<std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>>();
        // }
        BOOST_VERIFY(opticalTrackerP != nullptr);
        BOOST_VERIFY(m_nextState != nullptr);
        oneKUKAiiwaFusionTrackMessage = grl::toFlatBuffer(m_logFileBufferBuilder, *opticalTrackerP, *m_nextState);
        KUKAiiwaFusionTrackMessageBuffer.push_back(oneKUKAiiwaFusionTrackMessage);
      }
      //// Save the recording data in a file with m_nextState
      /// Save recording

      {
      std::string timestamp = current_date_and_time_string();
      std::string fileID(grl::flatbuffer::LogKUKAiiwaFusionTrackIdentifier());
			std::stringstream filename;
			filename << timestamp << "_flatbuffer_"<< counter++ <<"." << fileID;
      // save_recording(filename.str());
      // save the recording to a file in a separate thread, memory will be freed up when file finishes saving

      auto states = m_logFileBufferBuilder.CreateVector(KUKAiiwaFusionTrackMessageBuffer);
      auto fbLogKUKAiiwaFusionTrack = grl::flatbuffer::CreateLogKUKAiiwaFusionTrack(m_logFileBufferBuilder, states);
      m_logFileBufferBuilder.Finish(fbLogKUKAiiwaFusionTrack, fileID.c_str());
      flatbuffers::DetachedBuffer logDataBuf = m_logFileBufferBuilder.Release();
      // Reset all the state in this FlatBufferBuilder so it can be reused to construct another buffer
      // https://google.github.io/flatbuffers/classflatbuffers_1_1_flat_buffer_builder.html#ae94b94ba71ea0aeb2d9a98c43b713412


      auto verifier = flatbuffers::Verifier(logDataBuf.data(), logDataBuf.size());
      bool success = grl::flatbuffer::VerifyLogKUKAiiwaFusionTrackBuffer(verifier);
      std::cout << "filename: " << filename.str() << " verifier success: " << success << std::endl;
      /// Write data to file
      flatbuffers::SaveFile(filename.str().c_str(), reinterpret_cast<const char *>(logDataBuf.data()), logDataBuf.size(), true);
      ///std::shared_ptr::reset(), the object becomes empty
      }

      oneKUKAiiwaFusionTrackMessage = 0;
      KUKAiiwaFusionTrackMessageBuffer.clear();
      m_logFileBufferBuilder.Clear();
      ///m_logFileBufferBuilder.Reset();
// m_logFileBufferBuilderP.reset();
//m_KUKAiiwaFusionTrackMessageBufferP.clear();

      /// Swaps the values.
      std::swap(m_receivedFrame, m_nextState);


    }
}

void saveRecording_main ()
{
  bool debug = false;
  if(debug) std::cout << "starting fusiontrack" << std::endl;
  ft.reset(new grl::sensor::FusionTrack(params_.FusionTrackParams));
  grl::sensor::FusionTrack::Params ftp = grl::sensor::FusionTrack::emptyDefaultParams();
  // ftp.retrieveLeftPixels = false;
  // ftp.retrieveRightPixels = false;
  // ftp.geometryFilenames = {"geometry004.ini", "geometry104.ini"};
  ftp.geometryFilenames = {"geometry0022.ini", "geometry0055.ini"};
  // grl::sensor::FusionTrack ft(ftp);
  auto serialNumbers = ft->getDeviceSerialNumbers();

  if(debug) std::cout << "allocating frame" << std::endl;
  // allocating frame object
  grl::sensor::FusionTrack::Frame frame(ft->makeFrame());
  std::cout << "makeframe imageheader_member_address: " << &frame.imageHeader << " ftkQueryImageHeader address: " << frame.FrameQueryP->imageHeader << "\n";
  // get a fixed total number of updates from each device
  int num_updates = 2;
  if(debug) std::cout << "entering data receive loop" << std::endl;

  flatbuffers::FlatBufferBuilder fbb;
  flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage> oneKUKAiiwaFusionTrackMessage = 0;
  std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>> KUKAiiwaFusionTrackMessage_vector;

  for(int i = 0; i < num_updates; ++i)
  {
    // loop through all connecteif (frame.Error == FTK_OK)d devices
    for(auto serialNumber : serialNumbers)
    {
      frame.SerialNumber = serialNumber;
      if(debug) std::cout << "SerialNumber: " << frame.SerialNumber << std::endl;

      ft->receive(frame);
       /// std::cout << "frame.Error = " << frame.Error << " FTK_OK = " << FTK_OK <<std::endl;
      if(frame.Error == FTK_OK)
      {
        if(debug) std::cout << "time_us_member: " << frame.imageHeader.timestampUS
                    << " time_us_ftkQuery: " << frame.FrameQueryP->imageHeader->timestampUS << "\n";
        if(debug) std::cout << " imageheader_member_address: " << &frame.imageHeader << " ftkQueryImageHeader address: " << frame.FrameQueryP->imageHeader << "\n";
        for(const ftkMarker &marker : frame.Markers)
        {
          Eigen::Affine3f fusionTrackToMarker = grl::sensor::ftkMarkerToAffine3f(marker);
          if(debug) std::cout << " matrix: " << fusionTrackToMarker.matrix();
        }
      }
      else
      {
        // handle problem
        std::cout << "Couldn't receive data from the FusionTrack device, something was wrong." << std::endl;
      }
      oneKUKAiiwaFusionTrackMessage = grl::toFlatBuffer(fbb, *ft, frame);
      KUKAiiwaFusionTrackMessage_vector.push_back(oneKUKAiiwaFusionTrackMessage);
    }

  } // End of updates loop

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> states = fbb.CreateVector(KUKAiiwaFusionTrackMessage_vector);
  flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack> fbLogKUKAiiwaFusionTrack = grl::flatbuffer::CreateLogKUKAiiwaFusionTrack(fbb, states);

  ///////////////////////////////
  /// Saving BINARY version of file

  /// Finish a buffer with given object
  /// Call `Finish()` to instruct the builder fbb that this frame is complete.
  const char *  	file_identifier = grl::flatbuffer::LogKUKAiiwaFusionTrackIdentifier();
  // fbb.Finish(oneKUKAiiwaFusionTrackMessage, file_identifier);
  fbb.Finish(fbLogKUKAiiwaFusionTrack, grl::flatbuffer::LogKUKAiiwaFusionTrackIdentifier());

  auto verifier = flatbuffers::Verifier(fbb.GetBufferPointer(), fbb.GetSize());
  bool success = grl::flatbuffer::VerifyLogKUKAiiwaFusionTrackBuffer(verifier);

  std::string filename_prefix = "test";
  std::string filename_binary_suffix = "_binary.flik";
  std::string filename_binary = filename_prefix + filename_binary_suffix;
  std::cout <<"=====================================================" << std::endl;
  std::cout << "filename_binary: " << filename_binary << " verifier success: " << success << std::endl;
  const uint8_t *buf = fbb.GetBufferPointer();

  flatbuffers::SaveFile(filename_binary.c_str(), reinterpret_cast<const char *>(buf), fbb.GetSize(), true);

  ///////////////////////////////
  /// Saving JSON version of file

  /// This part is to get the relative path of the fbs file, then load and parse it, with the parsed format,
  /// write the data from flatbuffer to json file on disc.
  std::string schemafile;
  std::string jsongen;
  /// Get the current working directory
  std::string fbs_filename("LogKUKAiiwaFusionTrack.fbs");
  std::string fbs_path = grl::getpathtofbsfile(fbs_filename);

  /// Concatenates a path with a filename, regardless of wether the path
  /// ends in a separator or not.
  /// ../src/robonetracker/build/LogKUKAiiwaFusionTrack.fbs
  std::string fbs_path_name = flatbuffers::ConCatPathFileName(fbs_path, fbs_filename);
  std::cout << "fbs_path_name: " << fbs_path_name << std::endl;
  bool binary = false;
  /// Can't step into this function, since flatbuffer library is release version.
  bool ok = flatbuffers::LoadFile(fbs_path_name.c_str(), binary, &schemafile);
  // std::cout << schemafile << std::endl;
  /// parse fbs schema first, so we can use it to parse the data after
  flatbuffers::Parser parser;
  const char *include_directories[] = {fbs_path.c_str(), nullptr};
  ok = parser.Parse(schemafile.c_str(), include_directories);
  /// now generate text from the flatbuffer binary
  GenerateText(parser, buf, &jsongen);

  std::string filename_json_suffix = "_text.json";
  std::string filename_json = filename_prefix + filename_json_suffix;
  /// Write the data get from flatbuffer binary to json file on disc.
  std::ofstream out(filename_json);

  out << jsongen.c_str();
  out.close();

  std::cout << " fbb.GetSize(): " << fbb.GetSize() << std::endl;

  std::cout << "End of the program" << std::endl;

}

};

int main(int argc, char **argv)
{
  FusionTrackFlatBuffer ftkflatbuffer;
  /// ftkflatbuffer->construct();
  bool runMain = false;
  if(runMain) ftkflatbuffer.saveRecording_main();
  else ftkflatbuffer.update();

return 0;
} // End of main function
