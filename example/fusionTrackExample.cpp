// system includes
#include <boost/test/unit_test.hpp>
#include <exception>
#include <iostream>
#include <vector>
#include <csignal>
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
/// The command to get the json file from flatbuffer binary file, these two files should be located in the same folder.
/// flatc -I . --json LogKUKAiiwaFusionTrack.fbs -- 2017_11_03_15_12_54_FusionTrack.flik

volatile std::sig_atomic_t gSignalStatus = 0;

// called when the user presses ctrl+c
void signal_handler(int signal)
{
  gSignalStatus = signal;
}

bool FinishAndVerifyBuffer(
  flatbuffers::FlatBufferBuilder& fbb,
  std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>& KUKAiiwaFusionTrackMessage_vector )
{

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> states = fbb.CreateVector(KUKAiiwaFusionTrackMessage_vector);
  flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack> fbLogKUKAiiwaFusionTrack = grl::flatbuffer::CreateLogKUKAiiwaFusionTrack(fbb, states);

  /////////////////////////////////////
  /// Saving BINARY version of file ///
  /////////////////////////////////////
  // Finish a buffer with given object
  // Call `Finish()` to instruct the builder fbb that this frame is complete.
  const char *file_identifier = grl::flatbuffer::LogKUKAiiwaFusionTrackIdentifier();
  // fbb.Finish(oneKUKAiiwaFusionTrackMessage, file_identifier);
  fbb.Finish(fbLogKUKAiiwaFusionTrack, file_identifier);

  auto verifier = flatbuffers::Verifier(fbb.GetBufferPointer(), fbb.GetSize());
  bool success = grl::flatbuffer::VerifyLogKUKAiiwaFusionTrackBuffer(verifier);

  return success;
}

/// @param file path without extension (extension is added automatically)
// Save data "buf" of length "len" bytes into a file
// "name" returning true if successful, false otherwise.
// If "binary_stream" is false data is written using ifstream's
// text mode, otherwise data is written with no transcoding.
bool WriteLogKUKAiiwaFusionTrackBuffer(
  flatbuffers::DetachedBuffer db,
  std::string binary_file_path,
  std::string json_file_path,
  std::string includePath = std::string(),
  bool read_binary_schema=false,
  bool write_binary_stream=true)
{
    const char *buf = reinterpret_cast<const char *> (db.data());

    flatbuffers::SaveFile(binary_file_path.c_str(), buf, db.size(), write_binary_stream);

    ///////////////////////////////////
    /// Saving JSON version of file ///
    ///////////////////////////////////
    std::string fbs_filename("LogKUKAiiwaFusionTrack.fbs");
    if(includePath.empty()) {
      includePath = grl::getpathtofbsfile(fbs_filename);
    }
    // Concatenates a path with a filename, regardless of wether the path
    // ends in a separator or not.
    // ../src/robonetracker/build/LogKUKAiiwaFusionTrack.fbs
    std::string fbs_fullpath = flatbuffers::ConCatPathFileName(includePath, fbs_filename);
    std::cout << "fbs_fullpath: " << fbs_fullpath << std::endl;


    // load FlatBuffer schema (.fbs) into a string, returning true if successful, false otherwise.
    // With debugger, the LoadFIle function can't be stepped into, since flatbuffer library is compiled in release version.
    std::string schemafile;
    bool loadfbsfile_ok = flatbuffers::LoadFile(fbs_fullpath.c_str(), read_binary_schema, &schemafile);
    // std::cout << schemafile << std::endl;
    // This part is to get the relative path of the fbs file, then load and parse it, with the parsed format,
    // write the data from flatbuffer to json file on disc.

    std::string jsongen;
    // Get the current working director
    // includePath = grl::getpathtofbsfile(fbs_filename)

    // parse fbs schema first, so we can use it to parse the data after
    flatbuffers::Parser parser;
    const char *include_directories[] = {includePath.c_str(), nullptr};
    loadfbsfile_ok = parser.Parse(schemafile.c_str(), include_directories);
    // now generate text from the flatbuffer binary
    GenerateText(parser, buf, &jsongen);
    // Write the data get from flatbuffer binary to json file on disc.
    std::ofstream out(json_file_path);
    out << jsongen.c_str();
    out.close();
}

int main(int argc, char **argv)
{

  const std::size_t single_buffer_limit_bytes = 536870912;
  // Install a signal handler to catch a signal when CONTROL+C
  std::signal(SIGINT, signal_handler);
  // std::raise(SIGINT);

  bool debug = true;
  std::cout << "starting fusiontrack, to stop logging and exit press ctrl+c" << std::endl;
  grl::sensor::FusionTrack::Params ftp = grl::sensor::FusionTrack::emptyDefaultParams();
  // ftp.retrieveLeftPixels = false;
  // ftp.retrieveRightPixels = false;
  // ftp.geometryFilenames = {"geometry004.ini", "geometry104.ini"};
  ftp.geometryFilenames = {"geometry0022.ini", "geometry0055.ini"};
  grl::sensor::FusionTrack ft(ftp);
  auto serialNumbers = ft.getDeviceSerialNumbers();

  if(debug) std::cout << "allocating frame" << std::endl;
  // allocating frame object
  grl::sensor::FusionTrack::Frame frame(ft.makeFrame());
  std::cout << "makeframe imageheader_member_address: " << &frame.imageHeader << " ftkQueryImageHeader address: " << frame.FrameQueryP->imageHeader << "\n";
  // get a fixed total number of updates from each device
  int num_updates = 2;
  if(debug) std::cout << "entering data receive loop" << std::endl;

  flatbuffers::FlatBufferBuilder fbb;
  std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>> KUKAiiwaFusionTrackMessage_vector;

  flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage> oneKUKAiiwaFusionTrackMessage = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

flatbuffers::FlatBufferBuilder test_fbb;
std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>> test_KUKAiiwaFusionTrackMessage_vector;

flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage> test_oneKUKAiiwaFusionTrackMessage = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  bool test = true;
  // for(int i = 0; i < num_updates; ++i)
  // loop forever until there is a signal.
  std::size_t previous_size = 0;
  std::size_t update_step = 0;
  // only write parameters on the first message
  bool writeParameters = true;
  // when data gets very large we write multiple buffers
  int buffer_num = 0;
  while(!gSignalStatus && test)
  {
     // loop through all connected devices
     for(auto serialNumber : serialNumbers)
     {
        frame.SerialNumber = serialNumber;
        if(debug) std::cout << "SerialNumber: " << frame.SerialNumber << std::endl;

        ft.receive(frame);
        if(frame.Error == FTK_OK)
        {
          if(debug) std::cout << "time_us_member: " << frame.imageHeader.timestampUS
                          << " time_us_ftkQuery: " << frame.FrameQueryP->imageHeader->timestampUS << "\n";
          if(debug) std::cout << " imageheader_member_address: " << &frame.imageHeader << " ftkQueryImageHeader address: " << frame.FrameQueryP->imageHeader << std::endl;
          for(const ftkMarker &marker : frame.Markers)
          {
             Eigen::Affine3f fusionTrackToMarker = grl::sensor::ftkMarkerToAffine3f(marker);
             if(debug) std::cout << " matrix: " << fusionTrackToMarker.matrix() << std::endl;
          }
        }
        else if(frame.Error != FTK_WAR_NO_FRAME)
        {
          // handle problem
          std::cout << "Couldn't receive data from the FusionTrack device, something was wrong. Look up the following error code in ftkErrors.h for details: " << frame.Error << std::endl;
        }

        if(frame.Error != FTK_WAR_NO_FRAME)
        {
          if(update_step > 0) writeParameters = false;
          // only log the parameters the first time step (writeParameters == true)
          // also log all other data *except* when there is no new frame available
          oneKUKAiiwaFusionTrackMessage = grl::toFlatBuffer(fbb, ft, frame, writeParameters);
          KUKAiiwaFusionTrackMessage_vector.push_back(oneKUKAiiwaFusionTrackMessage);

          test_oneKUKAiiwaFusionTrackMessage = grl::toFlatBuffer(test_fbb, ft, frame);
          test_KUKAiiwaFusionTrackMessage_vector.push_back(test_oneKUKAiiwaFusionTrackMessage);
        }

        std::size_t builder_size_bytes = fbb.GetSize();
        // const double byteToMiB = 1/1048576;
        std::size_t newData = builder_size_bytes - previous_size;
        std::cout << "Buffer Size (bytes): " << builder_size_bytes  << " New data size (bytes): " << newData << std::endl;
        if(builder_size_bytes > single_buffer_limit_bytes)
        {
            bool success = FinishAndVerifyBuffer(fbb, KUKAiiwaFusionTrackMessage_vector);

            // test_binary.fltk should now be named test_binary_0.fltk, test_binary_1.fltk, etc...
            std::string binary_file_prefix = "test_binary_";
            std::string binary_file_suffix = ".flik";
            std::string binary_file_path = binary_file_prefix + std::to_string(buffer_num) + binary_file_suffix;
            std::string json_file_prefix = "test_text_";
            std::string json_file_suffix = ".json";
            std::string json_file_path = json_file_prefix + std::to_string(buffer_num) + json_file_suffix;
            buffer_num++;
            std::cout << "binary_file_path: " << binary_file_path << " verifier success: " << success << std::endl;
            std::cout << " fbb.GetSize(): " << fbb.GetSize() << std::endl;
            std::string includePath = "";
            WriteLogKUKAiiwaFusionTrackBuffer(
              fbb.Release(),
              binary_file_path,
              json_file_path,
              includePath,
              false,
              true);
        }
        previous_size = builder_size_bytes;

     }
    update_step++;
  } // End of updates loop

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> test_states = grl::toFlatBuffer(test_fbb, test_KUKAiiwaFusionTrackMessage_vector);
  flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack> test_fbLogKUKAiiwaFusionTrack = grl::toFlatBuffer(test_fbb, test_states);

  test_fbb.Finish(test_fbLogKUKAiiwaFusionTrack);
  auto test_verifier = flatbuffers::Verifier(test_fbb.GetBufferPointer(), test_fbb.GetSize());
  bool test_success = test_verifier.VerifyBuffer<grl::flatbuffer::LogKUKAiiwaFusionTrack>();
  std::cout <<" verifier test_success for LogKUKAiiwaFusionTrack: " << test_success << std::endl;

  std::cout << "End of the program" << std::endl;
  return 0;
} // End of main function
