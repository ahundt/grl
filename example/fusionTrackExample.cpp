// system includes
#include <boost/test/unit_test.hpp>
#include <exception>
#include <iostream>
#include <vector>
#include <csignal>
//// local includes
#include <ftkInterface.h>
#include "grl/flatbuffer/flatbuffer.hpp"
#include "grl/sensor/FusionTrack.hpp"
#include "grl/sensor/FusionTrackToEigen.hpp"
#include "grl/sensor/FusionTrackToFlatbuffer.hpp"
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/util.h"
#include "flatbuffers/idl.h"

#include "grl/time.hpp"

#include <boost/exception/all.hpp>
#include <boost/lexical_cast.hpp>
/// The command to get the json file from flatbuffer binary file, these two files should be located in the same folder.
/// flatc -I . --json LogKUKAiiwaFusionTrack.fbs -- 2017_11_03_15_12_54_FusionTrack.flik

volatile std::sig_atomic_t signalStatusG = 0;

// called when the user presses ctrl+c
void signal_handler(int signal)
{
  signalStatusG = signal;
}

int main(int argc, char **argv)
{

  const std::size_t MegaByte = 1024*1024;
  // If we write too large a flatbuffer
  const std::size_t single_buffer_limit_bytes = 512*MegaByte;
  // Install a signal handler to catch a signal when CONTROL+C
  std::signal(SIGINT, signal_handler);
  // std::raise(SIGINT);

  bool debug = false;
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

// flatbuffers::FlatBufferBuilder test_fbb;
// std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>> test_KUKAiiwaFusionTrackMessage_vector;

// flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage> test_oneKUKAiiwaFusionTrackMessage = 0;

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
  // period over which to print some additional status information
  int print_status_period = 10000;

  std::string binary_file_prefix = "test_binary_";
  std::string binary_file_suffix = ".flik";
  std::string json_file_prefix = "test_text_";
  std::string json_file_suffix = ".json";
  std::string fbs_filename("LogKUKAiiwaFusionTrack.fbs");
  // std::string includePath = grl::getpathtofbsfile(fbs_filename);
  std::string includePath = "/home/chunting/src/robonetracker/build/";


  while(!signalStatusG && test)
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

          // test_oneKUKAiiwaFusionTrackMessage = grl::toFlatBuffer(test_fbb, ft, frame);
          // test_KUKAiiwaFusionTrackMessage_vector.push_back(test_oneKUKAiiwaFusionTrackMessage);
        }

        std::size_t builder_size_bytes = fbb.GetSize();
        if(debug || update_step % print_status_period == 0)
        {
            std::size_t newData = builder_size_bytes - previous_size;
            std::cout << "Most recent frame receive status code:" << frame.Error << std::endl <<
                "Most recent single message data size (bytes): " << newData << std::endl <<
                "Current Log Buffer Size (MegaBytes): " << static_cast<double>(builder_size_bytes)/MegaByte << std::endl;

        }
        // const double byteToMiB = 1/1048576;
        if(builder_size_bytes > single_buffer_limit_bytes)
        {
            bool success = grl::FinishAndVerifyBuffer(fbb, KUKAiiwaFusionTrackMessage_vector);
            std::cout << "verifier success " << buffer_num << " : "<< success << std::endl;
            // test_binary.fltk should now be named test_binary_0.fltk, test_binary_1.fltk, etc...
            std::string binary_file_path = binary_file_prefix + std::to_string(buffer_num) + binary_file_suffix;
            std::string json_file_path = json_file_prefix + std::to_string(buffer_num) + json_file_suffix;
            std::cout << "Reached single buffer capacity limit of " << static_cast<double>(single_buffer_limit_bytes)/MegaByte <<
                "MB, writing binary log:" << binary_file_path << std::endl;

            std::string fbs_filename("LogKUKAiiwaFusionTrack.fbs");
            grl::SaveFlatBufferFile(
                fbb.GetBufferPointer(),
                fbb.GetSize(),
                binary_file_path,
                fbs_filename,
                json_file_path);
            buffer_num++;
            KUKAiiwaFusionTrackMessage_vector.clear();
            fbb.Reset();
        }
        previous_size = builder_size_bytes;

        // TODO(ahundt) get atracsys to fix this flaw in their library design
        // the fusionTrack library cannot handle being slammed
        // with update calls, so yield processor time with the
        // shortest possible sleep. If you call as fast as is possible
        // they will write to their .log file, causing all sorts of
        // slowdowns and other problems.
        std::this_thread::yield();
     }

    update_step++;
  } // End of updates loop

  // bool success = FinishAndVerifyBuffer(fbb, KUKAiiwaFusionTrackMessage_vector);
  // flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> states = fbb.CreateVector(KUKAiiwaFusionTrackMessage_vector);
  // flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack> fbLogKUKAiiwaFusionTrack = grl::flatbuffer::CreateLogKUKAiiwaFusionTrack(fbb, states);
  // const char *file_identifier = grl::flatbuffer::LogKUKAiiwaFusionTrackIdentifier();
  // // fbb.Finish(oneKUKAiiwaFusionTrackMessage, file_identifier);
  // fbb.Finish(fbLogKUKAiiwaFusionTrack, file_identifier);

  // auto verifier = flatbuffers::Verifier(fbb.GetBufferPointer(), fbb.GetSize());
  // bool success = grl::flatbuffer::VerifyLogKUKAiiwaFusionTrackBuffer(verifier);

  // std::cout << "verifier success " << buffer_num << " : "<< success << std::endl;
  // test_binary.fltk should now be named test_binary_0.fltk, test_binary_1.fltk, etc...

  bool success = grl::FinishAndVerifyBuffer(fbb, KUKAiiwaFusionTrackMessage_vector);
  std::string binary_file_path = binary_file_prefix + std::to_string(buffer_num) + binary_file_suffix;
  std::string json_file_path = json_file_prefix + std::to_string(buffer_num) + json_file_suffix;
  std::cout << " fbb.GetSize(): " << fbb.GetSize() << std::endl;
  std::cout << "FinishAndVerifyBuffer: " << success << std::endl;

  success = success && grl::SaveFlatBufferFile(
    fbb.GetBufferPointer(),
    fbb.GetSize(),
    binary_file_path,
    fbs_filename,
    json_file_path);


    std::cout << "Saved binary_file_path: " << binary_file_path << " json_file_path: " << json_file_path << " saved with success result: " << success << std::endl;

  // flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> test_states = grl::toFlatBuffer(test_fbb, test_KUKAiiwaFusionTrackMessage_vector);
  // flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack> test_fbLogKUKAiiwaFusionTrack = grl::toFlatBuffer(test_fbb, test_states);

  // test_fbb.Finish(test_fbLogKUKAiiwaFusionTrack);
  // auto test_verifier = flatbuffers::Verifier(test_fbb.GetBufferPointer(), test_fbb.GetSize());
  // bool test_success = test_verifier.VerifyBuffer<grl::flatbuffer::LogKUKAiiwaFusionTrack>();
  // std::cout <<" verifier test_success for LogKUKAiiwaFusionTrack: " << test_success << std::endl;

  std::cout << "End of the program" << std::endl;
  return success;
} // End of main function
