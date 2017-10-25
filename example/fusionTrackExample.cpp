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

int main(int argc, char **argv)
{
  bool debug = true;
  if (debug)
    std::cout << "starting fusiontrack" << std::endl;
  grl::sensor::FusionTrack::Params ftp = grl::sensor::FusionTrack::emptyDefaultParams();
  // ftp.retrieveLeftPixels = false;
  // ftp.retrieveRightPixels = false;
  // ftp.geometryFilenames = {"geometry004.ini", "geometry104.ini"};
  ftp.geometryFilenames = {"geometry0022.ini", "geometry0055.ini"};
  grl::sensor::FusionTrack ft(ftp);
  auto serialNumbers = ft.getDeviceSerialNumbers();

  if (debug)
    std::cout << "allocating frame" << std::endl;
  // allocating frame object
  grl::sensor::FusionTrack::Frame frame(ft.makeFrame());
  std::cout << "makeframe imageheader_member_address: " << &frame.imageHeader << " ftkQueryImageHeader address: " << frame.FrameQueryP->imageHeader << "\n";
  // get a fixed total number of updates from each device
  int num_updates = 3;
  if (debug)
    std::cout << "entering data receive loop" << std::endl;

  flatbuffers::FlatBufferBuilder fbb;
  std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>> KUKAiiwaFusionTrackMessage_vector;

  flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage> oneKUKAiiwaFusionTrackMessage = 0;
  flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack> ftk_loc_LogKUKAiiwaFusionTrack = 0;
  for (int i = 0; i < num_updates; ++i)
  {
    // loop through all connecteif (frame.Error == FTK_OK)d devices
    for (auto serialNumber : serialNumbers)
    {
      frame.SerialNumber = serialNumber;
      std::cout << "SerialNumber: " << frame.SerialNumber << std::endl;

      ft.receive(frame);
      if (frame.Error == FTK_OK)
      {
        if (debug)
          std::cout << "time_us_member: " << frame.imageHeader.timestampUS
                    << " time_us_ftkQuery: " << frame.FrameQueryP->imageHeader->timestampUS << "\n";
        if (debug)
          std::cout << " imageheader_member_address: " << &frame.imageHeader << " ftkQueryImageHeader address: " << frame.FrameQueryP->imageHeader << "\n";
        for (const ftkMarker &marker : frame.Markers)
        {
          Eigen::Affine3f fusionTrackToMarker = grl::sensor::ftkMarkerToAffine3f(marker);
          if (debug)
            std::cout << " matrix: " << fusionTrackToMarker.matrix();
        }
      }
      else
      {
        // handle problem
        std::cout << "Couldn't receive data from the FusionTrack device, something was wrong." << std::endl;
      }
      oneKUKAiiwaFusionTrackMessage = grl::toFlatBuffer(fbb, ft, frame);
      KUKAiiwaFusionTrackMessage_vector.push_back(oneKUKAiiwaFusionTrackMessage);
    }

  } // End of updates loop

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> states = fbb.CreateVector(KUKAiiwaFusionTrackMessage_vector);
  flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack> fbLogKUKAiiwaFusionTrack = grl::flatbuffer::CreateLogKUKAiiwaFusionTrack(fbb, states);
  /// Finish a buffer with given object
  /// Call `Finish()` to instruct the builder fbb that this frame is complete.
  fbb.Finish(oneKUKAiiwaFusionTrackMessage);
  fbb.Finish(fbLogKUKAiiwaFusionTrack);

  std::string filename = "test.flik";
  uint8_t *buf = fbb.GetBufferPointer();

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
  /// Write the data get from flatbuffer binary to json file on disc.
  std::ofstream out("LogKUKAiiwaFusionTrack.json");
  out << jsongen.c_str();
  out.close();

  flatbuffers::SaveFile(filename.c_str(), reinterpret_cast<const char *>(buf), fbb.GetSize(), true);
  std::cout << " fbb.GetSize(): " << fbb.GetSize() << std::endl;

  std::cout << "End of the program" << std::endl;
} // End of main function
