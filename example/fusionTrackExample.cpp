// system includes
#include <boost/test/unit_test.hpp>
#include <exception>
#include <iostream>
#include <vector>

//// local includes
#include "flatbuffers/util.h"
#include "grl/sensor/FusionTrack.hpp"
#include "grl/sensor/FusionTrackToEigen.hpp"
#include "grl/sensor/FusionTrackToFlatbuffer.hpp"
#include <ftkInterface.h>

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
  int num_updates = 100;
  if (debug)
    std::cout << "entering data receive loop" << std::endl;
  /******************************************************************/

  flatbuffers::FlatBufferBuilder fbb;
  std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>> KUKAiiwaFusionTrackMessage_vector;

  // shortcut for creating FusionTrackMessage with all fields set.
  // ftk_loc_ = FusionTrack_local, stored in local
  flatbuffers::Offset<grl::flatbuffer::FusionTrackMessage> ftk_loc_message = 0;
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
  //ftk_loc_LogKUKAiiwaFusionTrack = grl::toFlatBuffer(fbb, ftk_local_KUKAiiwaFusionTrackMessage_vector);
  // Call `Finish()` to instruct the builder fbb that this frame is complete.
 
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> states = fbb.CreateVector(KUKAiiwaFusionTrackMessage_vector);
  flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack> LogKUKAiiwaFusionTrack = grl::toFlatBuffer(fbb, states);
  fbb.Finish(LogKUKAiiwaFusionTrack);
  
  // print byte data for debugging:
  //flatbuffers::SaveFile("test.flik", fbb.GetBufferPointer(), fbb.GetSize());
  std::string filename = "test.flik";
  uint8_t *buf = fbb.GetBufferPointer();
  flatbuffers::SaveFile(filename.c_str(), reinterpret_cast<const char*> (buf), fbb.GetSize(), true);
  std::cout << " fbb.GetSize(): " << fbb.GetSize() << std::endl;
  /*
  for (flatbuffers::uoffset_t i = 0; i < fbb.GetSize(); i++)
  {
    std::cout << static_cast<int>(*(p + i)) << ", ";
  }
  */
  std::cout << std::endl;

  std::cout << "End of the program" << std::endl;
} // End of main function
