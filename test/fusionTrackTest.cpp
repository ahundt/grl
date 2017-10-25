#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE grlTest

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

BOOST_AUTO_TEST_SUITE(FusionTrackTest)

BOOST_AUTO_TEST_CASE(initialization)
{

  std::cout << "sizeof(ftkBuffer     ): " << sizeof(ftkBuffer) << std::endl;
  std::cout << "sizeof(ftkStatus     ): " << sizeof(ftkStatus) << std::endl;
  std::cout << "sizeof(ftkVersionSize): " << sizeof(ftkVersionSize) << std::endl;
  std::cout << "sizeof(ftkImageHeader): " << sizeof(ftkImageHeader) << std::endl;
  std::cout << "sizeof(ftkRawData    ): " << sizeof(ftkRawData) << std::endl;
  std::cout << "sizeof(ftk3DPoint    ): " << sizeof(ftk3DPoint) << std::endl;
  std::cout << "sizeof(ftk3DFiducial ): " << sizeof(ftk3DFiducial) << std::endl;
  std::cout << "sizeof(ftkGeometry   ): " << sizeof(ftkGeometry) << std::endl;
  std::cout << "sizeof(ftkMarker     ): " << sizeof(ftkMarker) << std::endl;
  std::cout << "sizeof(ftkFrameQuery ): " << sizeof(ftkFrameQuery) << std::endl;

  //    grl::sensor::FusionTrack ft;

  //// uncomment to test error output
  //        try {
  //            BOOST_LOG_TRIVIAL(info) << "Starting KUKA LBR iiwa plugin connection to Kuka iiwa\n";
  //            auto kukaPluginPG = std::make_shared<grl::KukaVrepPlugin>();
  //            kukaPluginPG->construct();
  //        } catch (boost::exception& e){
  //            // log the error and print it to the screen, don't release the exception
  //            BOOST_LOG_TRIVIAL(error) <<  boost::diagnostic_information(e);
  //        }
}

BOOST_AUTO_TEST_CASE(runRepeatedly)
{
  bool debug = false;
  if (debug) std::cout << "starting fusiontrack" << std::endl;

  grl::sensor::FusionTrack::Params ftp = grl::sensor::FusionTrack::emptyDefaultParams();
  ftp.geometryFilenames = {"geometry004.ini", "geometry104.ini"};
  //ftp.geometryFilenames = {"geometry0022.ini", "geometry0055.ini"};
  grl::sensor::FusionTrack ft(ftp);
  auto serialNumbers = ft.getDeviceSerialNumbers();
  // allocating frame object
  grl::sensor::FusionTrack::Frame frame(ft.makeFrame());
  std::cout << "makeframe imageheader_member_address: " << &frame.imageHeader << " ftkQueryImageHeader address: " << frame.FrameQueryP->imageHeader << "\n";
  // get a fixed total number of updates from each device
  int num_updates = 100;
  flatbuffers::FlatBufferBuilder fbb;
  std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>> KUKAiiwaFusionTrackMessage_vector;
  flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage> oneKUKAiiwaFusionTrackMessage = 0;
  flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack> ftk_loc_LogKUKAiiwaFusionTrack = 0;
  for (int i = 0; i < num_updates; ++i)
  {
    // loop through all connected devices
    for (auto serialNumber : serialNumbers)
    {
      frame.SerialNumber = serialNumber;
      ft.receive(frame);
      if (frame.Error == FTK_OK)
      {
        if (debug) std::cout << "time_us_member: " << frame.imageHeader.timestampUS
                             << " time_us_ftkQuery: " << frame.FrameQueryP->imageHeader->timestampUS << "\n";
        if (debug) std::cout << " imageheader_member_address: " << &frame.imageHeader << " ftkQueryImageHeader address: " << frame.FrameQueryP->imageHeader << "\n";
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
      }
      oneKUKAiiwaFusionTrackMessage = grl::toFlatBuffer(fbb, ft, frame);
      KUKAiiwaFusionTrackMessage_vector.push_back(oneKUKAiiwaFusionTrackMessage);
    }
  } // End of the loop
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> states = fbb.CreateVector(KUKAiiwaFusionTrackMessage_vector);
  flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack> LogKUKAiiwaFusionTrack = grl::toFlatBuffer(fbb, states);
  // Finish a buffer with a given root object
  fbb.Finish(LogKUKAiiwaFusionTrack);
  
  // print byte data for debugging:
  //flatbuffers::SaveFile("test.flik", fbb.GetBufferPointer(), fbb.GetSize());
  std::string filename = "test.flik";
  uint8_t *buf = fbb.GetBufferPointer();
  flatbuffers::SaveFile(filename.c_str(), reinterpret_cast<const char*> (buf), fbb.GetSize(), true);
  std::cout << " fbb.GetSize(): " << fbb.GetSize() << std::endl;
}

BOOST_AUTO_TEST_SUITE_END()
