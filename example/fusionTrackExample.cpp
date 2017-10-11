

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


int main(int argc, char** argv){

bool debug = true;
if(debug) std::cout << "starting fusiontrack" << std::endl;
grl::sensor::FusionTrack::Params ftp = grl::sensor::FusionTrack::emptyDefaultParams();
// ftp.retrieveLeftPixels = false;
// ftp.retrieveRightPixels = false;
// ftp.geometryFilenames = {"geometry004.ini", "geometry104.ini"};
ftp.geometryFilenames = {"geometry0022.ini", "geometry0055.ini"};
grl::sensor::FusionTrack ft(ftp);
auto serialNumbers = ft.getDeviceSerialNumbers();

// allocating frame object
grl::sensor::FusionTrack::Frame frame(ft.makeFrame());
std::cout << "makeframe imageheader_member_address: " << &frame.imageHeader << " ftkQueryImageHeader address: " << frame.FrameQueryP->imageHeader << "\n";
// get a fixed total number of updates from each device
int num_updates = 3;


/******************************************************************/


flatbuffers::FlatBufferBuilder fbb;
for(int i = 0; i < num_updates; ++i) {
  // loop through all connecteif (frame.Error == FTK_OK)d devices
  for(auto serialNumber : serialNumbers) {
    frame.SerialNumber = serialNumber;
    std::cout << "SerialNumber: " << frame.SerialNumber << std::endl;
    ft.receive(frame);
    if (frame.Error == FTK_OK)
    {
      if(debug) std::cout << "time_us_member: " << frame.imageHeader.timestampUS
                <<  " time_us_ftkQuery: "<< frame.FrameQueryP->imageHeader->timestampUS << "\n";
      if(debug) std::cout << " imageheader_member_address: " << &frame.imageHeader << " ftkQueryImageHeader address: " << frame.FrameQueryP->imageHeader << "\n";
      for(const ftkMarker& marker : frame.Markers){
        Eigen::Affine3f fusionTrackToMarker = grl::sensor::ftkMarkerToAffine3f(marker);
        if(debug) std::cout << " matrix: " << fusionTrackToMarker.matrix();
      }
      
      // shortcut for creating frame with all fields set.
      // ftk_loc = FusionTrack_local, stored in local
      auto ftk_loc = grl::toFlatBuffer(fbb, frame);
      // Call `Finish()` to instruct the builder fbb that this frame is complete.
      fbb.Finish(ftk_loc);
      // print byte data for debugging:
      auto p = fbb.GetBufferPointer();
      std::cout << " fbb.GetSize(): " << fbb.GetSize() << std::endl;
      for (flatbuffers::uoffset_t i = 0; i < fbb.GetSize(); i++) {
        std::cout << p[i];
      }
      std::cout << std::endl;
    } else {
      // handle problem
      std::cout << "There is something wrong" << std::endl;
    }
  }
}


std::cout << "End of the program" << std::endl;

}