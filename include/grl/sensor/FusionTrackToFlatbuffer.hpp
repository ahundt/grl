#ifndef GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
#define GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER

#include "FusionTrackToEigen.hpp"
#include "FusionTrack.hpp"
#include "ftkInterface.h"
#include "grl/flatbuffer/FusionTrack_generated.h"

namespace grl
{

grl::flatbuffer::Vector3d toFlatBuffer(const ::ftk3DPoint &pt)
{
    return grl::flatbuffer::Vector3d(pt.x, pt.y, pt.z);
}

grl::flatbuffer::Vector3d toFlatBuffer(const Eigen::Vector3d &pt)
{
    return grl::flatbuffer::Vector3d(pt.x(), pt.y(), pt.z());
}

grl::flatbuffer::Quaternion toFlatBuffer(Eigen::Quaterniond q)
{
    return grl::flatbuffer::Quaternion(q.x(), q.y(), q.z(), q.w());
}

grl::flatbuffer::Pose toFlatBuffer(Eigen::Affine3d tf)
{
    Eigen::Vector3d pos = tf.translation();
    Eigen::Quaterniond eigenQuat(tf.rotation());
    // rl::flatbuffer::Quaternion
    return grl::flatbuffer::Pose(toFlatBuffer(pos), toFlatBuffer(eigenQuat));
}

grl::flatbuffer::Pose toFlatBuffer(Eigen::Affine3f tf)
{
    return toFlatBuffer(tf.cast<double>());
}

flatbuffers::Offset<grl::flatbuffer::ftkGeometry>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const ::ftkGeometry &geometry, const std::string name = "")
{
    std::array<grl::flatbuffer::Vector3d, FTK_MAX_FIDUCIALS> fiducials;
    for (int i = 0; i < geometry.pointsCount; i++) {
        fiducials[i] = toFlatBuffer(geometry.positions[i]);
    }

    return grl::flatbuffer::CreateftkGeometry(
        fbb,
        fbb.CreateString(name),
        geometry.geometryId,
        geometry.version,
        fbb.CreateVectorOfStructs<grl::flatbuffer::Vector3d>(fiducials.begin(), geometry.pointsCount));
}
/* 
*       Convert uint32 Mask to a std::vector.
*       Refer to https://stackoverflow.com/a/2686571
*/
std::vector<uint32> bitMaskToVector(uint32 x)
{
    std::vector<uint32> ret;
    while (x)
    {
        if (x & 1) {
            ret.push_back(1);
        } else {
            ret.push_back(0);
        }
        x >>= 1;
    }
    reverse(ret.begin(), ret.end());
    return ret;
}

flatbuffers::Offset<grl::flatbuffer::ftkMarker>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const ::ftkMarker &marker, const std::string name = "")
{
    std::vector<uint32_t> MaskVector =  bitMaskToVector(marker.geometryPresenceMask);
    int vectorsize = MaskVector.size();
    auto fbmask = fbb.CreateVector(&MaskVector[0], vectorsize);
    auto fbPose = toFlatBuffer(grl::sensor::ftkMarkerToAffine3f(marker));
    return grl::flatbuffer::CreateftkMarker(
        fbb,
        fbb.CreateString(name),
        marker.id,
        marker.geometryId,
        fbmask,
        &fbPose //const Pose *transform = 0;
        );
}
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkMarker>>>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const std::vector<::ftkMarker> &ftkMarkers, const std::vector<std::string> &markername)
{
    std::vector<flatbuffers::Offset<grl::flatbuffer::ftkMarker>> fbMarkers;
    int markersize = ftkMarkers.size();
    for (int i = 0; i < markersize; i++) {
        fbMarkers.push_back(toFlatBuffer(fbb, ftkMarkers[i], markername[i]));
    }
        
    /// @todo TODO(ahundt) IN PROGRESS
    auto fbmarkervector = fbb.CreateVector(&fbMarkers[0], markersize);
    return fbmarkervector;
}
/*
    Converte a global variable ::ftk3DFiducial to grl::flatbutter::ftk3DFiducial.
*/
flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const ::ftk3DFiducial &Fiducial, const std::string markername = "")
{
    // The type of Fiducial.positionMM is ftk3DPoint.
    auto positiont3Dpoint = toFlatBuffer(Fiducial.positionMM);
    return grl::flatbuffer::Createftk3DFiducial(
        fbb,
        fbb.CreateString(markername),
        Fiducial.leftIndex,
        Fiducial.rightIndex,
        &positiont3Dpoint,
        Fiducial.epipolarErrorPixels,
        Fiducial.triangulationErrorMM,
        Fiducial.probability);
}
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>>>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const std::vector<::ftk3DFiducial> &fiducials, const std::vector<std::string> &markername)
{
    std::vector<flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>> fbfiducials;
    int fiducialsize = fiducials.size();
    int marker_name_size = markername.size();

    for (int i = 0; i < fiducialsize; i++)
    {
        fbfiducials.push_back(toFlatBuffer(fbb, fiducials[i], markername[i]));
    }

    /// @todo TODO(ahundt) IN PROGRESS
    auto fbfiducialvector = fbb.CreateVector(&fbfiducials[0], fiducialsize);
    return fbfiducialvector;
}

flatbuffers::Offset<grl::flatbuffer::FusionTrackFrame>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const grl::sensor::FusionTrack::Frame &frame)
{
    /// @todo TODO(ahundt) IN PROGRESS
    /// Here we should get the markers'name
    std::vector<std::string> markerNames;
    for (auto& fiducial : frame.Fiducials)
    {
        // @todo TODO(ahundt) look up actual marker names and set with "id_geometryID" here, or "" if no marker.
        markerNames.push_back("");
    }
   
    static const double microsecToSec = 1 / 1000000;
    flatbuffers::FlatBufferBuilder &_fbb = fbb;
    double timestamp = frame.imageHeader.timestampUS * microsecToSec;
    uint64_t serialNumber = frame.SerialNumber;
    uint64_t hardwareTimestampUS = 0;
    uint64_t desynchroUS = frame.imageHeader.desynchroUS;
    uint32_t counter = frame.imageHeader.counter;
    uint32_t format = 0;
    uint32_t width = frame.imageHeader.width;
    uint32_t height = frame.imageHeader.height;
    int32_t imageStrideInBytes = frame.imageHeader.imageStrideInBytes;
    uint32_t imageHeaderVersion = 0;
    int32_t imageHeaderStatus = frame.FrameQueryP->imageHeaderStat;
    flatbuffers::Offset<flatbuffers::String> imageLeftPixels = frame.CameraImageLeftP ? fbb.CreateString(reinterpret_cast<const char *>(frame.CameraImageLeftP->begin()), sizeof(frame.CameraImageLeftP)) : 0;
    uint32_t imageLeftPixelsVersion = frame.FrameQueryP->imageLeftVersionSize.Version;
    int32_t imageLeftStatus = frame.FrameQueryP->imageLeftStat;
    flatbuffers::Offset<flatbuffers::String> imageRightPixels = frame.CameraImageRightP ? fbb.CreateString(reinterpret_cast<const char *>(frame.CameraImageRightP->begin()), sizeof(frame.CameraImageRightP)) : 0;
    uint32_t imageRightPixelsVersion = frame.FrameQueryP->imageRightVersionSize.Version;
    int32_t imageRightStatus = frame.FrameQueryP->imageRightStat;
    // It should use the flatbuffers vector instead of standard one.
    // flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<ftkRegionOfInterest>>>
    const std::vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>> *regionsOfInterestLeft = nullptr;
    uint32_t regionsOfInterestLeftVersion = 0;
    int32_t regionsOfInterestLeftStatus = 0;
    const std::vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>> *regionsOfInterestRight = nullptr;
    uint32_t regionsOfInterestRightVersion = 0;
    int32_t regionsOfInterestRightStatus = 0;
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>>> threeDFiducials = toFlatBuffer(fbb, frame.Fiducials, markerNames);
    uint32_t threeDFiducialsVersion = frame.FrameQueryP->threeDFiducialsVersionSize.Version;
    int32_t threeDFiducialsStatus = frame.FrameQueryP->threeDFiducialsStat;
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkMarker>>> markers = toFlatBuffer(fbb, frame.Markers, markerNames);
    uint32_t markersVersion = frame.FrameQueryP->markersVersionSize.Version;
    int32_t markersStatus = frame.FrameQueryP->markersStat;
    int32_t deviceType = 0;
    int64_t ftkError = frame.Error;

    return grl::flatbuffer::CreateFusionTrackFrame(
        _fbb,
        timestamp,
        serialNumber,
        hardwareTimestampUS,
        desynchroUS,
        counter,
        format,
        width,
        height,
        imageStrideInBytes,
        imageHeaderVersion,
        imageHeaderStatus,
        imageLeftPixels,
        imageLeftPixelsVersion,
        imageLeftStatus,
        imageRightPixels,
        imageRightPixelsVersion,
        imageRightStatus,
        regionsOfInterestLeft ? _fbb.CreateVector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>>(*regionsOfInterestLeft) : 0,
        regionsOfInterestLeftVersion,
        regionsOfInterestLeftStatus,
        regionsOfInterestRight ? _fbb.CreateVector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>>(*regionsOfInterestRight) : 0,
        regionsOfInterestRightVersion,
        regionsOfInterestRightStatus,
        threeDFiducials,
        threeDFiducialsVersion,
        threeDFiducialsStatus,
        markers,
        markersVersion,
        markersStatus,
        deviceType,
        ftkError);
}
}

#endif // GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
