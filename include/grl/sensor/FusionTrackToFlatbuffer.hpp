#ifndef GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
#define GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER

#include "FusionTrackToEigen.hpp"
#include "FusionTrack.hpp"
#include "ftkInterface.h"
#include "grl/flatbuffer/FusionTrack_generated.h"
#include "grl/flatbuffer/Time_generated.h"
#include "grl/flatbuffer/LogKUKAiiwaFusionTrack_generated.h"
#include "flatbuffers/util.h"
#include "flatbuffers/idl.h"
#include <typeinfo>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <cassert>

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
    return grl::flatbuffer::Pose(toFlatBuffer(pos), toFlatBuffer(eigenQuat));
}

grl::flatbuffer::Pose toFlatBuffer(Eigen::Affine3f tf)
{
    return toFlatBuffer(tf.cast<double>());
}

flatbuffers::Offset<grl::flatbuffer::ftkGeometry>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const ::ftkGeometry &geometry,
             const std::string name = "")
{
    std::array<grl::flatbuffer::Vector3d, FTK_MAX_FIDUCIALS> fiducials;
    for(int i = 0; i < geometry.pointsCount; i++)
    {
        fiducials[i] = toFlatBuffer(geometry.positions[i]);
    }

    ////////////////////////////////////////////////////////////////////////////
    /*** This block is to verify the specific flatbuffer object ***/
    flatbuffers::FlatBufferBuilder _fbb;

    flatbuffers::Offset<grl::flatbuffer::ftkGeometry> _ftkGeometry =
    grl::flatbuffer::CreateftkGeometry(
        _fbb,
        _fbb.CreateString(name),
        geometry.geometryId,
        geometry.version,
        _fbb.CreateVectorOfStructs<grl::flatbuffer::Vector3d>(fiducials.begin(), geometry.pointsCount));
    _fbb.Finish(_ftkGeometry);
    auto verifier = flatbuffers::Verifier(_fbb.GetBufferPointer(), _fbb.GetSize());
    bool success = verifier.VerifyBuffer<grl::flatbuffer::ftkGeometry>();
    std::cout <<" verifier success for ftkGeometry: " << success << std::endl;
    ///////////////////////////////////////////////////////////////////////////

    return grl::flatbuffer::CreateftkGeometry(
        fbb,
        fbb.CreateString(name),
        geometry.geometryId,
        geometry.version,
        fbb.CreateVectorOfStructs<grl::flatbuffer::Vector3d>(fiducials.begin(), geometry.pointsCount));
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkGeometry>>>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const grl::sensor::FusionTrack::Params &params)
{
    flatbuffers::Offset<grl::flatbuffer::ftkGeometry> fbGeometry;
    std::vector<flatbuffers::Offset<grl::flatbuffer::ftkGeometry>> geometryvector;
    int geometry_size = params.markerIDs.size();
    std::vector<grl::flatbuffer::Vector3d> fiducials;
    std::vector<ftk3DPoint> MarkerModelGeometry;

    for(int i = 0; i < geometry_size; i++)
    {
        MarkerModelGeometry = params.markerModelGeometries[i];
        for(int j = 0; j < MarkerModelGeometry.size(); j++)
        {
            fiducials.push_back(toFlatBuffer(MarkerModelGeometry[j]));
        }
        fbGeometry = grl::flatbuffer::CreateftkGeometry(
            fbb,
            fbb.CreateString(params.markerNames[i]),
            params.markerIDs[i],
            0, ///version,
            fbb.CreateVectorOfStructs<grl::flatbuffer::Vector3d>(fiducials));
        geometryvector.push_back(fbGeometry);
    }
    auto fbGeometryvector = fbb.CreateVector(&geometryvector[0], geometryvector.size());
    return fbGeometryvector;
}

/// Convert uint32 Mask to a std::vector.
/// Refer to https://stackoverflow.com/a/2686571
std::vector<uint32> bitMaskToVector(uint32 x)
{
    std::vector<uint32> ret;
    while (x)
    {
        if(x & 1)
        {
            ret.push_back(1);
        }
        else
        {
            ret.push_back(0);
        }
        x >>= 1;
    }
    reverse(ret.begin(), ret.end());
    return ret;
}

flatbuffers::Offset<grl::flatbuffer::ftkMarker>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const ::ftkMarker &marker,
             const std::string name = "")
{
    std::vector<uint32_t> MaskVector = bitMaskToVector(marker.geometryPresenceMask);
    int vectorsize = MaskVector.size();
    auto fbmask = fbb.CreateVector(&MaskVector[0], vectorsize);
    auto fbPose = toFlatBuffer(grl::sensor::ftkMarkerToAffine3f(marker));

    ////////////////////////////////////////////////////////////////////////////
    /*** This block is to verify the specific flatbuffer object ***/
    flatbuffers::FlatBufferBuilder _fbb;
    auto _fbmask = _fbb.CreateVector(&MaskVector[0], vectorsize);
    auto _fbPose = toFlatBuffer(grl::sensor::ftkMarkerToAffine3f(marker));
    flatbuffers::Offset<grl::flatbuffer::ftkMarker> _ftkMarker =
    grl::flatbuffer::CreateftkMarker(
        _fbb,
        _fbb.CreateString(name),
        marker.id,
        marker.geometryId,
        _fbmask,
        &_fbPose //const Pose *transform = 0;
    );
    _fbb.Finish(_ftkMarker);
    auto verifier = flatbuffers::Verifier(_fbb.GetBufferPointer(), _fbb.GetSize());
    bool success = verifier.VerifyBuffer<grl::flatbuffer::ftkMarker>();
    if(!success) {
        std::cout <<" verifier success for ftkMarker: " << success << std::endl;
    }
    ///////////////////////////////////////////////////////////////////////////

    return grl::flatbuffer::CreateftkMarker(
        fbb,
        fbb.CreateString(name),
        marker.id,
        marker.geometryId,
        fbmask,
        &fbPose //const Pose *transform = 0;
        );
}

/// Convert ftk marker objects along with the name to flatbuffers
/// @see FusionTrack.hpp for details
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkMarker>>>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const std::vector<::ftkMarker> &ftkMarkers,
             const std::vector<std::string> &markername)
{
    std::vector<flatbuffers::Offset<grl::flatbuffer::ftkMarker>> fbMarkers;
    int markersize = ftkMarkers.size();
    for(int i = 0; i < markersize; i++)
    {
        fbMarkers.push_back(toFlatBuffer(fbb, ftkMarkers[i], markername[i]));
    }
    auto fbMarkerVector = fbb.CreateVector(&fbMarkers[0], markersize);
    return fbMarkerVector;
}

/// Convert a global variable ::ftk3DFiducial to grl::flatbutter::ftk3DFiducial.
/// @see FusionTrack.hpp for details
flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const ::ftk3DFiducial &Fiducial,
             const uint64_t markerID = 0)
{
    /// The type of Fiducial.positionMM is ftk3DPoint.
    auto positiont3Dpoint = toFlatBuffer(Fiducial.positionMM);
    ////////////////////////////////////////////////////////////////////////////
    /*** This block is to verify the specific flatbuffer object ***/
    flatbuffers::FlatBufferBuilder _fbb;
    auto _positiont3Dpoint = toFlatBuffer(Fiducial.positionMM);
    flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial> _ftk3DFiducial = grl::flatbuffer::Createftk3DFiducial(
        _fbb,
        markerID,
        Fiducial.leftIndex,
        Fiducial.rightIndex,
        &_positiont3Dpoint,
        Fiducial.epipolarErrorPixels,
        Fiducial.triangulationErrorMM,
        Fiducial.probability);
    _fbb.Finish(_ftk3DFiducial);
    auto verifier = flatbuffers::Verifier(_fbb.GetBufferPointer(), _fbb.GetSize());
    bool success = verifier.VerifyBuffer<grl::flatbuffer::ftk3DFiducial>();
    if(!success) {
        std::cout <<" verifier success for ftk3DFiducial: " << success << std::endl;
    }
    ///////////////////////////////////////////////////////////////////////////

    return grl::flatbuffer::Createftk3DFiducial(
        fbb,
        markerID,
        Fiducial.leftIndex,
        Fiducial.rightIndex,
        &positiont3Dpoint,
        Fiducial.epipolarErrorPixels,
        Fiducial.triangulationErrorMM,
        Fiducial.probability);
}


/// Convert ftk3DFiducial representation of an optical traker's reflective ball or ir LED to a flatbuffer
/// @see FusionTrack.hpp for details
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>>>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const std::vector<::ftk3DFiducial> &fiducials,
             const std::vector<uint64_t> &markerID)
{
    std::vector<flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>> fbfiducials;
    int fiducialsize = fiducials.size();
    int marker_name_size = markerID.size();

    for(int i = 0; i < fiducialsize; i++)
    {
        fbfiducials.push_back(toFlatBuffer(fbb, fiducials[i], markerID[i]));
    }
    auto fbfiducialvector = fbb.CreateVector(&fbfiducials[0], fiducialsize);
    return fbfiducialvector;
}


/// Convert ftkRegionOfInterest, aka ftkRawData to flatbuffer formatted version
flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const ::ftkRawData &ftkRegionOfInterest)
{
    double _centerXPixels = ftkRegionOfInterest.centerXPixels;
    double _centerYPixels = ftkRegionOfInterest.centerYPixels;
    uint32_t _RightEdge = ftkRegionOfInterest.status.RightEdge;
    uint32_t _BottomEdge = ftkRegionOfInterest.status.BottomEdge;
    uint32_t _LeftEdge = ftkRegionOfInterest.status.LeftEdge;
    uint32_t _TopEdge = ftkRegionOfInterest.status.TopEdge;
    uint32_t _pixelsCount = ftkRegionOfInterest.pixelsCount;
    double _probability = ftkRegionOfInterest.probability;
    ////////////////////////////////////////////////////////////////////////////
    /*** This block is to verify the specific flatbuffer object ***/
    flatbuffers::FlatBufferBuilder _fbb;
    flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest> _ftkRegionOfInterest =
    grl::flatbuffer::CreateftkRegionOfInterest(
        _fbb,
        _centerXPixels,
        _centerYPixels,
        _RightEdge,
        _BottomEdge,
        _LeftEdge,
        _TopEdge,
        _pixelsCount,
        _probability);
    _fbb.Finish(_ftkRegionOfInterest);
    auto verifier = flatbuffers::Verifier(_fbb.GetBufferPointer(), _fbb.GetSize());
    bool success = verifier.VerifyBuffer<grl::flatbuffer::ftkRegionOfInterest>();
    if(!success) {
        std::cout <<" verifier success for ftkRegionOfInterest: " << success << std::endl;
    }
    ///////////////////////////////////////////////////////////////////////////
    return grl::flatbuffer::CreateftkRegionOfInterest(
        fbb,
        _centerXPixels,
        _centerYPixels,
        _RightEdge,
        _BottomEdge,
        _LeftEdge,
        _TopEdge,
        _pixelsCount,
        _probability);
}


/// Convert Atracsys FusionTrack optical tracker library's ftkRawData to flatbuffer
/// @see FusionTrack.hpp for details
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>>>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const std::vector<::ftkRawData> &ftkRawData)
{
    std::vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>> ftkRegionOfInterests;
    int ftkRegionOfInterest_size = ftkRawData.size();

    for(int i = 0; i < ftkRegionOfInterest_size; i++)
    {
        ftkRegionOfInterests.push_back(toFlatBuffer(fbb, ftkRawData[i]));
    }
    return fbb.CreateVector(ftkRegionOfInterests);
}

/// grl::sensor::FusionTrack::Frame to flatbuffer
/// @see FusionTrack.hpp for details
flatbuffers::Offset<grl::flatbuffer::FusionTrackFrame>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const grl::sensor::FusionTrack::Frame &frame,
             bool writeRegionOfInterest = true,
             bool writeFiducials = true,
             bool writeMarkers = true)
{
    /// @todo TODO(ahundt) IN PROGRESS Here we should get the markers'name
    std::vector<std::string> fiducialIndexToMarkerNames;
    std::vector<uint64_t>    fiducialIndexToMarkerIDs;
    for(auto &fiducial : frame.Fiducials)
    {
        /// @todo TODO(ahundt) look up actual marker names and set with "id_geometryID" here, or "" if no marker.
        fiducialIndexToMarkerNames.push_back("");
        fiducialIndexToMarkerIDs.push_back(0);
    }

    std::vector<std::string> markerIndexToMarkerNames;
    std::vector<uint64_t>    markerIndexToMarkerIDs;
    for(auto & marker : frame.Markers)
    {
        /// @todo TODO(ahundt) look up actual marker names and set with "id_geometryID" here, or "" if no marker.
        markerIndexToMarkerNames.push_back("");
        markerIndexToMarkerIDs.push_back(0);

    }

    static const double microsecToSec = 1 / 1000000;
    double timestamp = frame.imageHeader.timestampUS * microsecToSec;
    uint64_t serialNumber = frame.SerialNumber;
    uint64_t hardwareTimestampUS = frame.imageHeader.timestampUS;
    uint64_t desynchroUS = frame.imageHeader.desynchroUS;
    uint32_t counter = frame.imageHeader.counter;
    uint32_t format = frame.imageHeader.format;
    uint32_t width = frame.imageHeader.width;
    uint32_t height = frame.imageHeader.height;
    int32_t imageStrideInBytes = frame.imageHeader.imageStrideInBytes;
    uint32_t imageHeaderVersion = frame.FrameQueryP->imageHeaderVersionSize.Version;
    int32_t imageHeaderStatus = frame.FrameQueryP->imageHeaderStat;
    flatbuffers::Offset<flatbuffers::String> imageLeftPixels = frame.CameraImageLeftP ? fbb.CreateString(reinterpret_cast<const char *>(frame.CameraImageLeftP->begin()), sizeof(frame.CameraImageLeftP)) : 0;
    uint32_t imageLeftPixelsVersion = frame.FrameQueryP->imageLeftVersionSize.Version;
    int32_t imageLeftStatus = frame.FrameQueryP->imageLeftStat;
    flatbuffers::Offset<flatbuffers::String> imageRightPixels = frame.CameraImageRightP ? fbb.CreateString(reinterpret_cast<const char *>(frame.CameraImageRightP->begin()), sizeof(frame.CameraImageRightP)) : 0;
    uint32_t imageRightPixelsVersion = frame.FrameQueryP->imageRightVersionSize.Version;
    int32_t imageRightStatus = frame.FrameQueryP->imageRightStat;

    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>>> regionsOfInterestLeft =
        (writeRegionOfInterest && frame.ImageRegionOfInterestBoxesLeft.size()) ? toFlatBuffer(fbb, frame.ImageRegionOfInterestBoxesLeft): 0;
    uint32_t regionsOfInterestLeftVersion = frame.FrameQueryP->rawDataLeftVersionSize.Version;
    int32_t regionsOfInterestLeftStatus = frame.FrameQueryP->rawDataLeftStat;
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>>> regionsOfInterestRight =
        (writeRegionOfInterest && frame.ImageRegionOfInterestBoxesRight.size()) ? toFlatBuffer(fbb, frame.ImageRegionOfInterestBoxesRight): 0;
    uint32_t regionsOfInterestRightVersion = frame.FrameQueryP->rawDataRightVersionSize.Version;
    int32_t regionsOfInterestRightStatus = frame.FrameQueryP->rawDataRightStat;
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>>> threeDFiducials =
        (writeFiducials && frame.Fiducials.size()) ? toFlatBuffer(fbb, frame.Fiducials, fiducialIndexToMarkerIDs) : 0;
    uint32_t threeDFiducialsVersion = frame.FrameQueryP->threeDFiducialsVersionSize.Version;
    int32_t threeDFiducialsStatus = frame.FrameQueryP->threeDFiducialsStat;
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkMarker>>> markers =
        (writeMarkers && frame.Markers.size()) ? toFlatBuffer(fbb, frame.Markers, markerIndexToMarkerNames) : 0;
    uint32_t markersVersion = frame.FrameQueryP->markersVersionSize.Version;
    int32_t markersStatus = frame.FrameQueryP->markersStat;
    int32_t deviceType = frame.DeviceType;
    int64_t ftkError = frame.Error;


    ////////////////////////////////////////////////////////////////////////////////////////////////
    /*** This block is to verify the specific flatbuffer object ***/
    flatbuffers::FlatBufferBuilder _fbb;
    flatbuffers::Offset<flatbuffers::String> _imageLeftPixels = frame.CameraImageLeftP ? _fbb.CreateString(reinterpret_cast<const char *>(frame.CameraImageLeftP->begin()), sizeof(frame.CameraImageLeftP)) : 0;
    flatbuffers::Offset<flatbuffers::String> _imageRightPixels = frame.CameraImageRightP ? _fbb.CreateString(reinterpret_cast<const char *>(frame.CameraImageRightP->begin()), sizeof(frame.CameraImageRightP)) : 0;
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>>> _regionsOfInterestLeft = toFlatBuffer(_fbb, frame.ImageRegionOfInterestBoxesLeft);
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>>> _regionsOfInterestRight = toFlatBuffer(_fbb, frame.ImageRegionOfInterestBoxesRight);
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>>> _threeDFiducials = toFlatBuffer(_fbb, frame.Fiducials, fiducialIndexToMarkerIDs);
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkMarker>>> _markers = toFlatBuffer(_fbb, frame.Markers, markerIndexToMarkerNames);

    flatbuffers::Offset<grl::flatbuffer::FusionTrackFrame> _fusionTrackFrame =
    grl::flatbuffer::CreateFusionTrackFrame(
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
        _imageLeftPixels,
        imageLeftPixelsVersion,
        imageLeftStatus,
        _imageRightPixels,
        imageRightPixelsVersion,
        imageRightStatus,
        _regionsOfInterestLeft,
        regionsOfInterestLeftVersion,
        regionsOfInterestLeftStatus,
        _regionsOfInterestRight,
        regionsOfInterestRightVersion,
        regionsOfInterestRightStatus,
        _threeDFiducials,
        threeDFiducialsVersion,
        threeDFiducialsStatus,
        _markers,
        markersVersion,
        markersStatus,
        deviceType,
        ftkError);
    _fbb.Finish(_fusionTrackFrame);
    auto verifier = flatbuffers::Verifier(_fbb.GetBufferPointer(), _fbb.GetSize());
    bool success = verifier.VerifyBuffer<grl::flatbuffer::FusionTrackFrame>();
    if(!success) {
        std::cout <<" verifier success for FusionTrackFrame: " << success << std::endl;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////

    return grl::flatbuffer::CreateFusionTrackFrame(
        fbb,
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
        regionsOfInterestLeft,
        regionsOfInterestLeftVersion,
        regionsOfInterestLeftStatus,
        regionsOfInterestRight,
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

flatbuffers::Offset<grl::flatbuffer::FusionTrackParameters>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const grl::sensor::FusionTrack &fusiontrack)
{
    grl::sensor::FusionTrack::Params _params = fusiontrack.getParams();

    // device serial numbers
    std::vector<uint64_t> m_deviceSerialNumbers = fusiontrack.getDeviceSerialNumbers();
    std::vector<uint8_t> m_device_types = fusiontrack.getDeviceTypes();
    flatbuffers::Offset<flatbuffers::String> name = fbb.CreateString(_params.name);
    flatbuffers::Offset<flatbuffers::String> deviceClockID = fbb.CreateString(_params.deviceClockID);
    flatbuffers::Offset<flatbuffers::String> localClockID = fbb.CreateString(_params.localClockID);
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkGeometry>>> geometries = toFlatBuffer(fbb, _params);
    auto geometryFilenames = fbb.CreateVectorOfStrings(_params.geometryFilenames);
    flatbuffers::Offset<flatbuffers::String> geometryDir = fbb.CreateString(_params.geometryDir);
    flatbuffers::Offset<flatbuffers::Vector<uint64_t>> TrackerDeviceIDs = fbb.CreateVector(&(_params.TrackerDeviceIDs[0]), _params.TrackerDeviceIDs.size());
    flatbuffers::Offset<flatbuffers::Vector<uint64_t>> markerIDs = fbb.CreateVector(&_params.markerIDs[0], _params.markerIDs.size());
    auto markerNames = fbb.CreateVectorOfStrings(_params.markerNames);
    flatbuffers::Offset<flatbuffers::Vector<uint64_t>> fb_m_deviceSerialNumbers = fbb.CreateVector(&m_deviceSerialNumbers[0], m_deviceSerialNumbers.size());
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> fb_m_device_types = fbb.CreateVector(&m_device_types[0], m_device_types.size());

    ////////////////////////////////////////////////////////////////////////////
    /*** This block is to verify the specific flatbuffer object ***/
    flatbuffers::FlatBufferBuilder _fbb;

    flatbuffers::Offset<flatbuffers::String> _name = _fbb.CreateString(_params.name);
    flatbuffers::Offset<flatbuffers::String> _deviceClockID = _fbb.CreateString(_params.deviceClockID);
    flatbuffers::Offset<flatbuffers::String> _localClockID = _fbb.CreateString(_params.localClockID);
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkGeometry>>> _geometries = toFlatBuffer(_fbb, _params);
    auto _geometryFilenames = _fbb.CreateVectorOfStrings(_params.geometryFilenames);
    flatbuffers::Offset<flatbuffers::String> _geometryDir = _fbb.CreateString(_params.geometryDir);
    flatbuffers::Offset<flatbuffers::Vector<uint64_t>> _TrackerDeviceIDs = _fbb.CreateVector(&(_params.TrackerDeviceIDs[0]), _params.TrackerDeviceIDs.size());
    flatbuffers::Offset<flatbuffers::Vector<uint64_t>> _markerIDs = _fbb.CreateVector(&_params.markerIDs[0], _params.markerIDs.size());
    auto _markerNames = _fbb.CreateVectorOfStrings(_params.markerNames);
    flatbuffers::Offset<flatbuffers::Vector<uint64_t>> _m_deviceSerialNumbers = _fbb.CreateVector(&m_deviceSerialNumbers[0], m_deviceSerialNumbers.size());
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> _m_device_types = _fbb.CreateVector(&m_device_types[0], m_device_types.size());
    flatbuffers::Offset<grl::flatbuffer::FusionTrackParameters> _FusionTrackParameters=
    grl::flatbuffer::CreateFusionTrackParameters(
        _fbb,
        _name,
        _deviceClockID,
        _localClockID,
        _geometries,
        _geometryFilenames,
        _geometryDir,
        _TrackerDeviceIDs,
        _markerIDs,
        _markerNames,
        _m_deviceSerialNumbers,
        _m_device_types);
     _fbb.Finish(_FusionTrackParameters);
     auto verifier = flatbuffers::Verifier(_fbb.GetBufferPointer(), _fbb.GetSize());
     bool success = verifier.VerifyBuffer<grl::flatbuffer::FusionTrackParameters>();
     if(!success){
        std::cout <<" verifier success for FusionTrackParameters: " << success << std::endl;
     }

     ///////////////////////////////////////////////////////////////////////////

    return grl::flatbuffer::CreateFusionTrackParameters(fbb,
                                                        name,
                                                        deviceClockID,
                                                        localClockID,
                                                        geometries,
                                                        geometryFilenames,
                                                        geometryDir,
                                                        TrackerDeviceIDs,
                                                        markerIDs,
                                                        markerNames,
                                                        fb_m_deviceSerialNumbers,
                                                        fb_m_device_types);
}

template <typename T>
typename T::value_type stringLength(const T &array)
{

    auto iter = std::find(array.begin(), array.end(), '\0');
    auto len = std::distance(array.begin(), iter);
    return len;
}

flatbuffers::Offset<grl::flatbuffer::TimeEvent>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const grl::TimeEvent &timeStamp)
{
    flatbuffers::Offset<flatbuffers::String> event_name = fbb.CreateString(const_cast<const char *>(timeStamp.event_name.begin()), stringLength(timeStamp.event_name));
    /// https://github.com/googlecartographer/cartographer/blob/master/cartographer/common/time.cc
    /// convert time to int64
    int64_t local_request_time = cartographer::common::ToUniversal(timeStamp.local_request_time);
    flatbuffers::Offset<flatbuffers::String> device_clock_id = fbb.CreateString(const_cast<const char *>(timeStamp.device_clock_id.begin()), stringLength(timeStamp.device_clock_id));
    int64_t device_time = cartographer::common::ToUniversal(timeStamp.device_time);
    flatbuffers::Offset<flatbuffers::String> local_clock_id = fbb.CreateString(const_cast<const char *>(timeStamp.local_clock_id.begin()), stringLength(timeStamp.local_clock_id));
    int64_t local_receive_time = cartographer::common::ToUniversal(timeStamp.local_receive_time);
    int64_t corrected_local_time = cartographer::common::ToUniversal(timeStamp.corrected_local_time);
    int64_t clock_skew = cartographer::common::ToSeconds(timeStamp.clock_skew);
    int64_t min_transport_delay = cartographer::common::ToSeconds(timeStamp.min_transport_delay);
    return grl::flatbuffer::CreateTimeEvent(
        fbb,
        event_name,
        local_request_time,
        device_clock_id,
        device_time,
        local_clock_id,
        local_receive_time,
        corrected_local_time,
        clock_skew,
        min_transport_delay);
}

flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const grl::sensor::FusionTrack &fusiontrack,
             const grl::sensor::FusionTrack::Frame &frame,
             bool writeParameters = true,
             bool writeRegionOfInterest = true,
             bool writeFiducials = true,
             bool writeMarkers = true)
{
    static const double microsecToSec = 1 / 1000000;
    double timestamp = frame.imageHeader.timestampUS * microsecToSec;
    // only write parameters if writeParameters is true
    flatbuffers::Offset<grl::flatbuffer::FusionTrackParameters> parameters = writeParameters ? toFlatBuffer(fbb, fusiontrack) : 0;
    flatbuffers::Offset<grl::flatbuffer::TimeEvent> timeEvent = toFlatBuffer(fbb, frame.TimeStamp);
    flatbuffers::Offset<grl::flatbuffer::FusionTrackFrame> fbframe = toFlatBuffer(fbb, frame);
    flatbuffers::Offset<grl::flatbuffer::FusionTrackMessage> message = grl::flatbuffer::CreateFusionTrackMessage(
        fbb,
        timestamp,
        parameters,
        timeEvent,
        fbframe);

    grl::flatbuffer::DeviceState deviceState_type = grl::flatbuffer::DeviceState::FusionTrackMessage;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /*** This block is to verify the specific flatbuffer object ***/
    flatbuffers::FlatBufferBuilder _fbb;
    flatbuffers::Offset<grl::flatbuffer::FusionTrackParameters> _parameters = toFlatBuffer(_fbb, fusiontrack);
    flatbuffers::Offset<grl::flatbuffer::TimeEvent> _timeEvent = toFlatBuffer(_fbb, frame.TimeStamp);
    flatbuffers::Offset<grl::flatbuffer::FusionTrackFrame> _fbframe = toFlatBuffer(_fbb, frame);
    flatbuffers::Offset<grl::flatbuffer::FusionTrackMessage> _message = grl::flatbuffer::CreateFusionTrackMessage(
        _fbb,
        timestamp,
        _parameters,
        _timeEvent,
        _fbframe);

    flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage> _oneKUKAiiwaFusionTrackMessage =
    grl::flatbuffer::CreateKUKAiiwaFusionTrackMessage(
            _fbb,
            timestamp,
            _timeEvent,
            deviceState_type,
            _message.Union());
    const char *file_identifier = grl::flatbuffer::LogKUKAiiwaFusionTrackIdentifier();
    // fbb.Finish(oneKUKAiiwaFusionTrackMessage, file_identifier);
    _fbb.Finish(_oneKUKAiiwaFusionTrackMessage);
    auto verifier = flatbuffers::Verifier(_fbb.GetBufferPointer(), _fbb.GetSize());
    bool success = verifier.VerifyBuffer<grl::flatbuffer::KUKAiiwaFusionTrackMessage>();
    ///assert(success==true && "KUKAiiwaFusionTrackMessage goes wrong/n");

    std::cout <<" verifier success for KUKAiiwaFusionTrackMessage: " << success << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    return grl::flatbuffer::CreateKUKAiiwaFusionTrackMessage(
        fbb,
        timestamp,
        timeEvent,
        deviceState_type,
        message.Union());
}


flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>> &KUKAiiwaFusionTrackMessage_vector) {
    // flatbuffers::FlatBufferBuilder _fbb;
    // flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> _KUKAiiwaFusionTrackMessage_vector = _fbb.CreateVector(KUKAiiwaFusionTrackMessage_vector);
    // _fbb.Finish(_KUKAiiwaFusionTrackMessage_vector);
    // auto verifier = flatbuffers::Verifier(_fbb.GetBufferPointer(), _fbb.GetSize());
    // bool success = verifier.VerifyBuffer<grl::flatbuffer::KUKAiiwaFusionTrackMessage>();
    // std::cout <<" ------------------verifier success for KUKAiiwaFusionTrackMessage: " << success << std::endl;

    return fbb.CreateVector(KUKAiiwaFusionTrackMessage_vector);
    }



flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> &states)
{
    // flatbuffers::FlatBufferBuilder _fbb;
    // flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack> _fbLogKUKAiiwaFusionTrack = grl::flatbuffer::CreateLogKUKAiiwaFusionTrack(_fbb, states);

    // // fbb.Finish(oneKUKAiiwaFusionTrackMessage, file_identifier);
    // _fbb.Finish(_fbLogKUKAiiwaFusionTrack);
    // auto verifier = flatbuffers::Verifier(_fbb.GetBufferPointer(), _fbb.GetSize());
    // bool success = verifier.VerifyBuffer<grl::flatbuffer::LogKUKAiiwaFusionTrack>();
    // std::cout <<" ------------------verifier success for LogKUKAiiwaFusionTrack: " << success << std::endl;

    return grl::flatbuffer::CreateLogKUKAiiwaFusionTrack(fbb, states);
}
/// Change from the absolute path to relative path.
std::string getpathtofbsfile(const std::string &filename)
{

    char buff[512];
    /// Get the current working directory
    /// ../src/robonetracker/build/bin
    getcwd(buff, 512);
    std::string current_working_dir(buff);
    /// std::cout << "Current working dir: " << buff << std::endl;
    /// Strip the last component of the path + separator.
    /// ../src/robonetracker/build
    return flatbuffers::StripFileName(buff);
}
} // End of grl namespace

#endif // GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
