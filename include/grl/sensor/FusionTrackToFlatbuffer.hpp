#ifndef GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
#define GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER

/// Before including any FlatBuffers related headers, you can add this #define.
/// You'll get an assert whenever the verifier fails, whose stack-trace can tell you exactly what check failed an on what field etc.
#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE

#include "FusionTrackToEigen.hpp"
#include "FusionTrack.hpp"
#include "grl/flatbuffer/FusionTrack_generated.h"
#include "grl/flatbuffer/Time_generated.h"
#include "grl/flatbuffer/LogKUKAiiwaFusionTrack_generated.h"
#include "grl/flatbuffer/HelperToFlatbuffer.hpp"
#include <flatbuffers/util.h>
#include <flatbuffers/idl.h>
#include <iostream>
#include <cstdio>
#include <cassert>
#include <ftkInterface.h>

namespace grl
{


/// Helper function for use when building up messages to save to a log file.
/// Call this just before SaveFlatBufferFile. See fusionTrackExample for how
/// and when to use it.
bool FinishAndVerifyBuffer(
    flatbuffers::FlatBufferBuilder& fbb,
    std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>& KUKAiiwaFusionTrackMessage_vector
)
{

    auto states = fbb.CreateVector(KUKAiiwaFusionTrackMessage_vector);
    auto fbLogKUKAiiwaFusionTrack = grl::flatbuffer::CreateLogKUKAiiwaFusionTrack(fbb, states);

    // Finish a buffer with given object
    // Call `Finish()` to instruct the builder fbb that this frame is complete.
    const char *file_identifier = grl::flatbuffer::LogKUKAiiwaFusionTrackIdentifier();
    // fbb.Finish(oneKUKAiiwaFusionTrackMessage, file_identifier);
    fbb.Finish(fbLogKUKAiiwaFusionTrack, file_identifier);

    auto verifier = flatbuffers::Verifier(fbb.GetBufferPointer(), fbb.GetSize());
    bool success = grl::flatbuffer::VerifyLogKUKAiiwaFusionTrackBuffer(verifier);

    return success;
}

grl::flatbuffer::Vector3d toFlatBuffer(const ::ftk3DPoint &pt)
{
    return grl::flatbuffer::Vector3d(pt.x, pt.y, pt.z);
}

// grl::flatbuffer::Vector3d toFlatBuffer(const Eigen::Vector3d &pt)
// {
//     return grl::flatbuffer::Vector3d(pt.x(), pt.y(), pt.z());
// }

// grl::flatbuffer::Quaternion toFlatBuffer(Eigen::Quaterniond q)
// {
//     return grl::flatbuffer::Quaternion(q.x(), q.y(), q.z(), q.w());
// }

// grl::flatbuffer::Pose toFlatBuffer(Eigen::Affine3d tf)
// {
//     Eigen::Vector3d pos = tf.translation();
//     Eigen::Quaterniond eigenQuat(tf.rotation());
//     return grl::flatbuffer::Pose(toFlatBuffer(pos), toFlatBuffer(eigenQuat));
// }

// grl::flatbuffer::Pose toFlatBuffer(Eigen::Affine3f tf)
// {
//     return toFlatBuffer(tf.cast<double>());
// }


///  the second parameter is merely a tag to uniquely identify this function so it compiles
/// its value is not utilized or modified.
grl::flatbuffer::ftkQueryStatus toFlatBuffer(const ::ftkQueryStatus queryStatus) {

    switch(queryStatus) {

        case QS_OK:
            return grl::flatbuffer::ftkQueryStatus::QS_OK;
        case QS_ERR_OVERFLOW:
            return grl::flatbuffer::ftkQueryStatus::QS_ERR_OVERFLOW;
        case QS_ERR_INVALID_RESERVED_SIZE:
            return grl::flatbuffer::ftkQueryStatus::QS_ERR_INVALID_RESERVED_SIZE;
        case QS_REPROCESS:
            return grl::flatbuffer::ftkQueryStatus::QS_REPROCESS;
        default:
            break;
        }
        return grl::flatbuffer::ftkQueryStatus::QS_WAR_SKIPPED;
}

/// the second parameter is merely a tag to identify this function so it compiles
/// its value is not utilized or modified.
grl::flatbuffer::ftkDeviceType toFlatBuffer(const ::ftkDeviceType deviceType, grl::flatbuffer::ftkDeviceType tag) {
// grl::flatbuffer::ftkDeviceType toFlatBuffer(const ::ftkDeviceType deviceType) {
    switch(deviceType) {
        case DEV_SIMULATOR: /*!< Internal use only */
              return grl::flatbuffer::ftkDeviceType::DEV_SIMULATOR;

        case DEV_INFINITRACK: /*!< Device is an infiniTrack */
              return grl::flatbuffer::ftkDeviceType::DEV_INFINITRACK;

        case DEV_FUSIONTRACK_500: /*!< Device is a fusionTrack 500 */
             return grl::flatbuffer::ftkDeviceType::DEV_FUSIONTRACK_500;

        case DEV_FUSIONTRACK_250: /*!< Device is a fusionTrack 250 */
             return grl::flatbuffer::ftkDeviceType::DEV_FUSIONTRACK_250;
        default: /**< Unknown device type. */
            break;
        };
        return grl::flatbuffer::ftkDeviceType::DEV_UNKNOWN_DEVICE;
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

    grl::flatbuffer::ftkQueryStatus imageHeaderStatus = toFlatBuffer(frame.FrameQueryP->imageHeaderStat);
    flatbuffers::Offset<flatbuffers::String> imageLeftPixels = frame.CameraImageLeftP ? fbb.CreateString(reinterpret_cast<const char *>(frame.CameraImageLeftP->begin()), sizeof(frame.CameraImageLeftP)) : 0;
    uint32_t imageLeftPixelsVersion = frame.FrameQueryP->imageLeftVersionSize.Version;
    // int32_t imageLeftStatus = frame.FrameQueryP->imageLeftStat;

    grl::flatbuffer::ftkQueryStatus imageLeftStatus = toFlatBuffer(frame.FrameQueryP->imageLeftStat);
    flatbuffers::Offset<flatbuffers::String> imageRightPixels = frame.CameraImageRightP ? fbb.CreateString(reinterpret_cast<const char *>(frame.CameraImageRightP->begin()), sizeof(frame.CameraImageRightP)) : 0;
    uint32_t imageRightPixelsVersion = frame.FrameQueryP->imageRightVersionSize.Version;
    // int32_t imageRightStatus = frame.FrameQueryP->imageRightStat;
    grl::flatbuffer::ftkQueryStatus imageRightStatus = toFlatBuffer(frame.FrameQueryP->imageRightStat);

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
    // int32_t deviceType = frame.DeviceType;
    auto deviceType = toFlatBuffer(frame.DeviceType, grl::flatbuffer::ftkDeviceType::DEV_UNKNOWN_DEVICE);
    // auto deviceType = grl::flatbuffer::ftkDeviceType::DEV_SIMULATOR;
    int64_t ftkError = frame.Error;

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

// template <typename T>
// typename T::value_type stringLength(const T &array)
// {

//     auto iter = std::find(array.begin(), array.end(), '\0');
//     auto len = std::distance(array.begin(), iter);
//     return len;
// }

// flatbuffers::Offset<grl::flatbuffer::TimeEvent>
// toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
//              const grl::TimeEvent &timeStamp)
// {
//     flatbuffers::Offset<flatbuffers::String> event_name = fbb.CreateString(const_cast<const char *>(timeStamp.event_name.begin()), stringLength(timeStamp.event_name));
//     /// https://github.com/googlecartographer/cartographer/blob/master/cartographer/common/time.cc
//     /// convert time to int64
//     int64_t local_request_time = cartographer::common::ToUniversal(timeStamp.local_request_time);
//     flatbuffers::Offset<flatbuffers::String> device_clock_id = fbb.CreateString(const_cast<const char *>(timeStamp.device_clock_id.begin()), stringLength(timeStamp.device_clock_id));
//     int64_t device_time = cartographer::common::ToUniversal(timeStamp.device_time);
//     flatbuffers::Offset<flatbuffers::String> local_clock_id = fbb.CreateString(const_cast<const char *>(timeStamp.local_clock_id.begin()), stringLength(timeStamp.local_clock_id));
//     int64_t local_receive_time = cartographer::common::ToUniversal(timeStamp.local_receive_time);
//     int64_t corrected_local_time = cartographer::common::ToUniversal(timeStamp.corrected_local_time);
//     int64_t clock_skew = cartographer::common::ToSeconds(timeStamp.clock_skew);
//     int64_t min_transport_delay = cartographer::common::ToSeconds(timeStamp.min_transport_delay);
//     return grl::flatbuffer::CreateTimeEvent(
//         fbb,
//         event_name,
//         local_request_time,
//         device_clock_id,
//         device_time,
//         local_clock_id,
//         local_receive_time,
//         corrected_local_time,
//         clock_skew,
//         min_transport_delay);
// }

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

    return grl::flatbuffer::CreateKUKAiiwaFusionTrackMessage(
        fbb,
        timestamp,
        timeEvent,
        deviceState_type,
        message.Union());
}


flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>> &KUKAiiwaFusionTrackMessage_vector) {
    return fbb.CreateVector(KUKAiiwaFusionTrackMessage_vector);
}



flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> &states)
{
    return grl::flatbuffer::CreateLogKUKAiiwaFusionTrack(fbb, states);
}


} // End of grl namespace

#endif // GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
