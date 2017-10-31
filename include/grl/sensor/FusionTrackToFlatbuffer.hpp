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
    for (int i = 0; i < geometry.pointsCount; i++)
    {
        fiducials[i] = toFlatBuffer(geometry.positions[i]);
    }

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

    for (int i = 0; i < geometry_size; i++)
    {
        MarkerModelGeometry = params.markerModelGeometries[i];
        for (int j = 0; j < MarkerModelGeometry.size(); j++)
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
    auto fbGemetryvector = fbb.CreateVector(&geometryvector[0], geometryvector.size());
    return fbGemetryvector;
}

/// Convert uint32 Mask to a std::vector.
/// Refer to https://stackoverflow.com/a/2686571
std::vector<uint32> bitMaskToVector(uint32 x)
{
    std::vector<uint32> ret;
    while (x)
    {
        if (x & 1)
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
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkMarker>>>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const std::vector<::ftkMarker> &ftkMarkers,
             const std::vector<std::string> &markername)
{
    std::vector<flatbuffers::Offset<grl::flatbuffer::ftkMarker>> fbMarkers;
    int markersize = ftkMarkers.size();
    for (int i = 0; i < markersize; i++)
    {
        fbMarkers.push_back(toFlatBuffer(fbb, ftkMarkers[i], markername[i]));
    }
    auto fbmarkervector = fbb.CreateVector(&fbMarkers[0], markersize);
    return fbmarkervector;
}
   /// Converte a global variable ::ftk3DFiducial to grl::flatbutter::ftk3DFiducial.
flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const ::ftk3DFiducial &Fiducial,
             const std::string markername = "")
{
    /// The type of Fiducial.positionMM is ftk3DPoint.
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
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const std::vector<::ftk3DFiducial> &fiducials,
             const std::vector<std::string> &markername)
{
    std::vector<flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>> fbfiducials;
    int fiducialsize = fiducials.size();
    int marker_name_size = markername.size();

    for (int i = 0; i < fiducialsize; i++)
    {
        fbfiducials.push_back(toFlatBuffer(fbb, fiducials[i], markername[i]));
    }
    auto fbfiducialvector = fbb.CreateVector(&fbfiducials[0], fiducialsize);
    return fbfiducialvector;
}
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

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>>>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const std::vector<::ftkRawData> &ftkRawDatas)
{
    std::vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>> ftkRegionOfInterests;
    int ftkRegionOfInterest_size = ftkRawDatas.size();

    for (int i = 0; i < ftkRegionOfInterest_size; i++)
    {
        ftkRegionOfInterests.push_back(toFlatBuffer(fbb, ftkRawDatas[i]));
    }
    return fbb.CreateVector(ftkRegionOfInterests);
}



flatbuffers::Offset<grl::flatbuffer::FusionTrackFrame>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const grl::sensor::FusionTrack::Frame &frame)
{
    /// @todo TODO(ahundt) IN PROGRESS
    /// Here we should get the markers'name
    std::vector<std::string> markerNames;
    for (auto &fiducial : frame.Fiducials)
    {
        /// @todo TODO(ahundt) look up actual marker names and set with "id_geometryID" here, or "" if no marker.
        markerNames.push_back("");
    }

    static const double microsecToSec = 1 / 1000000;
    flatbuffers::FlatBufferBuilder &_fbb = fbb;
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
    
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>>> regionsOfInterestLeft = toFlatBuffer(fbb, frame.ImageRegionOfInterestBoxesLeft);
    uint32_t regionsOfInterestLeftVersion = frame.FrameQueryP->rawDataLeftVersionSize.Version;
    int32_t regionsOfInterestLeftStatus = frame.FrameQueryP->rawDataLeftStat;
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkRegionOfInterest>>> regionsOfInterestRight = toFlatBuffer(fbb, frame.ImageRegionOfInterestBoxesRight);
    uint32_t regionsOfInterestRightVersion = frame.FrameQueryP->rawDataRightVersionSize.Version;
    int32_t regionsOfInterestRightStatus = frame.FrameQueryP->rawDataRightStat;
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftk3DFiducial>>> threeDFiducials = toFlatBuffer(fbb, frame.Fiducials, markerNames);
    uint32_t threeDFiducialsVersion = frame.FrameQueryP->threeDFiducialsVersionSize.Version;
    int32_t threeDFiducialsStatus = frame.FrameQueryP->threeDFiducialsStat;
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkMarker>>> markers = toFlatBuffer(fbb, frame.Markers, markerNames);
    uint32_t markersVersion = frame.FrameQueryP->markersVersionSize.Version;
    int32_t markersStatus = frame.FrameQueryP->markersStat;
    int32_t deviceType = frame.DeviceType;
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
    int geometryFiles_size = _params.geometryFilenames.size();
    std::vector<flatbuffers::Offset<flatbuffers::String>> geometryFilenames;
    std::vector<flatbuffers::Offset<flatbuffers::String>> markerNames;
    for (int i = 0; i < geometryFiles_size; i++)
    {
        geometryFilenames.push_back(fbb.CreateString(_params.geometryFilenames[i]));
        markerNames.push_back(fbb.CreateString(_params.markerNames[i]));
    }

    // device serial numbers
    std::vector<uint64_t> _m_deviceSerialNumbers = fusiontrack.getDeviceSerialNumbers();
    std::vector<uint8_t> _m_device_types = fusiontrack.getDeviceTypes();
    flatbuffers::Offset<flatbuffers::String> _name = fbb.CreateString(_params.name);
    flatbuffers::Offset<flatbuffers::String> _deviceClockID = fbb.CreateString(_params.deviceClockID);
    flatbuffers::Offset<flatbuffers::String> _localClockID = fbb.CreateString(_params.localClockID);
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ftkGeometry>>> _geometries = toFlatBuffer(fbb, _params);
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>> _geometryFilenames = fbb.CreateVector(&geometryFilenames[0], geometryFiles_size);
    flatbuffers::Offset<flatbuffers::String> _geometryDir = fbb.CreateString(_params.geometryDir);
    flatbuffers::Offset<flatbuffers::Vector<uint64_t>> _TrackerDeviceIDs = fbb.CreateVector(&(_params.TrackerDeviceIDs[0]), _params.TrackerDeviceIDs.size());
    flatbuffers::Offset<flatbuffers::Vector<uint64_t>> _markerIDs = fbb.CreateVector(&_params.markerIDs[0], _params.markerIDs.size());
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>> _markerNames = fbb.CreateVector(&markerNames[0], geometryFiles_size);
    flatbuffers::Offset<flatbuffers::Vector<uint64_t>> m_deviceSerialNumbers = fbb.CreateVector(&_m_deviceSerialNumbers[0], _m_deviceSerialNumbers.size());
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> m_device_types = fbb.CreateVector(&_m_device_types[0], _m_device_types.size());
    return grl::flatbuffer::CreateFusionTrackParameters(fbb,
                                                        _name,
                                                        _deviceClockID,
                                                        _localClockID,
                                                        _geometries,
                                                        _geometryFilenames,
                                                        _geometryDir,
                                                        _TrackerDeviceIDs,
                                                        _markerIDs,
                                                        _markerNames,
                                                        m_deviceSerialNumbers,
                                                        m_device_types);
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
    // std::cout << "Type: " << typeid(timeStamp.local_request_time).name() << std::endl;
    /// https://github.com/googlecartographer/cartographer/blob/master/cartographer/common/time.cc
    /// convert time to int64
    int64_t local_request_time = 0; //cartographer::common::ToUniversal(timeStamp.local_request_time);
    flatbuffers::Offset<flatbuffers::String> device_clock_id = fbb.CreateString(const_cast<const char *>(timeStamp.device_clock_id.begin()), stringLength(timeStamp.device_clock_id));
    int64_t device_time = 0; //ToUniversal(timeStamp.device_time);
    flatbuffers::Offset<flatbuffers::String> local_clock_id = fbb.CreateString(const_cast<const char *>(timeStamp.local_clock_id.begin()), stringLength(timeStamp.local_clock_id));
    int64_t local_receive_time = 0;   //ToUniversal(timeStamp.local_receive_time);
    int64_t corrected_local_time = 0; //ToUniversal(timeStamp.corrected_local_time);
    int64_t clock_skew = 0;           //cartographer::common::ToSeconds(timeStamp.clock_skew);
    int64_t min_transport_delay = 0;  //cartographer::common::ToSeconds(timeStamp.min_transport_delay);
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
             const grl::sensor::FusionTrack::Frame &frame)
{
    static const double microsecToSec = 1 / 1000000;
    double timestamp = frame.imageHeader.timestampUS * microsecToSec;
    flatbuffers::Offset<grl::flatbuffer::FusionTrackParameters> parameters = toFlatBuffer(fbb, fusiontrack);
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

flatbuffers::Offset<grl::flatbuffer::LogKUKAiiwaFusionTrack>
toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb,
             const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaFusionTrackMessage>>> states)
{
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
