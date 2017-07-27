#ifndef GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
#define GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER

#include "FusionTrackToEigen.hpp"
#include "FusionTrack.hpp"
#include "ftkInterface.h"

namespace grl{

    grl::flatbuffer::Vector3d toFlatBuffer(const ::ftk3DPoint& pt)
    {
        return grl::flatbuffer::Vector3d(pt.x,pt.y,pt.z);
    }


    grl::flatbuffer::Vector3d toFlatBuffer(const Eigen::Vector3d& pt)
    {
        return grl::flatbuffer::Vector3d(pt.x(),pt.y(),pt.z());
    }

    grl::flatbuffer::Quaternion toFlatBuffer(Eigen::Quaterniond q)
    {
        return grl::flatbuffer::Quaternion(q.x(), q.y(), q.z(), q.w());
    }

    grl::flatbuffer::Pose toFlatBuffer(Eigen::Affine3d tf)
    {
        return grl::flatbuffer::Pose(toFlatBuffer(tf.translation()), toFlatBuffer(tf.rotation()));
    }

    grl::flatbuffer::Pose toFlatBuffer(Eigen::Affine3f tf)
    {
        return toFlatBuffer(tf.cast<double>());
    }

    flatbuffers::Offset<grl::flatbuffer::ftkGeometry>
    toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const ::ftkGeometry& geometry, const std::string name = "")
    {
        std::array<grl::flatbuffer::Vector3d, FTK_MAX_FIDUCIALS> fiducials;
        for(int i = 0; i < geometry.pointsCount; i++) fiducials[i] = toFlatBuffer(geometry.positions[i]);

        return grl::flatbuffer::CreateftkGeometry(
            fbb,
            fbb.CreateString(name),
            geometry.geometryId,
            geometry.version,
            fbb.CreateVectorOfStructs<grl::flatbuffer::Vector3d>(fiducials.begin(), geometry.pointsCount));

    }

    flatbuffers::Offset<grl::flatbuffer::ftkMarker>
    toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const ::ftkMarker& marker, const std::string name = "" )
    {
        return grl::flatbuffer::CreateftkMarker(
            fbb,
            name,
            marker.id,
            marker.geometryId,
            marker.geometryPresenceMask,
            fbb.CreateVector(marker,FTK_MAX_FIDUCIALS),
            toFlatBuffer(ftkMarkerToAffine3f(marker))
        );
    }

    flatbuffers::Offset<grl::flatbuffer::ftkGeometry>
    toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const std::vector<::ftk3DFiducials>& geometry, const std::vector<std::string> & name)
    {
        std::vector<grl::flatbuffer::ftk3DFiducial> fiducials;
        for(int i = 0; i < geometry.pointsCount; i++) fiducials[i] = toFlatBuffer(geometry.positions[i]);
        /// @todo TODO(ahundt) IN PROGRESS
    }

    flatbuffers::Offset<grl::flatbuffer::ftkMarker>
    toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const std::vector<::ftkMarker>& marker, const std::vector<std::string> & name)
    {
        std::vector<grl::flatbuffer::ftk3DFiducial> fiducials;
        for(int i = 0; i < geometry.pointsCount; i++) fiducials[i] = toFlatBuffer(marker);
        /// @todo TODO(ahundt) IN PROGRESS
    }

    flatbuffers::Offset<grl::flatbuffer::FusionTrackFrame>
    toFlatBuffer(flatbuffers::FlatBufferBuilder &fbb, const grl::sensor::FusionTrack::Frame& frame)
    {
        static const double microsecToSec = 1/1000000;
        return grl::flatbuffer::CreateFusionTrackFrame(
            fbb,
            frame.imageHeader.timestampUS*microsecToSec, // happens to claim unix time, so we can just multiply.
            frame.SerialNumber,
            frame.imageHeader.desynchroUS,
            frame.imageHeader.counter,
            frame.imageHeader.format,
            frame.imageHeader.width,
            frame.imageHeader.height,
            frame.imageHeader.imageStrideInBytes,
            frame.FrameQueryP->imageHeaderStat,
            frame.CameraImageLeftP ? fbb.CreateString(frame.CameraImageLeft->begin()):0,
            frame.FrameQueryP->imageLeftVersionSize.Version,
            frame.FrameQueryP->imageLeftStat,
            frame.CameraImageRightP ? fbb.CreateString(frame.CameraImageRight->begin()):0,
            frame.FrameQueryP->imageRightVersionSize.Version,
            frame.FrameQueryP->imageRightStat,
            0, /// @todo TODO(ahundt) Region Of Interest aka ftkRawData
            0, /// @todo TODO(ahundt) Region Of Interest aka ftkRawData
            0, /// @todo TODO(ahundt) Region Of Interest aka ftkRawData
            0, /// @todo TODO(ahundt) Region Of Interest aka ftkRawData
            0, /// @todo TODO(ahundt) Region Of Interest aka ftkRawData
            0, /// @todo TODO(ahundt) Region Of Interest aka ftkRawData
            toFlatBuffer(frame.Fiducials),
            frame.FrameQueryP->threeDFiducialsVersionSize.Version,
            frame.FrameQueryP->threeDFiducialsStat,
            toFlatBuffer(frame.Markers),
            frame.FrameQueryP->markersVersionSize.Version,
            frame.FrameQueryP->markersStat,
            0, /// @todo TODO(ahundt) set the device type correctly
            frame.Error
        );
    }
}

#endif // GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
