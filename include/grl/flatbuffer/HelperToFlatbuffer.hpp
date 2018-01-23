#ifndef GRL_HELPER_TO_FLATBUFFER
#define GRL_HELPER_TO_FLATBUFFER

#include "grl/flatbuffer/Time_generated.h"
#include "grl/flatbuffer/Geometry_generated.h"
#include "grl/TimeEvent.hpp"
#include <flatbuffers/util.h>
#include <flatbuffers/idl.h>
#include <iostream>
#include <cstdio>
#include <cassert>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace grl
{


/// Helper function for both KUKA and FusionTrack
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

} // End of grl namespace

#endif // GRL_HELPER_TO_FLATBUFFER
