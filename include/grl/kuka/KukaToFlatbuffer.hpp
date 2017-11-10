#ifndef GRL_KUKA_TO_FLATBUFFER
#define GRL_KUKA_TO_FLATBUFFER

#include "grl/flatbuffer/JointState_generated.h"
#include "grl/flatbuffer/ArmControlState_generated.h"
#include "grl/flatbuffer/KUKAiiwa_generated.h"
namespace grl { namespace robot { namespace arm {

flatbuffers::Offset<grl::flatbuffer::EulerTranslationParams> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const double x = 0.0,
    const double y = 0.0,
    const double z = 0.0)
{
    return grl::flatbuffer::CreateEulerTranslationParams(fbb, x, y, z);
}

flatbuffers::Offset<grl::flatbuffer::EulerRotationParams> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const double r1 = 0.0,
    const double r2 = 0.0,
    const double r3 = 0.0,
    grl::flatbuffer::EulerOrder &eulerOrder)
{
    return grl::flatbuffer::CreateEulerRotationParams(fbb, r1, r2, r3, eulerOrder);
}

flatbuffers::Offset<grl::flatbuffer::EulerPoseParams> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const grl::flatbuffer::Vector3d &position,
    const grl::flatbuffer::EulerRotation &rotation)
{
    return grl::flatbuffer::CreateEulerPoseParams(fbb, position, rotation);
}

flatbuffers::Offset<grl::flatbuffer::CartesianImpedenceControlMode> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const grl::flatbuffer::EulerPose &stiffness,
    const grl::flatbuffer::EulerPose &damping,
    const double nullspaceStiffness,
    const double nullspaceDamping,
    const grl::flatbuffer::EulerPose &maxPathDeviation,
    const grl::flatbuffer::EulerPose &maxCartesianVelocity,
    const grl::flatbuffer::EulerPose &maxControlForce,
    const bool maxControlForceExceededStop)
{
    return grl::flatbuffer::CreateCartesianImpedenceControlMode(
        fbb,
        stiffness,
        damping,
        nullspaceStiffness,
        nullspaceDamping,
        maxPathDeviation,
        maxCartesianVelocity,
        maxControlForce,
        maxControlForceExceededStop);
}

flatbuffers::Offset<grl::flatbuffer::CreateJointImpedenceControlMode> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    std::vector<double> &joint_stiffness,
    std::vector<double> &joint_damping)
{
    auto jointStiffnessBuffer = fbb.CreateVector(joint_stiffness.data(),joint_stiffness.size());
    auto jointDampingBuffer = fbb.CreateVector(joint_damping.data(),joint_damping.size());
    return grl::flatbuffer::CreateJointImpedenceControlMode(fbb, jointStiffnessBuffer, jointDampingBuffer);
}

flatbuffers::Offset<grl::flatbuffer::FRI> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    grl::flatbuffer::EOverlayType overlayType,
    const int32_t sendPeriodMillisec,
    const int32_t setReceiveMultiplier,
    const bool updatePortOnRemote,
    const int16_t portOnRemote,
    const bool updatePortOnController,
    const int16_t portOnController)
{

    return grl::flatbuffer::CreateFRI(
        fbb,
        EOverlayType,
        sendPeriodMillisec,
        setReceiveMultiplier,
        updatePortOnRemote,
        portOnRemote,
        updatePortOnController,
        portOnController
    );
}

// flatbuffers::Offset<CartesianImpedenceControlMode> toFlatBuffer( )
// {

// }

}  // End of arm namespace
}  // End of robot namespace
}  // End of grl namespace


#endif // GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
