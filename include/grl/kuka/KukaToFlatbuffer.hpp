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
    const ::OverlayType &overlayType,
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

flatbuffers::Offset<grl::flatbuffer::SmartServo> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::vector<double> &jointAccelerationRel,
    const std::vector<double> &jointVelocityRel,
    const bool updateMinimumTrajectoryExecutionTime,
    const double minimumTrajectoryExecutionTime)
{
    auto jointAccelerationRelBuffer = fbb.CreateVector(jointAccelerationRel.data(),jointAccelerationRel.size());
    auto jointVelocityRelBuffer = fbb.CreateVector(jointVelocityRel.data(),jointVelocityRel.size());
    return grl::flatbuffer::CreateSmartServo(
        fbb,
        jointAccelerationRelBuffer,
        jointAccelerationRelBuffer,
        updateMinimumTrajectoryExecutionTime,
        minimumTrajectoryExecutionTime
    );
}

flatbuffers::Offset<grl::flatbuffer::ProcessData> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::string &dataType,
    const std::string &defaultValue,
    const std::string &displayName,
    const std::string &id,
    const std::string &min,
    const std::string &max,
    const std::string &unit,
    const std::string &value,
    bool shouldRemove = false,
    bool shouldUpdate = false)
{
    auto jointAccelerationRelBuffer = fbb.CreateVector(jointAccelerationRel.data(),jointAccelerationRel.size());
    auto jointVelocityRelBuffer = fbb.CreateVector(jointVelocityRel.data(),jointVelocityRel.size());
    return grl::flatbuffer::CreateProcessData(
        fbb,
        fbb.CreateString(datatype),
        fbb.CreateString(defaultValue),
        fbb.CreateString(displayName),
        fbb.CreateString(id),
        fbb.CreateString(min),
        fbb.CreateString(max),
        fbb.CreateString(unit),
        fbb.CreateString(value),
        fbb.CreateString(datatype),
        shouldRemove,
        shouldUpdate
    );
}


flatbuffers::Offset<grl::flatbuffer::LinkObject> toFlatBuffer(
    flatbuffers::FlatBufferBuilder fbb,
    const std::string &name,
    const std::string &parent,
    const grl::flatbuffer::Pose &pose,
    const grl::flatbuffer::Inertia &inertia)
    {
        return grl::flatbuffer::CreateLinkObject(
            fbb,
            fbb.CreateString(name),
            fbb.CreateString(parent),
            pose,
            inertia
        );
    }




flatbuffers::Offset<grl::flatbuffer::KUKAiiwaArmConfiguration> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::string &name,
    const grl::flatbuffer::KUKAiiwaInterface &commandInterface,
    const grl::flatbuffer::KUKAiiwaInterface &monitorInterface,
    // const grl::flatbuffer::ClientCommandMode clientCommandMode,
    const ::ClientCommandMode &clientCommandMode,
    const ::OverlayType &overlayType,
    const ::ControlMode &controlMode,
    flatbuffers::Offset<grl::flatbuffer::CartesianImpedenceControlMode> &setCartImpedance,
    flatbuffers::Offset<grl::flatbuffer::JointImpedenceControlMode> &setJointImpedance,
    flatbuffers::Offset<grl::flatbuffer::SmartServo> &smartServoConfig,
    flatbuffers::Offset<grl::flatbuffer::FRI> &FRIConfig,
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::LinkObject>>> &tools,
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ProcessData>>> &processData,
    const std::string &currentMotionCenter,
    const bool requestMonitorProcessData)
{
    auto jointAccelerationRelBuffer = fbb.CreateVector(jointAccelerationRel.data(),jointAccelerationRel.size());
    auto jointVelocityRelBuffer = fbb.CreateVector(jointVelocityRel.data(),jointVelocityRel.size());
    return  grl::flatbuffer::CreateKUKAiiwaArmConfiguration(
      fbb,
      fbb.CreateString(name),
      commandInterface,
      monitorInterface,
      clientCommandMode,
      overlayType,
      controlMode,
      setCartImpedance,
      setJointImpedance,
      smartServoConfig,
      FRIConfig,
      tools,
      processData,
      fbb.CreateString(currentMotionCenter),
      requestMonitorProcessData);
}

flatbuffers::Offset<grl::flatbuffer::JointState> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::vector<double> &position,
    const std::vector<double> &velocity,
    const std::vector<double> &acceleration,
    const std::vector<double> &torque)
{
    return grl::flatbuffer::CreateJointState(
        fbb,
        position ? fbb.CreateVector<double>(position) : 0,
        velocity ? fbb.CreateVector<double>(velocity) : 0,
        acceleration ? fbb.CreateVector<double>(acceleration) : 0,
        torque ? fbb.CreateVector<double>(torque) : 0);
}
flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorState> toFlatBuffer(
    flatbuffers::FlatBufferBuilder fbb,
    flatbuffers::Offset<grl::flatbuffer::JointState> &measuredState,
    const grl::flatbuffer::Pose &cartesianFlangePose,
    flatbuffers::Offset<grl::flatbuffer::JointState> &jointStateReal,
    flatbuffers::Offset<grl::flatbuffer::JointState> &jointStateInterpolated,
    flatbuffers::Offset<grl::flatbuffer::JointState> &externalState,
    const ::OperationMode &operationMode,
    const grl::flatbuffer::Wrench &CartesianWrench)
{
    return grl::flatbuffer::CreateJointState(
        fbb,
        measuredState,
        cartesianFlangePose,
        jointStateReal,
        jointStateInterpolated,
        externalState,
        operationMode,
        CartesianWrench);
}


// flatbuffers::Offset<CartesianImpedenceControlMode> toFlatBuffer( )
// {

// }

}  // End of arm namespace
}  // End of robot namespace
}  // End of grl namespace


#endif // GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
