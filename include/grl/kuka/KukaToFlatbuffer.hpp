#ifndef GRL_KUKA_TO_FLATBUFFER
#define GRL_KUKA_TO_FLATBUFFER

#include "grl/flatbuffer/JointState_generated.h"
#include "grl/flatbuffer/ArmControlState_generated.h"
#include "grl/flatbuffer/KUKAiiwa_generated.h"
#include "grl/flatbuffer/LinkObject_generated.h"
namespace grl { namespace robot { namespace arm {

flatbuffers::Offset<grl::flatbuffer::EulerTranslationParams> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const double x,
    const double y,
    const double z)
{
    return grl::flatbuffer::CreateEulerTranslationParams(fbb, x, y, z);
}

flatbuffers::Offset<grl::flatbuffer::EulerRotationParams> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const double r1,
    const double r2,
    const double r3,
    grl::flatbuffer::EulerOrder &eulerOrder)
{
    return grl::flatbuffer::CreateEulerRotationParams(fbb, r1, r2, r3, eulerOrder);
}
// Helper function is also defined in FusionTrackToFlatbuffer.hpp
grl::flatbuffer::Vector3d toFlatBuffer(const Eigen::Vector3d &pt)
{
    return grl::flatbuffer::Vector3d(pt.x(), pt.y(), pt.z());
}

// How to distinguish different eulerOrder?
grl::flatbuffer::EulerRotation toFlatBuffer(
    const Eigen::Vector3d &pt,
    grl::flatbuffer::EulerOrder eulerOrder )
{
    return grl::flatbuffer::EulerRotation(pt.x(), pt.y(), pt.z(), eulerOrder);
}

grl::flatbuffer::EulerPose toFlatBuffer(
    const grl::flatbuffer::Vector3d &positon,
    const grl::flatbuffer::EulerRotation &eulerRotation)
{
    return grl::flatbuffer::EulerPose(positon, eulerRotation);
}

// With ::ftk3DPoint, we can get the structure Vector3D, so do we need overwrite this function with input parameter ::ftk3DPoint?
// table EulerPose and EulerPoseParams are both defined in Euler.fbs.
flatbuffers::Offset<grl::flatbuffer::EulerPoseParams> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const grl::flatbuffer::Vector3d &position,
    const grl::flatbuffer::EulerRotation &rotation)
{
    return grl::flatbuffer::CreateEulerPoseParams(fbb, position, rotation);
}

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
    const ::ConnectionInfo &connectionInfo,
    // const int32_t sendPeriodMillisec,
    // const int32_t setReceiveMultiplier,
    const bool updatePortOnRemote,
    const int16_t portOnRemote,
    const bool updatePortOnController,
    const int16_t portOnController)
{
    return grl::flatbuffer::CreateFRI(
        fbb,
        EOverlayType,
        connectionInfo.sendPeriod,
        connectionInfo.receiveMultiplier,
        updatePortOnRemote,
        portOnRemote,
        updatePortOnController,
        portOnController
    );
}

/// KUKAiiwa.fbs
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

/// KUKAiiwa.fbs
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

/// LinkObject.fbs
flatbuffers::Offset<grl::flatbuffer::LinkObject> toFlatBuffer(
    flatbuffers::FlatBufferBuilder fbb,
    const std::string &name,
    const std::string &parent,
    const grl::flatbuffer::Pose &pose,  // Using the helper function toFlatBuffer to get this parameter?
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



/// KUKAiiwa.fbs
flatbuffers::Offset<grl::flatbuffer::KUKAiiwaArmConfiguration> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::string &name,
    const grl::flatbuffer::KUKAiiwaInterface &commandInterface, // enum defined in KUKAiiwa.fbs
    const grl::flatbuffer::KUKAiiwaInterface &monitorInterface,
    // const grl::flatbuffer::ClientCommandMode clientCommandMode,
    const ::ClientCommandMode &clientCommandMode, // enum
    const ::OverlayType &overlayType, // enum
    const ::ControlMode &controlMode, // enum
    flatbuffers::Offset<grl::flatbuffer::CartesianImpedenceControlMode> &setCartImpedance,
    flatbuffers::Offset<grl::flatbuffer::JointImpedenceControlMode> &setJointImpedance,
    flatbuffers::Offset<grl::flatbuffer::SmartServo> &smartServoConfig,
    flatbuffers::Offset<grl::flatbuffer::FRI> &FRIConfig,
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::LinkObject>>> &tools,
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::ProcessData>>> &processData,
    const std::string &currentMotionCenter,
    const bool requestMonitorProcessData)
{
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

/// JointState.fbs
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

/// KUKAiiwa.fbs
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

/// KUKAiiwa.fbs
flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::string &name,
    const td::string &destination,
    const td::string &source
    const double timestamp,
    const bool setArmControlState,
    flatbuffers::Offset<grl::flatbuffer:ArmControlState> &armControlState,
    const bool setArmConfiguration,
    flatbuffers::Offset<grl::flatbuffer:KUKAiiwaArmConfiguration> &armConfiguration,
    const bool hasMonitorState ,
    flatbuffers::Offset<grl::flatbuffer:KUKAiiwaMonitorState> &monitorState,
    const bool hasMonitorConfig,
    flatbuffers::Offset<grl::flatbuffer:KUKAiiwaMonitorConfiguration> &monitorConfig)
{
    return grl::flatbuffer::CreateKUKAiiwaState(
      fbb,
      fbb.CreateString(name),
      fbb.CreateString(destination),
      fbb.CreateString(source),
      timestamp,
      setArmControlState,
      armControlState,
      setArmConfiguration,
      armConfiguration,
      hasMonitorState,
      monitorState,
      hasMonitorConfig,
      monitorConfig);
}

/// ArmControlState.fbs
flatbuffers::Offset<grl::flatbuffer::MoveArmTrajectory> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    std::vector<flatbuffers::Offset<grl::flatbuffer::JointState>> &traj
   )
{
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<grl::flatbuffer::JointState>>> traj_vector = fbb.CreateVector(traj.data(), traj.size());
    return grl::flatbuffer::CreateMoveArmTrajectory(
      fbb,
      traj_vector);
}

/// ArmControlState.fbs
flatbuffers::Offset<grl::flatbuffer::MoveArmJointServo> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<grl::flatbuffer::JointState> &goal
   )
{
    return grl::flatbuffer::CreateMoveArmJointServo(
      fbb,
      goal);
}

/// ArmControlState.fbs
flatbuffers::Offset<grl::flatbuffer::MoveArmJointServo> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::string *parent,
    const grl::flatbuffer::Pose &goal)
{
    return grl::flatbuffer::CreateMoveArmJointServo(
      fbb,
      fbb.CreateString(parent),
      goal);
}


/// ArmControlState.fbs
// flatbuffers::Offset<grl::flatbuffer::ArmControlState> toFlatBuffer(
//     flatbuffers::FlatBufferBuilder &fbb,
//     const std::string &name,
//     int64_t sequenceNumber,
//     double timeStamp)
// {
//     // The parameter of ArmState is undetermined.
//     grl::flatbuffer::ArmState state_type = grl::flatbuffer::ArmStat::StartArm;
//     return grl::flatbuffer::CreateArmControlState(
//       fbb,
//       fbb.CreateString(name),
//       sequenceNumber,
//       timeStamp,
//       _state_type,
//       _state_type.Union());
// }
}  // End of arm namespace
}  // End of robot namespace
}  // End of grl namespace


#endif // GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
