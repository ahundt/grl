#ifndef GRL_KUKA_TO_FLATBUFFER
#define GRL_KUKA_TO_FLATBUFFER

/// Before including any FlatBuffers related headers, you can add this #define.
/// You'll get an assert whenever the verifier fails, whose stack-trace can tell you exactly what check failed an on what field etc.
#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE

#include <boost/range/algorithm/copy.hpp>

#include "grl/flatbuffer/JointState_generated.h"
#include "grl/flatbuffer/ArmControlState_generated.h"
#include "grl/flatbuffer/KUKAiiwa_generated.h"
#include "grl/flatbuffer/LinkObject_generated.h"
#include "grl/flatbuffer/Euler_generated.h"
#include "grl/flatbuffer/HelperToFlatbuffer.hpp"
#include "KukaFRIdriver.hpp"
#include "Kuka.hpp"
#include "KukaFRIalgorithm.hpp"
#include <FRIMessages.pb.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
namespace grl {

/// faltbuffer enum objects
/// 1. Which element in the enum should be selected as the defaut return value?
/// 2. Is the argument like enum, int, double etc. passed by value or by reference?

grl::flatbuffer::ESessionState toFlatBuffer(const ::FRISessionState sessionState) {

    switch(sessionState) {
        case FRISessionState_MONITORING_WAIT:
            return grl::flatbuffer::ESessionState::MONITORING_WAIT;
        case FRISessionState_MONITORING_READY:
            return grl::flatbuffer::ESessionState::MONITORING_READY;
        case FRISessionState_COMMANDING_WAIT:
            return grl::flatbuffer::ESessionState::COMMANDING_WAIT;
         case FRISessionState_COMMANDING_ACTIVE:
            return grl::flatbuffer::ESessionState::COMMANDING_ACTIVE;
        default:
            break;
        }
        return grl::flatbuffer::ESessionState::IDLE;
}

grl::flatbuffer::EConnectionQuality toFlatBuffer(const ::FRIConnectionQuality connectionQuality) {

    switch(connectionQuality) {
        case FRIConnectionQuality_POOR:
            return grl::flatbuffer::EConnectionQuality::POOR;
        case FRIConnectionQuality_FAIR:
            return grl::flatbuffer::EConnectionQuality::FAIR;
        case FRIConnectionQuality_GOOD:
            return grl::flatbuffer::EConnectionQuality::GOOD;

        default:
            break;
        }
        return grl::flatbuffer::EConnectionQuality::EXCELLENT;
}

grl::flatbuffer::ESafetyState toFlatBuffer(const ::SafetyState safetyState) {

    switch(safetyState) {
        case SafetyState_SAFETY_STOP_LEVEL_0:
            return grl::flatbuffer::ESafetyState::SAFETY_STOP_LEVEL_0;
        case SafetyState_SAFETY_STOP_LEVEL_1:
            return grl::flatbuffer::ESafetyState::SAFETY_STOP_LEVEL_1;
        case SafetyState_SAFETY_STOP_LEVEL_2:
            return grl::flatbuffer::ESafetyState::SAFETY_STOP_LEVEL_2;

        default:
            break;
        }
        return grl::flatbuffer::ESafetyState::NORMAL_OPERATION;
}

grl::flatbuffer::EOperationMode toFlatBuffer(const ::OperationMode operationMode) {

    switch(operationMode) {
        case OperationMode_TEST_MODE_1:
            return grl::flatbuffer::EOperationMode::TEST_MODE_1;
        case OperationMode_TEST_MODE_2:
            return grl::flatbuffer::EOperationMode::TEST_MODE_2;

        default:
            break;
        }
        return grl::flatbuffer::EOperationMode::AUTOMATIC_MODE;
}

grl::flatbuffer::EDriveState toFlatBuffer(const ::DriveState driveState) {

    switch(driveState) {
        case DriveState_OFF:
            return grl::flatbuffer::EDriveState::OFF;
        case DriveState_TRANSITIONING:
            return grl::flatbuffer::EDriveState::TRANSITIONING;

        default: //  DriveState_ACTIVE
            break;
        }
        return grl::flatbuffer::EDriveState::ACTIVE;
}

grl::flatbuffer::EControlMode toFlatBuffer(const ::ControlMode controlMode) {

    switch(controlMode) {
        case ControlMode_POSITION_CONTROLMODE:
            return grl::flatbuffer::EControlMode::POSITION_CONTROL_MODE;
        case ControlMode_CARTESIAN_IMPEDANCE_CONTROLMODE:
            return grl::flatbuffer::EControlMode::CART_IMP_CONTROL_MODE;
        case ControlMode_JOINT_IMPEDANCE_CONTROLMODE:
            return grl::flatbuffer::EControlMode::JOINT_IMP_CONTROL_MODE;
        default: //  ControlMode_NO_CONTROLMODE
            break;
        }
        return grl::flatbuffer::EControlMode::NO_CONTROL;
}

grl::flatbuffer::EClientCommandMode toFlatBuffer(const ::ClientCommandMode  clientcommandMode) {

    switch(clientcommandMode) {
        case ClientCommandMode_POSITION:
            return grl::flatbuffer::EClientCommandMode::POSITION;
        case ClientCommandMode_WRENCH:
            return grl::flatbuffer::EClientCommandMode::WRENCH;
         case ClientCommandMode_TORQUE:
            return grl::flatbuffer::EClientCommandMode::TORQUE;

        default: //  ClientCommandMode_NO_COMMAND_MODE
            break;
        }
        return grl::flatbuffer::EClientCommandMode::NO_COMMAND_MODE;
}

grl::flatbuffer::EOverlayType toFlatBuffer(const ::OverlayType  overlayType) {

    switch(overlayType) {

        case OverlayType_JOINT:
            return grl::flatbuffer::EOverlayType::JOINT;
         case OverlayType_CARTESIAN:
            return grl::flatbuffer::EOverlayType::CARTESIAN;

        default: //  OverlayType_NO_OVERLAY
            break;
        }
        return grl::flatbuffer::EOverlayType::NO_OVERLAY;
}
/// Euler.fbs
flatbuffers::Offset<grl::flatbuffer::EulerTranslationParams> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const double x,
    const double y,
    const double z)
{
    return grl::flatbuffer::CreateEulerTranslationParams(fbb, x, y, z);
}
/// Euler.fbs
flatbuffers::Offset<grl::flatbuffer::EulerRotationParams> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const double r1,
    const double r2,
    const double r3,
    grl::flatbuffer::EulerOrder &eulerOrder)
{
    return grl::flatbuffer::CreateEulerRotationParams(fbb, r1, r2, r3, eulerOrder);
}

/// Euler.fbs, struct EulerRotation
grl::flatbuffer::EulerRotation toFlatBuffer(
    const Eigen::Vector3d &pt,
    grl::flatbuffer::EulerOrder eulerOrder )
{
    return grl::flatbuffer::EulerRotation(pt.x(), pt.y(), pt.z(), eulerOrder);
}
/// Euler.fbs, struct EulerPose
grl::flatbuffer::EulerPose toFlatBuffer(
    const grl::flatbuffer::Vector3d &positon,
    const grl::flatbuffer::EulerRotation &eulerRotation)
{
    return grl::flatbuffer::EulerPose(positon, eulerRotation);
}
/// Euler.fbs
/// Overload the above function
/// TODO (@Chunting) Check if ptr has the same physical meaning with pt, if so, discard it.
grl::flatbuffer::EulerPose toFlatBuffer(
    const Eigen::Vector3d &pt,
    const Eigen::Vector3d &ptr,
    grl::flatbuffer::EulerOrder eulerOrder)
{
    auto positon = grl::toFlatBuffer(pt);
    auto eulerRotation = toFlatBuffer(ptr,eulerOrder);
    return grl::flatbuffer::EulerPose(positon, eulerRotation);
}

/// Euler.fbs
/// tables EulerPose and EulerPoseParams are both defined in Euler.fbs.
/// Like EulerPose, this function can be also overloaded with Eigen arguments.
flatbuffers::Offset<grl::flatbuffer::EulerPoseParams> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const grl::flatbuffer::Vector3d &position,
    const grl::flatbuffer::EulerRotation &rotation)
{
    return grl::flatbuffer::CreateEulerPoseParams(fbb, &position, &rotation);
}

/// LinkObject.fbs
flatbuffers::Offset<grl::flatbuffer::LinkObject> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::string &name,
    const std::string &parent,
    const grl::flatbuffer::Pose &pose,  // Using the helper function toFlatBuffer to get this parameter?
    const grl::flatbuffer::Inertia &inertia)
{
    return grl::flatbuffer::CreateLinkObject(
        fbb,
        fbb.CreateString(name),
        fbb.CreateString(parent),
        std::addressof(pose),
        std::addressof(inertia));
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
        position.empty() ? fbb.CreateVector<double>(position) : 0,
        velocity.empty() ? fbb.CreateVector<double>(velocity) : 0,
        acceleration.empty() ? fbb.CreateVector<double>(acceleration) : 0,
        torque.empty() ? fbb.CreateVector<double>(torque) : 0);
}
/// JointState.fbs
flatbuffers::Offset<grl::flatbuffer::JointState> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::vector<grl::robot::arm::KukaState>& kukaStates,
    const std::vector<double> &velocity,
    const std::vector<double> &acceleration)
{
    std::vector<double> position;
    std::vector<double> torque;
    std::size_t sizeofStates = kukaStates.size();
    for(auto &kukaState : kukaStates){

        boost::copy(kukaState.position, &position[0]);
        boost::copy(kukaState.torque, &torque[0]);
    }
    return grl::flatbuffer::CreateJointState(
        fbb,
        position.empty() ? fbb.CreateVector<double>(position) : 0,
        velocity.empty() ? fbb.CreateVector<double>(velocity) : 0,
        acceleration.empty() ? fbb.CreateVector<double>(acceleration) : 0,
        torque.empty() ? fbb.CreateVector<double>(torque) : 0);
}


/// ArmControlState.fbs
flatbuffers::Offset<grl::flatbuffer::MoveArmTrajectory> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::vector<flatbuffers::Offset<grl::flatbuffer::JointState>> &traj)
{
    return grl::flatbuffer::CreateMoveArmTrajectory(
      fbb,
      traj.empty()?fbb.CreateVector<flatbuffers::Offset<grl::flatbuffer::JointState>>(traj):0);
}

/// ArmControlState.fbs
flatbuffers::Offset<grl::flatbuffer::MoveArmJointServo> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<grl::flatbuffer::JointState> &goal)
{
    return grl::flatbuffer::CreateMoveArmJointServo (fbb, goal);
}
// /// ArmControlState.fbs
flatbuffers::Offset<grl::flatbuffer::MoveArmCartesianServo> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::string &parent,
    const grl::flatbuffer::Pose &goal)
{
    return grl::flatbuffer::CreateMoveArmCartesianServo(
      fbb,
      fbb.CreateString(parent),
      std::addressof(goal));
}


/// ArmControlState.fbs
flatbuffers::Offset<grl::flatbuffer::ArmControlState> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::string &name,
    int64_t sequenceNumber,
    double timeStamp,
    grl::flatbuffer::ArmState &armControlMode)
{
    // The parameter of ArmState is undetermined.
    // grl::flatbuffer::ArmState armstate_type = grl::flatbuffer::ArmState::armstate;
    // auto command = grl::flatbuffer::CreateStartArm(fbb);
    switch (armControlMode) {
        case grl::flatbuffer::ArmState::StartArm: {
            return grl::flatbuffer::CreateArmControlState(
               fbb,
               fbb.CreateString(name),
               sequenceNumber,
               timeStamp,
               armControlMode,
               grl::flatbuffer::CreateStartArm(fbb).Union());
        }
        case grl::flatbuffer::ArmState::TeachArm: {
            return grl::flatbuffer::CreateArmControlState(
              fbb,
              fbb.CreateString(name),
              sequenceNumber,
              timeStamp,
              armControlMode,
              grl::flatbuffer::CreateTeachArm(fbb).Union());
        }
        case grl::flatbuffer::ArmState::PauseArm: {
            return grl::flatbuffer::CreateArmControlState(
              fbb,
              fbb.CreateString(name),
              sequenceNumber,
              timeStamp,
              armControlMode,
              grl::flatbuffer::CreatePauseArm(fbb).Union());
        }
        case grl::flatbuffer::ArmState::StopArm: {
            return grl::flatbuffer::CreateArmControlState(
              fbb,
              fbb.CreateString(name),
              sequenceNumber,
              timeStamp,
              armControlMode,
              grl::flatbuffer::CreateStopArm(fbb).Union());
        }
        case grl::flatbuffer::ArmState::ShutdownArm: {
            return grl::flatbuffer::CreateArmControlState(
              fbb,
              fbb.CreateString(name),
              sequenceNumber,
              timeStamp,
              armControlMode,
              grl::flatbuffer::CreateShutdownArm(fbb).Union());
        }
        case grl::flatbuffer::ArmState::NONE: {
           std::cout << "Waiting for interation mode... (currently NONE)" << std::endl;
           break;
        }
        default:
           std::cout<< "C++ KukaJAVAdriver: unsupported use case: " << EnumNameArmState(armControlMode) << std::endl;
    }
}

/// ArmControlState.fbs
flatbuffers::Offset<grl::flatbuffer::ArmControlState> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::string &name,
    int64_t sequenceNumber,
    double timeStamp,
    grl::robot::arm::KukaState &armState,
    grl::flatbuffer::ArmState &armControlMode)
{
    switch (armControlMode) {
        std::cout<< "C++ KukaJAVAdriver: sending armposition command: " << armState.commandedPosition_goal << std::endl;
        case grl::flatbuffer::ArmState::MoveArmJointServo: {
            auto armPositionBuffer = fbb.CreateVector(armState.commandedPosition_goal.data(), armState.commandedPosition_goal.size());
            auto commandedTorque = fbb.CreateVector(armState.commandedTorque.data(), armState.commandedTorque.size());
            auto goalJointState = grl::flatbuffer::CreateJointState(fbb,armPositionBuffer,0/*no velocity*/,0/*no acceleration*/,commandedTorque);
            auto moveArmJointServo = grl::flatbuffer::CreateMoveArmJointServo(fbb,goalJointState);
            return grl::flatbuffer::CreateArmControlState(
             fbb,
             fbb.CreateString(name),
             sequenceNumber,
             timeStamp,
             armControlMode,
             moveArmJointServo.Union());
        }
        default:
            toFlatBuffer(fbb, name, sequenceNumber, timeStamp, armControlMode);
    }
}

/// ArmControlState.fbs
flatbuffers::Offset<grl::flatbuffer::ArmControlSeries> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::vector<flatbuffers::Offset<grl::flatbuffer::ArmControlState>> &armcontrolstates)
{
    return grl::flatbuffer::CreateArmControlSeries(
      fbb,
      armcontrolstates.empty()?fbb.CreateVector<flatbuffers::Offset<grl::flatbuffer::ArmControlState>>(armcontrolstates):0);
}
/// KUKAiiwa.fbs
flatbuffers::Offset<grl::flatbuffer::CartesianImpedenceControlMode> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const grl::flatbuffer::EulerPose& stiffness,
    const grl::flatbuffer::EulerPose& damping,
    const double nullspaceStiffness,
    const double nullspaceDamping,
    const grl::flatbuffer::EulerPose &maxPathDeviation,
    const grl::flatbuffer::EulerPose &maxCartesianVelocity,
    const grl::flatbuffer::EulerPose &maxControlForce,
    const bool maxControlForceExceededStop)
{
    return grl::flatbuffer::CreateCartesianImpedenceControlMode(
        fbb,
        std::addressof(stiffness),
        std::addressof(damping),
        nullspaceStiffness,
        nullspaceDamping,
        std::addressof(maxPathDeviation),
        std::addressof(maxCartesianVelocity),
        std::addressof(maxControlForce),
        maxControlForceExceededStop);
}
/// KUKAiiwa.fbs
flatbuffers::Offset<grl::flatbuffer::JointImpedenceControlMode> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    std::vector<double> &joint_stiffness,
    std::vector<double> &joint_damping)
{
    auto jointStiffnessBuffer = fbb.CreateVector(joint_stiffness.data(),joint_stiffness.size());
    auto jointDampingBuffer = fbb.CreateVector(joint_damping.data(),joint_damping.size());
    return grl::flatbuffer::CreateJointImpedenceControlMode(fbb, jointStiffnessBuffer, jointDampingBuffer);
}
/// KUKAiiwa.fbs
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
    // auto _overlayType = toFlatBuffer(overlayType);
    return grl::flatbuffer::CreateFRI(
        fbb,
        toFlatBuffer(overlayType),
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
        fbb.CreateString(dataType),
        fbb.CreateString(defaultValue),
        fbb.CreateString(displayName),
        fbb.CreateString(id),
        fbb.CreateString(min),
        fbb.CreateString(max),
        fbb.CreateString(unit),
        fbb.CreateString(value),
        shouldRemove,
        shouldUpdate);
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
    const std::vector<flatbuffers::Offset<grl::flatbuffer::LinkObject>> &tools,
    const std::vector<flatbuffers::Offset<grl::flatbuffer::ProcessData>> &processData,
    const std::string &currentMotionCenter,
    bool requestMonitorProcessData = false)
{
    return  grl::flatbuffer::CreateKUKAiiwaArmConfiguration(
      fbb,
      fbb.CreateString(name),
      commandInterface,
      monitorInterface,
      toFlatBuffer(clientCommandMode),
      toFlatBuffer(overlayType),
      toFlatBuffer(controlMode),
      setCartImpedance,
      setJointImpedance,
      smartServoConfig,
      FRIConfig,
      tools.empty() ? fbb.CreateVector<flatbuffers::Offset<grl::flatbuffer::LinkObject>>(tools):0,
      processData.empty() ? fbb.CreateVector<flatbuffers::Offset<grl::flatbuffer::ProcessData>>(processData) : 0,
      currentMotionCenter.empty() ? fbb.CreateString(currentMotionCenter) : 0,
      requestMonitorProcessData);
}



/// KUKAiiwa.fbs
flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorState> toFlatBuffer(
    flatbuffers::FlatBufferBuilder fbb,
    flatbuffers::Offset<grl::flatbuffer::JointState> &measuredState,
    const grl::flatbuffer::Pose &cartesianFlangePose,
    flatbuffers::Offset<grl::flatbuffer::JointState> &jointStateReal,
    flatbuffers::Offset<grl::flatbuffer::JointState> &jointStateInterpolated,
    flatbuffers::Offset<grl::flatbuffer::JointState> &externalState,
    const ::FRISessionState &sessionState,
    const ::OperationMode &operationMode,
    const grl::flatbuffer::Wrench &CartesianWrench)
{
    return grl::flatbuffer::CreateKUKAiiwaMonitorState(
        fbb,
        measuredState,
        std::addressof(cartesianFlangePose),
        jointStateReal,
        jointStateInterpolated,
        externalState,

        toFlatBuffer(operationMode),
        toFlatBuffer(sessionState),
        std::addressof(CartesianWrench)
        );
}

/// KUKAiiwa.fbs
flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::string &name,
    const std::string &destination,
    const std::string &source,
    const double timestamp,
    const bool setArmControlState,
    flatbuffers::Offset<grl::flatbuffer::ArmControlState> &armControlState,
    const bool setArmConfiguration,
    flatbuffers::Offset<grl::flatbuffer::KUKAiiwaArmConfiguration> &armConfiguration,
    const bool hasMonitorState,  // MessageMonitorData monitorData in FRIMessage.pb.h
    flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorState> &monitorState,
    const bool hasMonitorConfig,
    flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorConfiguration> &monitorConfig)
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



/// KUKAiiwa.fbs
flatbuffers::Offset<grl::flatbuffer::FRIMessageLog> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const ::FRISessionState &sessionState,
    const ::FRIConnectionQuality &connectionQuality,
    const ::ControlMode &controlMode, // enum
    const ::FRIMonitoringMessage &friMonitoringMessage,
    const grl::TimeEvent &timeEvent)  // There are two times (TimeStamp and TimeEvent) here, which one should be kept? both?
{
    auto _sessionState = toFlatBuffer(sessionState);
    auto _connectionQuality = toFlatBuffer(connectionQuality);
    auto _controlMode = toFlatBuffer(controlMode);
    auto _messageIdentifier = friMonitoringMessage.header.messageIdentifier;
    auto _sequenceCounter = friMonitoringMessage.header.sequenceCounter;
    auto _reflectedSequenceCounter = friMonitoringMessage.header.reflectedSequenceCounter;
    // auto _sec = timeStamp.sec;
    // auto _nanosec = timeStamp.nanosec;
    std::vector<double> data;
    // get measured joint position
    grl::robot::arm::copy(friMonitoringMessage, std::back_inserter(data), grl::revolute_joint_angle_open_chain_state_tag());
    flatbuffers::Offset<flatbuffers::Vector<double>> _measuredJointPosition = fbb.CreateVector(data);
    data.clear();
    // get measured joint torque
    grl::robot::arm::copy(friMonitoringMessage, std::back_inserter(data), grl::revolute_joint_torque_open_chain_state_tag());
    flatbuffers::Offset<flatbuffers::Vector<double>> _measuredTorque = fbb.CreateVector(data);
    data.clear();
    // get measured joint torque
    grl::robot::arm::copy(friMonitoringMessage, std::back_inserter(data), grl::revolute_joint_torque_open_chain_command_tag());
    flatbuffers::Offset<flatbuffers::Vector<double>> _commandedJointPosition = fbb.CreateVector(data);
    data.clear();
    // get commanded joint torque
    grl::robot::arm::copy(friMonitoringMessage, std::back_inserter(data), grl::revolute_joint_torque_open_chain_command_tag());
    flatbuffers::Offset<flatbuffers::Vector<double>> _commandedTorque = fbb.CreateVector(data);
    data.clear();
    // get measured external torque
    grl::robot::arm::copy(friMonitoringMessage, std::back_inserter(data), grl::revolute_joint_torque_external_open_chain_state_tag());
    flatbuffers::Offset<flatbuffers::Vector<double>> _externalTorque = fbb.CreateVector(data);
    data.clear();
    // get interpolated joint state
    grl::robot::arm::copy(friMonitoringMessage, std::back_inserter(data), grl::revolute_joint_angle_interpolated_open_chain_state_tag());
    flatbuffers::Offset<flatbuffers::Vector<double>> _jointStateInterpolated = fbb.CreateVector(data);
    flatbuffers::Offset<grl::flatbuffer::TimeEvent> _timeEvent = grl::toFlatBuffer(fbb, timeEvent);
    auto _overlayType = toFlatBuffer(friMonitoringMessage.ipoData.overlayType);
     return grl::flatbuffer::CreateFRIMessageLog(
      fbb,
      _sessionState,
      _connectionQuality,
      _controlMode,
      _messageIdentifier,
      _sequenceCounter,
      _reflectedSequenceCounter,
      // _sec,
      // _nanosec,
      _measuredJointPosition,
      _measuredTorque,
      _commandedJointPosition,
      _commandedTorque,
      _externalTorque,
      _jointStateInterpolated,
      _timeEvent);
}
/// KUKAiiwa.fbs
flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::string &name,
    const std::string &destination,
    const std::string &source,
    const double timestamp,
    const bool setArmControlState,
    const flatbuffers::Offset<grl::flatbuffer::ArmControlState> &armControlState,
    const bool setArmConfiguration,
    const flatbuffers::Offset<grl::flatbuffer::KUKAiiwaArmConfiguration> &armConfiguration,
    const bool hasMonitorState,
    const flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorState> &monitorState,
    const bool hasMonitorConfig,
    const flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorConfiguration> &monitorConfig,
    const flatbuffers::Offset<grl::flatbuffer::FRIMessageLog> &FRIMessage)
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
        monitorConfig,
        FRIMessage);
}
/// KUKAiiwa.fbs
flatbuffers::Offset<grl::flatbuffer::KUKAiiwaStates> toFlatBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    const std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState>> &kukaiiwastates)
{
    return grl::flatbuffer::CreateKUKAiiwaStates(
      fbb,
      kukaiiwastates.empty()?fbb.CreateVector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState>>(kukaiiwastates):0);
}
// }  // End of arm namespace
// }  // End of robot namespace
}  // End of grl namespace


#endif // GRL_ATRACSYS_FUSION_TRACK_TO_FLATBUFFER
