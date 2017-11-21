/// KukaFRIDriver.hpp handles communication with the Kuka over FRI.
/// If you are new to this code base you are most likely looking for KukaDriver.hpp
#ifndef _KUKA_FRI_DRIVER
#define _KUKA_FRI_DRIVER

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/config.hpp>
#include <boost/exception/all.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <tuple>

#include <boost/math/special_functions/sign.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/transform.hpp>

//#ifdef BOOST_NO_CXX11_ATOMIC_SMART_PTR
#include <boost/thread.hpp>
//#endif

// friClientData is found in the kuka connectivity FRI cpp zip file
#include "grl/kuka/Kuka.hpp"
#include "grl/kuka/KukaFRI.hpp"
#include "grl/exception.hpp"
#include "grl/vector_ostream.hpp"
#include "grl/kuka/KukaFRIalgorithm.hpp"
// used for time synchronization
#include "grl/TimeEvent.hpp"


/// @todo TODO(ahundt) REMOVE SPDLOG FROM LOW LEVEL CODE
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

struct KukaState;

namespace grl {
namespace robot {
namespace arm {

/// @brief internal function to decode KUKA FRI message buffer (using nanopb
/// decoder) for the KUKA FRI
///
/// @note encode needs to be updated for each additional supported command type
/// and when updating to newer FRI versions
void decode(KUKA::FRI::ClientData &friData, std::size_t msg_size) {
  // The decoder was given a pointer to the monitoringMessage at initialization
  if (!friData.decoder.decode(friData.receiveBuffer, msg_size)) {
    BOOST_THROW_EXCEPTION(std::runtime_error("Error decoding received FRI data, the message may be missing or corrupted. This error is most likely due to the application running on the KUKA robot's side of the connection going down or disabling FRI, so check the robot and the JAVA side of the system."));
  }

  // check message type
  if (friData.expectedMonitorMsgID !=
      friData.monitoringMsg.header.messageIdentifier) {
    BOOST_THROW_EXCEPTION(std::invalid_argument(
        std::string("KukaFRI.hpp: Problem reading buffer, id code: ") +
        boost::lexical_cast<std::string>(
            static_cast<int>(friData.monitoringMsg.header.messageIdentifier)) +
        std::string(" does not match expected id code: ") +
        boost::lexical_cast<std::string>(
            static_cast<int>(friData.expectedMonitorMsgID)) +
        std::string("\n")));
    return;
  }

  friData.lastState =
      grl::robot::arm::get(friData.monitoringMsg, KUKA::FRI::ESessionState());
}

/// @brief Default LowLevelStepAlgorithmType
/// This algorithm is designed to be changed out
/// @todo Generalize this class using C++ techinques "tag dispatching" and "type
/// traits". See boost.geometry access and coorinate_type classes for examples.
/// Also perhaps make this the outer class which accepts drivers at the template param?
struct LinearInterpolation {

  enum ParamIndex {
    JointAngleDest,
    TimeDurationToDestMS
  };

  typedef std::tuple<boost::container::static_vector<double,7>,std::size_t> Params;

  // extremely conservative default timeframe to reach destination plus no goal position
  static const Params defaultParams() {
    boost::container::static_vector<double,7> nopos;
    return std::make_tuple(nopos,10000);
  }
  /// Default constructor
  /// @todo verify this doesn't corrupt the state of the system
  LinearInterpolation() {
  };

  // no action by default
  template <typename ArmDataType, typename CommandModeType>
  void lowLevelTimestep(ArmDataType &, CommandModeType &) {
    // need to tag dispatch here
    BOOST_VERIFY(false); // not yet supported
  }

  template <typename ArmData>
  void lowLevelTimestep(ArmData &friData,
                  revolute_joint_angle_open_chain_command_tag) {

    // no updates if no goal has been set
    if(goal_position.size() == 0) return;
    // switch (friData_->monitoringMsg.robotInfo.controlMode) {
    // case ControlMode_POSITION_CONTROLMODE:
    // case ControlMode_JOINT_IMPEDANCE_CONTROLMODE:

    KukaState::joint_state ipoJointPos;
    KukaState::joint_state currentJointPos;
    KukaState::joint_state currentMinusIPOJointPos;
    KukaState::joint_state goalPlusIPOJointPos;
    KukaState::joint_state diffToGoal;
    KukaState::joint_state amountToMove;
    KukaState::joint_state commandToSend;
    KukaState::joint_state commandToSendPlusIPOJointPos;

    double ripoJointPos[7];
    double rcurrentJointPos[7];
    double rcurrentMinusIPOJointPos[7];
    double rgoalPlusIPOJointPos[7];
    double rcommandedGoal[7];
    double rdiffToGoal[7];
    double ramountToMove[7];
    double rcommandToSend[7];
    double rvelocity_limits[7];
    double rcommandToSendPlusIPOJointPos[7];

    // the current "holdposition" joint angles
    /// @todo maybe this should be the
    /// revolute_joint_angle_interpolated_open_chain_state_tag()? @see
    /// kukaFRIalgorithm.hpp
    grl::robot::arm::copy(friData.monitoringMsg,
                          std::back_inserter(currentJointPos),
                          revolute_joint_angle_open_chain_state_tag());
    grl::robot::arm::copy(friData.monitoringMsg,
                          std::back_inserter(ipoJointPos),
                          revolute_joint_angle_interpolated_open_chain_state_tag());

    // copy value for debugging
    boost::copy(ipoJointPos, &ripoJointPos[0]);
    boost::copy(currentJointPos, &rcurrentJointPos[0]);
    boost::copy(goal_position, &rcommandedGoal[0]);

        boost::transform(currentJointPos, ipoJointPos,
                         std::back_inserter(currentMinusIPOJointPos), std::minus<double>());
        boost::transform(goal_position, ipoJointPos,
                         std::back_inserter(goalPlusIPOJointPos), std::plus<double>());


    boost::copy(currentMinusIPOJointPos, &rcurrentMinusIPOJointPos[0]);
    boost::copy(goal_position, &rcommandedGoal[0]);
    boost::copy(goalPlusIPOJointPos, &rgoalPlusIPOJointPos[0]);

    // only move if there is time left to reach the goal
    if(goal_position_command_time_duration_remaining > 0)
    {
        // single timestep in ms
        int thisTimeStepMS(grl::robot::arm::get(friData.monitoringMsg, grl::time_step_tag()));
        double thisTimeStepS = (static_cast<double>(thisTimeStepMS) / 1000);
        //double secondsPerTick = std::chrono::duration_cast<std::chrono::seconds>(thisTimeStep).count();

        // the fraction of the distance to the goal that should be traversed this
        // tick
        double fractionOfDistanceToTraverse =
            static_cast<double>(thisTimeStepMS) /
            static_cast<double>(goal_position_command_time_duration_remaining);

        // makes viewing in a debugger easier
        boost::copy(goal_position, &rcommandedGoal[0]);
        // get the angular distance to the goal
        // use current time and time to destination to interpolate (scale) goal
        // joint position
        boost::transform(goal_position, currentJointPos,
                         std::back_inserter(diffToGoal),
                         [&](double commanded_angle, double current_angle) {
                           return (commanded_angle - current_angle) *
                                  fractionOfDistanceToTraverse;
                         });
        boost::copy(diffToGoal, &rdiffToGoal[0]);

        goal_position_command_time_duration_remaining -= thisTimeStepMS;

        /// @todo correctly pass velocity limits from outside, use "copy" fuction in
        /// Kuka.hpp, correctly account for differing robot models. This  *should*
        /// be in KukaFRIdriver at the end of this file.

        // R820 velocity limits
        // A1 - 85 °/s  == 1.483529864195 rad/s
        // A2 - 85 °/s  == 1.483529864195 rad/s
        // A3 - 100 °/s == 1.745329251994 rad/s
        // A4 - 75 °/s  == 1.308996938996 rad/s
        // A5 - 130 °/s == 2.268928027593 rad/s
        // A6 - 135 °/s == 2.356194490192 rad/s
        // A1 - 135 °/s == 2.356194490192 rad/s
        KukaState::joint_state velocity_limits;
        velocity_limits.push_back(1.483529864195*thisTimeStepS);
        velocity_limits.push_back(1.483529864195*thisTimeStepS);
        velocity_limits.push_back(1.745329251994*thisTimeStepS);
        velocity_limits.push_back(1.308996938996*thisTimeStepS);
        velocity_limits.push_back(2.268928027593*thisTimeStepS);
        velocity_limits.push_back(2.356194490192*thisTimeStepS);
        velocity_limits.push_back(2.356194490192*thisTimeStepS);

        boost::copy(velocity_limits, &rvelocity_limits[0]);
        // clamp the commanded velocities to below the system limits
        // use std::min to ensure commanded change in position remains under the
        // maximum possible velocity for a single timestep
        boost::transform(
            diffToGoal, velocity_limits, std::back_inserter(amountToMove),
            [&](double diff, double maxvel) {
              return boost::math::copysign(std::min(std::abs(diff), maxvel), diff);
            });

        boost::copy(amountToMove, &ramountToMove[0]);


        // add the current joint position to the amount to move to get the actual
        // position command to send
        boost::transform(currentJointPos, amountToMove,
                         std::back_inserter(commandToSend), std::plus<double>());

        boost::copy(currentMinusIPOJointPos, &rcurrentMinusIPOJointPos[0]);
        boost::copy(commandToSend, &rcommandToSend[0]);


        boost::transform(commandToSend, ipoJointPos,
                         std::back_inserter(commandToSendPlusIPOJointPos), std::plus<double>());


        boost::copy(commandToSendPlusIPOJointPos, &rcommandToSendPlusIPOJointPos[0]);
        // send the command
//        grl::robot::arm::set(friData.commandMsg, commandToSend,
//                             grl::revolute_joint_angle_open_chain_command_tag());
        // send the command
        grl::robot::arm::set(friData.commandMsg, commandToSend,
                             grl::revolute_joint_angle_open_chain_command_tag());
    }
    // break;
  }

  void setGoal(const Params& params ) {
      /// @todo TODO(ahundt) support param tag structs for additional control modes
      goal_position_command_time_duration_remaining = std::get<TimeDurationToDestMS>(params);
      goal_position = std::get<JointAngleDest>(params);

  }

  /// @todo look in FRI_Client_SDK_Cpp.zip to see if position must be set for
  /// joint torques. Ref files: LBRTorqueSineOverlayClient.cpp,
  /// LBRTorqueSineOverlayClient.h, friLBRCommand.cpp, friLBRCommand.h
  template <typename ArmData>
  void lowLevelTimestep(ArmData &friData,
                  revolute_joint_torque_open_chain_command_tag) {

    //not yet supported
    BOOST_VERIFY(false);
    // case ControlMode_JOINT_IMPEDANCE_CONTROLMODE:
//    grl::robot::arm::set(friData.commandMsg, armState.commandedTorque,
//                         grl::revolute_joint_torque_open_chain_command_tag());

    /// @note encode() needs to be updated for each additional supported command
    /// type
    // break;
  }

  /// @todo look in FRI_Client_SDK_Cpp.zip to see if position must be set for
  /// cartesian wrench. Ref files: LBRWrenchSineOverlayClient.cpp,
  /// LBRWrenchSineOverlayClient.h, friLBRCommand.cpp, friLBRCommand.h
  template <typename ArmData>
  void lowLevelTimestep(ArmData &friData, cartesian_wrench_command_tag) {

    //not yet supported
    BOOST_VERIFY(false);
    // case ControlMode_CARTESIAN_IMPEDANCE_CONTROLMODE:
    // not yet supported
//    grl::robot::arm::set(friData.commandMsg,
//                         armState.commandedCartesianWrenchFeedForward,
//                         grl::cartesian_wrench_command_tag());
    // break;
  }

  /// @todo make this accessible via a nonmember function
  bool hasCommandData() {
    /// @todo check if duration remaining should be greater than zero or greater
    /// than the last tick size
    return goal_position_command_time_duration_remaining > 0;
  }
  //    template<typename ArmData>
  //    void operator()(ArmData& clientData,
  //    revolute_joint_angle_open_chain_command_tag){
  //              default:
  //                break;
  //            }
private:
  // the armstate at initialization of this object
  KukaState::joint_state velocity_limits;
  KukaState::joint_state goal_position;
  double goal_position_command_time_duration_remaining; // milliseconds
};

/// @brief encode data in the class KUKA::FRI::ClientData into the send buffer
/// for the KUKA FRI.
/// this preps the information for transport over the network
///
/// @note encode needs to be updated for each additional supported command type
/// and when updating to newer FRI versions
template <typename LowLevelStepAlgorithmType = LinearInterpolation>
std::size_t encode(LowLevelStepAlgorithmType &step_alg,
                   KUKA::FRI::ClientData &friData,
                   boost::system::error_code &ec) {
  // reset send counter
  friData.lastSendCounter = 0;

  // set sequence counters
  friData.commandMsg.header.sequenceCounter = friData.sequenceCounter++;
  friData.commandMsg.header.reflectedSequenceCounter =
      friData.monitoringMsg.header.sequenceCounter;

  KUKA::FRI::ESessionState sessionState =
      grl::robot::arm::get(friData.monitoringMsg, KUKA::FRI::ESessionState());

  if ((step_alg.hasCommandData() &&
       (sessionState == KUKA::FRI::COMMANDING_WAIT ||
        sessionState == KUKA::FRI::COMMANDING_ACTIVE))) {
    KUKA::FRI::EClientCommandMode commandMode = grl::robot::arm::get(
        friData.monitoringMsg, KUKA::FRI::EClientCommandMode());
    switch (commandMode) {
    case ClientCommandMode_POSITION:
      step_alg.lowLevelTimestep(friData, revolute_joint_angle_open_chain_command_tag());
      break;
    case ClientCommandMode_WRENCH:
      step_alg.lowLevelTimestep(friData, cartesian_wrench_command_tag());
      break;
    case ClientCommandMode_TORQUE:
      step_alg.lowLevelTimestep(friData, revolute_joint_torque_open_chain_command_tag());
      break;
    default:
      // this is unhandled at the moment...
      BOOST_VERIFY(false);
      // BOOST_THROW_EXCEPTION_CURRENT_FUNCTION;
      // ClientCommandMode_NO_COMMAND_MODE, anything else that is added in the
      // future and unimplemented
      /// @todo do nothing if in an unsupported command mode? Or do the same as
      /// the next else if step?
      break;
    }

  } else if (!(friData.commandMsg.has_commandData &&
               step_alg.hasCommandData() &&
               (sessionState == KUKA::FRI::COMMANDING_WAIT ||
                sessionState == KUKA::FRI::COMMANDING_ACTIVE))) {
    // copy current measured joint position to commanded position only if we
    // *don't* have new command data

    /// @todo should this be different if it is in torque mode?
    /// @todo allow copying of data directly between commandmsg and
    /// monitoringMsg
    std::vector<double> msg;
    copy(friData.monitoringMsg, std::back_inserter(msg),
         revolute_joint_angle_open_chain_command_tag());
    // copy the previously recorded command over
    set(friData.commandMsg, msg,
        grl::revolute_joint_angle_open_chain_command_tag());
  }

  int buffersize = KUKA::FRI::FRI_COMMAND_MSG_MAX_SIZE;
  if (!friData.encoder.encode(friData.sendBuffer, buffersize)) {
    // @todo figure out PB_GET_ERROR, integrate with error_code type supported
    // by boost
    ec = boost::system::errc::make_error_code(boost::system::errc::bad_message);
    return 0;
  }

  return buffersize;
}

/// @brief Actually talk over the network to receive an update and send out a
/// new KUKA FRI command
///
/// Receives an update, performs the necessary checks, then sends a message if
/// appropriate.
///
/// @pre socket must already have the endpoint resolved and "connected". While
/// udp is technically stateless the asio socket supports the connection api
/// components for convenience.
template <typename LowLevelStepAlgorithmType = LinearInterpolation>
void update_state(boost::asio::ip::udp::socket &socket,
                  LowLevelStepAlgorithmType &step_alg,
                  KUKA::FRI::ClientData &friData,
                  boost::system::error_code &receive_ec,
                  std::size_t &receive_bytes_transferred,
                  boost::system::error_code &send_ec,
                  std::size_t &send_bytes_transferred,
                  grl::TimeEvent& timeEvent,
                  boost::asio::ip::udp::endpoint sender_endpoint = boost::asio::ip::udp::endpoint()) {

  static const int message_flags = 0;
        // get a local clock timestamp, then the latest frame from the device, then another timestamp
  timeEvent.local_request_time = cartographer::common::UniversalTimeScaleClock::now();
  receive_bytes_transferred = socket.receive_from(
      boost::asio::buffer(friData.receiveBuffer,
                          KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE),
      sender_endpoint, message_flags, receive_ec);
  timeEvent.local_receive_time = cartographer::common::UniversalTimeScaleClock::now();
  decode(friData, receive_bytes_transferred);

  friData.lastSendCounter++;
  // Check whether to send a response
  if (friData.lastSendCounter >=
      friData.monitoringMsg.connectionInfo.receiveMultiplier) {
    send_bytes_transferred = encode(step_alg, friData, send_ec);
    if (send_ec)
      return;
    socket.send(boost::asio::buffer(friData.sendBuffer, send_bytes_transferred),
                message_flags, send_ec);
  }
}


/// @brief don't use this
/// @deprecated this is an old implemenation that will be removed in the future
void copy(const FRIMonitoringMessage &monitoringMsg, KukaState &state) {
  state.clear();
  copy(monitoringMsg, std::back_inserter(state.position),
       revolute_joint_angle_open_chain_state_tag());
  copy(monitoringMsg, std::back_inserter(state.torque),
       revolute_joint_torque_open_chain_state_tag());
  copy(monitoringMsg, std::back_inserter(state.commandedPosition),
       revolute_joint_angle_open_chain_command_tag());
  copy(monitoringMsg, std::back_inserter(state.commandedTorque),
       revolute_joint_torque_open_chain_command_tag());
  copy(monitoringMsg, std::back_inserter(state.ipoJointPosition),
       revolute_joint_angle_interpolated_open_chain_state_tag());
  state.sessionState = static_cast<flatbuffer::ESessionState>(
      get(monitoringMsg, KUKA::FRI::ESessionState()));
  state.connectionQuality = static_cast<flatbuffer::EConnectionQuality>(
      get(monitoringMsg, KUKA::FRI::EConnectionQuality()));
  state.safetyState = static_cast<flatbuffer::ESafetyState>(
      get(monitoringMsg, KUKA::FRI::ESafetyState()));
  state.operationMode = static_cast<flatbuffer::EOperationMode>(
      get(monitoringMsg, KUKA::FRI::EOperationMode()));
  state.driveState = static_cast<flatbuffer::EDriveState>(
      get(monitoringMsg, KUKA::FRI::EDriveState()));

  /// @todo fill out missing state update steps
}

/// @brief Simple low level driver to communicate over the Kuka iiwa FRI
/// interface using KUKA::FRI::ClientData status objects
///
/// @note If you aren't sure, see KukaDriver in KukaDriver.hpp.
///
/// @note If you want to change how the lowest level high rate updates are
/// performed see KukaFRIdriver
///
/// One important aspect of this design is the is_running_automatically flag. If
/// you are unsure,
/// the suggested default is run_automatically (true/enabled). When it is
/// enabled,
/// the driver will create a thread internally and run the event loop
/// (io_service) itself.
/// If run manually, you are expected to call io_service.run() on the io_service
/// you provide,
/// or on the run() member function. When running manually you are also expected
/// to call
/// async_getLatestState(handler) frequently enought that the 5ms response
/// requirement of the KUKA
/// FRI interface is met.
template <typename LowLevelStepAlgorithmType = LinearInterpolation>
class KukaFRIClientDataDriver
    : public std::enable_shared_from_this<
          KukaFRIClientDataDriver<LowLevelStepAlgorithmType>>,
      public KukaUDP {

public:
  using KukaUDP::ParamIndex;
  using KukaUDP::ThreadingRunMode;
  using KukaUDP::Params;
  using KukaUDP::defaultParams;

  KukaFRIClientDataDriver(boost::asio::io_service &ios,
                          Params params = defaultParams())
      : params_(params), m_shouldStop(false), isConnectionEstablished_(false),
        io_service_(ios)
  //,socketP_(std::move(connect(params, io_service_,sender_endpoint_))) ///<
  //@todo this breaks the assumption that the object can be constructed without
  // hardware issues being a porblem
  {
    construct(params);
  }

  KukaFRIClientDataDriver(Params params = defaultParams())
      : params_(params), m_shouldStop(false), isConnectionEstablished_(false),
        optional_internal_io_service_P(new boost::asio::io_service),
        io_service_(*optional_internal_io_service_P)
  //,socketP_(std::move(connect(params, io_service_,sender_endpoint_))) ///<
  //@todo this breaks the assumption that the object can be constructed without
  // hardware issues being a porblem
  {
    construct(params);
  }

  /// Call this to initialize the object after the constructor has been called
  void construct(Params params = defaultParams()) {
    try {

      ///////////
      // initialize all of the states
      latestStateForUser_ = make_valid_LatestState();
      spareStates_.emplace_back(std::move(make_valid_LatestState()));
      spareStates_.emplace_back(std::move(make_valid_LatestState()));

      // start up the driver thread since the io_service_ is internal only
      if (std::get<is_running_automatically>(params)) {
        driver_threadP_.reset(new std::thread([&] { update(); }));
      }

    } catch (boost::exception &e) {
      add_details_to_connection_error(e, params);
      throw;
    }
  }

  /// @brief blocking call to communicate with the robot continuously
  /// @pre construct() should be called before run()
  void run() { update(); }

  /// @brief Updates the passed friData shared pointer to a pointer to newly
  /// updated data, plus any errors that occurred.
  ///
  /// We recommend you supply a valid shared_ptr to friData, even if all command
  /// elements are set to false.
  /// The friData pointer you pass in can contain a command to send to the arm.
  /// To update with new state and optional input state, you give up lifetime
  /// control of the input,
  /// and assume liftime control of the output.
  ///
  /// This function is designed for single threaded use to quickly receive and
  /// send "non-blocking" updates to the robot.
  /// It is not thread safe cannot be called simultaneously from multiple
  /// threads.
  ///
  ///
  /// @note { An error code is set if update_state is called with no new data
  /// available.
  ///         In this special case, all error codes and bytes_transferred are 0,
  ///         because
  ///         there was no new data available for the user.
  ///       }
  ///
  /// @warning Do not pound this call continuously in a very tight loop, because
  /// then the driver won't be able to acquire the lock and send updates to the
  /// robot.
  ///
  /// @param[in,out] friData supply a new command, receive a new update of the
  /// robot state. Pointer is null if no new data is available.
  ///
  /// @pre If friData!=nullptr it is assumed valid for use and this class will
  /// take control of the object.
  ///
  /// @return isError = false if you have new data, true when there is either an
  /// error or no new data
  bool update_state(std::shared_ptr<typename LowLevelStepAlgorithmType::Params> &step_alg_params,
                    std::shared_ptr<KUKA::FRI::ClientData> &friData,
                    boost::system::error_code &receive_ec,
                    std::size_t &receive_bytes_transferred,
                    boost::system::error_code &send_ec,
                    std::size_t &send_bytes_transferred,
                    grl::TimeEvent& timeEvent) {

    if (exceptionPtr) {
      /// @note this exception most likely came from the update() call running
      /// the kuka driver
      std::rethrow_exception(exceptionPtr);
    }

    bool haveNewData = false;

    if (!isConnectionEstablished_ ||
        !std::get<latest_receive_monitor_state>(latestStateForUser_)) {
      // no new data, so immediately return results accordingly
      std::tie(step_alg_params, friData, receive_ec, receive_bytes_transferred, send_ec,
               send_bytes_transferred) = make_LatestState(step_alg_params,friData);
      return !haveNewData;
    }

    // ensure we have valid data for future updates
    // need to copy this over because friData will be set as an output value
    // later
    // and allocate/initialize data if null
    auto validFriDataLatestState = make_valid_LatestState(step_alg_params,friData);

    // get the latest state from the driver thread
    {
      boost::lock_guard<boost::mutex> lock(ptrMutex_);

      // get the update if one is available
      // the user has provided new data to send to the device
      if (std::get<latest_receive_monitor_state>(validFriDataLatestState)
              ->commandMsg.has_commandData) {
        std::swap(validFriDataLatestState, newCommandForDriver_);
      }
      // newCommandForDriver_ is finalized

      if (spareStates_.size() < spareStates_.capacity() &&
          std::get<latest_receive_monitor_state>(validFriDataLatestState)) {
        spareStates_.emplace_back(std::move(validFriDataLatestState));
      }

      if (std::get<latest_receive_monitor_state>(latestStateForUser_)) {
        // return the latest state to the caller
        std::tie(step_alg_params,friData, receive_ec, receive_bytes_transferred, send_ec,
                 send_bytes_transferred) = std::move(latestStateForUser_);
        haveNewData = true;
      } else if (std::get<latest_receive_monitor_state>(
                     validFriDataLatestState)) {
        // all storage is full, return the spare data to the user
        std::tie(step_alg_params, friData, receive_ec, receive_bytes_transferred, send_ec,
                 send_bytes_transferred, timeEvent) = validFriDataLatestState;
      }
    }

    // let the user know if we aren't in the best possible state
    return !haveNewData || receive_bytes_transferred == 0 || receive_ec ||
           send_ec;
  }

  void destruct() {
    m_shouldStop = true;
    if (driver_threadP_) {
      driver_threadP_->join();
    }
  }

  ~KukaFRIClientDataDriver() { destruct(); }

  /// Is everything ok?
  /// @return true if the kuka fri connection is actively running without any
  /// issues
  /// @todo consider expanding to support real error codes
  bool is_active() { return !exceptionPtr && isConnectionEstablished_; }

private:
  /// Reads data off of the real kuka fri device in a separate thread
  /// @todo consider switching to single producer single consumer queue to avoid
  /// locking overhead, but keep latency in mind
  /// https://github.com/facebook/folly/blob/master/folly/docs/ProducerConsumerQueue.md
  void update() {
    try {

      LowLevelStepAlgorithmType step_alg;
      /// nextState is the object currently being loaded with data off the
      /// network
      /// the driver thread should access this exclusively in update()
      LatestState nextState = make_valid_LatestState();
      LatestState latestCommandBackup = make_valid_LatestState();

      boost::asio::ip::udp::endpoint sender_endpoint;
      boost::asio::ip::udp::socket socket(
          connect(params_, io_service_, sender_endpoint));
      KukaState kukastate; ///< @todo TODO(ahundt) remove this line when new
                           /// api works completely since old one is deprecated

      /////////////
      // run the primary update loop in a separate thread
      while (!m_shouldStop) {
        /// @todo maybe there is a more convienient way to set this that is
        /// easier for users? perhaps initializeClientDataForiiwa()?

        // nextState and latestCommandBackup should never be null
        BOOST_VERIFY(std::get<latest_receive_monitor_state>(nextState));
        BOOST_VERIFY(
            std::get<latest_receive_monitor_state>(latestCommandBackup));

        // set the flag that must always be there
        std::get<latest_receive_monitor_state>(nextState)
            ->expectedMonitorMsgID = KUKA::LBRState::LBRMONITORMESSAGEID;

        auto lowLevelAlgorithmParamP = std::get<latest_low_level_algorithm_params>(nextState);

        // if there is a valid low level algorithm param command set the new goal
        if(lowLevelAlgorithmParamP) step_alg.setGoal(*lowLevelAlgorithmParamP);

        // actually talk over the network to receive an update and send out a
        // new command
        grl::robot::arm::update_state(
            socket, step_alg,
            *std::get<latest_receive_monitor_state>(nextState),
            std::get<latest_receive_ec>(nextState),
            std::get<latest_receive_bytes_transferred>(nextState),
            std::get<latest_send_ec>(nextState),
            std::get<latest_send_bytes_transferred>(nextState),
            std::get<latest_time_event_data>(nextState));

        /// @todo use atomics to eliminate the global mutex lock for this object
        // lock the mutex to communicate with the user thread
        // if it cannot lock, simply send the previous message
        // again
        if (ptrMutex_.try_lock()) {

          //////////////////////////////////////////////
          // right now this is the state of everything:
          //////////////////////////////////////////////
          //
          // Always Valid:
          //
          //     nextState: valid, contains the latest update
          //     latestCommandBackup: should always be valid (though hasCommand
          //     might be false)
          //
          // Either Valid or Null:
          //    latestStateForUser_ : null if the user took data out, valid
          //    otherwise
          //    newCommandForDriver_: null if there is no new command data from
          //    the user, vaild otherwise

          // 1) set the outgoing latest state for the user to pick up
          //    latestStateForUser_ is finalized
          std::swap(latestStateForUser_, nextState);

          // 2) get a new incoming command if available and set incoming command
          // variable to null
          if (std::get<latest_receive_monitor_state>(newCommandForDriver_)) {
            // 3) back up the new incoming command
            // latestCommandBackup is finalized, newCommandForDriver_ needs to
            // be cleared out
            std::swap(latestCommandBackup, newCommandForDriver_);

            // nextState may be null
            if (!std::get<latest_receive_monitor_state>(nextState)) {
              nextState = std::move(newCommandForDriver_);
            } else if (!(spareStates_.size() == spareStates_.capacity())) {
              spareStates_.emplace_back(std::move(newCommandForDriver_));
            } else {
              std::get<latest_receive_monitor_state>(newCommandForDriver_)
                  .reset();
            }
          }

          // finalized: latestStateForUser_, latestCommandBackup,
          // newCommandForDriver_ is definitely null
          // issues to be resolved:
          // nextState: may be null right now, and it should be valid
          // newCommandForDriver_: needs to be cleared with 100% certainty
          BOOST_VERIFY(spareStates_.size() > 0);

          if (!std::get<latest_receive_monitor_state>(nextState) &&
              spareStates_.size()) {
            // move the last element out and shorten the vector
            nextState = std::move(*(spareStates_.end() - 1));
            spareStates_.pop_back();
          }

          BOOST_VERIFY(std::get<latest_receive_monitor_state>(nextState));

          KUKA::FRI::ClientData &nextClientData =
              *std::get<latest_receive_monitor_state>(nextState);
          KUKA::FRI::ClientData &latestClientData =
              *std::get<latest_receive_monitor_state>(latestStateForUser_);

          // copy essential data from latestStateForUser_ to nextState
          nextClientData.lastState = latestClientData.lastState;
          nextClientData.sequenceCounter = latestClientData.sequenceCounter;
          nextClientData.lastSendCounter = latestClientData.lastSendCounter;
          nextClientData.expectedMonitorMsgID =
              latestClientData.expectedMonitorMsgID;

          // copy command from latestCommandBackup to nextState aka
          // nextClientData
          KUKA::FRI::ClientData &latestCommandBackupClientData =
              *std::get<latest_receive_monitor_state>(latestCommandBackup);
          set(nextClientData.commandMsg,
              latestCommandBackupClientData.commandMsg);

          // if there are no error codes and we have received data,
          // then we can consider the connection established!
          /// @todo perhaps data should always send too?
          if (!std::get<latest_receive_ec>(nextState) &&
              !std::get<latest_send_ec>(nextState) &&
              std::get<latest_receive_bytes_transferred>(nextState)) {
            isConnectionEstablished_ = true;
          }

          ptrMutex_.unlock();
        }
      }

    } catch (...) {
      // transport the exception to the main thread in a safe manner
      exceptionPtr = std::current_exception();
      m_shouldStop = true;
      isConnectionEstablished_ = false;
    }

    isConnectionEstablished_ = false;
  }

  enum LatestStateIndex {
    latest_low_level_algorithm_params,
    latest_receive_monitor_state,
    latest_receive_ec,
    latest_receive_bytes_transferred,
    latest_send_ec,
    latest_send_bytes_transferred,
    latest_time_event_data
  };

  /// this is the object that stores all data for the latest device state
  /// including the KUKA defined ClientData object, and a grl defined TimeEvent
  /// which stores the time data needed for synchronization.
  typedef std::tuple<std::shared_ptr<typename LowLevelStepAlgorithmType::Params>,
                     std::shared_ptr<KUKA::FRI::ClientData>,
                     boost::system::error_code, std::size_t,
                     boost::system::error_code, std::size_t,
                     grl::TimeEvent>
      LatestState;

  /// Creates a default LatestState Object
  static LatestState
  make_LatestState(std::shared_ptr<typename LowLevelStepAlgorithmType::Params> lowLevelAlgorithmParams,
                   std::shared_ptr<KUKA::FRI::ClientData> &clientData) {
    return std::make_tuple(lowLevelAlgorithmParams, clientData, boost::system::error_code(),
                           std::size_t(), boost::system::error_code(),
                           std::size_t(), grl::TimEvent());
  }

  /// creates a shared_ptr to KUKA::FRI::ClientData with all command message
  /// status explicitly set to false
  /// @post std::shared_ptr<KUKA::FRI::ClientData> will be non-null
  static std::shared_ptr<KUKA::FRI::ClientData> make_shared_valid_ClientData(
    std::shared_ptr<KUKA::FRI::ClientData> &friData) {
    if (friData.get() == nullptr) {
      friData = std::make_shared<KUKA::FRI::ClientData>(KUKA::LBRState::NUM_DOF);
      // there is no commandMessage data on a new object
      friData->resetCommandMessage();
    }

    return friData;
  }

  static std::shared_ptr<KUKA::FRI::ClientData> make_shared_valid_ClientData() {
    std::shared_ptr<KUKA::FRI::ClientData> friData;
    return make_shared_valid_ClientData(friData);
  }

  /// Initialize valid shared ptr to LatestState object with a valid allocated
  /// friData. Note that lowLevelAlgorithmParams will remain null!
  static LatestState
  make_valid_LatestState(
    std::shared_ptr<typename LowLevelStepAlgorithmType::Params> lowLevelAlgorithmParams,
    std::shared_ptr<KUKA::FRI::ClientData> &friData
  ) {
    if (!friData) {
      friData = make_shared_valid_ClientData();
  }

    return make_LatestState(lowLevelAlgorithmParams,friData);
  }

  static LatestState make_valid_LatestState() {
    std::shared_ptr<typename LowLevelStepAlgorithmType::Params> emptyLowLevelAlgParams;
    std::shared_ptr<KUKA::FRI::ClientData> friData;
    return make_valid_LatestState(emptyLowLevelAlgParams,friData);
  }

  Params params_;

  /// @todo replace with unique_ptr
  /// the latest state we have available to give to the user
  LatestState latestStateForUser_;
  LatestState newCommandForDriver_;

  /// should always be valid, never null
  boost::container::static_vector<LatestState, 2> spareStates_;

  std::atomic<bool> m_shouldStop;
  std::exception_ptr exceptionPtr;
  std::atomic<bool> isConnectionEstablished_;

  /// may be null, allows the user to choose if they want to provide an
  /// io_service
  std::unique_ptr<boost::asio::io_service> optional_internal_io_service_P;

  // other things to do somewhere:
  // - get list of control points
  // - get the control point in the arm base coordinate system
  // - load up a configuration file with ip address to send to, etc.
  boost::asio::io_service &io_service_;
  std::unique_ptr<std::thread> driver_threadP_;
  boost::mutex ptrMutex_;

  typename LowLevelStepAlgorithmType::Params step_alg_params_;
};

/// @brief Primary Kuka FRI driver, only talks over realtime network FRI KONI
/// ethernet port
///
///
/// @note If you aren't sure, see KukaDriver in KukaDriver.hpp.
///
/// @note If you want to change how the lowest level high rate updates are
/// performed, make another version of this class or update so
/// LowLevelStepAlgorithmType, like LinearInterpolation,
/// is configurable. @see KukaFRIdriver
///
///
/// KukaFRIdriver is a low level driver at a slightly "higher level" than the
/// the "lowest level" KukaFRIClientDataDriver
/// to communicate. This is the class you will want to replace if you want to
/// change how low level position
/// updates are changed between FRI update steps, which occur at a configurable
/// duration of 1-5 ms.
///
/// For position based motion to work, you must set both the position you want
/// and the time you want it to get there.
/// This is required because the robot can move extremely fast, so accidentally
/// setting the velocity to the max is
/// very dangerous. If the time point is in the past, the robot will not move.
/// If the time point is too near in the future
/// to make it, the robot will move at the max speed towards that position.
///
///
/// While velocity limits are not provided explicitly by KUKA in their low level
/// C++ API,
/// if you send a command that violates the velocity limits specified in KUKA's
/// documenation
/// the arm stops immediately with an error, even over FRI.
///
/// @todo support generic read/write
/// @todo ensure commands stay within machine limits to avoid lockup
/// @todo reset and resume if lockup occurs whenever possible
/// @todo in classes that use this driver, make the use of this class templated
/// so that the low level update algorithm can change.
/// @todo add getter for number of milliseconds between fri updates (1-5) aka
/// sync_period aka send_period aka ms per tick
template <typename LowLevelStepAlgorithmType = LinearInterpolation>
class KukaFRIdriver : public std::enable_shared_from_this<
                          KukaFRIdriver<LowLevelStepAlgorithmType>>,
                      public KukaUDP {

public:
  using KukaUDP::ParamIndex;
  using KukaUDP::ThreadingRunMode;
  using KukaUDP::Params;
  using KukaUDP::defaultParams;

  KukaFRIdriver(Params params = defaultParams()) : params_(params) {}

  //      KukaFRIdriver(boost::asio::io_service&
  //      device_driver_io_service__,Params params = defaultParams())
  //        :
  //        device_driver_io_service(device_driver_io_service__),
  //        params_(params)
  //      {}

  void construct() { construct(params_); }

  /// @todo create a function that calls simGetObjectHandle and throws an
  /// exception when it fails
  /// @warning getting the ik group is optional, so it does not throw an
  /// exception
  void construct(Params params) {

    params_ = params;
    // keep driver threads from exiting immediately after creation, because they
    // have work to do!
    device_driver_workP_.reset(
        new boost::asio::io_service::work(device_driver_io_service));

    kukaFRIClientDataDriverP_.reset(
        new grl::robot::arm::KukaFRIClientDataDriver<LowLevelStepAlgorithmType>(
            device_driver_io_service,
            std::make_tuple(std::string(std::get<RobotModel>(params)),
                            std::string(std::get<localhost>(params)),
                            std::string(std::get<localport>(params)),
                            std::string(std::get<remotehost>(params)),
                            std::string(std::get<remoteport>(params)),
                            grl::robot::arm::KukaFRIClientDataDriver<
                                LowLevelStepAlgorithmType>::run_automatically))

            );
  }

  const Params &getParams() { return params_; }

  ~KukaFRIdriver() {
    device_driver_workP_.reset();

    if (driver_threadP) {
      device_driver_io_service.stop();
      driver_threadP->join();
    }
  }

  /// gets the number of seconds in one message exchange "tick" aka "cycle",
  /// "time step" of the robot arm's low level controller
  double getSecondsPerTick() {
    return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::milliseconds(grl::robot::arm::get(
                   friData_->monitoringMsg, grl::time_step_tag())))
        .count();
  }

  /// @todo make this configurable for different specific robots. Currently set
  /// for kuka lbr iiwa 14kg R820
  KukaState::joint_state getMaxVel() {
    KukaState::joint_state maxVel;
    /// get max velocity constraint parameter for this robot model
    copy(std::get<RobotModel>(params_), std::back_inserter(maxVel),
         grl::revolute_joint_velocity_open_chain_state_constraint_tag());

    // scale velocity down to a single timestep. In other words multiply each
    // velocity by the number of seconds in a tick, likely 0.001-0.005
    boost::transform(
        maxVel, maxVel.begin(),
        std::bind2nd(std::multiplies<KukaState::joint_state::value_type>(),
                     getSecondsPerTick()));

    return maxVel;
  }

  /**
   * spin once, this is what you call each time you synchronize the client with
   * the robot over UDP
   * it is expected to be called at least once per send_period_millisec, which
   * is the time between
   * each FRI udp packet.
   *
   */
  bool run_one() {
    grl::TimeEvent time_event_stamp;
    // note: this one sends *and* receives the joint data!
    BOOST_VERIFY(kukaFRIClientDataDriverP_.get() != nullptr);
    /// @todo use runtime calculation of NUM_JOINTS instead of constant
    if (!friData_) {
      friData_ = std::make_shared<KUKA::FRI::ClientData>(KUKA::LBRState::NUM_DOF);
    }

    bool haveNewData = false;

    static const std::size_t minimumConsecutiveSuccessesBeforeSendingCommands = 100;

    std::shared_ptr<typename LowLevelStepAlgorithmType::Params> lowLevelStepAlgorithmCommandParamsP;

    /// @todo probably only need to set this once
    armState.velocity_limits.clear();
    armState.velocity_limits = getMaxVel();

    // This is the key point where the arm's motion goal command is updated and
    // sent to the robot
    // Set the FRI to the simulated joint positions
    if (this->m_haveReceivedRealDataCount >
        minimumConsecutiveSuccessesBeforeSendingCommands) {
      /// @todo TODO(ahundt) Need to eliminate this allocation
      boost::lock_guard<boost::mutex> lock(jt_mutex);

      boost::container::static_vector<double, 7> jointStateToCommand;
      boost::copy(armState.commandedPosition,std::back_inserter(jointStateToCommand));
      // pass time to reach specified goal for position control
      lowLevelStepAlgorithmCommandParamsP = std::make_shared<grl::robot::arm::LinearInterpolation::Params>(std::make_tuple(jointStateToCommand,armState.goal_position_command_time_duration));
      /// @todo construct new low level command object and pass to
      /// KukaFRIClientDataDriver
      /// this is where we used to setup a new FRI command

      // std::cout << "commandToSend: " << commandToSend << "\n" <<
      // "currentJointPos: " << currentJointPos << "\n" << "amountToMove: " <<
      // amountToMove << "\n" << "maxVel: " << maxvel << "\n";
    } else {
      /// @todo TODO(ahundt) BUG: Need way to pass time to reach specified goal for position control and eliminate this allocation
      lowLevelStepAlgorithmCommandParamsP.reset(new typename LowLevelStepAlgorithmType::Params());
    }

    BOOST_VERIFY(lowLevelStepAlgorithmCommandParamsP != nullptr);
    boost::system::error_code send_ec, recv_ec;
    std::size_t send_bytes, recv_bytes;
    // sync with device over network
    haveNewData = !kukaFRIClientDataDriverP_->update_state(
        lowLevelStepAlgorithmCommandParamsP, friData_, recv_ec, recv_bytes, send_ec,
        send_bytes, time_event_stamp);
    m_attemptedCommunicationCount++;

    if (haveNewData) {
      boost::lock_guard<boost::mutex> lock(jt_mutex);
      // if there were problems sending commands, start by sending the current
      // position
      //            if(this->m_haveReceivedRealDataCount >
      //            minimumConsecutiveSuccessesBeforeSendingCommands-1)
      //            {
      //              boost::lock_guard<boost::mutex> lock(jt_mutex);
      //              // initialize arm commands to current arm position
      //              armState.clearCommands();
      ////              armState.commandedPosition.clear();
      ////              armState.commandedTorque.clear();
      ////              grl::robot::arm::copy(friData_->monitoringMsg,
      /// std::back_inserter(armState.commandedPosition),
      /// grl::revolute_joint_angle_open_chain_command_tag());
      ////              grl::robot::arm::copy(friData_->monitoringMsg,
      /// std::back_inserter(armState.commandedTorque)   ,
      /// grl::revolute_joint_torque_open_chain_command_tag());
      //            }

      m_attemptedCommunicationConsecutiveSuccessCount++;
      this->m_attemptedCommunicationConsecutiveFailureCount = 0;
      this->m_haveReceivedRealDataCount++;

      // We have the real kuka state read from the device now
      // update real joint angle data
      armState.position.clear();
      grl::robot::arm::copy(friData_->monitoringMsg,
                            std::back_inserter(armState.position),
                            grl::revolute_joint_angle_open_chain_state_tag());

      armState.torque.clear();
      grl::robot::arm::copy(friData_->monitoringMsg,
                            std::back_inserter(armState.torque),
                            grl::revolute_joint_torque_open_chain_state_tag());

      armState.externalTorque.clear();
      grl::robot::arm::copy(
          friData_->monitoringMsg, std::back_inserter(armState.externalTorque),
          grl::revolute_joint_torque_external_open_chain_state_tag());

// only supported for kuka sunrise OS 1.9
#ifdef KUKA_SUNRISE_1_9
      armState.externalForce.clear();
      grl::robot::arm::copy(friData_->monitoringMsg,
                            std::back_inserter(armState.externalForce),
                            grl::cartesian_external_force_tag());
#endif // KUKA_SUNRISE_1_9
      armState.ipoJointPosition.clear();
      grl::robot::arm::copy(
          friData_->monitoringMsg,
          std::back_inserter(armState.ipoJointPosition),
          grl::revolute_joint_angle_interpolated_open_chain_state_tag());

      armState.sendPeriod = std::chrono::milliseconds(
          grl::robot::arm::get(friData_->monitoringMsg, grl::time_step_tag()));

      armState.time_event_stamp = time_event_stamp;

      //              std::cout << "Measured Torque: ";
      //              std::cout << std::setw(6);
      //              for (float t:armState.torque) {
      //                  std::cout << t << " ";
      //              }
      //              std::cout << '\n';
      //
      //              std::cout << "External Torque: ";
      //              std::cout << std::setw(6);
      //              for (float t:armState.externalTorque) {
      //                  std::cout << t << " ";
      //              }
      //              std::cout << '\n';
      //
      //              std::cout << "External Force: ";
      //              for (float t:armState.externalForce) {
      //                  std::cout << t << " ";
      //              }
      //              std::cout << '\n';

    } else {
      m_attemptedCommunicationConsecutiveFailureCount++;
      std::cerr << "No new FRI data available, is an FRI application running "
                   "on the Kuka arm? \n Total sucessful transfers: "
                << this->m_haveReceivedRealDataCount
                << "\n Total attempts: " << m_attemptedCommunicationCount
                << "\n Consecutive Failures: "
                << m_attemptedCommunicationConsecutiveFailureCount
                << "\n Consecutive Successes: "
                << m_attemptedCommunicationConsecutiveSuccessCount << "\n";
      m_attemptedCommunicationConsecutiveSuccessCount = 0;
      /// @todo TODO(ahundt) should the results of getlatest state even be possible to call
      /// without receiving real data? should the library change?
      /// @todo TODO(ahundt) use spdlog library instead of cerr?
    }

    return haveNewData;
  }

  /**
   * \brief Set the joint positions for the current interpolation step.
   *
   * This method is only effective when the robot is in a commanding state
   * and the set time point for reaching the destination is in the future.
   * This function sets the goal time point for a motion to the epoch, aka "time
   * 0" (which is in the past) for safety.
   *
   *
   * For position based motion to work, you must set both the position you want
   * and the time you want it to get there.
   * This is required because the robot can move extremely fast, so accidentally
   * setting the velocity to the max is
   * very dangerous. If the time point is in the past, the robot will not move.
   * If the time point is too near in the future
   * to make it, the robot will move at the max speed towards that position.
   *
   * @see KukaFRIdriver::set(TimePoint && duration_to_goal_command, time_duration_command_tag) to set
   * the destination time point in the future so the position motion can start.
   *
   * @param state Object which stores the current state of the robot, including
   * the command to send next
   * @param range Array with the new joint positions (in radians)
   * @param tag identifier object indicating that revolute joint angle commands
   * should be modified
   */
  template <typename Range>
  void set(Range &&range, grl::revolute_joint_angle_open_chain_command_tag) {
    boost::lock_guard<boost::mutex> lock(jt_mutex);
    armState.clearCommands();
    boost::copy(range, std::back_inserter(armState.commandedPosition));
    boost::copy(range, std::back_inserter(armState.commandedPosition_goal));
  }

  /**
   * @brief Set the time duration expected between new position commands in ms
   *
   * The driver will likely be updated every so often, such as every 25ms, and
   * the lowest level of the
   * driver may update even more frequently, such as every 1ms. By providing as
   * accurate an
   * estimate between high level updates the low level driver can smooth out the
   * motion through
   * interpolation (the default), or another algorithm. See
   * LowLevelStepAlgorithmType template parameter
   * in the KukaFRIdriver class if you want to change out the low level
   * algorithm.
   *
   * @see KukaFRIdriver::get(time_duration_command_tag)
   *
   * @param duration_to_goal_command std::chrono time format representing the
   * time duration between updates
   *
   */
  void set(double duration_to_goal_command, time_duration_command_tag) {
    boost::lock_guard<boost::mutex> lock(jt_mutex);
    armState.goal_position_command_time_duration = duration_to_goal_command;
  }

  /**
   * @brief Get the timestamp of the most recent armState
   *
   *
   *
   * @see KukaFRIdriver::set(Range&& range,
   * grl::revolute_joint_angle_open_chain_command_tag)
   *
   */
  KukaState::time_point_type get(time_point_tag) {
    boost::lock_guard<boost::mutex> lock(jt_mutex);
    return armState.timestamp;
  }

  /**
   * \brief Set the applied joint torques for the current interpolation step.
   *
   * This method is only effective when the client is in a commanding state.
   * The ControlMode of the robot has to be joint impedance control mode. The
   * Client Command Mode has to be torque.
   *
   * @param state Object which stores the current state of the robot, including
   * the command to send next
   * @param torques Array with the applied torque values (in Nm)
   * @param tag identifier object indicating that the torqe value command should
   * be modified
   */
  template <typename Range>
  void set(Range &&range, grl::revolute_joint_torque_open_chain_command_tag) {
    boost::lock_guard<boost::mutex> lock(jt_mutex);
    armState.clearCommands();
    boost::copy(range, armState.commandedTorque);
  }

  /**
   * \brief Set the applied wrench vector of the current interpolation step.
   *
   * The wrench vector consists of:
   * [F_x, F_y, F_z, tau_A, tau_B, tau_C]
   *
   * F ... forces (in N) applied along the Cartesian axes of the
   * currently used motion center.
   * tau ... torques (in Nm) applied along the orientation angles
   * (Euler angles A, B, C) of the currently used motion center.
   *
   * This method is only effective when the client is in a commanding state.
   * The ControlMode of the robot has to be Cartesian impedance control mode.
   * The
   * Client Command Mode has to be wrench.
   *
   * @param state object storing the command data that will be sent to the
   * physical device
   * @param range wrench Applied Cartesian wrench vector, in x, y, z, roll,
   * pitch, yaw force measurments.
   * @param tag identifier object indicating that the wrench value command
   * should be modified
   *
   * @todo perhaps support some specific more useful data layouts
   */
  template <typename Range>
  void set(Range &&range, grl::cartesian_wrench_command_tag) {
    boost::lock_guard<boost::mutex> lock(jt_mutex);
    armState.clearCommands();
    std::copy(range, armState.commandedCartesianWrenchFeedForward);
  }

  /// @todo should this exist, is it a good design? is it written correctly?
  void get(KukaState &state) {
    boost::lock_guard<boost::mutex> lock(jt_mutex);
    state = armState;
  }

  // The total number of times the FRI interface has successfully received a UDP
  // packet
  // from the robot since this class was initialized.
  volatile std::size_t m_haveReceivedRealDataCount = 0;
  // The total number of times the FRI interface has attempted to receive a UDP
  // packet
  // from the robot since this class was initialized, regardless of if it was
  // successful or not.
  volatile std::size_t m_attemptedCommunicationCount = 0;
  // The number of consecutive FRI receive calls that have failed to get data
  // successfully, resets to 0 on a single success.
  volatile std::size_t m_attemptedCommunicationConsecutiveFailureCount = 0;
  // The number of consecutive FRI receive calls that have received data
  // successfully, resets to 0 on a single failure.
  volatile std::size_t m_attemptedCommunicationConsecutiveSuccessCount = 0;

  boost::asio::io_service device_driver_io_service;
  std::unique_ptr<boost::asio::io_service::work> device_driver_workP_;
  std::unique_ptr<std::thread> driver_threadP;
  std::shared_ptr<
      grl::robot::arm::KukaFRIClientDataDriver<LowLevelStepAlgorithmType>>
      kukaFRIClientDataDriverP_;

private:
  KukaState armState;
  boost::mutex jt_mutex;

  Params params_;
  std::shared_ptr<KUKA::FRI::ClientData> friData_;
};

/// @brief nonmember wrapper function to help integrate KukaFRIdriver objects
/// with generic programming interface
template <typename LowLevelStepAlgorithmType = LinearInterpolation,
          typename Range, typename T>
static inline void set(KukaFRIdriver<LowLevelStepAlgorithmType> &state,
                       Range &&range, T t) {
  state.set(range, t);
}
}
}
} // namespace grl::robot::arm

#endif
