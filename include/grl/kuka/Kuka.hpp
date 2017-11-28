/// Kuka.hpp provides some internal KUKA specific definitions and utility implementation functions.
/// If you are not familiar with this code, you are most likely looking for KukaDriver.hpp.
#ifndef GRL_KUKA_HPP
#define GRL_KUKA_HPP

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/container/static_vector.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/exception/all.hpp>
#include <boost/asio.hpp>


#include "grl/flatbuffer/KUKAiiwa_generated.h"
#include "grl/tags.hpp"
#include "grl/exception.hpp"
// used for time synchronization
#include "grl/TimeEvent.hpp"

#include <chrono>
#include <ctime>

namespace KUKA {
namespace LBRState {
/// @todo replace all instances of this with the getter now provided
const int NUM_DOF = 7;
const int LBRMONITORMESSAGEID = 0x245142;
}
namespace LBRCommand {
// Following from Kuka friLBRCommand.cpp
const int LBRCOMMANDMESSAGEID = 0x34001;
}

} // namespace KUKA

namespace grl {
namespace robot {
namespace arm {

/// @brief Internal implementation class for driver use only, stores all the
/// kuka state data in a simple object
/// @todo replace with something generic
/// @deprecated this is an old implemenation that will be removed in the future,
/// do not depend on this struct directly.
/// @todo commandedPosition and commandedPosition_goal are used a bit
/// ambiguously, figure out the difference and clean it up.
/// @TODO(ahundt) add support for a vector of Transformation data @see FRIMessages.pb.h,
/// but store as quaternon + vector, not rotation matrix
struct KukaState {
  typedef boost::container::static_vector<double, KUKA::LBRState::NUM_DOF>
      joint_state;
  // cartesian state entries consist of a vector x,y,z and a quaternion [x, y, z, w]
  typedef boost::container::static_vector<double, 7> cartesian_state;
  /// Class std::chrono::high_resolution_clock represents the clock with the smallest tick period provided by the implementation.
  typedef std::chrono::time_point<std::chrono::high_resolution_clock> time_point_type;

  /// measured position, identified by revolute_joint_angle_open_chain_state_tag
  /// @see grl::revolute_joint_angle_open_chain_state_tag in grl/tags.hpp, which identifies this data
  joint_state position;
  /// measured torque, identified by revolute_joint_torque_open_chain_state_tag
  /// @see grl::revolute_joint_torque_open_chain_state_tag in grl/tags.hpp, which identifies this data
  joint_state torque;
  /// torque applied to the robot from an outside source,
  /// @see grl::revolute_joint_torque_open_chain_state_tag in grl/tags.hpp, which identifies this data
  joint_state externalTorque;
  /// measured external force, only available in sunrise OS version 1.9
  cartesian_state externalForce;

  /// commanded joint angles
  /// @see grl::revolute_joint_angle_open_chain_command_tag in grl/tags.hpp, which identifies this data
  joint_state commandedPosition;
  /// cartesian_wrench_command_tag
  cartesian_state commandedCartesianWrenchFeedForward;
  joint_state commandedTorque;

  joint_state ipoJointPosition;
  joint_state ipoJointPositionOffsets;
  std::chrono::milliseconds sendPeriod; ///< time duration between each FRI low
                                        /// level UDP packet loop update

  //  Each of the following have an equivalent in kuka's friClientIf.h
  //  which needed to be reimplemented due to licensing restrictions
  //  in the corresponding C++ code
  flatbuffer::ESessionState sessionState; // KUKA::FRI::ESessionState
  flatbuffer::EConnectionQuality
      connectionQuality;                    // KUKA::FRI::EConnectionQuality
  flatbuffer::ESafetyState safetyState;     // KUKA::FRI::ESafetyState
  flatbuffer::EOperationMode operationMode; // KUKA::FRI::EOperationMode
  flatbuffer::EDriveState driveState;       // KUKA::FRI::EDriveState

  // The point in time associated with the current measured
  // state of the arm (position, torque, etc.). When commanding
  // the arm use commanded_goal_timestamp.
  /// TODO(Chunting) remove this and make all code that uses it instead set the time_event_stamp
  /// most likely the device_time, but double check the correctness of that
  /// time_point_type timestamp;

  /////////////////////////////////////////////////////////////////////////////////////////////
  // members below here define the driver state and are not part of the FRI arm
  // message format
  /////////////////////////////////////////////////////////////////////////////////////////////

  /// we need to mind arm constraints, so we set a goal then work towards it
  joint_state commandedPosition_goal;

  /// time duration over which commandedPosition_goal is expected to be reached.
  /// this will be used when computing the trajectory with which the low level
  /// algorithm will approach the goal position.
  /// This should be strictly greater than or equal to timestamp.
  ///
  /// @note: this is part of the driver wrapper and is not present in the
  /// underlying kuka APIs.
  double goal_position_command_time_duration;

  /// velocity_limits we need to mind arm constraints, so we set a goal then
  /// work towards it. While velocity limits are not provided explicitly by KUKA
  /// in their low level C++ API, if you send a command that violates the
  /// velocity limits the arm stops immediately with an error.
  joint_state velocity_limits;

  /// Time event records the hardware time,
  /// the local computer time before receiving data
  /// and the local computer time after receiving data
  /// This makes it possible to more accurately synchronize
  /// time between multiple hardware devices.
  grl::TimeEvent time_event_stamp;

  void clear() {
    position.clear();
    torque.clear();
    externalTorque.clear();
    externalForce.clear();
    commandedPosition.clear();
    commandedTorque.clear();
    commandedCartesianWrenchFeedForward.clear();
    ipoJointPosition.clear();
    commandedPosition_goal.clear();
    goal_position_command_time_duration = 0;
  }

  void clearCommands() {
    commandedPosition.clear();
    commandedTorque.clear();
    commandedCartesianWrenchFeedForward.clear();
    commandedPosition_goal.clear();
  }
};

/// constexpr objects are const and are initiallized with values known during compilation
constexpr auto KUKA_LBR_IIWA_14_R820 = "KUKA_LBR_IIWA_14_R820";
/// constexpr auto KUKA_LBR_IIWA_7_R8, identified by00 = "KUKA_LBR_IIWA_7_R800";
constexpr auto KUKA_LBR_IIWA_7_R8 = "KUKA_LBR_IIWA_7_R800";

/// @brief copy vector of joint velocity limits in radians/s
///
/// @todo R800 velocity limits aren't correct!
/// @todo find a better design where this can be expanded for more models of
/// robot in the future, maybe a std::unordered_map?
template <typename OutputIterator>
OutputIterator
copy(std::string model, OutputIterator it,
     grl::revolute_joint_velocity_open_chain_state_constraint_tag) {
  if (boost::iequals(model, KUKA_LBR_IIWA_14_R820)) {

    // R820 velocity limits
    // A1 - 85 °/s  == 1.483529864195 rad/s
    // A2 - 85 °/s  == 1.483529864195 rad/s
    // A3 - 100 °/s == 1.745329251994 rad/s
    // A4 - 75 °/s  == 1.308996938996 rad/s
    // A5 - 130 °/s == 2.268928027593 rad/s
    // A6 - 135 °/s == 2.356194490192 rad/s
    // A1 - 135 °/s == 2.356194490192 rad/s

    KukaState::joint_state maxVel;
    maxVel.push_back(1.483529864195); // RK: Commented out secondsPerTick
    maxVel.push_back(1.483529864195);
    maxVel.push_back(1.745329251994);
    maxVel.push_back(1.308996938996);
    maxVel.push_back(2.268928027593);
    maxVel.push_back(2.356194490192);
    maxVel.push_back(2.356194490192);

    return boost::copy(maxVel, it);
    /// KUKA_LBR_IIWA_7_R800 to KUKA_LBR_IIWA_7_R820
  } else if (boost::iequals(model, "KUKA_LBR_IIWA_7_R800")) {

    /// @RK: updated the right joint velocity information based
    //  on the 800 model from the KUKA manual

    // R800 velocity limits
    // A1 - 98 °/s   == 1.71042 rad/s
    // A2 - 98 °/s   == 1.71042 rad/s
    // A3 - 100 °/s  == 1.74533 rad/s
    // A4 - 130 °/s  == 2.26893 rad/s
    // A5 - 140 °/s  == 2.44346 rad/s
    // A6 - 180 °/s  == 3.14159 rad/s
    // A1 - 180 °/s  == 3.14159 rad/s

    KukaState::joint_state maxVel;
    maxVel.push_back(1.71042);
    maxVel.push_back(1.71042);
    maxVel.push_back(1.74533);
    maxVel.push_back(2.26893);
    maxVel.push_back(2.44346);
    maxVel.push_back(3.14159);
    maxVel.push_back(3.14159);

    return boost::copy(maxVel, it);
  }

  else
    return it;
}


/// @brief Internal class, defines some default status variables
///
/// This class defines some connection functions and parameter definitions
/// that are shared amongst many of the KUKA API components
class KukaUDP {

public:
  enum ParamIndex {
    RobotModel,              // RobotModel (options are KUKA_LBR_IIWA_14_R820,
                             // KUKA_LBR_IIWA_7_R800)
    localhost,               // 192.170.10.100
    localport,               // 30200
    remotehost,              // 192.170.10.2
    remoteport,              // 30200
    is_running_automatically // true by default, this means that an internal
                             // thread will be created to run the driver.
  };

  enum ThreadingRunMode { run_manually = 0, run_automatically = 1 };

  typedef std::tuple<std::string, std::string, std::string, std::string,
                     std::string, ThreadingRunMode>
      Params;

  static const Params defaultParams() {
    return std::make_tuple(KUKA_LBR_IIWA_14_R820, std::string("192.170.10.100"),
                           std::string("30200"), std::string("192.170.10.2"),
                           std::string("30200"), run_automatically);
  }

  /// Advanced functionality, do not use without a great reason
  template <typename T>
  static boost::asio::ip::udp::socket
  connect(T &params, boost::asio::io_service &io_service_,
          boost::asio::ip::udp::endpoint &sender_endpoint) {
    std::string localhost(std::get<localhost>(params));
    std::string lp(std::get<localport>(params));
    short localport = boost::lexical_cast<short>(lp);
    std::string remotehost(std::get<remotehost>(params));
    std::string rp(std::get<remoteport>(params));
    short remoteport = boost::lexical_cast<short>(rp);
    std::cout << "using: "
              << " " << localhost << " " << localport << " " << remotehost
              << " " << remoteport << "\n";

    boost::asio::ip::udp::socket s(
        io_service_,
        boost::asio::ip::udp::endpoint(
            boost::asio::ip::address::from_string(localhost), localport));

    boost::asio::ip::udp::resolver resolver(io_service_);
    sender_endpoint =
        *resolver.resolve({boost::asio::ip::udp::v4(), remotehost, rp});
    s.connect(sender_endpoint);

    return std::move(s);
  }

  static void add_details_to_connection_error(boost::exception &e,
                                              Params &params) {
    e << errmsg_info(
        "KukaUDP: Unable to connect to Kuka FRI Koni UDP device "
        "using boost::asio::udp::socket configured with localhost:localport "
        "@ " +
        std::get<localhost>(params) + ":" + std::get<localport>(params) +
        " and remotehost:remoteport @ " + std::get<remotehost>(params) + ":" +
        std::get<remoteport>(params) + "\n");
  }
};

}
}
} // namespace grl::robot::arm

#endif
