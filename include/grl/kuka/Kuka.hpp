#ifndef GRL_KUKA_HPP
#define GRL_KUKA_HPP

#include <boost/algorithm/string/predicate.hpp>
#include <boost/container/static_vector.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/transform.hpp>

#include "grl/flatbuffer/KUKAiiwa_generated.h"
#include "grl/tags.hpp"

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
struct KukaState {
  typedef boost::container::static_vector<double, KUKA::LBRState::NUM_DOF>
      joint_state;
  typedef boost::container::static_vector<double, 7> cartesian_state;
  typedef std::chrono::time_point<std::chrono::high_resolution_clock>
      time_point_type;

  joint_state position;
  joint_state torque;
  joint_state externalTorque;
  cartesian_state externalForce;
  joint_state commandedPosition;
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
  time_point_type timestamp;

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

constexpr auto KUKA_LBR_IIWA_14_R820 = "KUKA_LBR_IIWA_14_R820";
constexpr auto KUKA_LBR_IIWA_7_R800 = "KUKA_LBR_IIWA_7_R800";

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
    KukaState::joint_state maxVel = {
        1.483529864195, 1.483529864195, 1.745329251994, 1.308996938996,
        2.268928027593, 2.356194490192, 2.356194490192};
    return boost::copy(maxVel, it);
  } else if (boost::iequals(model, KUKA_LBR_IIWA_7_R800)) {
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
    KukaState::joint_state maxVel = {1.71042, 1.71042, 1.74533, 2.26893,
                                     2.44346, 3.14159, 3.14159};
    return boost::copy(maxVel, it);
  }

  else
    return it;
}
}
}
} // namespace grl::robot::arm

#endif
