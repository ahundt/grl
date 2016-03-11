#ifndef GRL_KUKA_HPP
#define GRL_KUKA_HPP

#include <boost/container/static_vector.hpp>

/// @todo the FRI headers below depend on code that can't be shared, find a way to remove them so the java interface can work without the FRI source.
#include "friClientIf.h"
#include "friClientData.h"
#include "FRIMessages.pb.h"
#include "friCommandMessageEncoder.h"
#include "friMonitoringMessageDecoder.h"

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

namespace grl { namespace robot { namespace arm {

/// @brief Internal implementation class for driver use only, stores all the kuka state data in a simple object
/// @todo replace with something generic
/// @deprecated this is an old implemenation that will be removed in the future
struct KukaState {
	typedef boost::container::static_vector<double,KUKA::LBRState::NUM_DOF+1> joint_state;
    typedef boost::container::static_vector<double,7> cartesian_state;
    
	joint_state     position;
	joint_state     torque;
    joint_state     externalTorque;
    cartesian_state externalForce;
	joint_state     commandedPosition;
    cartesian_state commandedCartesianWrenchFeedForward;
	joint_state     commandedTorque;
    
	joint_state     ipoJointPosition;
    joint_state     ipoJointPositionOffsets;
	KUKA::FRI::ESessionState      sessionState;
	KUKA::FRI::EConnectionQuality connectionQuality;
	KUKA::FRI::ESafetyState       safetyState;
	KUKA::FRI::EOperationMode     operationMode;
	KUKA::FRI::EDriveState        driveState;
	
	std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
    
    // members bewlow here define the driver state and are not part of the FRI arm message format
    
    /// we need to mind arm constraints, so we set a goal then work towards it
    joint_state     commandedPosition_goal;
    /// velocity_limits we need to mind arm constraints, so we set a goal then work towards it
    joint_state     velocity_limits;
    
    void clear(){
      position.clear();
      torque.clear();
      externalTorque.clear();
      externalForce.clear();
      commandedPosition.clear();
      commandedTorque.clear();
      commandedCartesianWrenchFeedForward.clear();
      ipoJointPosition.clear();
      commandedPosition_goal.clear();
    }
    
    void clearCommands(){
      commandedPosition.clear();
      commandedTorque.clear();
      commandedCartesianWrenchFeedForward.clear();
      commandedPosition_goal.clear();
    }
};


}}}// namespace grl::robot::arm


#endif