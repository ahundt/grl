#ifndef GRL_KUKA_HPP
#define GRL_KUKA_HPP

#include <boost/container/static_vector.hpp>
#include "grl/flatbuffer/KUKAiiwa_generated.h"

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
	typedef boost::container::static_vector<double,KUKA::LBRState::NUM_DOF> joint_state;
    typedef boost::container::static_vector<double,7> cartesian_state;
    
	joint_state     position;
	joint_state     torque;
	joint_state     commandedPosition;
    cartesian_state commandedCartesianWrenchFeedForward;
	joint_state     commandedTorque;
    
	joint_state     ipoJointPosition;
    joint_state     ipoJointPositionOffsets;
    
    //  Each of the following have an equivalent in kuka's friClientIf.h
    //  which needed to be reimplemented due to licensing restrictions
    //  in the corresponding C++ code
	flatbuffer::ESessionState      sessionState;      // KUKA::FRI::ESessionState
	flatbuffer::EConnectionQuality connectionQuality; // KUKA::FRI::EConnectionQuality
	flatbuffer::ESafetyState       safetyState;       // KUKA::FRI::ESafetyState
	flatbuffer::EOperationMode     operationMode;     // KUKA::FRI::EOperationMode
	flatbuffer::EDriveState        driveState;        // KUKA::FRI::EDriveState
	
	std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
    
    // members bewlow here define the driver state and are not part of the FRI arm message format
    
    /// we need to mind arm constraints, so we set a goal then work towards it
    joint_state     commandedPosition_goal;
    /// velocity_limits we need to mind arm constraints, so we set a goal then work towards it
    joint_state     velocity_limits;
    
    void clear(){
      position.clear();
      torque.clear();
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