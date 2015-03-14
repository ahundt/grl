#ifndef _KUKA_FRI_CLIENT_DATA
#define _KUKA_FRI_CLIENT_DATA

#include "friClientData.h"
#include "robone/KukaFRI.hpp"

namespace robone { namespace robot { namespace arm {
    // Decode message buffer (using nanopb decoder)
void decode(KUKA::FRI::ClientData& friData, std::size_t msg_size){
/// @todo FRI_MONITOR_MSG_MAX_SIZE may not be the right size... probably need the actual size received
    if (!friData.decoder.decode(friData.receiveBuffer, msg_size)) {
        throw std::runtime_error( "Error decoding received data");
    }
	

    // check message type
    if (friData.expectedMonitorMsgID != friData.monitoringMsg.header.messageIdentifier)
    {
		throw std::invalid_argument(std::string("KukaFRI.hpp: Problem reading buffer, id code: ") +
			                         boost::lexical_cast<std::string>(static_cast<int>(friData.monitoringMsg.header.messageIdentifier)) + 
									std::string(" does not match expected id code: ") +
				                    boost::lexical_cast<std::string>(static_cast<int>(friData.expectedMonitorMsgID)) + std::string("\n")
									);
        return;
    }
}


/// @todo replace with something generic
struct KukaState {
	typedef boost::container::static_vector<double,KUKA::LBRState::NUM_DOF> joint_state;
	joint_state position;
	joint_state torque;
	joint_state commandedPosition;
	joint_state commandedTorque;
	joint_state ipoJointPosition;
	KUKA::FRI::ESessionState      sessionState;
	KUKA::FRI::EConnectionQuality connectionQuality;
	KUKA::FRI::ESafetyState       safetyState;
	KUKA::FRI::EOperationMode     operationMode;
	KUKA::FRI::EDriveState        driveState;
	
	std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
};


void copy(const FRIMonitoringMessage& monitoringMsg, KukaState& state ){
	copy(monitoringMsg,state.position.begin(),revolute_joint_angle_multi_state_tag());
	copy(monitoringMsg,state.torque.begin(),revolute_joint_torque_multi_state_tag());
	copy(monitoringMsg,state.commandedPosition.begin(),revolute_joint_angle_multi_command_tag());
	copy(monitoringMsg,state.commandedTorque.begin(),revolute_joint_torque_multi_command_tag());
	copy(monitoringMsg,state.ipoJointPosition.begin(),revolute_joint_angle_interpolated_multi_state_tag());
	state.sessionState = getSessionState(monitoringMsg);
	state.connectionQuality = getConnectionQuality(monitoringMsg);
	state.safetyState = getSafetyState(monitoringMsg);
	state.operationMode = getOperationMode(monitoringMsg);
	state.driveState = getDriveState(monitoringMsg);
		
	/// @todo fill out missing state update steps
	
}

/// encode data in the class into the send buffer
/// @todo update the statements in here to run on the actual data types available
std::size_t encode(KUKA::FRI::ClientData& friData,KukaState& state){
    // Check whether to send a response
    friData.lastSendCounter = 0;
  
    // set sequence counters
    friData.commandMsg.header.sequenceCounter = friData.sequenceCounter++;
    friData.commandMsg.header.reflectedSequenceCounter = friData.monitoringMsg.header.sequenceCounter;

    // copy current joint position to commanded position
    friData.commandMsg.has_commandData = true;
    friData.commandMsg.commandData.has_jointPosition = true;
    tRepeatedDoubleArguments *dest  = (tRepeatedDoubleArguments*)friData.commandMsg.commandData.jointPosition.value.arg;
	if ((state.sessionState == KUKA::FRI::COMMANDING_WAIT) || (state.sessionState == KUKA::FRI::COMMANDING_ACTIVE))
	    std::copy(state.ipoJointPosition.begin(),state.ipoJointPosition.end(),dest->value); /// @todo is this the right thing to copy?
    else
	    std::copy(state.commandedPosition.begin(),state.commandedPosition.end(),dest->value);  /// @todo is this the right thing to copy?
	
	int buffersize = KUKA::FRI::FRI_COMMAND_MSG_MAX_SIZE;
        if (!friData.encoder.encode(friData.sendBuffer, buffersize))
            return 0;
	
	return buffersize;
}


/// @todo implment async version of this, probably in a small class
/// @todo implement sending state
void update_state(boost::asio::ip::udp::socket& socket, KUKA::FRI::ClientData& friData, KukaState& state){
	std::size_t buf_size = socket.receive(boost::asio::buffer(friData.receiveBuffer,KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE));
	decode(friData,buf_size);
	copy(friData.monitoringMsg,state);

    friData.lastSendCounter++;
    // Check whether to send a response
    if (friData.lastSendCounter >= friData.monitoringMsg.connectionInfo.receiveMultiplier){
    	buf_size = encode(friData,state);
		socket.send(boost::asio::buffer(friData.sendBuffer,buf_size));
    }
}

}}} // namespace robone::robot::arm

#endif