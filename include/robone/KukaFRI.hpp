#ifndef _KUKA_FRI_
#define _KUKA_FRI_

#include <chrono>
#include <stdexcept>

#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/units/physical_dimensions/plane_angle.hpp>
#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>

// Kuka include files
#include "friClientIf.h"
#include "friClientData.h"

namespace KUKA {
	namespace LBRState {
		const int NUM_DOF = 7;
		const int LBRMONITORMESSAGEID = 0x245142;
	}
	namespace LBRCommand {
		// Following from Kuka friLBRCommand.cpp
		const int LBRCOMMANDMESSAGEID = 0x34001;
	}
}

namespace robone { namespace robot { 
	    
		struct state_tag{}; /// @todo consider moving to separate tag header
		struct command_tag{};
		struct external_state_tag{}; /// @todo consider eliding external and internal sensor tag
		struct interpolated_state_tag{};
	
	namespace arm {
		
		namespace kuka {
				// Following from Kuka example program
				const int default_port_id = 30200;
		
			namespace detail {
				template<typename OutputIterator>
				void copyJointState(tRepeatedDoubleArguments* values,OutputIterator it){
					std::copy(static_cast<double*>(values->value),static_cast<double*>(values->value)+KUKA::LBRState::NUM_DOF,it);
				}
			}
		}
	
	/// copy measured joint angle to output iterator
	template<typename OutputIterator>
	void copy(const KUKA::FRI::ClientData& friData, OutputIterator it, boost::units::plane_angle_base_dimension, robone::robot::state_tag){

        if (friData.monitoringMsg.monitorData.has_measuredJointPosition) {
           kuka::detail::copyJointState(static_cast<tRepeatedDoubleArguments*>(friData.monitoringMsg.monitorData.measuredJointPosition.value.arg),it);
        }
	}
	
	
	/// copy commanded joint angle to output iterator
	template<typename OutputIterator>
	void copy(const KUKA::FRI::ClientData& friData, OutputIterator it, boost::units::plane_angle_base_dimension, robone::robot::command_tag){

        if (friData.monitoringMsg.monitorData.has_commandedJointPosition) {
           kuka::detail::copyJointState(static_cast<tRepeatedDoubleArguments*>(friData.monitoringMsg.monitorData.commandedJointPosition.value.arg),it);
        }
	}
	
	
	/// copy measured joint torque to output iterator
	template<typename OutputIterator>
	void copy(const KUKA::FRI::ClientData& friData, OutputIterator it, boost::units::torque_dimension, robone::robot::state_tag){
        if (friData.monitoringMsg.monitorData.has_measuredTorque) {
           kuka::detail::copyJointState(static_cast<tRepeatedDoubleArguments*>(friData.monitoringMsg.monitorData.measuredTorque.value.arg),it);
        }
	}
	
	/// copy measured external joint torque to output iterator
	template<typename OutputIterator>
	void copy(const KUKA::FRI::ClientData& friData, OutputIterator it, boost::units::torque_dimension, external_state_tag){
        if (friData.monitoringMsg.monitorData.has_externalTorque) {
           kuka::detail::copyJointState(static_cast<tRepeatedDoubleArguments*>(friData.monitoringMsg.monitorData.externalTorque.value.arg),it);
        }
	}
	
	/// copy commanded  joint torque to output iterator
	template<typename OutputIterator>
	void copy(const KUKA::FRI::ClientData& friData, OutputIterator it, boost::units::torque_dimension, command_tag){
        if (friData.monitoringMsg.monitorData.has_commandedTorque) {
           kuka::detail::copyJointState(static_cast<tRepeatedDoubleArguments*>(friData.monitoringMsg.monitorData.commandedTorque.value.arg),it);
        }
	}
	
	/// copy interpolated commanded joint angles
	template<typename OutputIterator>
	void copy(const KUKA::FRI::ClientData& friData, OutputIterator it,boost::units::plane_angle_base_dimension,interpolated_state_tag){
        if (friData.monitoringMsg.ipoData.has_jointPosition) {
           kuka::detail::copyJointState(static_cast<tRepeatedDoubleArguments*>(friData.monitoringMsg.ipoData.jointPosition.value.arg),it);
        }

	}
	
	/// @todo consider changing these get* functions to get(data,type_tag());
	KUKA::FRI::ESafetyState getSafetyState(const KUKA::FRI::ClientData& friData) {
		KUKA::FRI::ESafetyState KukaSafetyState = KUKA::FRI::ESafetyState::NORMAL_OPERATION;
	    if (friData.monitoringMsg.has_robotInfo) {
	        if (friData.monitoringMsg.robotInfo.has_safetyState)
	            KukaSafetyState = static_cast<KUKA::FRI::ESafetyState>(friData.monitoringMsg.robotInfo.safetyState);
	    }
		return KukaSafetyState;
	}
	
	
	/// @todo consider changing these get* functions to get(data,type_tag());
	KUKA::FRI::EOperationMode getOperationMode(const KUKA::FRI::ClientData& friData) {
		KUKA::FRI::EOperationMode KukaSafetyState = KUKA::FRI::EOperationMode::TEST_MODE_1;
	    if (friData.monitoringMsg.has_robotInfo) {
	        if (friData.monitoringMsg.robotInfo.has_operationMode)
	            KukaSafetyState = static_cast<KUKA::FRI::EOperationMode>(friData.monitoringMsg.robotInfo.operationMode);
	    }
		return KukaSafetyState;
	}
	
	
	/// @todo consider changing these get* functions to get(data,type_tag());
		/// @todo this one requires a callback... figure it out
	KUKA::FRI::EDriveState getDriveState(const KUKA::FRI::ClientData& friData) {
		KUKA::FRI::EDriveState KukaSafetyState = KUKA::FRI::EDriveState::OFF;
	    if (friData.monitoringMsg.has_robotInfo) {
	         //   KukaSafetyState = static_cast<KUKA::FRI::EDriveState>(friData.monitoringMsg.robotInfo.driveState);
	    }
		return KukaSafetyState;
	}
	
#if 0 // original getConnectionInfo
	getConnectionInfo(const KUKA::FRI::ClientData& friData){

	    if (friData.monitoringMsg.has_connectionInfo) {
	        KukaSessionState = friData.monitoringMsg.connectionInfo.sessionState;
	        KukaQuality = friData.monitoringMsg.connectionInfo.quality;
	        if (friData.monitoringMsg.connectionInfo.has_sendPeriod)
	            KukaSendPeriod = friData.monitoringMsg.connectionInfo.sendPeriod;
	        if (friData.monitoringMsg.connectionInfo.has_receiveMultiplier)
	            KukaReceiveMultiplier = friData.monitoringMsg.connectionInfo.receiveMultiplier;
	    }
	}
#endif
	
	KUKA::FRI::ESessionState getSessionState(const KUKA::FRI::ClientData& friData){
		KUKA::FRI::ESessionState KukaSessionState = KUKA::FRI::ESessionState::IDLE;
	    if (friData.monitoringMsg.has_connectionInfo) {
	        KukaSessionState = static_cast<KUKA::FRI::ESessionState>(friData.monitoringMsg.connectionInfo.sessionState);
	    }
		return KukaSessionState;
	}
	
	KUKA::FRI::EConnectionQuality getConnectionQuality(const KUKA::FRI::ClientData& friData){
		KUKA::FRI::EConnectionQuality KukaQuality = KUKA::FRI::EConnectionQuality::POOR;
	    if (friData.monitoringMsg.has_connectionInfo) {
	        KukaQuality = static_cast<KUKA::FRI::EConnectionQuality>(friData.monitoringMsg.connectionInfo.quality);
	    }
		return KukaQuality;
	}
	
	uint32_t getSendPeriod(const KUKA::FRI::ClientData& friData){
		uint32_t KukaSendPeriod = 0;
	    if (friData.monitoringMsg.has_connectionInfo) {
	        if (friData.monitoringMsg.connectionInfo.has_sendPeriod)
	            KukaSendPeriod = friData.monitoringMsg.connectionInfo.sendPeriod;
	    }
		return KukaSendPeriod;
	}
	
	std::size_t getReceiveMultiplier(const KUKA::FRI::ClientData& friData){
		std::size_t KukaReceiveMultiplier = 0;
	    if (friData.monitoringMsg.has_connectionInfo) {
	        if (friData.monitoringMsg.connectionInfo.has_receiveMultiplier)
	            KukaReceiveMultiplier = friData.monitoringMsg.connectionInfo.receiveMultiplier;
	    }
		return KukaReceiveMultiplier;
	}
	
	std::chrono::time_point<std::chrono::high_resolution_clock> getTimeStamp(const KUKA::FRI::ClientData& friData){
		// defaults to the epoch
		std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
        if (friData.monitoringMsg.monitorData.has_timestamp) {
            timestamp += std::chrono::seconds(friData.monitoringMsg.monitorData.timestamp.sec) +
				         std::chrono::nanoseconds(friData.monitoringMsg.monitorData.timestamp.nanosec);
        }
		return timestamp;
	}
	
	/// @todo replace with something generic
	struct KukaState {
		typedef boost::array<double,KUKA::LBRState::NUM_DOF> joint_state;
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
	

	    // Decode message buffer (using nanopb decoder)
	void decode(KUKA::FRI::ClientData& friData){
	/// @todo FRI_MONITOR_MSG_MAX_SIZE may not be the right size... probably need the actual size received
	    if (!friData.decoder.decode(friData.receiveBuffer, KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE)) {
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
	
	/// encode data in the class into the send buffer
	/// @todo update the statements in here to run on the actual data types available
	void encode(KUKA::FRI::ClientData& friData,KukaState& state){
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
	            return;
	}
	
	void copy(const KUKA::FRI::ClientData& friData, KukaState& state ){
		copy(friData,state.position.begin(),boost::units::plane_angle_base_dimension(),robone::robot::state_tag());
		copy(friData,state.torque.begin(),boost::units::torque_dimension(),robone::robot::state_tag());
		copy(friData,state.commandedPosition.begin(),boost::units::plane_angle_base_dimension(),robone::robot::command_tag());
		copy(friData,state.commandedTorque.begin(),boost::units::torque_dimension(),robone::robot::command_tag());
		copy(friData,state.ipoJointPosition.begin(),boost::units::plane_angle_base_dimension(),robone::robot::interpolated_state_tag());
		state.sessionState = getSessionState(friData);
		state.connectionQuality = getConnectionQuality(friData);
		state.safetyState = getSafetyState(friData);
		state.operationMode = getOperationMode(friData);
		/// @todo this one requires a callback... figure it out
		//state.driveState = getDriveState(friData);
			
		/// @todo fill out missing state update steps
		
	}
	
	
	/// @todo implment async version of this, probably in a small class
	/// @todo implement sending state
	void update_state(boost::asio::ip::udp::socket& socket, KUKA::FRI::ClientData& friData, KukaState& state){
		std::size_t buf_size = socket.receive(boost::asio::buffer(friData.receiveBuffer,KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE));
		copy(friData,state);

	    friData.lastSendCounter++;
	    // Check whether to send a response
	    if (friData.lastSendCounter >= friData.monitoringMsg.connectionInfo.receiveMultiplier){
	    	encode(friData,state);
			socket.send(boost::asio::buffer(friData.sendBuffer,KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE));
	    }
	}
	
	

}}} /// namespace robone::robot::arm

#endif

