#ifndef _KUKA_FRI_
#define _KUKA_FRI_

#include <chrono>
#include <stdexcept>

#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/units/physical_dimensions/plane_angle.hpp>
#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/container/static_vector.hpp>

// Kuka include files
#include "friClientIf.h"
#include "friClientData.h"
#include "robone/tags.hpp"

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
	    
	
	namespace arm {
		
		namespace kuka {
				// Following from Kuka example program
				const int default_port_id = 30200;
		
			namespace detail {
				template<typename T, typename OutputIterator>
				void copyJointState(T values,OutputIterator it, bool dataAvailable = true){
					if(dataAvailable) std::copy(static_cast<double*>(static_cast<tRepeatedDoubleArguments*>(values)->value),static_cast<double*>(static_cast<tRepeatedDoubleArguments*>(values)->value)+KUKA::LBRState::NUM_DOF,it);
				}
			}
		}
	
	/// copy measured joint angle to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, boost::units::plane_angle_base_dimension, state_tag){
 	   kuka::detail::copyJointState(monitoringMsg.monitorData.measuredJointPosition.value.arg,it,monitoringMsg.monitorData.has_measuredJointPosition);
	}
	
	
	/// copy commanded joint angle to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, boost::units::plane_angle_base_dimension, command_tag){
		kuka::detail::copyJointState(monitoringMsg.monitorData.commandedJointPosition.value.arg,it, monitoringMsg.monitorData.has_commandedJointPosition);
    }
	
	
	/// copy measured joint torque to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, boost::units::torque_dimension, state_tag){
           kuka::detail::copyJointState(monitoringMsg.monitorData.measuredTorque.value.arg,it, monitoringMsg.monitorData.has_measuredTorque);
	}
	
	/// copy measured external joint torque to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, boost::units::torque_dimension, external_state_tag){
           kuka::detail::copyJointState(monitoringMsg.monitorData.externalTorque.value.arg,it, monitoringMsg.monitorData.has_externalTorque);
	}
	
	/// copy commanded  joint torque to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, boost::units::torque_dimension, command_tag){
           kuka::detail::copyJointState(monitoringMsg.monitorData.commandedTorque.value.arg,it, monitoringMsg.monitorData.has_commandedTorque);
	}
	
	/// copy interpolated commanded joint angles
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it,boost::units::plane_angle_base_dimension,interpolated_state_tag){
        if (monitoringMsg.ipoData.has_jointPosition) {
           kuka::detail::copyJointState(monitoringMsg.ipoData.jointPosition.value.arg,it);
        }

	}
	
	/// @todo consider changing these get* functions to get(data,type_tag());
	KUKA::FRI::ESafetyState getSafetyState(const FRIMonitoringMessage& monitoringMsg) {
		KUKA::FRI::ESafetyState KukaSafetyState = KUKA::FRI::ESafetyState::NORMAL_OPERATION;
	    if (monitoringMsg.has_robotInfo) {
	        if (monitoringMsg.robotInfo.has_safetyState)
	            KukaSafetyState = static_cast<KUKA::FRI::ESafetyState>(monitoringMsg.robotInfo.safetyState);
	    }
		return KukaSafetyState;
	}
	
	
	/// @todo consider changing these get* functions to get(data,type_tag());
	KUKA::FRI::EOperationMode getOperationMode(const FRIMonitoringMessage& monitoringMsg) {
		KUKA::FRI::EOperationMode KukaSafetyState = KUKA::FRI::EOperationMode::TEST_MODE_1;
	    if (monitoringMsg.has_robotInfo) {
	        if (monitoringMsg.robotInfo.has_operationMode)
	            KukaSafetyState = static_cast<KUKA::FRI::EOperationMode>(monitoringMsg.robotInfo.operationMode);
	    }
		return KukaSafetyState;
	}
	
	
	/// @todo consider changing these get* functions to get(data,type_tag());
		/// @todo this one requires a callback... figure it out
	KUKA::FRI::EDriveState getDriveState(const FRIMonitoringMessage& monitoringMsg) {
		KUKA::FRI::EDriveState KukaSafetyState = KUKA::FRI::EDriveState::OFF;
	    if (monitoringMsg.has_robotInfo) {
	         //   KukaSafetyState = static_cast<KUKA::FRI::EDriveState>(monitoringMsg.robotInfo.driveState);
	    }
		return KukaSafetyState;
	}
	
#if 0 // original getConnectionInfo
	getConnectionInfo(const FRIMonitoringMessage& monitoringMsg){

	    if (monitoringMsg.has_connectionInfo) {
	        KukaSessionState = monitoringMsg.connectionInfo.sessionState;
	        KukaQuality = monitoringMsg.connectionInfo.quality;
	        if (monitoringMsg.connectionInfo.has_sendPeriod)
	            KukaSendPeriod = monitoringMsg.connectionInfo.sendPeriod;
	        if (monitoringMsg.connectionInfo.has_receiveMultiplier)
	            KukaReceiveMultiplier = monitoringMsg.connectionInfo.receiveMultiplier;
	    }
	}
#endif
	
	KUKA::FRI::ESessionState getSessionState(const FRIMonitoringMessage& monitoringMsg){
		KUKA::FRI::ESessionState KukaSessionState = KUKA::FRI::ESessionState::IDLE;
	    if (monitoringMsg.has_connectionInfo) {
	        KukaSessionState = static_cast<KUKA::FRI::ESessionState>(monitoringMsg.connectionInfo.sessionState);
	    }
		return KukaSessionState;
	}
	
	KUKA::FRI::EConnectionQuality getConnectionQuality(const FRIMonitoringMessage& monitoringMsg){
		KUKA::FRI::EConnectionQuality KukaQuality = KUKA::FRI::EConnectionQuality::POOR;
	    if (monitoringMsg.has_connectionInfo) {
	        KukaQuality = static_cast<KUKA::FRI::EConnectionQuality>(monitoringMsg.connectionInfo.quality);
	    }
		return KukaQuality;
	}
	
	uint32_t getSendPeriod(const FRIMonitoringMessage& monitoringMsg){
		uint32_t KukaSendPeriod = 0;
	    if (monitoringMsg.has_connectionInfo) {
	        if (monitoringMsg.connectionInfo.has_sendPeriod)
	            KukaSendPeriod = monitoringMsg.connectionInfo.sendPeriod;
	    }
		return KukaSendPeriod;
	}
	
	std::size_t getReceiveMultiplier(const FRIMonitoringMessage& monitoringMsg){
		std::size_t KukaReceiveMultiplier = 0;
	    if (monitoringMsg.has_connectionInfo) {
	        if (monitoringMsg.connectionInfo.has_receiveMultiplier)
	            KukaReceiveMultiplier = monitoringMsg.connectionInfo.receiveMultiplier;
	    }
		return KukaReceiveMultiplier;
	}
	
	std::chrono::time_point<std::chrono::high_resolution_clock> getTimeStamp(const FRIMonitoringMessage& monitoringMsg){
		// defaults to the epoch
		std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
        if (monitoringMsg.monitorData.has_timestamp) {
            timestamp += std::chrono::seconds(monitoringMsg.monitorData.timestamp.sec) +
				         std::chrono::nanoseconds(monitoringMsg.monitorData.timestamp.nanosec);
        }
		return timestamp;
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
	
	void copy(const FRIMonitoringMessage& monitoringMsg, KukaState& state ){
		copy(monitoringMsg,state.position.begin(),boost::units::plane_angle_base_dimension(),state_tag());
		copy(monitoringMsg,state.torque.begin(),boost::units::torque_dimension(),state_tag());
		copy(monitoringMsg,state.commandedPosition.begin(),boost::units::plane_angle_base_dimension(),command_tag());
		copy(monitoringMsg,state.commandedTorque.begin(),boost::units::torque_dimension(),command_tag());
		copy(monitoringMsg,state.ipoJointPosition.begin(),boost::units::plane_angle_base_dimension(),interpolated_state_tag());
		state.sessionState = getSessionState(monitoringMsg);
		state.connectionQuality = getConnectionQuality(monitoringMsg);
		state.safetyState = getSafetyState(monitoringMsg);
		state.operationMode = getOperationMode(monitoringMsg);
		/// @todo this one requires a callback... figure it out
		//state.driveState = getDriveState(monitoringMsg);
			
		/// @todo fill out missing state update steps
		
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
	
	namespace kuka {
	
	class iiwa : std::enable_shared_from_this<iiwa>
	{
	
	};
	
	
	}

}}} /// namespace robone::robot::arm

#endif

