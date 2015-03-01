#ifndef _KUKA_FRI_
#define _KUKA_FRI_

#include <chrono>
#include <stdexcept>

#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/units/physical_dimensions/plane_angle.hpp>
#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/lexical_cast.hpp>

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
	
	namespace arm {
		
		namespace kuka {
				// Following from Kuka example program
				const int default_port_id = 30200;
				struct interpolated_joint_angle_data_tag{};
		
			namespace detail {
				template<typename OutputIterator>
				copyJointState(tRepeatedDoubleArguments* values,it){
					std::copy(static_cast<double*>(values->value),static_cast<double*>(values->value)+KUKA::LBRState::NUM_DOF,it);
				}
			}
		}
	
	/// copy measured joint angle to output iterator
	template<typename OutputIterator,typename Interaction = robone::robot::state_tag>
	copy(const KUKA::FRI::ClientDATA& friData, OutputIterator it, boost::units::plane_angle_base_dimension(), Interaction()){

        if (friData->monitoringMsg.monitorData.has_measuredJointPosition) {
           kuka::detail::copyJointState(static_cast<tRepeatedDoubleArguments*>(friData.monitoringMsg.monitorData.measuredJointPosition.value.arg));
        }
	}
	
	
	/// copy measured joint torque to output iterator
	template<typename OutputIterator,typename Interaction = robone::robot::state_tag>
	copy(const KUKA::FRI::ClientDATA& friData, OutputIterator it, boost::units::torque_dimension(), Interaction()){
        if (friData->monitoringMsg.monitorData.has_measuredTorque) {
           kuka::detail::copyJointState(static_cast<tRepeatedDoubleArguments*>(friData.monitoringMsg.monitorData.measuredTorque.value.arg));
        }
	}
	
	/// copy measured external joint torque to output iterator
	template<typename OutputIterator>
	copy(const KUKA::FRI::ClientDATA& friData, OutputIterator it, boost::units::torque_dimension(), external_state_tag()){
        if (friData->monitoringMsg.monitorData.has_externalTorque) {
           kuka::detail::copyJointState(static_cast<tRepeatedDoubleArguments*>(friData.monitoringMsg.monitorData.externalTorque.value.arg));
        }
	}
	
	/// copy commanded  joint torque to output iterator
	template<typename OutputIterator>
	copy(const KUKA::FRI::ClientDATA& friData, OutputIterator it, boost::units::torque_dimension(), command_tag()){
        if (friData->monitoringMsg.monitorData.has_commandedTorque) {
           kuka::detail::copyJointState(static_cast<tRepeatedDoubleArguments*>(friData.monitoringMsg.monitorData.commandedTorque.value.arg));
        }
	}
	
	/// copy interpolated commanded joint angles
	copy(const KUKA::FRI::ClientDATA& friData, OutputIterator it,boost::units::plane_angle_base_dimension(),interpolated_joint_angle_data_tag()){
        if (friData->monitoringMsg.ipoData.has_jointPosition) {
           kuka::detail::copyJointState(static_cast<tRepeatedDoubleArguments*>(friData.ipoData.jointPosition.value.arg));
        }
		
	}
	
	
	getSafetyState(const KUKA::FRI::ClientDATA& friData) {
	    if (friData->monitoringMsg.has_robotInfo) {
	        if (friData->monitoringMsg.robotInfo.has_safetyState)
	            KukaSafetyState = friData->monitoringMsg.robotInfo.safetyState;
	    }
	}
	
	getConnectionInfo(const KUKA::FRI::ClientDATA& friData){

	    if (friData->monitoringMsg.has_connectionInfo) {
	        KukaSessionState = friData->monitoringMsg.connectionInfo.sessionState;
	        KukaQuality = friData->monitoringMsg.connectionInfo.quality;
	        if (friData->monitoringMsg.connectionInfo.has_sendPeriod)
	            KukaSendPeriod = friData->monitoringMsg.connectionInfo.sendPeriod;
	        if (friData->monitoringMsg.connectionInfo.has_receiveMultiplier)
	            KukaReceiveMultiplier = friData->monitoringMsg.connectionInfo.receiveMultiplier;
	    }
	}
	
	std::chrono::time_point<std::chrono::high_resolution_clock> getTimeStamp(const KUKA::FRI::ClientDATA& friData){
		// defaults to the epoch
		std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
        if (friData->monitoringMsg.monitorData.has_timestamp) {
            timestamp += std::chrono::seconds(friData->monitoringMsg.monitorData.timestamp.sec) +
				         std::chrono::nanoseconds(friData->monitoringMsg.monitorData.timestamp.nanosec);
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
		ESessionState      sessionState;
		EConnectionQuality connectionQuality;
		ESafetyState       safetyState;
		EOperationMode     operationMode;
		EDriveState        driveState;
		
		std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
	};
	

	    // Decode message buffer (using nanopb decoder)
	void decode(KUKA::FRI::ClientDATA& friData){
	    if (!friData.decoder.decode(friData->receiveBuffer, size)) {
	        throw std::runtime_error( "Error decoding received data");
	    }
		

	    // check message type
	    if (friData->expectedMonitorMsgID != friData->monitoringMsg.header.messageIdentifier)
	    {
			throw std::invalid_argument("KukaFRI.hpp: Problem reading buffer, id code: " +
				                         boost::lexical_cast<std::string>(static_cast<int>(friData->monitoringMsg.header.messageIdentifier)) + 
										" does not match expected id code:" + 
					                    boost::lexical_cast<std::string>(static_cast<int>(friData->expectedMonitorMsgID)) + 
										);
	        return;
	    }
	}
	
	/// encode data in the class into the send buffer
	/// @todo update the statements in here to run on the actual data types available
	void encode(KUKA::FRI::ClientDATA& friData){
	    // Check whether to send a response
	        friData->lastSendCounter = 0;
      
	        // set sequence counters
	        friData->commandMsg.header.sequenceCounter = friData->sequenceCounter++;
	        friData->commandMsg.header.reflectedSequenceCounter = friData->monitoringMsg.header.sequenceCounter;

	        // copy current joint position to commanded position
	        friData->commandMsg.has_commandData = true;
	        friData->commandMsg.commandData.has_jointPosition = true;
	        tRepeatedDoubleArguments *dest
	            = (tRepeatedDoubleArguments*)friData->commandMsg.commandData.jointPosition.value.arg;
			if ((KukaSessionState == KUKA::FRI::COMMANDING_WAIT) || (KukaSessionState == KUKA::FRI::COMMANDING_ACTIVE))
	            memcpy(dest->value, JointPosIpo.Pointer(), NUM_DOF * sizeof(double));
	        else
	            memcpy(dest->value, JointPos.Pointer(), NUM_DOF * sizeof(double));
      
	        if (!friData->encoder.encode(friData->sendBuffer, size))
	            return;
	}
	
	copy(KUKA::FRI::ClientDATA& friData, KukaState state ){
		copy(friData,state.position.begin(),boost::units::plane_angle_base_dimension(),robone::robot::state_tag());
		copy(friData,state.torque.begin(),boost::units::torque_dimension(),robone::robot::state_tag());
		copy(friData,state.commandedPosition.begin(),boost::units::plane_angle_base_dimension(),robone::robot::command_tag());
		copy(friData,state.commandedTorque.begin(),boost::units::torque_dimension(),robone::robot::command_tag());
		/// @todo fill out missing state update steps
		
	}
	
	
	/// @todo implment async version of this, probably in a small class
	update_state(boost::asio::udp::socket& socket, KUKA::FRI::ClientDATA& friData, KukaState& state){
		socket.receive(boost::asio::buffer(friData->receiveBuffer),FRI_MONITOR_MSG_MAX_SIZE);
		copy(friData,state);

	    friData->lastSendCounter++;
	    // Check whether to send a response
	    if (friData->lastSendCounter >= friData->monitoringMsg.connectionInfo.receiveMultiplier){
	    	encode(friData);
	    }
	}
	
	

}}} /// namespace robone::robot::arm

#endif

