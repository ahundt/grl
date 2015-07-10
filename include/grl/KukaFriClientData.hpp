#ifndef _KUKA_FRI_CLIENT_DATA
#define _KUKA_FRI_CLIENT_DATA

#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/transform.hpp>
#include "friClientData.h"
#include "grl/KukaFRI.hpp"
#include <boost/log/trivial.hpp>

/// @todo move back to KukaFRIClientDataTest.cpp
template<typename T,size_t N>
std::ostream& operator<< (std::ostream& out, const boost::container::static_vector<T,N>& v) {
    out << "[";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last) 
            out << ", ";
    }
    out << "]";
    return out;
}

/// @todo move back to KukaFRIClientDataTest.cpp
template<typename T,std::size_t U>
inline boost::log::formatting_ostream& operator<<(boost::log::formatting_ostream& out,  boost::container::static_vector<T,U>& v)
{
    out << "[";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last) 
            out << ", ";
    }
    out << "]";
    return out;
}


namespace grl { namespace robot { namespace arm {






    // Decode message buffer (using nanopb decoder)
void decode(KUKA::FRI::ClientData& friData, std::size_t msg_size){
    // The decoder was given a pointer to the monitoringMessage at initialization
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
    
    friData.lastState = grl::robot::arm::get(friData.monitoringMsg,KUKA::FRI::ESessionState());
}




/////////////////////////////////
//  moving out to user code
//    // copy current joint position to commanded position
//	if ((state.ipoJointPosition.size() == KUKA::LBRState::NUM_DOF) &&
//        (((friData.sessionState == KUKA::FRI::COMMANDING_WAIT)) || (state.sessionState == KUKA::FRI::COMMANDING_ACTIVE))
//       ) {
//
//         KukaState::joint_state jointStateToCommand;
//        // vector addition between ipoJointPosition and ipoJointPositionOffsets, copying the result into jointStateToCommand
//        boost::transform ( state.ipoJointPosition, state.ipoJointPositionOffsets, std::back_inserter(jointStateToCommand), std::plus<KukaState::joint_state::value_type>());
//        
//        // copy the commanded position into the command message
//        set(friData.commandMsg, jointStateToCommand, grl::revolute_joint_angle_open_chain_command_tag());
//        
//		//BOOST_LOG_TRIVIAL(trace) << "ENCODE position: " << state.position << " connectionQuality: " << state.connectionQuality << " operationMode: " << state.operationMode << " sessionState: " << state.sessionState << " driveState: " << state.driveState << " ipoJointPosition: " << state.ipoJointPosition << " ipoJointPositionOffsets: " << state.ipoJointPositionOffsets << " jointStateToCommand: " << jointStateToCommand << "\n";
//    }else {
//        set(friData.commandMsg, state.commandedPosition, grl::revolute_joint_angle_open_chain_command_tag());
//    }





/// encode data in the class into the send buffer
std::size_t encode(KUKA::FRI::ClientData& friData,boost::system::error_code& ec){
    // reset send counter
    friData.lastSendCounter = 0;
  
    // set sequence counters
    friData.commandMsg.header.sequenceCounter = friData.sequenceCounter++;
    friData.commandMsg.header.reflectedSequenceCounter = friData.monitoringMsg.header.sequenceCounter;
	
	int buffersize = KUKA::FRI::FRI_COMMAND_MSG_MAX_SIZE;
    if (!friData.encoder.encode(friData.sendBuffer, buffersize)){
        // @todo figure out PB_GET_ERROR
        ec = boost::system::errc::make_error_code(boost::system::errc::bad_message);
        return 0;
    }
	
	return buffersize;
}


void update_state(boost::asio::ip::udp::socket& socket, KUKA::FRI::ClientData& friData, boost::system::error_code& receive_ec,std::size_t& receive_bytes_transferred, boost::system::error_code& send_ec, std::size_t& send_bytes_transferred){
    
    boost::asio::ip::udp::endpoint sender_endpoint;
    int message_flags = 0;
	receive_bytes_transferred = socket.receive_from(boost::asio::buffer(friData.receiveBuffer,KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE),sender_endpoint,message_flags,receive_ec);
	decode(friData,receive_bytes_transferred);

    friData.lastSendCounter++;
    // Check whether to send a response
    if (friData.lastSendCounter >= friData.monitoringMsg.connectionInfo.receiveMultiplier){
    	send_bytes_transferred = encode(friData,send_ec);
        if(send_ec) return;
		socket.send_to(boost::asio::buffer(friData.sendBuffer,send_bytes_transferred), sender_endpoint, message_flags, send_ec);
    }
}








/// @todo replace with something generic
struct KukaState {
	typedef boost::container::static_vector<double,KUKA::LBRState::NUM_DOF> joint_state;
    typedef boost::container::static_vector<double,6> cartesian_state;
    
//    KukaState()
//    :
//    position(0,KUKA::LBRState::NUM_DOF),
//    torque(0,KUKA::LBRState::NUM_DOF),
//    commandedPosition(0,KUKA::LBRState::NUM_DOF),
//    commandedTorque(0,KUKA::LBRState::NUM_DOF),
//    ipoJointPosition(0,KUKA::LBRState::NUM_DOF){}
    
	joint_state     position;
	joint_state     torque;
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
    
    void clear(){
      position.clear();
      torque.clear();
      commandedPosition.clear();
      commandedTorque.clear();
      ipoJointPosition.clear();
    }
};



void copy(const FRIMonitoringMessage& monitoringMsg, KukaState& state ){
    state.clear();
	copy(monitoringMsg,std::back_inserter(state.position),revolute_joint_angle_open_chain_state_tag());
	copy(monitoringMsg,std::back_inserter(state.torque),revolute_joint_torque_open_chain_state_tag());
	copy(monitoringMsg,std::back_inserter(state.commandedPosition),revolute_joint_angle_open_chain_command_tag());
	copy(monitoringMsg,std::back_inserter(state.commandedTorque),revolute_joint_torque_open_chain_command_tag());
	copy(monitoringMsg,std::back_inserter(state.ipoJointPosition),revolute_joint_angle_interpolated_open_chain_state_tag());
	state.sessionState = get(monitoringMsg,KUKA::FRI::ESessionState());
	state.connectionQuality = get(monitoringMsg,KUKA::FRI::EConnectionQuality());
	state.safetyState = get(monitoringMsg,KUKA::FRI::ESafetyState());
	state.operationMode = get(monitoringMsg,KUKA::FRI::EOperationMode());
	state.driveState = get(monitoringMsg,KUKA::FRI::EDriveState());
		
	/// @todo fill out missing state update steps
	
}

}}} // namespace grl::robot::arm

#endif