#ifndef _KUKA_FRI_CLIENT_DATA
#define _KUKA_FRI_CLIENT_DATA

#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/transform.hpp>
#include "friClientData.h"
#include "grl/KukaFRI.hpp"
#include "grl/KukaFRIThreadSeparator.hpp"
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
        BOOST_THROW_EXCEPTION(std::runtime_error( "Error decoding received data"));
    }
	

    // check message type
    if (friData.expectedMonitorMsgID != friData.monitoringMsg.header.messageIdentifier)
    {
		BOOST_THROW_EXCEPTION( std::invalid_argument(std::string("KukaFRI.hpp: Problem reading buffer, id code: ") +
			                         boost::lexical_cast<std::string>(static_cast<int>(friData.monitoringMsg.header.messageIdentifier)) + 
									std::string(" does not match expected id code: ") +
				                    boost::lexical_cast<std::string>(static_cast<int>(friData.expectedMonitorMsgID)) + std::string("\n")
									));
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
    
    KUKA::FRI::ESessionState sessionState = grl::robot::arm::get(friData.monitoringMsg,KUKA::FRI::ESessionState());
        // copy current joint position to commanded position
	if (sessionState == KUKA::FRI::COMMANDING_WAIT || sessionState == KUKA::FRI::COMMANDING_ACTIVE)
    {
           /// @todo check that the user set things, if not, copy from the monitormessage
       
//         KukaState::joint_state jointStateToCommand;
//        
//        // vector addition between ipoJointPosition and ipoJointPositionOffsets, copying the result into jointStateToCommand
//        boost::transform ( state.ipoJointPosition, state.ipoJointPositionOffsets, std::back_inserter(jointStateToCommand), std::plus<KukaState::joint_state::value_type>());
//
//        // copy the commanded position into the command message
//        set(friData.commandMsg, jointStateToCommand, grl::revolute_joint_angle_open_chain_command_tag());
//        
//		//BOOST_LOG_TRIVIAL(trace) << "ENCODE position: " << state.position << " connectionQuality: " << state.connectionQuality << " operationMode: " << state.operationMode << " sessionState: " << state.sessionState << " driveState: " << state.driveState << " ipoJointPosition: " << state.ipoJointPosition << " ipoJointPositionOffsets: " << state.ipoJointPositionOffsets << " jointStateToCommand: " << jointStateToCommand << "\n";
    } else {
        /// @todo allow copying of data directly between commandmsg and monitoringMsg
        std::vector<double> msg;
        copy(friData.monitoringMsg,std::back_inserter(msg),revolute_joint_angle_open_chain_command_tag());
        // copy the previously recorded command over
        set(friData.commandMsg,msg, grl::revolute_joint_angle_open_chain_command_tag());
    }
    
	
	int buffersize = KUKA::FRI::FRI_COMMAND_MSG_MAX_SIZE;
    if (!friData.encoder.encode(friData.sendBuffer, buffersize)){
        // @todo figure out PB_GET_ERROR
        ec = boost::system::errc::make_error_code(boost::system::errc::bad_message);
        return 0;
    }
	
	return buffersize;
}

/// @pre socket must already have the endpoint resolved and "connected". While udp is technically stateless the asio socket supports the connection api components for convenience.
void update_state(boost::asio::ip::udp::socket& socket, KUKA::FRI::ClientData& friData, boost::system::error_code& receive_ec,std::size_t& receive_bytes_transferred, boost::system::error_code& send_ec, std::size_t& send_bytes_transferred){
    
    int message_flags = 0;
	receive_bytes_transferred = socket.receive(boost::asio::buffer(friData.receiveBuffer,KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE),message_flags,receive_ec);
	decode(friData,receive_bytes_transferred);

    friData.lastSendCounter++;
    // Check whether to send a response
    if (friData.lastSendCounter >= friData.monitoringMsg.connectionInfo.receiveMultiplier){
    	send_bytes_transferred = encode(friData,send_ec);
        if(send_ec) return;
		socket.send(boost::asio::buffer(friData.sendBuffer,send_bytes_transferred), message_flags, send_ec);
    }
}


class KukaFRIClientDataDriver : public std::enable_shared_from_this<KukaFRIClientDataDriver>, public KukaFRI {
    
public:

    using KukaFRI::ParamIndex;
    using KukaFRI::ThreadingRunMode;
    using KukaFRI::Params;
    using KukaFRI::defaultParams;
    
    
    
	KukaFRIClientDataDriver(boost::asio::io_service& ios, Params params = defaultParams())
        :
        isConnectionEstablished_(false),
        io_service_(ios)
    {
      construct(params);
	}
    
    
	KukaFRIClientDataDriver(Params params = defaultParams())
        :
        isConnectionEstablished_(false),
        optional_internal_io_service_P(new boost::asio::io_service),
        io_service_(*optional_internal_io_service_P)
    {
        construct(params);
	}

    /// Call this to initialize the object
    void construct(Params params = defaultParams())
    {

        try
        {
            connect(params, io_service_);
            // start up the driver thread since the io_service_ is internal only
            if(std::get<is_running_automatically>(params))
            {
              driver_threadP_.reset(new std::thread([&]{ update(); }));
            }
        
        } catch( boost::exception &e) {
            add_details_to_connection_error(e,params);
            throw;
        }
        
    }
    
    /// @brief blocking call to communicate with the robot continuously
    /// @pre construct() should
    void run(){
      update();
    }

    /// @brief Updates the passed friData shared pointer to a pointer to newly updated data, plus any errors that occurred.
    ///
    /// @note This function is not thread safe cannot be called simultaneously from multiple threads.
    ///
    /// @param[in,out] friData the pointer to update with new state and optional input state.
    ///
    /// @pre If friData!=nullptr it is assumed valid for use and this class will take control of the object.
    ///
    /// @return false if everything is ok, true otherwise (simplifies checking all the ec and bytes_transferred when there isn't a problem)
    bool update_state(std::shared_ptr<KUKA::FRI::ClientData>& friData, boost::system::error_code& receive_ec,std::size_t& receive_bytes_transferred, boost::system::error_code& send_ec, std::size_t& send_bytes_transferred){
       
        
        // ensure we have valid data for future updates
        std::shared_ptr<KUKA::FRI::ClientData> tempFriData;
        if(friData.get() == nullptr)
        {
          tempFriData = std::make_shared<KUKA::FRI::ClientData>(KUKA::LBRState::NUM_DOF);
        }
        else
        {
          tempFriData = friData;
        }
        std::atomic_exchange(&userThreadStateP_,latestStateP_);
        
        if(userThreadStateP_.get()!=nullptr)
        {
          std::tie(friData,receive_ec,receive_bytes_transferred,send_ec,send_bytes_transferred) = *userThreadStateP_;
          std::atomic_exchange(&std::get<latest_receive_monitor_state>(*userThreadStateP_),tempFriData);
        }
        else
        {
          std::tie(friData,receive_ec,receive_bytes_transferred,send_ec,send_bytes_transferred) = LatestState();
        }
        
        return !(receive_bytes_transferred > 0 && !receive_ec && !send_ec); // note send_bytes_transferred can be 0 without any problems
    }

    void destruct(){
       m_shouldStop = true;
       if(driver_threadP_){
         driver_threadP_->join();
       }
    }
    
    ~KukaFRIClientDataDriver(){
       destruct();
    }

    /// Is everything ok?
    /// @return true if the kuka fri connection is actively running without any issues
    /// @todo consider expanding to support real error codes
    bool is_active()
    {
      return !exceptionPtr && isConnectionEstablished_;
    }

private:
    /// Reads data off of the real kuka fri device in a separate thread
    void update() {
       try {
            // initialize all of the states
            latestStateP_     = make_shared_valid_LatestState();
            nextStateP_       = make_shared_valid_LatestState();
            userThreadStateP_ = make_shared_valid_LatestState();

            // run the primary update loop in a separate thread
            while (!m_shouldStop)
            {
                grl::robot::arm::update_state  (
                                                *socketP_,
                                                *std::get<latest_receive_monitor_state>(*nextStateP_),
                                                std::get<latest_receive_ec>(*nextStateP_),
                                                std::get<latest_receive_bytes_transferred>(*nextStateP_),
                                                std::get<latest_send_ec>(*nextStateP_),
                                                std::get<latest_send_bytes_transferred>(*nextStateP_)
                                               );
                isConnectionEstablished_ = true;
                std::atomic_exchange(&latestStateP_,nextStateP_);
            }
           
       } catch(...) {
            // transport the exception to the main thread in a safe manner
            exceptionPtr = std::current_exception();
            m_shouldStop = true;
       }
       
       isConnectionEstablished_ = false;
    }

    enum LatestStateIndex{
     latest_receive_monitor_state,
     latest_receive_ec,
     latest_receive_bytes_transferred,
     latest_send_ec,
     latest_send_bytes_transferred
    };
    
    typedef std::tuple<std::shared_ptr<KUKA::FRI::ClientData>,boost::system::error_code, std::size_t,boost::system::error_code, std::size_t>
    LatestState;
    
    /// Creates a default LatestState Object
    /// @todo num_dof should not be a magic number
    /// @note the clientData will never be null
    static LatestState make_LatestState(std::shared_ptr<KUKA::FRI::ClientData> clientData){
      return std::make_tuple(clientData,boost::system::error_code(), std::size_t(),boost::system::error_code(), std::size_t());
    }
    
    /// Initialize valid shared ptr to LatestState object with a valid allocated friData
    static std::shared_ptr<LatestState> make_shared_valid_LatestState(std::shared_ptr<KUKA::FRI::ClientData> friData = std::make_shared<KUKA::FRI::ClientData>(KUKA::LBRState::NUM_DOF)){
        
        std::shared_ptr<LatestState> tempLatestStateP;
        
        if(friData.get()==nullptr)
        {
          tempLatestStateP = std::make_shared<LatestState>(make_LatestState(std::make_shared<KUKA::FRI::ClientData>(KUKA::LBRState::NUM_DOF)));
        }
        else
        {
          tempLatestStateP = std::make_shared<LatestState>(make_LatestState(friData));
        }
        
        return tempLatestStateP;
    }
    

    /// @todo replace with unique_ptr
    /// the latest state we have available
    std::shared_ptr<LatestState> latestStateP_;
    std::shared_ptr<LatestState> nextStateP_;
    std::shared_ptr<LatestState> userThreadStateP_;
    std::atomic<bool> m_shouldStop;
    std::exception_ptr exceptionPtr;
    std::atomic<bool> isConnectionEstablished_;

    /// may be null, allows the user to choose if they want to provide an io_service
    std::unique_ptr<boost::asio::io_service> optional_internal_io_service_P;
    
	// other things to do somewhere:
	// - get list of control points
	// - get the control point in the arm base coordinate system
	// - load up a configuration file with ip address to send to, etc.
	boost::asio::io_service& io_service_;
    std::unique_ptr<boost::asio::ip::udp::socket> socketP_;
    std::unique_ptr<std::thread> driver_threadP_;
};





/// @brief don't use this
/// @todo replace with something generic
/// @deprecated this is an old implemenation that will be removed in the future
struct KukaState {
	typedef boost::container::static_vector<double,KUKA::LBRState::NUM_DOF> joint_state;
    typedef boost::container::static_vector<double,6> cartesian_state;
    
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


/// @brief don't use this
/// @deprecated this is an old implemenation that will be removed in the future
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


/// @deprecated this is an old implemenation that will be removed in the future
/// encode data in the class into the send buffer
/// @todo update the statements in here to run on the actual data types available
std::size_t encode(KUKA::FRI::ClientData& friData,KukaState& state){
    // Check whether to send a response
    friData.lastSendCounter = 0;
  
    // set sequence counters
    friData.commandMsg.header.sequenceCounter = friData.sequenceCounter++;
    friData.commandMsg.header.reflectedSequenceCounter = friData.monitoringMsg.header.sequenceCounter;

    // copy current joint position to commanded position
	if ((state.ipoJointPosition.size() == KUKA::LBRState::NUM_DOF) &&
        (((state.sessionState == KUKA::FRI::COMMANDING_WAIT)) || (state.sessionState == KUKA::FRI::COMMANDING_ACTIVE))
       ) {
       
         KukaState::joint_state jointStateToCommand;
        
        // vector addition between ipoJointPosition and ipoJointPositionOffsets, copying the result into jointStateToCommand
        boost::transform ( state.ipoJointPosition, state.ipoJointPositionOffsets, std::back_inserter(jointStateToCommand), std::plus<KukaState::joint_state::value_type>());
        
        // copy the commanded position into the command message
        set(friData.commandMsg, jointStateToCommand, grl::revolute_joint_angle_open_chain_command_tag());
        
		//BOOST_LOG_TRIVIAL(trace) << "ENCODE position: " << state.position << " connectionQuality: " << state.connectionQuality << " operationMode: " << state.operationMode << " sessionState: " << state.sessionState << " driveState: " << state.driveState << " ipoJointPosition: " << state.ipoJointPosition << " ipoJointPositionOffsets: " << state.ipoJointPositionOffsets << " jointStateToCommand: " << jointStateToCommand << "\n";
    }else {
        set(friData.commandMsg, state.commandedPosition, grl::revolute_joint_angle_open_chain_command_tag());
    }
	
	int buffersize = KUKA::FRI::FRI_COMMAND_MSG_MAX_SIZE;
        if (!friData.encoder.encode(friData.sendBuffer, buffersize))
            return 0;
	
	return buffersize;
}

/// @deprecated this is an old implemenation that will be removed in the future
/// @todo implment async version of this, probably in a small class
/// @todo implement sending state
void update_state(boost::asio::ip::udp::socket& socket, KUKA::FRI::ClientData& friData, KukaState& state){
    
    boost::asio::ip::udp::endpoint sender_endpoint;
	std::size_t buf_size = socket.receive_from(boost::asio::buffer(friData.receiveBuffer,KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE),sender_endpoint);
	decode(friData,buf_size);
    KukaState::joint_state ipoJointCommand;
    
    // copy state over
	copy(friData.monitoringMsg,state);
    
    // todo: figure out if there is a better place/time to do this
    //state.ipoJointPosition.clear();

    friData.lastSendCounter++;
    // Check whether to send a response
    if (friData.lastSendCounter >= friData.monitoringMsg.connectionInfo.receiveMultiplier){
    	buf_size = encode(friData,state);
		socket.send(boost::asio::buffer(friData.sendBuffer,buf_size));
    }
}


}}} // namespace grl::robot::arm

#endif