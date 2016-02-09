#ifndef _KUKA_FRI_DRIVER
#define _KUKA_FRI_DRIVER

#include <tuple>
#include <memory>
#include <thread>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/exception/all.hpp>
#include <boost/config.hpp>

#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/math/special_functions/sign.hpp>


#ifdef BOOST_NO_CXX11_ATOMIC_SMART_PTR
#include <boost/thread.hpp>
#endif

// friClientData is found in the kuka connectivity FRI cpp zip file
#include "friClientIf.h"
#include "friClientData.h"
#include "grl/kuka/KukaFRIalgorithm.hpp"
#include "grl/exception.hpp"

#include "Kuka.hpp"

struct KukaState;

/// @todo move somewhere that won't cause conflicts
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




/// encode data in the class into the send buffer
std::size_t encode(KUKA::FRI::ClientData& friData,boost::system::error_code& ec){
    // reset send counter
    friData.lastSendCounter = 0;
  
    // set sequence counters
    friData.commandMsg.header.sequenceCounter = friData.sequenceCounter++;
    friData.commandMsg.header.reflectedSequenceCounter = friData.monitoringMsg.header.sequenceCounter;
    
    KUKA::FRI::ESessionState sessionState = grl::robot::arm::get(friData.monitoringMsg,KUKA::FRI::ESessionState());
        // copy current measured joint position to commanded position only if we *don't* have new command data
	if (
        !(friData.commandMsg.has_commandData &&
        (sessionState == KUKA::FRI::COMMANDING_WAIT || sessionState == KUKA::FRI::COMMANDING_ACTIVE))
       )
    {
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
void update_state(boost::asio::ip::udp::socket& socket, KUKA::FRI::ClientData& friData, boost::system::error_code& receive_ec,std::size_t& receive_bytes_transferred, boost::system::error_code& send_ec, std::size_t& send_bytes_transferred, boost::asio::ip::udp::endpoint sender_endpoint = boost::asio::ip::udp::endpoint()){
    
    int message_flags = 0;
	receive_bytes_transferred = socket.receive_from(boost::asio::buffer(friData.receiveBuffer,KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE),sender_endpoint);
	decode(friData,receive_bytes_transferred);

    friData.lastSendCounter++;
    // Check whether to send a response
    if (friData.lastSendCounter >= friData.monitoringMsg.connectionInfo.receiveMultiplier){
    	send_bytes_transferred = encode(friData,send_ec);
        if(send_ec) return;
		socket.send(boost::asio::buffer(friData.sendBuffer,send_bytes_transferred), message_flags, send_ec);
    }
}


/// @brief Internal class, defines some default status variables 
class KukaFRI {
    
public:

    enum ParamIndex {
        localhost,  // 192.170.10.100
        localport,  // 30200
        remotehost, // 192.170.10.2
        remoteport,  // 30200
        send_period_millisec, // number of milliseconds between fri updates (1-5)
        is_running_automatically // true by default, this means that an internal thread will be created to run the driver.
    };
    
    enum ThreadingRunMode {
      run_manually = 0,
      run_automatically = 1
    };
    
    typedef std::tuple<std::string,std::string,std::string,std::string,std::size_t,ThreadingRunMode> Params;
    
    
    static const Params defaultParams(){
        return std::make_tuple(std::string("192.170.10.100"),std::string("30200"),std::string("192.170.10.2"),std::string("30200"),1/* ms per tick */,run_automatically);
    }
    
    
    /// Advanced functionality, do not use without a great reason
    template<typename T>
    static boost::asio::ip::udp::socket connect(T& params, boost::asio::io_service& io_service_,boost::asio::ip::udp::endpoint& sender_endpoint){
              std::string localhost(std::get<localhost>(params));
              std::string lp(std::get<localport>(params));
              short localport = boost::lexical_cast<short>(lp);
              std::string remotehost(std::get<remotehost>(params));
              std::string rp(std::get<remoteport>(params));
              short remoteport = boost::lexical_cast<short>(rp);
              std::cout << "using: "<< " " <<  localhost << " " << localport << " " <<  remotehost << " " << remoteport << "\n";
        
            boost::asio::ip::udp::socket s(io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(localhost), localport));

            boost::asio::ip::udp::resolver resolver(io_service_);
            sender_endpoint = *resolver.resolve({boost::asio::ip::udp::v4(), remotehost, rp});
            s.connect(sender_endpoint);
        
            return std::move(s);
    }
    
    static void add_details_to_connection_error(boost::exception& e, Params& params){
                e << errmsg_info("KukaFRIThreadSeparator: Unable to connect to Kuka FRI Koni UDP device using boost::asio::udp::socket configured with localhost:localport @ " +
                                   std::get<localhost>(params) + ":" + std::get<localport>(params) + " and remotehost:remoteport @ " +
                                   std::get<remotehost>(params) + ":" + std::get<remoteport>(params) + "\n");
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


/// @brief Simple low level driver to communicate over the Kuka iiwa FRI interface using KUKA::FRI::ClientData status objects
///
/// One important aspect of this design is the is_running_automatically flag. If you are unsure,
/// the suggested default is run_automatically (true/enabled). When it is enabled,
/// the driver will create a thread internally and run the event loop (io_service) itself.
/// If run manually, you are expected to call io_service.run() on the io_service you provide,
/// or on the run() member function. When running manually you are also expected to call
/// async_getLatestState(handler) frequently enought that the 5ms response requirement of the KUKA
/// FRI interface is met.
class KukaFRIClientDataDriver : public std::enable_shared_from_this<KukaFRIClientDataDriver>, public KukaFRI {
    
public:

    using KukaFRI::ParamIndex;
    using KukaFRI::ThreadingRunMode;
    using KukaFRI::Params;
    using KukaFRI::defaultParams;
    
    
    
	KukaFRIClientDataDriver(boost::asio::io_service& ios, Params params = defaultParams())
        :
        params_(params),
        m_shouldStop(false),
        isConnectionEstablished_(false),
        io_service_(ios)
        //,socketP_(std::move(connect(params, io_service_,sender_endpoint_))) ///< @todo this breaks the assumption that the object can be constructed without hardware issues being a porblem
    {
        construct(params);
	}
    
    
	KukaFRIClientDataDriver(Params params = defaultParams())
        :
        params_(params),
        m_shouldStop(false),
        isConnectionEstablished_(false),
        optional_internal_io_service_P(new boost::asio::io_service),
        io_service_(*optional_internal_io_service_P)
        //,socketP_(std::move(connect(params, io_service_,sender_endpoint_))) ///< @todo this breaks the assumption that the object can be constructed without hardware issues being a porblem
    {
        construct(params);
	}

    /// Call this to initialize the object
    void construct(Params params = defaultParams())
    {
        try
        {
        
            ///////////
            // initialize all of the states
            latestStateForUser_      = make_valid_LatestState();
            spareStates_.emplace_back(std::move(make_valid_LatestState()));
            spareStates_.emplace_back(std::move(make_valid_LatestState()));
            
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
    /// @pre construct() should be called before run()
    void run(){
      update();
    }

    /// @brief Updates the passed friData shared pointer to a pointer to newly updated data, plus any errors that occurred.
    ///
    /// We recommend you supply a valid shared_ptr to friData, even if all command elements are set to false.
    /// The friData pointer you pass in can contain a command to send to the arm.
    /// To update with new state and optional input state, you give up lifetime control of the input,
    /// and assume liftime control of the output.
    ///
    /// This function is designed for single threaded use to quickly receive and send "non-blocking" updates to the robot.
    /// It is not thread safe cannot be called simultaneously from multiple threads.
    ///
    ///
    /// @note { An error code is set if update_state is called with no new data available.
    ///         In this special case, all error codes and bytes_transferred are 0, because
    ///         there was no new data available for the user.
    ///       }
    ///
    /// @warning Do not pound this call continuously in a very tight loop, because then the driver won't be able to acquire the lock and send updates to the robot.
    ///
    /// @param[in,out] friData supply a new command, receive a new update of the robot state. Pointer is null if no new data is available.
    ///
    /// @pre If friData!=nullptr it is assumed valid for use and this class will take control of the object.
    ///
    /// @return isError = false if you have new data, true when there is either an error or no new data
    bool update_state(std::shared_ptr<KUKA::FRI::ClientData>& friData, boost::system::error_code& receive_ec,std::size_t& receive_bytes_transferred, boost::system::error_code& send_ec, std::size_t& send_bytes_transferred){
       
       if(exceptionPtr) {
         /// @note this exception most likely came from the update() call running the kuka driver
         std::rethrow_exception(exceptionPtr);
       }
       
       bool haveNewData = false;
       
       if(!isConnectionEstablished_ || !std::get<latest_receive_monitor_state>(latestStateForUser_))
       {
          // no new data, so immediately return results accordingly
          std::tie(friData,receive_ec,receive_bytes_transferred,send_ec,send_bytes_transferred) = make_LatestState(friData);
          return !haveNewData;
       }
       
        // ensure we have valid data for future updates
        // need to copy this over because friData will be set as an output value later
        // and allocate/initialize data if null
        auto validFriDataLatestState = make_valid_LatestState(friData);
        
        // get the latest state from the driver thread
        {
            boost::lock_guard<boost::mutex> lock(ptrMutex_);
            
            // get the update if one is available
            // the user has provided new data to send to the device
            if(std::get<latest_receive_monitor_state>(validFriDataLatestState)->commandMsg.has_commandData)
            {
                  std::swap(validFriDataLatestState,newCommandForDriver_);
            }
            // newCommandForDriver_ is finalized
        
            if(spareStates_.size() < spareStates_.capacity() && std::get<latest_receive_monitor_state>(validFriDataLatestState))
            {
               spareStates_.emplace_back(std::move(validFriDataLatestState));
            }
        
            if(std::get<latest_receive_monitor_state>(latestStateForUser_))
            {
                // return the latest state to the caller
                std::tie(friData,receive_ec,receive_bytes_transferred,send_ec,send_bytes_transferred) = std::move(latestStateForUser_);
                haveNewData = true;
            }
            else if(std::get<latest_receive_monitor_state>(validFriDataLatestState))
            {
                // all storage is full, return the spare data to the user
                std::tie(friData,receive_ec,receive_bytes_transferred,send_ec,send_bytes_transferred) = validFriDataLatestState;
            }
        
        }
        
        // let the user know if we aren't in the best possible state
        return !haveNewData || receive_bytes_transferred == 0 || receive_ec || send_ec;
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
    /// @todo consider switching to single producer single consumer queue to avoid locking overhead, but keep latency in mind https://github.com/facebook/folly/blob/master/folly/docs/ProducerConsumerQueue.md
    void update() {
       try
       {

            /// nextState is the object currently being loaded with data off the network
            /// the driver thread should access this exclusively in update()
            LatestState nextState           = make_valid_LatestState();
            LatestState latestCommandBackup = make_valid_LatestState();

            boost::asio::ip::udp::endpoint sender_endpoint;
            boost::asio::ip::udp::socket socket(connect(params_, io_service_,sender_endpoint));
            KukaState kukastate; ///< @todo remove this line when new api works completely since old one is deprecated
           
            /////////////
            // run the primary update loop in a separate thread
            while (!m_shouldStop)
            {
                /// @todo maybe there is a more convienient way to set this that is easier for users? perhaps initializeClientDataForiiwa()?
            
                // nextState and latestCommandBackup should never be null
                BOOST_VERIFY(std::get<latest_receive_monitor_state>(nextState));
                BOOST_VERIFY(std::get<latest_receive_monitor_state>(latestCommandBackup));
            
                // set the flag that must always be there
                std::get<latest_receive_monitor_state>(nextState)->expectedMonitorMsgID = KUKA::LBRState::LBRMONITORMESSAGEID;
            
                // actually talk over the network to receive an update and send out a new command
                grl::robot::arm::update_state  (
                                                socket
                                                ,*std::get<latest_receive_monitor_state>(nextState)
                                                ,std::get<latest_receive_ec>(nextState)
                                                ,std::get<latest_receive_bytes_transferred>(nextState)
                                                ,std::get<latest_send_ec>(nextState)
                                                ,std::get<latest_send_bytes_transferred>(nextState)
                                               );
            
                /// @todo use atomics to eliminate the global mutex lock for this object
                // lock the mutex to communicate with the user thread
                // if it cannot lock, simply send the previous message
                // again
                if (ptrMutex_.try_lock())
                {
                    
                    //////////////////////////////////////////////
                    // right now this is the state of everything:
                    //////////////////////////////////////////////
                    //
                    // Always Valid:
                    //
                    //     nextState: valid, contains the latest update
                    //     latestCommandBackup: should always be valid (though hasCommand might be false)
                    //
                    // Either Valid or Null:
                    //    latestStateForUser_ : null if the user took data out, valid otherwise
                    //    newCommandForDriver_: null if there is no new command data from the user, vaild otherwise
                    
                    
                    // 1) set the outgoing latest state for the user to pick up
                    //    latestStateForUser_ is finalized
                    std::swap(latestStateForUser_, nextState);
                    
                    // 2) get a new incoming command if available and set incoming command variable to null
                    if(std::get<latest_receive_monitor_state>(newCommandForDriver_))
                    {
                       // 3) back up the new incoming command
                       // latestCommandBackup is finalized, newCommandForDriver_ needs to be cleared out
                       std::swap(latestCommandBackup,newCommandForDriver_);
                    
                       // nextState may be null
                       if(!std::get<latest_receive_monitor_state>(nextState))
                       {
                          nextState = std::move(newCommandForDriver_);
                       }
                       else if(!(spareStates_.size()==spareStates_.capacity()))
                       {
                          spareStates_.emplace_back(std::move(newCommandForDriver_));
                       }
                       else
                       {
                         std::get<latest_receive_monitor_state>(newCommandForDriver_).reset();
                       }
                    }
                    
                    // finalized: latestStateForUser_, latestCommandBackup,  newCommandForDriver_ is definitely null
                    // issues to be resolved:
                    // nextState: may be null right now, and it should be valid
                    // newCommandForDriver_: needs to be cleared with 100% certainty
                    BOOST_VERIFY(spareStates_.size()>0);
                    
                    if(
                       !std::get<latest_receive_monitor_state>(nextState)
                       && spareStates_.size()
                      )
                    {
                      // move the last element out and shorten the vector
                      nextState = std::move(*(spareStates_.end()-1));
                      spareStates_.pop_back();
                    }
                    
                    BOOST_VERIFY(std::get<latest_receive_monitor_state>(nextState));
                    
                    KUKA::FRI::ClientData& nextClientData = *std::get<latest_receive_monitor_state>(nextState);
                    KUKA::FRI::ClientData& latestClientData = *std::get<latest_receive_monitor_state>(latestStateForUser_);
                    
                    // copy essential data from latestStateForUser_ to nextState
                    nextClientData.lastState            = latestClientData.lastState;
                    nextClientData.sequenceCounter      = latestClientData.sequenceCounter;
                    nextClientData.lastSendCounter      = latestClientData.lastSendCounter;
                    nextClientData.expectedMonitorMsgID = latestClientData.expectedMonitorMsgID;
                    
                    // copy command from latestCommandBackup to nextState aka nextClientData
                    KUKA::FRI::ClientData& latestCommandBackupClientData = *std::get<latest_receive_monitor_state>(latestCommandBackup);
                    set(nextClientData.commandMsg, latestCommandBackupClientData.commandMsg);
                    
                    
                    // if there are no error codes and we have received data,
                    // then we can consider the connection established!
                    /// @todo perhaps data should always send too?
                    if(!std::get<latest_receive_ec>(nextState) &&
                       !std::get<latest_send_ec>(nextState)    &&
                       std::get<latest_receive_bytes_transferred>(nextState)
                      )
                    {
                        isConnectionEstablished_ = true;
                    }
                
                    ptrMutex_.unlock();
                }
            
            
            }
           
       } catch(...) {
            // transport the exception to the main thread in a safe manner
            exceptionPtr = std::current_exception();
            m_shouldStop = true;
            isConnectionEstablished_ = false;
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
    
    typedef std::tuple
            <
                std::shared_ptr<KUKA::FRI::ClientData>
               ,boost::system::error_code
               ,std::size_t
               ,boost::system::error_code
               ,std::size_t
            >
    LatestState;
    
    /// Creates a default LatestState Object
    static LatestState
    make_LatestState(std::shared_ptr<KUKA::FRI::ClientData>& clientData)
    {
      return std::make_tuple(clientData,boost::system::error_code(), std::size_t(),boost::system::error_code(), std::size_t());
    }
    
    /// creates a shared_ptr to KUKA::FRI::ClientData with all command message status explicitly set to false
    /// @post std::shared_ptr<KUKA::FRI::ClientData> will be non-null
    static std::shared_ptr<KUKA::FRI::ClientData>
    make_shared_valid_ClientData(std::shared_ptr<KUKA::FRI::ClientData>& friData)
    {
        if(friData.get() == nullptr)
        {
          friData = std::make_shared<KUKA::FRI::ClientData>(KUKA::LBRState::NUM_DOF);
          // there is no commandMessage data on a new object
          friData->resetCommandMessage();
        }
    
        return friData;
    }
    
    static std::shared_ptr<KUKA::FRI::ClientData>
    make_shared_valid_ClientData()
    {
      std::shared_ptr<KUKA::FRI::ClientData> friData;
      return make_shared_valid_ClientData(friData);
    }
    
    /// Initialize valid shared ptr to LatestState object with a valid allocated friData
    static LatestState
    make_valid_LatestState(std::shared_ptr<KUKA::FRI::ClientData>& friData)
    {
        if(!friData) friData = make_shared_valid_ClientData();
        
        return make_LatestState(friData);
    }
    
    static LatestState
    make_valid_LatestState()
    {
      std::shared_ptr<KUKA::FRI::ClientData> friData;
      return make_valid_LatestState(friData);
    }
    
    Params params_;

    /// @todo replace with unique_ptr
    /// the latest state we have available to give to the user
    LatestState latestStateForUser_;
    LatestState newCommandForDriver_;
    
    /// should always be valid, never null
    boost::container::static_vector<LatestState,2> spareStates_;
    
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
    std::unique_ptr<std::thread> driver_threadP_;
    boost::mutex ptrMutex_;
};




    enum RobotMode {
      MODE_TEACH, MODE_SERVO, MODE_IDLE
    };


/// @brief Primary Kuka FRI driver, only talks over realtime network FRI KONI ethernet port
///
/// @todo support generic read/write
/// @todo ensure commands stay within machine limits to avoid lockup
/// @todo reset and resume if lockup occurs whenever possible
class KukaFRIdriver : public std::enable_shared_from_this<KukaFRIdriver>, public KukaFRI {
    
public:

    using KukaFRI::ParamIndex;
    using KukaFRI::ThreadingRunMode;
    using KukaFRI::Params;
    using KukaFRI::defaultParams;


      KukaFRIdriver(Params params = defaultParams())
        : params_(params)
      {}
    
//      KukaFRIdriver(boost::asio::io_service& device_driver_io_service__,Params params = defaultParams())
//        :
//        device_driver_io_service(device_driver_io_service__),
//        params_(params)
//      {}
    

      void construct(){ construct(params_);}

      /// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
      /// @warning getting the ik group is optional, so it does not throw an exception
      void construct(Params params) {

        params_ = params;
        // keep driver threads from exiting immediately after creation, because they have work to do!
        device_driver_workP_.reset(new boost::asio::io_service::work(device_driver_io_service));
        
        


          kukaFRIClientDataDriverP_.reset(
              new grl::robot::arm::KukaFRIClientDataDriver(
                  device_driver_io_service,
                  std::make_tuple(
                      std::string(std::get<localhost >                  (params)),
                      std::string(std::get<localport >                  (params)),
                      std::string(std::get<remotehost>                  (params)),
                      std::string(std::get<remoteport>                  (params)),
                                  std::get<send_period_millisec>        (params) ,
                      grl::robot::arm::KukaFRIClientDataDriver::run_automatically
                      )
                  )

              );
      }



      const Params & getParams(){
        return params_;
      }


      ~KukaFRIdriver(){
        device_driver_workP_.reset();

        if(driver_threadP){
          device_driver_io_service.stop();
          driver_threadP->join();
        }
      }

    KukaState::joint_state getMaxVel()
    {
    
                    /// @todo make maxVel a parameter rather than hardcoded
                    /// @todo 0.1 just makes it move at a much smaller fraction of max speed. Properly integrate safety.
                    double secondsPerTick = (std::get<send_period_millisec>(params_)/1000.0);
                    // R820 velocity limits
                    //A1 - 85 °/s  == 1.483529864195 rad/s
                    //A2 - 85 °/s  == 1.483529864195 rad/s
                    //A3 - 100 °/s == 1.745329251994 rad/s
                    //A4 - 75 °/s  == 1.308996938996 rad/s
                    //A5 - 130 °/s == 2.268928027593 rad/s
                    //A6 - 135 °/s == 2.356194490192 rad/s
                    //A1 - 135 °/s == 2.356194490192 rad/s
                    KukaState::joint_state maxVel = {
                                                        1.483529864195*secondsPerTick,
                                                        1.483529864195*secondsPerTick,
                                                        1.745329251994*secondsPerTick,
                                                        1.308996938996*secondsPerTick,
                                                        2.268928027593*secondsPerTick,
                                                        2.356194490192*secondsPerTick,
                                                        2.356194490192*secondsPerTick
                                                        };
                    return maxVel;
    }
        

     /**
      * spin once 
      *
      */
     bool run_one(){
          // note: this one sends *and* receives the joint data!
          BOOST_VERIFY(kukaFRIClientDataDriverP_.get()!=nullptr);
          /// @todo use runtime calculation of NUM_JOINTS instead of constant
          if(!friData_) friData_ = std::make_shared<KUKA::FRI::ClientData>(KUKA::LBRState::NUM_DOF);
         
          bool haveNewData = false;

          static const std::size_t minimumConsecutiveSuccessesBeforeSendingCommands = 100;
          
                    KukaState::joint_state ipoJointPos;
                    KukaState::joint_state currentJointPos;
                    KukaState::joint_state diffToGoal;
                    KukaState::joint_state amountToMove;
                    KukaState::joint_state commandToSend;
                    KukaState::joint_state maxvel = getMaxVel();
                    
         

          // Set the FRI to the simulated joint positions
          if(this->m_haveReceivedRealDataCount > minimumConsecutiveSuccessesBeforeSendingCommands){
            boost::lock_guard<boost::mutex> lock(jt_mutex);
            switch (friData_->monitoringMsg.robotInfo.controlMode) {
              case ControlMode_POSITION_CONTROLMODE:
                    
                    // the current "holdposition" joint angles
                    grl::robot::arm::copy(friData_->monitoringMsg,std::back_inserter(currentJointPos),revolute_joint_angle_open_chain_state_tag());
                    
                    boost::transform ( armState.commandedPosition_goal, currentJointPos, std::back_inserter(diffToGoal), std::minus<double>());
                    
                    boost::transform(diffToGoal,maxvel,std::back_inserter(amountToMove), [&](double diff,double maxvel) { return boost::math::copysign(std::min(std::abs(diff),maxvel),diff); } );
                    boost::transform ( currentJointPos, amountToMove, std::back_inserter(commandToSend), std::plus<double>());
                    if(0) grl::robot::arm::set(friData_->commandMsg, commandToSend, grl::revolute_joint_angle_open_chain_command_tag());
                    std::cout << "commandToSend: " << commandToSend << "\n" << "currentJointPos: " << currentJointPos << "\n" << "amountToMove: " << amountToMove << "\n" << "maxVel: " << maxvel << "\n";
                break;
              case ControlMode_JOINT_IMPEDANCE_CONTROLMODE:
                grl::robot::arm::set(friData_->commandMsg, armState.commandedTorque, grl::revolute_joint_torque_open_chain_command_tag());
                break;
              case ControlMode_CARTESIAN_IMPEDANCE_CONTROLMODE:
                // not yet supported
                grl::robot::arm::set(friData_->commandMsg, armState.commandedCartesianWrenchFeedForward, grl::cartesian_wrench_command_tag());
                break;

              default:
                break;
            }
          }

          boost::system::error_code send_ec,recv_ec;
          std::size_t send_bytes, recv_bytes;
          // sync with device over network
          haveNewData = !kukaFRIClientDataDriverP_->update_state(friData_,recv_ec,recv_bytes,send_ec,send_bytes);
          m_attemptedCommunicationCount++;

          if(haveNewData)
          {
              // if there were problems sending commands, start by sending the current position
//            if(this->m_haveReceivedRealDataCount > minimumConsecutiveSuccessesBeforeSendingCommands-1)
//            {
//              boost::lock_guard<boost::mutex> lock(jt_mutex);
//              // initialize arm commands to current arm position
//              armState.clearCommands();
////              armState.commandedPosition.clear();
////              armState.commandedTorque.clear();
////              grl::robot::arm::copy(friData_->monitoringMsg, std::back_inserter(armState.commandedPosition), grl::revolute_joint_angle_open_chain_command_tag());
////              grl::robot::arm::copy(friData_->monitoringMsg, std::back_inserter(armState.commandedTorque)   , grl::revolute_joint_torque_open_chain_command_tag());
//            }
          
            m_attemptedCommunicationConsecutiveSuccessCount++;
            this->m_attemptedCommunicationConsecutiveFailureCount = 0;
            this->m_haveReceivedRealDataCount++;
          

            // We have the real kuka state read from the device now
            // update real joint angle data
            armState.position.clear();
            grl::robot::arm::copy(friData_->monitoringMsg, std::back_inserter(armState.position), grl::revolute_joint_angle_open_chain_state_tag());

            armState.torque.clear();
            grl::robot::arm::copy(friData_->monitoringMsg, std::back_inserter(armState.torque), grl::revolute_joint_torque_open_chain_state_tag());

          }
          else
          {
            m_attemptedCommunicationConsecutiveFailureCount++;
            std::cerr << "No new FRI data available, is an FRI application running on the Kuka arm? \n Total sucessful transfers: " << this->m_haveReceivedRealDataCount << "\n Total attempts: "<< m_attemptedCommunicationCount <<"\n Consecutive Failures: "<< m_attemptedCommunicationConsecutiveFailureCount<<"\n Consecutive Successes: "<<
            m_attemptedCommunicationConsecutiveSuccessCount <<"\n";
            m_attemptedCommunicationConsecutiveSuccessCount=0;
            /// @todo should the results of getlatest state even be possible to call without receiving real data? should the library change?
          }
          
          return haveNewData;
         
        }
    
        
 
     /**
      * \brief Set the joint positions for the current interpolation step.
      * 
      * This method is only effective when the client is in a commanding state.
      * @param state Object which stores the current state of the robot, including the command to send next
      * @param range Array with the new joint positions (in radians)
      * @param tag identifier object indicating that revolute joint angle commands should be modified
      */
   template<typename Range>
   void set(Range&& range, grl::revolute_joint_angle_open_chain_command_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       armState.clearCommands();
       boost::copy(range, std::back_inserter(armState.commandedPosition));
       boost::copy(range, std::back_inserter(armState.commandedPosition_goal));
    }
  
     /**
      * \brief Set the applied joint torques for the current interpolation step.
      * 
      * This method is only effective when the client is in a commanding state.
      * The ControlMode of the robot has to be joint impedance control mode. The
      * Client Command Mode has to be torque.
      * 
      * @param state Object which stores the current state of the robot, including the command to send next
      * @param torques Array with the applied torque values (in Nm)
      * @param tag identifier object indicating that the torqe value command should be modified
      */
   template<typename Range>
   void set(Range&& range, grl::revolute_joint_torque_open_chain_command_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       armState.clearCommands();
      boost::copy(range, armState.commandedTorque);
    }
 
   
     /**
      * \brief Set the applied wrench vector of the current interpolation step.
      * 
      * The wrench vector consists of:
      * [F_x, F_y, F_z, tau_A, tau_B, tau_C]
      * 
      * F ... forces (in N) applied along the Cartesian axes of the 
      * currently used motion center.
      * tau ... torques (in Nm) applied along the orientation angles 
      * (Euler angles A, B, C) of the currently used motion center.
      *  
      * This method is only effective when the client is in a commanding state.
      * The ControlMode of the robot has to be Cartesian impedance control mode. The
      * Client Command Mode has to be wrench.
      * 
      * @param state object storing the command data that will be sent to the physical device
      * @param range wrench Applied Cartesian wrench vector, in x, y, z, roll, pitch, yaw force measurments.
      * @param tag identifier object indicating that the wrench value command should be modified
      *
      * @todo perhaps support some specific more useful data layouts
      */
   template<typename Range>
   void set(Range&& range, grl::cartesian_wrench_command_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       armState.clearCommands();
      std::copy(range,armState.commandedCartesianWrenchFeedForward);
    }
    
   /// @todo should this exist? is it written correctly?
   void get(KukaState & state)
   {
     boost::lock_guard<boost::mutex> lock(jt_mutex);
     state = armState;
   }

      volatile std::size_t m_haveReceivedRealDataCount = 0;
      volatile std::size_t m_attemptedCommunicationCount = 0;
      volatile std::size_t m_attemptedCommunicationConsecutiveFailureCount = 0;
      volatile std::size_t m_attemptedCommunicationConsecutiveSuccessCount = 0;

      boost::asio::io_service device_driver_io_service;
      std::unique_ptr<boost::asio::io_service::work> device_driver_workP_;
      std::unique_ptr<std::thread> driver_threadP;
      std::shared_ptr<grl::robot::arm::KukaFRIClientDataDriver> kukaFRIClientDataDriverP_;

    private:

      KukaState armState;
      boost::mutex jt_mutex;

      Params params_;
      std::shared_ptr<KUKA::FRI::ClientData> friData_;

    };
    
 

  template<typename Range,typename T>
  static inline void set(KukaFRIdriver & state, Range&& range, T t) {
      state.set(range,t);
   }
}}} // namespace grl::robot::arm

#endif
