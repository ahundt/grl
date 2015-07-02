#ifndef _KUKA_FRI_
#define _KUKA_FRI_

#include <chrono>
#include <stdexcept>

#include <boost/range/adaptor/copied.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range.hpp>
#include <boost/geometry/core/access.hpp>
#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/units/physical_dimensions/plane_angle.hpp>
#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/container/static_vector.hpp>

// Kuka include files
#include "friClientIf.h"
#include "FRIMessages.pb.h"
#include "friCommandMessageEncoder.h"
#include "friMonitoringMessageDecoder.h"
#include "grl/tags.hpp"
#include "grl/KukaNanopb.hpp"

namespace KUKA {
	namespace LBRState {
		const int NUM_DOF = 7;
		const int LBRMONITORMESSAGEID = 0x245142;
	}
	namespace LBRCommand {
		// Following from Kuka friLBRCommand.cpp
		const int LBRCOMMANDMESSAGEID = 0x34001;
	}
} // namespace KUKA

namespace grl { namespace robot { 
	    
	
	namespace arm {
		
		namespace kuka {
				// Following from Kuka example program
				const int default_port_id = 30200;
				struct send_period{};
				struct receive_multiplier{};
		
			namespace detail {
    
                /// @todo replace with joint_iterator<tRepeatedDoubleArguments,T>()
				template<typename T, typename OutputIterator>
				void copyJointState(T values,OutputIterator it, bool dataAvailable = true){
					if(dataAvailable) std::copy(static_cast<double*>(static_cast<tRepeatedDoubleArguments*>(values)->value),static_cast<double*>(static_cast<tRepeatedDoubleArguments*>(values)->value)+KUKA::LBRState::NUM_DOF,it);
				}
				
			    /// @todo handle dataAvaliable = false case
				/// @todo support tRepeatedIntArguments, and perhaps const versions
				template<typename T>
				boost::iterator_range<T> get(T& values, bool dataAvailable = true){
				  return boost::iterator_range<T>(*values);
				}
				
			}
		} // namespace kuka
	
    
	/// copy measured joint angle to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, revolute_joint_angle_open_chain_state_tag){
 	   kuka::detail::copyJointState(monitoringMsg.monitorData.measuredJointPosition.value.arg,it,monitoringMsg.monitorData.has_measuredJointPosition);
	}
	
	/// copy interpolated commanded joint angles
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it,revolute_joint_angle_interpolated_open_chain_state_tag){
           kuka::detail::copyJointState(monitoringMsg.ipoData.jointPosition.value.arg,it,monitoringMsg.ipoData.has_jointPosition);
	}
	
	/// copy commanded joint angle to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, revolute_joint_angle_open_chain_command_tag){
		kuka::detail::copyJointState(monitoringMsg.monitorData.commandedJointPosition.value.arg,it, monitoringMsg.monitorData.has_commandedJointPosition);
    }
	
	
	/// copy measured joint torque to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, revolute_joint_torque_open_chain_state_tag){
           kuka::detail::copyJointState(monitoringMsg.monitorData.measuredTorque.value.arg,it, monitoringMsg.monitorData.has_measuredTorque);
	}
	
	/// copy measured external joint torque to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, revolute_joint_torque_external_open_chain_state_tag){
           kuka::detail::copyJointState(monitoringMsg.monitorData.externalTorque.value.arg,it, monitoringMsg.monitorData.has_externalTorque);
	}
	
	/// copy commanded  joint torque to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, revolute_joint_torque_open_chain_command_tag){
           kuka::detail::copyJointState(monitoringMsg.monitorData.commandedTorque.value.arg,it, monitoringMsg.monitorData.has_commandedTorque);
	}
	
	/// @todo consider using another default value, or perhaps boost::optional<Kuka::FRI::ESafetyState>?
	KUKA::FRI::ESafetyState get(const FRIMonitoringMessage& monitoringMsg, const KUKA::FRI::ESafetyState) {
		KUKA::FRI::ESafetyState state = KUKA::FRI::ESafetyState::NORMAL_OPERATION;
	    if (monitoringMsg.has_robotInfo) {
	        if (monitoringMsg.robotInfo.has_safetyState)
	            return static_cast<KUKA::FRI::ESafetyState>(monitoringMsg.robotInfo.safetyState);
	    }
		return state;
	}
	
	
	KUKA::FRI::EOperationMode get(const FRIMonitoringMessage& monitoringMsg, const KUKA::FRI::EOperationMode) {
		KUKA::FRI::EOperationMode state = KUKA::FRI::EOperationMode::TEST_MODE_1;
	    if (monitoringMsg.has_robotInfo) {
	        if (monitoringMsg.robotInfo.has_operationMode)
	            state = static_cast<KUKA::FRI::EOperationMode>(monitoringMsg.robotInfo.operationMode);
	    }
		return state;
	}
	
	
	KUKA::FRI::EControlMode get(const FRIMonitoringMessage& monitoringMsg, const KUKA::FRI::EControlMode) {
		KUKA::FRI::EControlMode state = KUKA::FRI::EControlMode::NO_CONTROL;
	    if (monitoringMsg.has_robotInfo) {
	        if (monitoringMsg.robotInfo.has_controlMode)
	            state = static_cast<KUKA::FRI::EControlMode>(monitoringMsg.robotInfo.controlMode);
	    }
		return state;
	}
	
	
	KUKA::FRI::EClientCommandMode get(const FRIMonitoringMessage& monitoringMsg, const KUKA::FRI::EClientCommandMode) {
		KUKA::FRI::EClientCommandMode state = KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE;
	    if (monitoringMsg.has_ipoData) {
	        if (monitoringMsg.ipoData.has_clientCommandMode)
	            state = static_cast<KUKA::FRI::EClientCommandMode>(monitoringMsg.ipoData.clientCommandMode);
	    }
		return state;
	}
	
	
	KUKA::FRI::EOverlayType get(const FRIMonitoringMessage& monitoringMsg, const KUKA::FRI::EOverlayType) {
		KUKA::FRI::EOverlayType state = KUKA::FRI::EOverlayType::NO_OVERLAY;
	    if (monitoringMsg.has_ipoData) {
	        if (monitoringMsg.ipoData.has_overlayType)
	            state = static_cast<KUKA::FRI::EOverlayType>(monitoringMsg.ipoData.overlayType);
	    }
		return state;
	}
	
	
	KUKA::FRI::EDriveState get(const FRIMonitoringMessage& _message, const KUKA::FRI::EDriveState)
	{
	   tRepeatedIntArguments *values =
			 (tRepeatedIntArguments *)_message.robotInfo.driveState.arg;
	   int firstState = (int)values->value[0];
	   for (int i=1; i<KUKA::LBRState::NUM_DOF; i++)
	   {
		  int state = (int)values->value[i];
		  if (state != firstState)
		  {
			 return KUKA::FRI::EDriveState::TRANSITIONING;
		  }
	   }
	   return (KUKA::FRI::EDriveState)firstState;
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
	
	KUKA::FRI::ESessionState get(const FRIMonitoringMessage& monitoringMsg, const KUKA::FRI::ESessionState){
		KUKA::FRI::ESessionState KukaSessionState = KUKA::FRI::ESessionState::IDLE;
	    if (monitoringMsg.has_connectionInfo) {
	        KukaSessionState = static_cast<KUKA::FRI::ESessionState>(monitoringMsg.connectionInfo.sessionState);
	    }
		return KukaSessionState;
	}
	
	KUKA::FRI::EConnectionQuality get(const FRIMonitoringMessage& monitoringMsg, const KUKA::FRI::EConnectionQuality){
		KUKA::FRI::EConnectionQuality KukaQuality = KUKA::FRI::EConnectionQuality::POOR;
	    if (monitoringMsg.has_connectionInfo) {
	        KukaQuality = static_cast<KUKA::FRI::EConnectionQuality>(monitoringMsg.connectionInfo.quality);
	    }
		return KukaQuality;
	}
	
	uint32_t get(const FRIMonitoringMessage& monitoringMsg, const kuka::send_period){
		uint32_t KukaSendPeriod = 0;
	    if (monitoringMsg.has_connectionInfo) {
	        if (monitoringMsg.connectionInfo.has_sendPeriod)
	            KukaSendPeriod = monitoringMsg.connectionInfo.sendPeriod;
	    }
		return KukaSendPeriod;
	}
	
	std::size_t get(const FRIMonitoringMessage& monitoringMsg,const kuka::receive_multiplier){
		std::size_t KukaReceiveMultiplier = 0;
	    if (monitoringMsg.has_connectionInfo) {
	        if (monitoringMsg.connectionInfo.has_receiveMultiplier)
	            KukaReceiveMultiplier = monitoringMsg.connectionInfo.receiveMultiplier;
	    }
		return KukaReceiveMultiplier;
	}
	
	std::chrono::time_point<std::chrono::high_resolution_clock> get(const FRIMonitoringMessage& monitoringMsg,const std::chrono::time_point<std::chrono::high_resolution_clock>){
		// defaults to the epoch
		std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
        if (monitoringMsg.monitorData.has_timestamp) {
            timestamp += std::chrono::seconds(monitoringMsg.monitorData.timestamp.sec) +
				         std::chrono::nanoseconds(monitoringMsg.monitorData.timestamp.nanosec);
        }
		return timestamp;
	}
        
    
      /**
       * \brief Set the joint positions for the current interpolation step.
       * 
       * This method is only effective when the client is in a commanding state.
       * @param range Array with the new joint positions (in radians)
       */
    template<typename Range>
    static inline void set(FRICommandMessage & state, Range&& range, grl::revolute_joint_angle_open_chain_command_tag) {
       state.has_commandData = true;
       state.commandData.has_jointPosition = true;
       tRepeatedDoubleArguments *dest =  (tRepeatedDoubleArguments*)state.commandData.jointPosition.value.arg;
       boost::copy(range, dest->value);
     }
     
      /**
       * \brief Set the applied joint torques for the current interpolation step.
       * 
       * This method is only effective when the client is in a commanding state.
       * The ControlMode of the robot has to be joint impedance control mode. The
       * Client Command Mode has to be torque.
       * 
       * @param torques Array with the applied torque values (in Nm)
       */
    template<typename Range>
    static inline void set(FRICommandMessage & state, Range&& range, grl::revolute_joint_torque_open_chain_command_tag) {
       state.has_commandData = true;
       state.commandData.has_jointTorque = true;
       tRepeatedDoubleArguments *dest =  (tRepeatedDoubleArguments*)state.commandData.jointTorque.value.arg;
       boost::copy(range, dest->value);
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
       * @param FRICommandMessage object storing the command data that will be sent to the physical device
       * @param range wrench Applied Cartesian wrench vector, in x, y, z, roll, pitch, yaw force measurments.
       *
       * @todo perhaps support some specific more useful data layouts
       * @note copies only the elements that will fit
       */
    template<typename Range>
    static inline void set(FRICommandMessage & state, Range&& range, grl::cartesian_wrench_command_tag) {
       state.has_commandData = true;
       state.commandData.has_cartesianWrenchFeedForward = true;
       std::copy_n(std::begin(range),std::min(std::distance(range),state.commandData.cartesianWrenchFeedForward.element_count), &state.commandData.cartesianWrenchFeedForward.element[0]);
     }
     
     
	
	
namespace kuka {
        
        

	
    /// kuka iiwa driver
    ///
    ///
    /// @internal
    ///   All async calls need to be wrapped in strand_.wrap(), this serializes the calls so there are no threading problems.
    /// @endinternal
	class iiwa : public std::enable_shared_from_this<iiwa>
    {
    public:
    struct MonitorState {
        MonitorState():monitoringMessage(), decoder(&monitoringMessage,KUKA::LBRState::NUM_DOF){}
        
        MonitorState(const MonitorState&) = delete;
        MonitorState& operator=(const MonitorState&) = delete;
        
        const FRIMonitoringMessage& get()const {return monitoringMessage;}
        
    private:
        FRIMonitoringMessage monitoringMessage;          //!< monitoring message struct
        KUKA::FRI::MonitoringMessageDecoder decoder;            //!< monitoring message decoder
        boost::container::static_vector<uint8_t,KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE> buf_;
        
        friend class iiwa;
    };
    
    struct CommandState {
        CommandState():
        commandMessage(),
        encoder(&commandMessage,KUKA::LBRState::NUM_DOF),
        buf_(0,buf_.capacity()){}
        
        CommandState(const CommandState&) = delete;
        CommandState& operator=(const CommandState&) = delete;
        
        FRICommandMessage& get() {return commandMessage;}
        
    private:
        FRICommandMessage commandMessage;          //!< command message struct
        KUKA::FRI::CommandMessageEncoder encoder;            //!< command message encoder
        boost::container::static_vector<uint8_t,KUKA::FRI::FRI_COMMAND_MSG_MAX_SIZE> buf_;
        
        friend class iiwa;
    };
    
    /// @todo make a simple constructor for those who don't care as much about asio details
    
    
	/// note that since a socket cannot be copied,
	/// you should initialize this object with
	/// @code
    /// boost::asio::io_service io_service;
	/// boost::asio::ip::udp::socket socket(io_service);
    /// // initialize socket state
	/// iiwa driver(std::move(socket));
    /// io_service.run();
	/// @endcode
	explicit iiwa(boost::asio::ip::udp::socket socket):
      sequenceCounter_(0),
      receivesSinceLastSendCounter_(0),
      strand_(socket.get_io_service()),
      socket_(std::move(socket))
      {}
    
		
	  /// Advanced function to receive data from the FRI. We recommend using async_update.
	  /// @param Handler is expected to be a fn with the signature f(boost::system::error_code, std::size_t bytes_transferred)
      ///
      /// @note The user is responsible for keeping the MonitorState valid for the duration of the asynchronous call
      ///
      /// Design philosopy: it is important that thread safety be maintained, so a boost::asio::strand is used to enforced serial calling of receive/send for this
	  template<typename Handler>
	  void async_receive(MonitorState& monitor, Handler handler){
	  
		  auto self(shared_from_this());
		  monitor.buf_.resize(monitor.buf_.capacity());
		  
		  socket_.async_receive_from(boost::asio::buffer(&monitor.buf_[0],monitor.buf_.size()),sender_endpoint_,
             /// @todo FIXME disabled strand_.wrap(
             //strand_.wrap( // serialize receive and send in a single thread
             [this,self,&monitor,handler](boost::system::error_code const ec, std::size_t bytes_transferred)
			 {
			   monitor.buf_.resize(bytes_transferred);
			   monitor.decoder.decode(reinterpret_cast<char*>(&(monitor.buf_[0])),bytes_transferred);
				 
             
               receivesSinceLastSendCounter_++;
               lastMonitorSequenceCounter_ = monitor.monitoringMessage.header.sequenceCounter;
               lastMonitorJointAngles_.clear();
               lastMonitorJointTorques_.clear();
               sessionState_ = get(monitor.get(),KUKA::FRI::ESessionState());
               receiveMultiplier_ = get(monitor.get(),receive_multiplier());
               copy(monitor.monitoringMessage, std::back_inserter(lastMonitorJointAngles_), revolute_joint_angle_open_chain_command_tag());
               copy(monitor.monitoringMessage, std::back_inserter(lastMonitorJointTorques_), revolute_joint_torque_open_chain_command_tag());
             
			   /// @todo make sure this matches up with the synchronous version and Dr. Kazanzides' version
			   /// @todo also wrap these calls in a strand so there aren't threading issues. That should still be fast enough, but it can be adjusted if further improvement is needed.
             
             
               /// @todo should we pass bytes_transferred?
               /// @todo FIXME
               handler(ec,bytes_transferred);
               //socket_.get_io_service().post(std::bind(handler,ec,bytes_transferred));
			  
			 }
             //)
             );
	  }
      
	  /// @brief Advanced function to send data to the FRI. We recommend using async_update.
      ///
	  /// @param Handler is expected to be a fn with the signature f(boost::system::error_code, std::size_t bytes_transferred)
      ///
      /// It is important to note that this functionality differs slightly from what kuka provides.
      /// The CommandState is expected to be set so that it has command data when you make the call.
      /// If you do not set the command data, it will be assumed that you simply wish to monitor.
      ///
      /// @todo should we pass bytes_transferred as well to f(boost::system::error_code,bytes_transferred)?
      /// @note the user is responsible for keeping the MonitorState valid for the duration of the asynchronous call
      template<typename Handler>
      void async_send(CommandState& command, Handler handler){
          /// @todo convert commandState_ to a parameter
          // Check whether to send a response
          
		  auto self(shared_from_this());
		  command.buf_.resize(command.buf_.capacity());

// porbably not worth the time to post this simple function
//          socket_.get_io_service().post(
//            /// @todo FIXME strand wrap disabled
//            //strand_.wrap( // serialize receive and send in a single thread
//            [this,self,&command,handler](){
              if (isCommandReadyToSend()){
                  command.buf_.resize(command.buf_.capacity());
                  std::size_t buf_size = encode(command);
                  command.buf_.resize(buf_size);
                  
                  auto self(shared_from_this());
                  
                  socket_.async_send_to(
                      boost::asio::buffer(&command.buf_[0],buf_size),sender_endpoint_,
                      strand_.wrap( // serialize receive and send in a single thread
                      [this,self,&command,handler](boost::system::error_code const ec, std::size_t bytes_transferred)
                      {
                          handler(ec,bytes_transferred);
                      }));
              } else {
                // didn't need to send data at this time
                // there is no error and no bytes were transferred
                handler(boost::system::error_code(),0);
              }
//           }
           //)
//           );
      }
    
      /// This is the recommended mechanism to communicate with the FRI
      /// @todo may need to differentiate between async_receive failing and asying_send failing
      template<typename Handler>
      void async_update(MonitorState& monitor, CommandState& command, Handler handler){
      
         auto self(shared_from_this());
         
         // receive the latest state
         async_receive(monitor,
           // strand_.wrap( /// @todo FIXME wrap this call
           [this,&command,self,handler](boost::system::error_code const ec, std::size_t bytes_transferred){
             if(!ec && isCommandReadyToSend()){
               // send the new command (if appropriate)
               async_send(command, std::bind(handler,ec,bytes_transferred,std::placeholders::_1,std::placeholders::_2));
             } else {
               handler(ec,bytes_transferred,boost::system::error_code(),0);
             }
           
           }/// ) /// @todo FIXME
         );
      }
    
	
	  boost::asio::ip::udp::socket& socket(){
	    return socket_;
	  }
    
    /// @todo provide a way for users to send commands (and not send commands) over FRI
    
	private:
    
    bool isCommandReadyToSend(){
       return receivesSinceLastSendCounter_ >= receiveMultiplier_;
    }
    
    std::size_t encode(CommandState& command){
          receivesSinceLastSendCounter_ = 0;
          sequenceCounter_++;
          command.commandMessage.header.sequenceCounter = sequenceCounter_;
          command.commandMessage.header.reflectedSequenceCounter = lastMonitorSequenceCounter_;
        
          // copy the monitor data if we are not in a commanding state
          // note that this differs slightly form the implementation kuka provides!
          if(!command.commandMessage.has_commandData || (sessionState_ != KUKA::FRI::COMMANDING_WAIT) || (sessionState_ != KUKA::FRI::COMMANDING_ACTIVE)) {
            set(command.commandMessage,lastMonitorJointAngles_,grl::revolute_joint_angle_open_chain_command_tag());
            set(command.commandMessage,lastMonitorJointTorques_,grl::revolute_joint_torque_open_chain_command_tag());
          }
          // copy current joint position to commanded position
          
          int buffersize = KUKA::FRI::FRI_COMMAND_MSG_MAX_SIZE;
          if (!command.encoder.encode(reinterpret_cast<char*>(&command.buf_[0]), buffersize))
              return 0;
        
          return buffersize;
      }
		
	    std::atomic<uint32_t> sequenceCounter_; //!< sequence counter for command messages
        std::atomic<uint32_t> lastMonitorSequenceCounter_;
	    std::atomic<uint32_t> receivesSinceLastSendCounter_; //!< steps since last send command
        std::atomic<uint32_t> receiveMultiplier_;
        std::atomic<KUKA::FRI::ESessionState> sessionState_;
        boost::container::static_vector<double,KUKA::LBRState::NUM_DOF> lastMonitorJointAngles_;
        boost::container::static_vector<double,KUKA::LBRState::NUM_DOF> lastMonitorJointTorques_;
        boost::asio::ip::udp::endpoint sender_endpoint_;
    
        boost::asio::io_service::strand strand_;
        boost::asio::ip::udp::socket socket_;
	};
    
	
    } // namespace kuka

        
    // unwrap the monitor state when calling get()
    template<typename T>
    T get(const kuka::iiwa::MonitorState & state, T&& t){
        return get(state.get(),std::forward<T>(t));
    }
    
    
    // unwrap the monitor state when calling copy()
    template<typename ...Params>
    void copy(kuka::iiwa::MonitorState &state, Params&&... params){
        copy(state.get(),std::forward<Params>(params)...);
    }
    
    
    
    // unwrap the monitor state when calling get()
    template<typename Range, typename Tag>
    void set(kuka::iiwa::CommandState & state, Range range, Tag tag){
        set(state.get(),range,tag);
    }


}}} // namespace grl::robot::arm



///////////////////////////////////////////////////////////////////////////////
// Register CoordinateBase as a model of the boost::geometry point concept.
// Registration does not add any dependencies
// until the templates are instantiated.
// @see boost.org/libs/geometry for details
//
// include is required for macro below:
// #include <boost/geometry/geometries/register/point.hpp>
// macro unrolled and modified to create registration below:
// BOOST_GEOMETRY_REGISTER_POINT_2D(nrec::spatial::CoordinateBase ,nrec::spatial::CoordinateBase::value_type,cs::cartesian, operator[](0), operator[](1) )
///////////////////////////////////////////////////////////////////////////////
namespace mpl_ {template< int N > struct int_;} // forward declaration
namespace boost {
// _MSC_VER is a workaround for MSVC2010
// where multiple using declarations cause
// multiple definition error code C2874
#ifdef _MSC_VER
namespace mpl {template< int N > struct int_;}
#else // _MSC_VER
namespace mpl { using ::mpl_::int_; }
#endif // _MSC_VER


namespace geometry {
namespace cs { struct cartesian; } // forward declaration
struct box_tag; // forward declaration
struct point_tag; // forward declaration

namespace traits {
    template<typename T, typename Enable> struct tag; // forward declaration
    template<typename T, typename Enable> struct dimension; // forward declaration
    template<typename T, typename Enable> struct coordinate_type; // forward declaration
    template<typename T, typename Enable> struct coordinate_system; // forward declaration
    template <typename Geometry, std::size_t Dimension, typename Enable > struct access; // forward declaration
	
    template<typename Enable> struct tag<FRIMonitoringMessage, Enable > { typedef grl::device_state_tag type; };
    template<typename Enable> struct dimension<FRIMonitoringMessage, Enable > : boost::mpl::int_<KUKA::LBRState::NUM_DOF> {};
    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };

	
	
//    template<typename GeometryTag,typename Enable> struct coordinate_type<FRIMonitoringMessage, GeometryTag, Enable >;
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_angle_open_chain_state_tag             , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_angle_interpolated_open_chain_state_tag, Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_torque_open_chain_state_tag            , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_torque_external_open_chain_state_tag   , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_angle_open_chain_command_tag           , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_torque_open_chain_command_tag          , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
	
    template<typename Enable> struct access<FRIMonitoringMessage , KUKA::LBRState::NUM_DOF, Enable>
    {
	    // angle
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,grl::revolute_joint_angle_open_chain_state_tag) {
				return grl::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.monitorData.commandedJointPosition.value.arg),monitoringMsg.monitorData.has_measuredJointPosition);
		}
		
		// interpolated angle
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,grl::revolute_joint_angle_interpolated_open_chain_state_tag) {
				return grl::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.ipoData.jointPosition.value.arg), monitoringMsg.ipoData.has_jointPosition);
		}
		
		// torque
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,grl::revolute_joint_torque_open_chain_state_tag) {
				return grl::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.monitorData.measuredTorque.value.arg),monitoringMsg.monitorData.has_measuredTorque);
		}
		
		// external torque
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,grl::revolute_joint_torque_external_open_chain_state_tag) {
				return grl::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.monitorData.externalTorque.value.arg),monitoringMsg.monitorData.has_externalTorque);
		}
		
		// commanded angle
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,grl::revolute_joint_angle_open_chain_command_tag) {
				return grl::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.ipoData.jointPosition.value.arg),monitoringMsg.ipoData.has_jointPosition);
		}
		
		// commanded torque
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,grl::revolute_joint_torque_open_chain_command_tag) {
				return grl::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.monitorData.commandedTorque.value.arg), monitoringMsg.monitorData.has_commandedTorque);
		}
		
        //static inline void set(nrec::spatial::Coordinate<Dim,T,U> & p, typename nrec::spatial::Coordinate<Dim,T,U>::value_type const& value) { p.operator[](Index) = value; }
		
		
		
		
		
		
		
		
//	    // angle
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_angle_open_chain_state_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_angle_open_chain_state_tag) {
//				return grl::robot::arm::kuka::get(monitoringMsg.monitorData.commandedJointPosition.value.arg,monitoringMsg.monitorData.has_measuredJointPosition);
//		}
//		
//		// interpolated angle
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_angle_interpolated_open_chain_state_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_angle_interpolated_open_chain_state_tag) {
//				return grl::robot::arm::kuka::get(monitoringMsg.ipoData.jointPosition.value.arg, monitoringMsg.ipoData.has_jointPosition);
//		}
//		
//		// torque
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_torque_open_chain_state_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_torque_open_chain_state_tag) {
//				return grl::robot::arm::kuka::get(monitoringMsg.monitorData.measuredTorque.value.arg,monitoringMsg.monitorData.has_measuredTorque);
//		}
//		
//		// external torque
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_torque_external_open_chain_state_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_torque_external_open_chain_state_tag) {
//				return grl::robot::arm::kuka::get(monitoringMsg.monitorData.externalTorque.value.arg,monitoringMsg.monitorData.has_externalTorque);
//		}
//		
//		// commanded angle
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_angle_open_chain_command_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_angle_open_chain_command_tag) {
//				return grl::robot::arm::kuka::get(monitoringMsg.ipoData.jointPosition.value.arg,monitoringMsg.ipoData.has_jointPosition);
//		}
//		
//		// commanded torque
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_torque_open_chain_command_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_torque_open_chain_command_tag) {
//				return grl::robot::arm::kuka::get(monitoringMsg.monitorData.commandedTorque.value.arg, monitoringMsg.monitorData.has_commandedTorque);
//		}
//		
    };
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    template<typename Enable> struct tag<FRICommandMessage, Enable > { typedef grl::device_command_tag type; };
    template<typename Enable> struct dimension<FRICommandMessage, Enable > : boost::mpl::int_<1> {}; // each joint is 1 dimensional
    template<typename Enable> struct coordinate_type<FRICommandMessage, Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    
    
    
    //    template<typename GeometryTag,typename Enable> struct coordinate_type<FRICommandMessage, GeometryTag, Enable >;
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_angle_open_chain_state_tag             , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_angle_interpolated_open_chain_state_tag, Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_torque_open_chain_state_tag            , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_torque_external_open_chain_state_tag   , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_angle_open_chain_command_tag           , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_torque_open_chain_command_tag          , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    
    template<typename Enable> struct access<FRICommandMessage , KUKA::LBRState::NUM_DOF, Enable>
    {
    // angle
//    const typename coordinate_type<FRICommandMessage,Enable>::type
//    get(FRICommandMessage  const& monitoringMsg,grl::revolute_joint_angle_open_chain_state_tag) {
//        return grl::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.monitorData.commandedJointPosition.value.arg),monitoringMsg.monitorData.has_measuredJointPosition);
//    }
    
    template<typename Range>
    static inline void set(FRICommandMessage & state, Range&& range, grl::revolute_joint_angle_open_chain_command_tag) {
       state.has_commandData = true;
       state.commandData.has_jointPosition = true;
       tRepeatedDoubleArguments *dest =  (tRepeatedDoubleArguments*)state.commandData.jointPosition.value.arg;
       boost::copy(range, dest->value);
     }
    
    
    
    };
    

}}} // boost::geometry::traits




#endif

