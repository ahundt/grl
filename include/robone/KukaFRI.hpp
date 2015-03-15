#ifndef _KUKA_FRI_
#define _KUKA_FRI_

#include <chrono>
#include <stdexcept>

#include <boost/geometry/core/access.hpp>
#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/units/physical_dimensions/plane_angle.hpp>
#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/container/static_vector.hpp>

// Kuka include files
#include "friClientIf.h"
#include "friMessages.pb.h"
#include "friCommandMessageEncoder.h"
#include "friMonitoringMessageDecoder.h"
#include "robone/tags.hpp"
#include "robone/KukaNanopb.hpp"

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
		}
	
    
	/// copy measured joint angle to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, revolute_joint_angle_multi_state_tag){
 	   kuka::detail::copyJointState(monitoringMsg.monitorData.measuredJointPosition.value.arg,it,monitoringMsg.monitorData.has_measuredJointPosition);
	}
	
	/// copy interpolated commanded joint angles
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it,revolute_joint_angle_interpolated_multi_state_tag){
           kuka::detail::copyJointState(monitoringMsg.ipoData.jointPosition.value.arg,it,monitoringMsg.ipoData.has_jointPosition);
	}
	
	/// copy commanded joint angle to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, revolute_joint_angle_multi_command_tag){
		kuka::detail::copyJointState(monitoringMsg.monitorData.commandedJointPosition.value.arg,it, monitoringMsg.monitorData.has_commandedJointPosition);
    }
	
	
	/// copy measured joint torque to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, revolute_joint_torque_multi_state_tag){
           kuka::detail::copyJointState(monitoringMsg.monitorData.measuredTorque.value.arg,it, monitoringMsg.monitorData.has_measuredTorque);
	}
	
	/// copy measured external joint torque to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, revolute_joint_torque_external_multi_state_tag){
           kuka::detail::copyJointState(monitoringMsg.monitorData.externalTorque.value.arg,it, monitoringMsg.monitorData.has_externalTorque);
	}
	
	/// copy commanded  joint torque to output iterator
	template<typename OutputIterator>
	void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, revolute_joint_torque_multi_command_tag){
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
        
    
	
	
namespace kuka {
        
        

	
    /// kuka iiwa driver
	class iiwa : std::enable_shared_from_this<iiwa>
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
        command_buf_(0,command_buf_.capacity()){}
        
        CommandState(const CommandState&) = delete;
        CommandState& operator=(const CommandState&) = delete;
        
        const FRICommandMessage& get() const {return commandMessage;}
        
    private:
        FRICommandMessage commandMessage;          //!< monitoring message struct
        KUKA::FRI::CommandMessageEncoder encoder;            //!< monitoring message decoder
        boost::container::static_vector<uint8_t,KUKA::FRI::FRI_COMMAND_MSG_MAX_SIZE> command_buf_;
        
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
      commandState_(new CommandState),
      sequenceCounter_(0),
      lastSendCounter_(0),
      receivedCommandForJointAngles_(false),
      strand_(socket.get_io_service()),
      socket_(std::move(socket))
      {}
		
		
	  /// @param Handler is expected to be a fn with the signature f(boost::system::error_code,std::unique_ptr<MonitorState>)
      ///
      /// Design philosopy: it is important that thread safety be maintained, so a boost::asio::strand is used to enforced serial calling of receive/send for this
	  template<typename Handler>
	  void async_receive(std::unique_ptr<MonitorState> monitor, Handler&& handler){
	  
		  auto self(shared_from_this());
		  monitor->buf_.resize(monitor->buf_.capacity());
		  
		  socket_.async_receive(boost::asio::buffer(&monitor->buf_[0],monitor->buf_.size()),
             strand_.wrap( // serialize receive and send in a single thread
		     std::bind([this,self](boost::system::error_code const ec, std::size_t bytes_transferred, Handler&& handler, std::unique_ptr<MonitorState>& moved_monitor)
			 {
			   moved_monitor->buf_.resize(bytes_transferred);
			   moved_monitor->decoder.decode(reinterpret_cast<char*>(&(moved_monitor->buf_[0])),bytes_transferred);
				 
             
               lastSendCounter_++;
               lastMonitorSequenceCounter_ = moved_monitor->monitoringMessage.header.sequenceCounter;
               copy(moved_monitor->monitoringMessage, lastMonitorJointAngles_.begin(), revolute_joint_angle_multi_command_tag());
             
             
             
			   /// @todo make sure this matches up with the synchronous version and Dr. Kazanzides' version
			   /// @todo need to update the state now that the data is received and set up a new data "send"
			   /// @todo however, some thought needs to go into when the async_send data is actually supplied, because the user needs to be able to send it.
			   /// @todo also wrap these calls in a strand so there aren't threading issues. That should still be fast enough, but it can be adjusted if further improvement is needed.
               if(commandState_) async_send();
             
               handler(ec,std::move(moved_monitor));
			   //moved_monitor->
			  
			 },std::move(monitor))));
	  }
	
	  boost::asio::ip::udp::socket& socket(){
	    return socket_;
	  }
    
    /// @todo provide a way for users to send commands over FRI
    
	private:
	
      /// @todo this should eventually run separately, and use a similar API to async_receive
      //template<typename Handler>
      void async_send(/*std::unique_ptr<CommandState> command, Handler&& handler*/){
          /// @todo convert commandState_ to a parameter
          // Check whether to send a response
          if (lastSendCounter_ >= receiveMultiplier_){
              commandState_->command_buf_.resize(commandState_->command_buf_.capacity());
              std::size_t buf_size = encode(*commandState_);
              commandState_->command_buf_.resize(buf_size);
              
              auto self(shared_from_this());
              
              socket_.async_send(boost::asio::buffer(&commandState_->command_buf_[0],buf_size),
                                 strand_.wrap( // serialize receive and send in a single thread
                                 [this,self](boost::system::error_code const ec, std::size_t bytes_transferred/*, Handler&& handler,*/)
                                 {
                                   /// @todo remove this when it actually goes to a real handler
                                     /// @todo actually send this to a real handler
                                     //handler(ec,bytes_transferred,std::move(moved_command));
                                     if(ec) std::cout << "iiwa send command error\n";
                                 }));
          }
      }
    
    std::size_t encode(CommandState& command){
          lastSendCounter_ = 0;
          sequenceCounter_++;
          command.commandMessage.header.sequenceCounter = sequenceCounter_;
          command.commandMessage.header.reflectedSequenceCounter = lastMonitorSequenceCounter_;
          
          // copy current joint position to commanded position
          command.commandMessage.has_commandData = true;
          command.commandMessage.commandData.has_jointPosition = true;
          tRepeatedDoubleArguments *dest  = (tRepeatedDoubleArguments*)command.commandMessage.commandData.jointPosition.value.arg;
          if (receivedCommandForJointAngles_ && (sessionState_ == KUKA::FRI::COMMANDING_WAIT) || (sessionState_ == KUKA::FRI::COMMANDING_ACTIVE))
              std::copy(lastMonitorJointAngles_.begin(),lastMonitorJointAngles_.end(),dest->value);
          else
              std::copy(lastCommandedJointAngles_.begin(),lastCommandedJointAngles_.end(),dest->value);
          
          int buffersize = KUKA::FRI::FRI_COMMAND_MSG_MAX_SIZE;
          if (!command.encoder.encode(reinterpret_cast<char*>(&command.command_buf_[0]), buffersize))
              return 0;
        
          return buffersize;
      }
		
	    /// @todo wrap everything here in a strand so there aren't threading issues
        std::unique_ptr<CommandState> commandState_;
	    uint32_t sequenceCounter_; //!< sequence counter for command messages
        uint32_t lastMonitorSequenceCounter_;
	    uint32_t lastSendCounter_; //!< steps since last send command
        uint32_t receiveMultiplier_;
        KUKA::FRI::ESessionState sessionState_;
        boost::container::static_vector<double,KUKA::LBRState::NUM_DOF> lastMonitorJointAngles_;
        bool receivedCommandForJointAngles_;
        boost::container::static_vector<double,KUKA::LBRState::NUM_DOF> lastCommandedJointAngles_;
    
        boost::asio::io_service::strand strand_;
        boost::asio::ip::udp::socket socket_;
	};
        
        
        // unwrap the monitor state when calling get()
        template<typename T>
        T get(const iiwa::MonitorState & state, T&& t){
            return get(state.get(),std::forward<T>(t));
        }
        
        
        // unwrap the monitor state when calling copy()
        template<typename ...Params>
        void copy(iiwa::MonitorState &state, Params&&... params){
            copy(state.get(),std::forward<Params>(params)...);
        }
	
    } // namespace kuka

}}} // namespace robone::robot::arm



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
	
    template<typename Enable> struct tag<FRIMonitoringMessage, Enable > { typedef robone::device_state_tag type; };
    template<typename Enable> struct dimension<FRIMonitoringMessage, Enable > : boost::mpl::int_<KUKA::LBRState::NUM_DOF> {};
    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };

	
	
//    template<typename GeometryTag,typename Enable> struct coordinate_type<FRIMonitoringMessage, GeometryTag, Enable >;
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_angle_multi_state_tag             , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_angle_interpolated_multi_state_tag, Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_torque_multi_state_tag            , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_torque_external_multi_state_tag   , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_angle_multi_command_tag           , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
//    template<typename Enable> struct coordinate_type<FRIMonitoringMessage, revolute_joint_torque_multi_command_tag          , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
	
    template<typename Enable> struct access<FRIMonitoringMessage , KUKA::LBRState::NUM_DOF, Enable>
    {
	    // angle
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,robone::revolute_joint_angle_multi_state_tag) {
				return robone::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.monitorData.commandedJointPosition.value.arg),monitoringMsg.monitorData.has_measuredJointPosition);
		}
		
		// interpolated angle
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,robone::revolute_joint_angle_interpolated_multi_state_tag) {
				return robone::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.ipoData.jointPosition.value.arg), monitoringMsg.ipoData.has_jointPosition);
		}
		
		// torque
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,robone::revolute_joint_torque_multi_state_tag) {
				return robone::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.monitorData.measuredTorque.value.arg),monitoringMsg.monitorData.has_measuredTorque);
		}
		
		// external torque
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,robone::revolute_joint_torque_external_multi_state_tag) {
				return robone::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.monitorData.externalTorque.value.arg),monitoringMsg.monitorData.has_externalTorque);
		}
		
		// commanded angle
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,robone::revolute_joint_angle_multi_command_tag) {
				return robone::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.ipoData.jointPosition.value.arg),monitoringMsg.ipoData.has_jointPosition);
		}
		
		// commanded torque
        const typename coordinate_type<FRIMonitoringMessage,Enable>::type
		get(FRIMonitoringMessage  const& monitoringMsg,robone::revolute_joint_torque_multi_command_tag) {
				return robone::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.monitorData.commandedTorque.value.arg), monitoringMsg.monitorData.has_commandedTorque);
		}
		
        //static inline void set(nrec::spatial::Coordinate<Dim,T,U> & p, typename nrec::spatial::Coordinate<Dim,T,U>::value_type const& value) { p.operator[](Index) = value; }
		
		
		
		
		
		
		
		
//	    // angle
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_angle_multi_state_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_angle_multi_state_tag) {
//				return robone::robot::arm::kuka::get(monitoringMsg.monitorData.commandedJointPosition.value.arg,monitoringMsg.monitorData.has_measuredJointPosition);
//		}
//		
//		// interpolated angle
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_angle_interpolated_multi_state_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_angle_interpolated_multi_state_tag) {
//				return robone::robot::arm::kuka::get(monitoringMsg.ipoData.jointPosition.value.arg, monitoringMsg.ipoData.has_jointPosition);
//		}
//		
//		// torque
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_torque_multi_state_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_torque_multi_state_tag) {
//				return robone::robot::arm::kuka::get(monitoringMsg.monitorData.measuredTorque.value.arg,monitoringMsg.monitorData.has_measuredTorque);
//		}
//		
//		// external torque
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_torque_external_multi_state_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_torque_external_multi_state_tag) {
//				return robone::robot::arm::kuka::get(monitoringMsg.monitorData.externalTorque.value.arg,monitoringMsg.monitorData.has_externalTorque);
//		}
//		
//		// commanded angle
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_angle_multi_command_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_angle_multi_command_tag) {
//				return robone::robot::arm::kuka::get(monitoringMsg.ipoData.jointPosition.value.arg,monitoringMsg.ipoData.has_jointPosition);
//		}
//		
//		// commanded torque
//        const typename coordinate_type<FRIMonitoringMessage,revolute_joint_torque_multi_command_tag,Enable>::type
//		get(FRIMonitoringMessage  const& monitoringMsg,revolute_joint_torque_multi_command_tag) {
//				return robone::robot::arm::kuka::get(monitoringMsg.monitorData.commandedTorque.value.arg, monitoringMsg.monitorData.has_commandedTorque);
//		}
//		
    };
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    template<typename Enable> struct tag<FRICommandMessage, Enable > { typedef robone::device_command_tag type; };
    template<typename Enable> struct dimension<FRICommandMessage, Enable > : boost::mpl::int_<KUKA::LBRState::NUM_DOF> {};
    template<typename Enable> struct coordinate_type<FRICommandMessage, Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    
    
    
    //    template<typename GeometryTag,typename Enable> struct coordinate_type<FRICommandMessage, GeometryTag, Enable >;
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_angle_multi_state_tag             , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_angle_interpolated_multi_state_tag, Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_torque_multi_state_tag            , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_torque_external_multi_state_tag   , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_angle_multi_command_tag           , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    //    template<typename Enable> struct coordinate_type<FRICommandMessage, revolute_joint_torque_multi_command_tag          , Enable > { typedef boost::iterator_range<repeatedDoubleArguments> type; };
    
    template<typename Enable> struct access<FRICommandMessage , KUKA::LBRState::NUM_DOF, Enable>
    {
    // angle
//    const typename coordinate_type<FRICommandMessage,Enable>::type
//    get(FRICommandMessage  const& monitoringMsg,robone::revolute_joint_angle_multi_state_tag) {
//        return robone::robot::arm::kuka::detail::get(*static_cast<tRepeatedDoubleArguments*>(monitoringMsg.monitorData.commandedJointPosition.value.arg),monitoringMsg.monitorData.has_measuredJointPosition);
//    }
    
    template<typename Range>
    static inline void set(FRICommandMessage & state, Range&& range, robone::revolute_joint_angle_multi_command_tag) {  }
    
    
    };
    

}}} // boost::geometry::traits




#endif

