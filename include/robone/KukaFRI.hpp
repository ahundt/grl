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
	
	struct iiwaMonitorState {
	  iiwaMonitorState():monitoringMessage(), decoder(&monitoringMessage,KUKA::LBRState::NUM_DOF){}
	  
      iiwaMonitorState(const iiwaMonitorState&) = delete;
      iiwaMonitorState& operator=(const iiwaMonitorState&) = delete;
	 
	  const FRIMonitoringMessage& get(){return monitoringMessage;}
		
	  private:
	  FRIMonitoringMessage monitoringMessage;          //!< monitoring message struct
      KUKA::FRI::MonitoringMessageDecoder decoder;            //!< monitoring message decoder
	};
	
	struct iiwaCommandState {
	  iiwaCommandState():commandMessage(), encoder(&commandMessage,KUKA::LBRState::NUM_DOF){}
	  
      iiwaCommandState(const iiwaCommandState&) = delete;
      iiwaCommandState& operator=(const iiwaCommandState&) = delete;
	 
	  const FRICommandMessage& get(){return commandMessage;}
		
	  private:
	  FRICommandMessage commandMessage;          //!< monitoring message struct
      KUKA::FRI::CommandMessageEncoder encoder;            //!< monitoring message decoder
	};
} // namespace kuka
	
	
	// unwrap the monitor state when calling get()
	template<typename T>
	T get(kuka::iiwaMonitorState & state, T&& t){
	  return get(state.get(),std::forward<T>(t));
	}
	
	
	
	namespace kuka {
	
	
	
	class iiwa : std::enable_shared_from_this<iiwa>
	{
	
	explicit iiwa(boost::asio::ip::udp::socket socket):
	  monitor_buf(0,monitor_buf.capacity()),
	  command_buf(0,command_buf.capacity()),
      sequenceCounter(0),
      lastSendCounter(0),
	  socket_(std::move(socket)){}
	
	
	private:
	  std::unique_ptr<iiwaMonitorState> monitorStateP;
	  std::unique_ptr<iiwaCommandState> commandStateP;
	  boost::container::static_vector<uint8_t,KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE> monitor_buf;
	  boost::container::static_vector<uint8_t,KUKA::FRI::FRI_COMMAND_MSG_MAX_SIZE> command_buf;
	  uint32_t sequenceCounter; //!< sequence counter for command messages
	  uint32_t lastSendCounter; //!< steps since last send command
		
	  boost::asio::ip::udp::socket socket_;
	};
	
	
	}

}}} /// namespace robone::robot::arm



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

}}} // boost::geometry::traits




#endif

