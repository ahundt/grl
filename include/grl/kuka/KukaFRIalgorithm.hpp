#ifndef _KUKA_FRI_ALGORITHM
#define _KUKA_FRI_ALGORITHM

#include <chrono>
#include <stdexcept>
#include <algorithm>

#include <boost/range/adaptor/copied.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/distance.hpp>
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
#include "grl/kuka/KukaNanopb.hpp"
#include "grl/kuka/Kuka.hpp"

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
                
               /**
                * copies double data from a _CartesianVector to
                * an Output iterator which is assumed to contain doubles
                * as well, for example std::back_inserter(std::vector<double>)
                * The wrench vector consists of:
                * [F_x, F_y, F_z, tau_A, tau_B, tau_C]
                *
                * F ... forces (in N) applied along the Cartesian axes of the
                * currently used motion center.
                * tau ... torques (in Nm) applied along the orientation angles
                * (Euler angles A, B, C) of the currently used motion center.
                template<typename T, typename OutputIterator>
                */
                template<typename T, typename OutputIterator>
                void copyCartesianState(T values,OutputIterator it, bool dataAvailable = true){
                    // min of element_count and 6 to prevent buffer overflow attack
                    if(dataAvailable) std::copy(&values.element[0],&values.element[0]+std::min(values.element_count,static_cast<std::size_t>(6)),it);
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
        
    /// copy measured external force to output iterator
    /// note that this has a strange layout and actually contains
    /// an array of values for cartesian space.
    /// order is from the kuka documentation:
    ///
    /// The wrench vector consists of: [F_x, F_y, F_z, tau_A, tau_B, tau_C]
    ///
    /// F ... forces (in N) applied along the Cartesian axes of the currently used
    /// motion center. tau ... torques (in Nm) applied along the orientation angles
    /// (Euler angles A, B, C) of the currently used motion center.
    template<typename OutputIterator>
    void copy(const FRIMonitoringMessage& monitoringMsg, OutputIterator it, cartesian_external_force_tag){
            kuka::detail::copyCartesianState(monitoringMsg.monitorData.externalForce,it, monitoringMsg.monitorData.has_externalForce);
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
	
    /// @brief Get FRI UDP packet time step (1-5ms) in milliseconds between each device sync
    ///
    /// "time step" is also known as "time duration", "milliseconds per tick", or "send period"
    /// gets the time duration between when udp packets are expected to be sent in milliseconds
	uint32_t get(const FRIMonitoringMessage& monitoringMsg, const grl::time_step_tag){
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
       * @param state Object which stores the current state of the robot, including the command to send next
       * @param range Array with the new joint positions (in radians)
       * @param tag identifier object indicating that revolute joint angle commands should be modified
       */
    template<typename Range>
    static inline void set(FRICommandMessage & state, Range&& range, grl::revolute_joint_angle_open_chain_command_tag) {
       if(boost::size(range))
       {
           state.has_commandData = true;
           state.commandData.has_jointPosition = true;
           tRepeatedDoubleArguments *dest =  (tRepeatedDoubleArguments*)state.commandData.jointPosition.value.arg;
           boost::copy(range, dest->value);
       }
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
    static inline void set(FRICommandMessage & state, Range&& range, grl::revolute_joint_torque_open_chain_command_tag) {
       if(boost::size(range))
       {
           state.has_commandData = true;
           state.commandData.has_jointTorque = true;
           tRepeatedDoubleArguments *dest =  (tRepeatedDoubleArguments*)state.commandData.jointTorque.value.arg;
           boost::copy(range, dest->value);
       }
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
       * @param tag identifier object indicating that the wrench value command should be modified
       *
       * @todo perhaps support some specific more useful data layouts
       * @note copies only the elements that will fit
       */
    template<typename Range>
    static inline void set(FRICommandMessage & state, Range&& range, grl::cartesian_wrench_command_tag) {
       if(boost::size(range))
       {
           state.has_commandData = true;
           state.commandData.has_cartesianWrenchFeedForward = true;
           std::copy_n(std::begin(range),std::min(boost::size(range),state.commandData.cartesianWrenchFeedForward.element_count), &state.commandData.cartesianWrenchFeedForward.element[0]);
       }
     }
     
     
    /// set the left destination FRICommandMessage state to be equal to the right source FRICommandMessage state
    static inline void set(FRICommandMessage & state, const FRICommandMessage& sourceState, grl::command_tag) {
       state.has_commandData = sourceState.has_commandData;
       
       // cartesianWrench
       state.commandData.has_cartesianWrenchFeedForward = state.commandData.has_cartesianWrenchFeedForward;
       std::copy_n(&state.commandData.cartesianWrenchFeedForward.element[0],std::min(state.commandData.cartesianWrenchFeedForward.element_count,sourceState.commandData.cartesianWrenchFeedForward.element_count), &state.commandData.cartesianWrenchFeedForward.element[0]);
       
       // for joint copying
       tRepeatedDoubleArguments *dest;
       tRepeatedDoubleArguments *source;
       
       
       
       // jointTorque
       state.commandData.has_jointTorque = state.commandData.has_jointTorque;
       dest   =  (tRepeatedDoubleArguments*)state.commandData.jointTorque.value.arg;
       source =  (tRepeatedDoubleArguments*)sourceState.commandData.jointTorque.value.arg;
       std::copy_n(source->value,std::min(source->size,dest->size), dest->value);
       
       
       // jointPosition
       state.commandData.has_jointPosition = state.commandData.has_jointPosition;
       dest   =  (tRepeatedDoubleArguments*)state.commandData.jointPosition.value.arg;
       source =  (tRepeatedDoubleArguments*)sourceState.commandData.jointPosition.value.arg;
       std::copy_n(source->value,std::min(source->size,dest->size), dest->value);
       
     }
     
     
    static inline void set(FRICommandMessage & state, const FRICommandMessage& sourceState) {
       set(state,sourceState, grl::command_tag());
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

