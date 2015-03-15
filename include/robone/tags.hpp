
#include <boost/geometry/core/tags.hpp>
#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/units/physical_dimensions/plane_angle.hpp>
#include <boost/units/physical_dimensions/torque.hpp>

namespace robone {
	
	struct joint_tag{};
	
	struct prismatic_joint_tag : joint_tag {};
	struct revolute_joint_tag : joint_tag {};
	
	
	
    
	struct state_tag{}; 
	struct command_tag{};
	struct external_state_tag{}; ///< @todo consider eliminating/replacing external state tag
	struct interpolated_state_tag{};  ///< @todo consider eliminating/replacing interpolated state tag
	
    
    struct device_state_tag : boost::geometry::multi_tag, state_tag {};
    struct device_command_tag : boost::geometry::multi_tag, command_tag {};
	
	// joint angle
	struct revolute_joint_angle_multi_state_tag : revolute_joint_tag, state_tag, boost::units::plane_angle_base_dimension, boost::geometry::multi_tag {};
	
	// interpolated joint angle
	struct revolute_joint_angle_interpolated_multi_state_tag : revolute_joint_tag, interpolated_state_tag, boost::units::plane_angle_base_dimension, boost::geometry::multi_tag {};
	
	// joint torque
	struct revolute_joint_torque_multi_state_tag : revolute_joint_tag, state_tag, boost::units::torque_dimension, boost::geometry::multi_tag {};
	
	// external joint torque
	struct revolute_joint_torque_external_multi_state_tag : revolute_joint_tag, external_state_tag, boost::units::torque_dimension, boost::geometry::multi_tag {};
	
	// commanded joint angle
	struct revolute_joint_angle_multi_command_tag : revolute_joint_tag, command_tag, boost::units::plane_angle_base_dimension, boost::geometry::multi_tag {};
	struct revolute_joint_torque_multi_command_tag : revolute_joint_tag, command_tag, boost::units::torque_dimension, boost::geometry::multi_tag {};
	
	/// @todo add single_tag_of implementation for all joint types @see boost/geometry/core/tags.hpp for reference.
}