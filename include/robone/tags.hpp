
#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/units/physical_dimensions/plane_angle.hpp>
#include <boost/units/physical_dimensions/torque.hpp>

namespace robone {
	
	struct joint_tag{};
	
	struct prismatic_joint_tag : joint_tag {};
	struct revolute_joint_tag : joint_tag {};
	
	
	
    
	struct state_tag{}; /// @todo consider moving to separate tag header
	struct command_tag{};
	struct external_state_tag{}; /// @todo consider eliding external and internal sensor tag
	struct interpolated_state_tag{};
	
	
	struct revolute_joint_angle_state_tag : revolute_joint_tag, state_tag, boost::units::plane_angle_base_dimension {};
	struct revolute_joint_angle_interpolated_state_tag : revolute_joint_tag, interpolated_state_tag, boost::units::plane_angle_base_dimension {};
	struct revolute_joint_torque_state_tag : revolute_joint_tag, state_tag, boost::units::torque_dimension {};
	struct revolute_joint_torque_external_state_tag : revolute_joint_tag, external_state_tag, boost::units::torque_dimension {};
	struct revolute_joint_angle_command_tag : revolute_joint_tag, command_tag, boost::units::plane_angle_base_dimension {};
	struct revolute_joint_torque_command_tag : revolute_joint_tag, command_tag, boost::units::torque_dimension {};
	
	
}