#ifndef _GRL_TAGS_HPP
#define _GRL_TAGS_HPP

#include <boost/geometry/core/tags.hpp>
#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/units/physical_dimensions/plane_angle.hpp>
#include <boost/units/physical_dimensions/torque.hpp>
#include <boost/units/physical_dimensions/force.hpp>
#include <boost/units/physical_dimensions/angular_velocity.hpp>
#include <boost/units/physical_dimensions/time.hpp>

namespace grl {

    struct time_step_tag : boost::units::time_base_dimension {};
	
	struct joint_tag{};
	
	struct prismatic_joint_tag : joint_tag {};
	struct revolute_joint_tag : joint_tag {};
	
	
	struct device_tag{};
    
	struct state_tag{}; 
	struct command_tag{};
    struct constraint_tag{}; ///< Represents a system contraint, such as a max velocity limit
	struct external_state_tag{}; ///< @todo consider eliminating/replacing external state tag
	struct interpolated_state_tag{};  ///< @todo consider eliminating/replacing interpolated state tag
    
    struct wrench_tag{}; // https://en.wikipedia.org/wiki/Screw_theory#Wrench
	
    /// @todo how close is this to a 1d linestring? Or maybe an nd linestring if you even represent dh params this way?
    struct open_chain_tag : boost::geometry::single_tag, state_tag {};
    
    struct device_state_tag : state_tag, device_tag {};
    struct device_command_tag : command_tag, state_tag, device_tag {};
	
	/// joint angle
	struct revolute_joint_angle_open_chain_state_tag : revolute_joint_tag,  boost::units::plane_angle_base_dimension, open_chain_tag {};
	
	/// interpolated joint angle
	struct revolute_joint_angle_interpolated_open_chain_state_tag : revolute_joint_tag, interpolated_state_tag, boost::units::plane_angle_base_dimension, open_chain_tag {};
    
	/// joint velocity
	struct revolute_joint_velocity_open_chain_state_tag : revolute_joint_tag,  boost::units::angular_velocity_dimension, open_chain_tag {};
    
    /// joint velocity constraint (ex: max velocity)
	struct revolute_joint_velocity_open_chain_state_constraint_tag : revolute_joint_velocity_open_chain_state_tag, constraint_tag{};
	
	/// joint torque
	struct revolute_joint_torque_open_chain_state_tag : revolute_joint_tag, boost::units::torque_dimension, open_chain_tag {};
	
	/// external joint torque, i.e. torques applied to an arm excluding those caused by the mass of the arm itself and gravity
	struct revolute_joint_torque_external_open_chain_state_tag : revolute_joint_tag, external_state_tag, boost::units::torque_dimension, open_chain_tag {};
    
    /// external forces, i.e. forces applied to an arm excluding those caused by the mass of the arm itself and gravity
    struct cartesian_external_force_tag : external_state_tag, boost::units::force_dimension, boost::geometry::cartesian_tag {};
	
	/// commanded joint angle
	struct revolute_joint_angle_open_chain_command_tag : revolute_joint_tag, command_tag, boost::units::plane_angle_base_dimension, open_chain_tag {};
	struct revolute_joint_torque_open_chain_command_tag : revolute_joint_tag, command_tag, boost::units::torque_dimension, open_chain_tag {};
	struct cartesian_wrench_command_tag : boost::geometry::single_tag, command_tag, boost::geometry::cartesian_tag {};
	/// @todo add single_tag_of implementation for all joint types @see boost/geometry/core/tags.hpp for reference.
}

#endif