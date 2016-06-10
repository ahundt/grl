/// @file tags.hpp
///
/// @brief defines tag dispatching types used throughout grl
///
/// http://www.boost.org/community/generic_programming.html#tag_dispatching
/// http://barendgehrels.blogspot.com/2010/10/tag-dispatching-by-type-tag-dispatching.html

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

    struct time_point_tag : boost::units::time_base_dimension {}; ///< single point in time relative to some start point
    struct time_duration_tag : boost::units::time_base_dimension {}; ///< used for periodic events

    struct time_step_tag : time_duration_tag {}; ///< time step between measurements

	struct joint_tag{}; // identifies a joint, such as on a robot arm

	struct prismatic_joint_tag : joint_tag {};
	struct revolute_joint_tag : joint_tag {};


	struct device_tag{}; ///< identifies a type of device

	struct state_tag{};   ///< identifies data representing the state of a sytem
	struct command_tag{}; ///< data for changing a system's physical state, such as joint angles to be sent to a robot arm.
  struct constraint_tag{}; ///< Represents a system contraint, such as a max velocity limit
	struct external_state_tag{}; ///< @todo consider eliminating/replacing external state tag
	struct interpolated_state_tag{};  ///< @todo consider eliminating/replacing interpolated state tag
  struct cart_stiffness_values{};
  struct cart_damping_values{};

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

  /// commanded time stamp, the time at which a command should be completed
  struct time_point_command_tag : time_point_tag, command_tag {};

  /// the time duration over which a command should be completed
  struct time_duration_command_tag : time_duration_tag, command_tag {};

	/// commanded joint angle
	struct revolute_joint_angle_open_chain_command_tag : revolute_joint_tag, command_tag, boost::units::plane_angle_base_dimension, open_chain_tag {};
	struct revolute_joint_torque_open_chain_command_tag : revolute_joint_tag, command_tag, boost::units::torque_dimension, open_chain_tag {};
	struct cartesian_wrench_command_tag : boost::geometry::single_tag, command_tag, boost::geometry::cartesian_tag {};
	/// @todo add single_tag_of implementation for all joint types @see boost/geometry/core/tags.hpp for reference.
}

#endif
