/* Copyright 2015-2017 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is part of mc_rbdyn_urdf.
 *
 * mc_rbdyn_urdf is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * mc_rbdyn_urdf is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with mc_rbdyn_urdf.  If not, see <http://www.gnu.org/licenses/>.
 */
/**
 * THIS IS A TEMPORAL FILE, AFTER FINISHING THE TEST, REMOVE IT.
 * @CHUNTING
 */
#pragma once

#include <mc_rbdyn_urdf/urdf.h>
#include <boost/math/constants/constants.hpp>

#include "RBDyn/FK.h"


std::string XYZSarmUrdf(
R"(<robot name="iiwa14" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <material name="Black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="Blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="Green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="Grey">
    <color rgba="0.4 0.4 0.4 1.0"/>
  </material>
  <material name="Orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="Brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="Red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="White">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  <!-- Fix to world just for testing -->
  <link name="world"/>
  <!--joint between {parent} and link_0-->
  <joint name="world_iiwa_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="iiwa_link_0"/>
  </joint>
  <link name="iiwa_link_0">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.1 0 0.07"/>
      <mass value="5"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.06" iyz="0" izz="0.03"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/visual/link_0.stl"/>
      </geometry>
      <material name="Grey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/collision/link_0.stl"/>
      </geometry>
      <material name="Grey"/>
    </collision>
    <self_collision_checking>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <capsule length="0.25" radius="0.15"/>
      </geometry>
    </self_collision_checking>
  </link>
  <!-- joint between link_0 and link_1 -->
  <joint name="iiwa_joint_1" type="revolute">
    <parent link="iiwa_link_0"/>
    <child link="iiwa_link_1"/>
    <origin rpy="0 0 0" xyz="0 0 0.1575"/>
    <axis xyz="0 0 1"/>
    <limit effort="300" lower="-2.96705972839" upper="2.96705972839" velocity="10"/>
    <safety_controller k_position="100" k_velocity="2" soft_lower_limit="-2.93215314335" soft_upper_limit="2.93215314335"/>
    <dynamics damping="0.5"/>
  </joint>
  <link name="iiwa_link_1">
    <inertial>
      <origin rpy="0 0 0" xyz="0 -0.03 0.12"/>
      <mass value="4"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.09" iyz="0" izz="0.02"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/visual/link_1.stl"/>
      </geometry>
      <material name="Orange"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/collision/link_1.stl"/>
      </geometry>
      <material name="Orange"/>
    </collision>
  </link>
  <!-- joint between link_1 and link_2 -->
  <joint name="iiwa_joint_2" type="revolute">
    <parent link="iiwa_link_1"/>
    <child link="iiwa_link_2"/>
    <origin rpy="1.57079632679   0 3.14159265359" xyz="0 0 0.2025"/>
    <axis xyz="0 0 1"/>
    <limit effort="300" lower="-2.09439510239" upper="2.09439510239" velocity="10"/>
    <safety_controller k_position="100" k_velocity="2" soft_lower_limit="-2.05948851735" soft_upper_limit="2.05948851735"/>
    <dynamics damping="0.5"/>
  </joint>
  <link name="iiwa_link_2">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0003 0.059 0.042"/>
      <mass value="4"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.018" iyz="0" izz="0.044"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/visual/link_2.stl"/>
      </geometry>
      <material name="Orange"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/collision/link_2.stl"/>
      </geometry>
      <material name="Orange"/>
    </collision>
  </link>
  <!-- joint between link_2 and link_3 -->
  <joint name="iiwa_joint_3" type="revolute">
    <parent link="iiwa_link_2"/>
    <child link="iiwa_link_3"/>
    <origin rpy="1.57079632679 0 3.14159265359" xyz="0 0.2045 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="300" lower="-2.96705972839" upper="2.96705972839" velocity="10"/>
    <safety_controller k_position="100" k_velocity="2" soft_lower_limit="-2.93215314335" soft_upper_limit="2.93215314335"/>
    <dynamics damping="0.5"/>
  </joint>
  <link name="iiwa_link_3">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0.03 0.13"/>
      <mass value="3"/>
      <inertia ixx="0.08" ixy="0" ixz="0" iyy="0.075" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/visual/link_3.stl"/>
      </geometry>
      <material name="Orange"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/collision/link_3.stl"/>
      </geometry>
      <material name="Orange"/>
    </collision>
  </link>
  <!-- joint between link_3 and link_4 -->
  <joint name="iiwa_joint_4" type="revolute">
    <parent link="iiwa_link_3"/>
    <child link="iiwa_link_4"/>
    <origin rpy="1.57079632679 0 0" xyz="0 0 0.2155"/>
    <axis xyz="0 0 1"/>
    <limit effort="300" lower="-2.09439510239" upper="2.09439510239" velocity="10"/>
    <safety_controller k_position="100" k_velocity="2" soft_lower_limit="-2.05948851735" soft_upper_limit="2.05948851735"/>
    <dynamics damping="0.5"/>
  </joint>
  <link name="iiwa_link_4">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0.067 0.034"/>
      <mass value="2.7"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.029"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/visual/link_4.stl"/>
      </geometry>
      <material name="Orange"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/collision/link_4.stl"/>
      </geometry>
      <material name="Orange"/>
    </collision>
  </link>
  <!-- joint between link_4 and link_5 -->
  <joint name="iiwa_joint_5" type="revolute">
    <parent link="iiwa_link_4"/>
    <child link="iiwa_link_5"/>
    <origin rpy="-1.57079632679 3.14159265359 0" xyz="0 0.1845 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="300" lower="-2.96705972839" upper="2.96705972839" velocity="10"/>
    <safety_controller k_position="100" k_velocity="2" soft_lower_limit="-2.93215314335" soft_upper_limit="2.93215314335"/>
    <dynamics damping="0.5"/>
  </joint>
  <link name="iiwa_link_5">
    <inertial>
      <origin rpy="0 0 0" xyz="0.0001 0.021 0.076"/>
      <mass value="1.7"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.018" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/visual/link_5.stl"/>
      </geometry>
      <material name="Orange"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/collision/link_5.stl"/>
      </geometry>
      <material name="Orange"/>
    </collision>
  </link>
  <!-- joint between link_5 and link_6 -->
  <joint name="iiwa_joint_6" type="revolute">
    <parent link="iiwa_link_5"/>
    <child link="iiwa_link_6"/>
    <origin rpy="1.57079632679 0 0" xyz="0 0 0.2155"/>
    <axis xyz="0 0 1"/>
    <limit effort="300" lower="-2.09439510239" upper="2.09439510239" velocity="10"/>
    <safety_controller k_position="100" k_velocity="2" soft_lower_limit="-2.05948851735" soft_upper_limit="2.05948851735"/>
    <dynamics damping="0.5"/>
  </joint>
  <link name="iiwa_link_6">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0.0006 0.0004"/>
      <mass value="1.8"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.0036" iyz="0" izz="0.0047"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/visual/link_6.stl"/>
      </geometry>
      <material name="Orange"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/collision/link_6.stl"/>
      </geometry>
      <material name="Orange"/>
    </collision>
  </link>
  <!-- joint between link_6 and link_7 -->
  <joint name="iiwa_joint_7" type="revolute">
    <parent link="iiwa_link_6"/>
    <child link="iiwa_link_7"/>
    <origin rpy="-1.57079632679 3.14159265359 0" xyz="0 0.081 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="300" lower="-3.05432619099" upper="3.05432619099" velocity="10"/>
    <safety_controller k_position="100" k_velocity="2" soft_lower_limit="-3.01941960595" soft_upper_limit="3.01941960595"/>
    <dynamics damping="0.5"/>
  </joint>
  <link name="iiwa_link_7">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0.02"/>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/visual/link_7.stl"/>
      </geometry>
      <material name="Grey"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://iiwa_description/meshes/iiwa14/collision/link_7.stl"/>
      </geometry>
      <material name="Grey"/>
    </collision>
  </link>
  <joint name="iiwa_joint_ee" type="fixed">
    <parent link="iiwa_link_7"/>
    <child link="iiwa_link_ee"/>
    <origin rpy="0 0 0" xyz="0 0 0.045"/>
  </joint>
  <link name="iiwa_link_ee">
    </link>
  <!-- Load Gazebo lib and set the robot namespace -->
  <gazebo>
    <plugin filename="libgazebo_ros_control.so" name="gazebo_ros_control">
      <robotNamespace>/iiwa</robotNamespace>
    </plugin>
  </gazebo>
  <!-- Link0 -->
  <gazebo reference="iiwa_link_0">
    <material>Gazebo/Grey</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>
  <!-- Link1 -->
  <gazebo reference="iiwa_link_1">
    <material>Gazebo/Orange</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>
  <!-- Link2 -->
  <gazebo reference="iiwa_link_2">
    <material>Gazebo/Orange</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>
  <!-- Link3 -->
  <gazebo reference="iiwa_link_3">
    <material>Gazebo/Orange</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>
  <!-- Link4 -->
  <gazebo reference="iiwa_link_4">
    <material>Gazebo/Orange</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>
  <!-- Link5 -->
  <gazebo reference="iiwa_link_5">
    <material>Gazebo/Orange</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>
  <!-- Link6 -->
  <gazebo reference="iiwa_link_6">
    <material>Gazebo/Orange</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>
  <!-- Link7 -->
  <gazebo reference="iiwa_link_7">
    <material>Gazebo/Grey</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>
  <transmission name="iiwa_tran_1">
    <robotNamespace>/iiwa</robotNamespace>
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="iiwa_joint_1">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="iiwa_motor_1">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  <transmission name="iiwa_tran_2">
    <robotNamespace>/iiwa</robotNamespace>
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="iiwa_joint_2">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="iiwa_motor_2">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  <transmission name="iiwa_tran_3">
    <robotNamespace>/iiwa</robotNamespace>
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="iiwa_joint_3">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="iiwa_motor_3">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  <transmission name="iiwa_tran_4">
    <robotNamespace>/iiwa</robotNamespace>
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="iiwa_joint_4">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="iiwa_motor_4">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  <transmission name="iiwa_tran_5">
    <robotNamespace>/iiwa</robotNamespace>
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="iiwa_joint_5">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="iiwa_motor_5">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  <transmission name="iiwa_tran_6">
    <robotNamespace>/iiwa</robotNamespace>
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="iiwa_joint_6">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="iiwa_motor_6">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  <transmission name="iiwa_tran_7">
    <robotNamespace>/iiwa</robotNamespace>
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="iiwa_joint_7">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="iiwa_motor_7">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
</robot>

)");

namespace mc_rbdyn_urdf
{
bool operator==( const Geometry::Mesh& m1, const Geometry::Mesh& m2)
{
  return m1.scale == m2.scale && m1.filename == m2.filename;
}

bool operator==( const Geometry::Box& b1, const Geometry::Box& b2)
{
  return b1.size == b2.size;
}

bool operator==( const Geometry::Sphere& b1, const Geometry::Sphere& b2)
{
  return b1.radius == b2.radius;
}

bool operator==( const Geometry::Cylinder& b1, const Geometry::Cylinder& b2)
{
  return b1.radius == b2.radius && b1.length == b2.length;
}

bool operator==(const Geometry& g1, const Geometry& g2)
{
  return g1.type == g2.type && g1.data == g2.data;
}
bool operator==(const Visual& v1, const Visual& v2) {
  return v1.name == v2.name && v1.origin == v2.origin && v1.geometry == v2.geometry;
}
} /* mc_rbdyn_urdf */


const double TOL = 1e-6;