
# To run a test driving the kuka iiwa into the straight up position,
# launch each of the following in separate terminals:
# roscore 
# grl_kuka_ros_driver
# rostopic pub /interaction_mode std_msgs/String "data: 'MoveArmJointServo'"
rostopic pub /joint_traj_pt_cmd trajectory_msgs/JointTrajectoryPoint "positions: [0,0,0,0,0,0,0.1]
velocities: [0,0,0,0,0,0,0]
accelerations: [0,0,0,0,0,0,0]
effort: [0,0,0,0,0,0,0]
time_from_start: {secs: 1, nsecs: 0}"
