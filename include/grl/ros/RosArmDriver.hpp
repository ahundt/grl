#ifndef GRL_ROS_BRIDGE_HPP_
#define GRL_ROS_BRIDGE_HPP_

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

namespace grl {
  namespace ros {

    enum RobotMode {

    };


    /** 
     *
     * This class contains code to offer a simple communication layer between ROS and the 
     *
     * Initally:
     * 
     *
     */
    class RosArmDriver {
    public:



    protected:

      ros::Subscriber jt_sub; // subscribes to joint state trajectories and executes them 
      ros::Publisher js_pub; // publish true joint states from the KUKA

    };


  }

}

#endif
