#ifndef GRL_ROS_BRIDGE_HPP_
#define GRL_ROS_BRIDGE_HPP_

#include <iostream>
#include <memory>
#include <array>

#include <boost/log/trivial.hpp>
#include <boost/exception/all.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include "grl/kuka/KukaDriver.hpp"
#include "grl/flatbuffer/JointState_generated.h"
#include "grl/flatbuffer/ArmControlState_generated.h"


/// @todo move elsewhere, because it will conflict with others' implementations of outputting vectors
template<typename T>
inline boost::log::formatting_ostream& operator<<(boost::log::formatting_ostream& out,  std::vector<T>& v)
{
  out << "[";
  size_t last = v.size() - 1;
  for(size_t i = 0; i < v.size(); ++i) {
    out << v[i];
    if (i != last) 
      out << ", ";
  }
  out << "]";
  return out;
}

namespace grl {
  namespace ros {

    enum RobotMode {
      MODE_TEACH, MODE_SERVO, MODE_IDLE
    };


    /** 
     *
     * This class contains code to offer a simple communication layer between ROS and the KUKA LBR iiwa
     *
     *
     */
    class KukaLBRiiwaROSPlugin : public std::enable_shared_from_this<KukaLBRiiwaROSPlugin> 
    {
    public:

      enum ParamIndex {
        RobotTipName,
        RobotTargetName,
        RobotTargetBaseName,
        LocalZMQAddress,
        RemoteZMQAddress,
        LocalHostKukaKoniUDPAddress,
        LocalHostKukaKoniUDPPort,
        RemoteHostKukaKoniUDPAddress,
        RemoteHostKukaKoniUDPPort,
        KukaCommandMode,
        KukaMonitorMode,
        IKGroupName
      };

      /// @todo allow default params
      typedef std::tuple<
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string,
        std::string
          > Params;


      static const Params defaultParams(){
        return std::make_tuple(
            "RobotMillTip"            , // RobotTipHandle,
            "RobotMillTipTarget"      , // RobotTargetHandle,
            "Robotiiwa"               , // RobotTargetBaseHandle,
            "tcp://0.0.0.0:30010"     , // LocalZMQAddress
            "tcp://172.31.1.147:30010", // RemoteZMQAddress
            "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
            "30200"                   , // LocalHostKukaKoniUDPPort,
            "192.170.10.2"            , // RemoteHostKukaKoniUDPAddress,
            "30200"                   , // RemoteHostKukaKoniUDPPort
            "JAVA"                    , // KukaCommandMode (options are FRI, JAVA)
            "FRI"                     , // KukaMonitorMode (options are FRI, JAVA)
            "IK_Group1_iiwa"            // IKGroupName
            );
      }


      /// unique tag type so State never
      /// conflicts with a similar tuple
      struct JointStateTag{};

      enum JointStateIndex {
        JointPosition,
        JointForce,
        JointTargetPosition,
        JointLowerPositionLimit,
        JointUpperPositionLimit,
        JointMatrix,
        JointStateTagIndex
      };


      typedef std::vector<double>               JointScalar;

      /// @see http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm#simGetJointMatrix for data layout information
      typedef std::array<double,12> TransformationMatrix;
      typedef std::vector<TransformationMatrix> TransformationMatrices;

      typedef std::tuple<
        JointScalar,            // jointPosition
        //  JointScalar             // JointVelocity  // no velocity yet
        JointScalar,            // jointForce
        JointScalar,            // jointTargetPosition
        JointScalar,            // JointLowerPositionLimit
        JointScalar,            // JointUpperPositionLimit
        TransformationMatrices, // jointTransformation
        JointStateTag           // JointStateTag unique identifying type so tuple doesn't conflict
          > State;


      KukaLBRiiwaROSPlugin(Params params = defaultParams())
        : params_(params), nh_()
      {
      }

      void construct(){ construct(params_);}

      /// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
      /// @warning getting the ik group is optional, so it does not throw an exception
      void construct(Params params) {

          ::ros::NodeHandle nh;
          js_pub_ = nh.advertise<sensor_msgs::JointState>("joint_state",100);
          jt_sub_ = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_traj_cmd", 1000, &KukaLBRiiwaROSPlugin::jt_callback, this);
          jt_pt_sub_ = nh.subscribe<trajectory_msgs::JointTrajectoryPoint>("joint_traj_pt_cmd", 1000, &KukaLBRiiwaROSPlugin::jt_pt_callback, this);
          mode_sub_ = nh.subscribe<std_msgs::String>("interaction_mode", 1000, &KukaLBRiiwaROSPlugin::mode_callback, this);
          //jt_sub_ = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_traj_cmd",1000,boost::bind(&KukaLBRiiwaROSPlugin::jt_callback, this, _1));

        params_ = params;
        // keep driver threads from exiting immediately after creation, because they have work to do!
        device_driver_workP_.reset(new boost::asio::io_service::work(device_driver_io_service));
        
        /// @todo properly support passing of io_service
        KukaDriverP_.reset(
            new grl::robot::arm::KukaDriver(
                //device_driver_io_service,
                params
                // std::make_tuple(
                //     std::string(std::get<LocalHostKukaKoniUDPAddress >        (params)),
                //     std::string(std::get<LocalHostKukaKoniUDPPort    >        (params)),
                //     std::string(std::get<RemoteHostKukaKoniUDPAddress>        (params)),
                //     std::string(std::get<RemoteHostKukaKoniUDPPort   >        (params)),
                //     grl::robot::arm::KukaFRIClientDataDriver::run_automatically
                //     )
                )

            );
          KukaDriverP_->construct();
      }




      bool setState(State& state) { return true; }


      const Params & getParams(){
        return params_;
      }

      /**
       * ROS joint trajectory callback
       * this code needs to execute the joint trajectory on the robot
       */
      void jt_callback(const trajectory_msgs::JointTrajectoryConstPtr &msg) {
        boost::lock_guard<boost::mutex> lock(jt_mutex);


        for(auto &pt: msg->points) {
          // positions velocities ac
          if (pt.positions.size() != KUKA::LBRState::NUM_DOF) {
            BOOST_THROW_EXCEPTION(std::runtime_error("Malformed joint trajectory request! Wrong number of joints."));
          }

          // handle
          //simJointPosition
          simJointPosition.clear();
          boost::copy(pt.positions,std::back_inserter(simJointPosition));
          //simJointVelocity
          simJointVelocity.clear();
          boost::copy(pt.velocities,std::back_inserter(simJointVelocity));
          //simJointForce
          simJointForce.clear();
          boost::copy(pt.effort,std::back_inserter(simJointForce));

          ///@todo: execute the rest of the trajectory
          break;
        }

      }


      /// ROS callback to set current interaction mode; determines whether commands will be send in SERVO, TEACH, etc
      void mode_callback(const std_msgs::StringConstPtr &msg) {
        boost::lock_guard<boost::mutex> lock(jt_mutex);
        if (msg->data == "teach") { 
          interaction_mode = MODE_TEACH;
        } else if (msg->data == "servo") {
          interaction_mode = MODE_SERVO;
        } else if (msg->data == "idle") {
          interaction_mode = MODE_IDLE;
        } else {
          interaction_mode = MODE_IDLE;
        }
      }

      /// ROS joint trajectory callback
      /// this code needs to execute the joint trajectory on the robot
      void jt_pt_callback(const trajectory_msgs::JointTrajectoryPointConstPtr &msg) {
        boost::lock_guard<boost::mutex> lock(jt_mutex);


          // positions velocities ac
          if (msg->positions.size() != KUKA::LBRState::NUM_DOF) {
            BOOST_THROW_EXCEPTION(std::runtime_error("Malformed joint trajectory request! Wrong number of joints."));
          }

          // handle
          //simJointPosition
          simJointPosition.clear();
          boost::copy(msg->positions,std::back_inserter(simJointPosition));
          //simJointVelocity
          simJointVelocity.clear();
          boost::copy(msg->velocities,std::back_inserter(simJointVelocity));
          //simJointForce
          simJointForce.clear();
          boost::copy(msg->effort,std::back_inserter(simJointForce));

          ///@todo: execute the rest of the trajectory
      }


      ~KukaLBRiiwaROSPlugin(){
        device_driver_workP_.reset();

        if(driver_threadP){
          device_driver_io_service.stop();
          driver_threadP->join();
        }
      }

     /// 
     /// @brief spin once, call this repeatedly to run the driver
     /// 
     /// 
     bool run_one()
     {

        bool haveNewData = false;

         if(KukaDriverP_){

             //grl::robot::arm::set(*KukaDriverP_, simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());
             if(simJointPosition.size()) KukaDriverP_->set( simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());
             if(simJointForce.size()) KukaDriverP_->set( simJointForce, grl::revolute_joint_torque_open_chain_command_tag());
             
             haveNewData = KukaDriverP_->run_one();

             if(haveNewData)
             {

               // We have the real kuka state read from the device now
               // update real joint angle data
               current_js_.position.clear();
               KukaDriverP_->get(std::back_inserter(current_js_.position), grl::revolute_joint_angle_open_chain_state_tag());

               current_js_.effort.clear();
               KukaDriverP_->get(std::back_inserter(current_js_.effort), grl::revolute_joint_torque_open_chain_state_tag());

               current_js_.velocity.clear();
               //grl::robot::arm::copy(friData_->monitoringMsg, std::back_inserter(current_js_.velocity), grl::revolute_joint_angle_open_chain_state_tag());

               js_pub_.publish(current_js_);
            }
        
        }
     
        return haveNewData;
      }

      boost::asio::io_service device_driver_io_service;
      std::unique_ptr<boost::asio::io_service::work> device_driver_workP_;
      std::unique_ptr<std::thread> driver_threadP;

      /// @todo replace all these simJoint elements with simple KukaLBRiiwaROSPlugin::State
      std::vector<double> simJointPosition;// = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
      std::vector<double> simJointVelocity = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
      std::vector<double> simJointForce;// = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
      std::vector<double> simJointTargetPosition = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
      KukaLBRiiwaROSPlugin::TransformationMatrices simJointTransformationMatrix;

      /// @note loss of precision! kuka sends double values, if you write custom code don't use these float values. Vrep uses floats internally which is why they are used here.
      std::vector<double> realJointPosition        = { 0, 0, 0, 0, 0, 0, 0 };
      // does not exist
      std::vector<double> realJointForce           = { 0, 0, 0, 0, 0, 0, 0 };

    private:

      RobotMode interaction_mode;

      boost::mutex jt_mutex;
      boost::shared_ptr<robot::arm::KukaDriver> KukaDriverP_;
      Params params_;
      
      ::ros::Subscriber jt_sub_; // subscribes to joint state trajectories and executes them 
      ::ros::Subscriber jt_pt_sub_; // subscribes to joint state trajectories and executes them 
      ::ros::Subscriber mode_sub_; // subscribes to interaction mode messages (strings for now)

      ::ros::Publisher js_pub_; // publish true joint states from the KUKA


      sensor_msgs::JointState current_js_;

      ::ros::NodeHandle nh_;

    };
  }
}


#endif
