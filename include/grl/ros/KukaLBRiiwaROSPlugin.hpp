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

#include <cartesian_impedance_msgs/SetCartesianImpedance.h>
#include <cartesian_impedance_msgs/SetCartesianForceCtrl.h>

#include "grl/kuka/KukaDriver.hpp"
#include "grl/flatbuffer/JointState_generated.h"
#include "grl/flatbuffer/ArmControlState_generated.h"
#include "grl/flatbuffer/KUKAiiwa_generated.h"

//this is for conversion fom meters to mm
#define meters_to_mm 1000


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

    /**
     *
     * This class contains code to offer a simple communication layer between ROS and the KUKA LBR iiwa
     *
     * @todo Main Loop Update Rate must be supplied to underlying Driver for FRI mode. see KukaLBRiiwaVrepPlugin for reference, particularly kukaDriverP_->set(simulationTimeStep_,time_duration_command_tag());
     */
    class KukaLBRiiwaROSPlugin : public std::enable_shared_from_this<KukaLBRiiwaROSPlugin>
    {
    public:

      enum ParamIndex {
        RobotName,
        RobotModel,
        LocalZMQAddress,
        RemoteZMQAddress,
        LocalHostKukaKoniUDPAddress,
        LocalHostKukaKoniUDPPort,
        RemoteHostKukaKoniUDPAddress,
        RemoteHostKukaKoniUDPPort,
        KukaCommandMode,
        KukaMonitorMode
      };

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
        std::string
          > Params;


      static const Params defaultParams(){
        return std::make_tuple(
            "Robotiiwa"               , // RobotName,
            "KUKA_LBR_IIWA_14_R820"      , // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800)
            "tcp://0.0.0.0:30010"     , // LocalZMQAddress
            "tcp://172.31.1.147:30010", // RemoteZMQAddress
            "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
            "30200"                   , // LocalHostKukaKoniUDPPort,
            "192.170.10.2"            , // RemoteHostKukaKoniUDPAddress,
            "30200"                   , // RemoteHostKukaKoniUDPPort
            "JAVA"                    , // KukaCommandMode (options are FRI, JAVA)
            "FRI"                       // KukaMonitorMode (options are FRI, JAVA)
            );
      }

      Params& loadRosParams(Params& params) {
            ::ros::NodeHandle nh_tilde("~");

            nh_tilde.getParam("RobotName",std::get<RobotName>(params));
            nh_tilde.getParam("RobotModel",std::get<RobotModel>(params));
            nh_tilde.getParam("LocalZMQAddress",std::get<LocalZMQAddress>(params));
            nh_tilde.getParam("RemoteZMQAddress",std::get<RemoteZMQAddress>(params));
            nh_tilde.getParam("LocalHostKukaKoniUDPAddress",std::get<LocalHostKukaKoniUDPAddress>(params));
            nh_tilde.getParam("LocalHostKukaKoniUDPPort",std::get<LocalHostKukaKoniUDPPort>(params));
            nh_tilde.getParam("RemoteHostKukaKoniUDPAddress",std::get<RemoteHostKukaKoniUDPAddress>(params));
            nh_tilde.getParam("RemoteHostKukaKoniUDPPort",std::get<RemoteHostKukaKoniUDPPort>(params));
            nh_tilde.getParam("KukaCommandMode",std::get<KukaCommandMode>(params));
            nh_tilde.getParam("KukaMonitorMode",std::get<KukaMonitorMode>(params));

          return params;
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
        robot::arm::KukaJAVAdriver::JointStateTag           // JointStateTag unique identifying type so tuple doesn't conflict
          > State;


      KukaLBRiiwaROSPlugin(Params params = defaultParams())
        : debug(false),params_(params), nh_("")
      {
        loadRosParams(params_);
      }

      void construct(){ construct(params_);}

      /// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
      /// @warning getting the ik group is optional, so it does not throw an exception
      void construct(Params params) {

          current_js_.name.resize(7);
          current_js_.name[0] = "iiwa_joint_1";
          current_js_.name[1] = "iiwa_joint_2";
          current_js_.name[2] = "iiwa_joint_3";
          current_js_.name[3] = "iiwa_joint_4";
          current_js_.name[4] = "iiwa_joint_5";
          current_js_.name[5] = "iiwa_joint_6";
          current_js_.name[6] = "iiwa_joint_7";
          current_js_.velocity.resize(7);
          current_js_.velocity[0] = 0.;
          current_js_.velocity[1] = 0.;
          current_js_.velocity[2] = 0.;
          current_js_.velocity[3] = 0.;
          current_js_.velocity[4] = 0.;
          current_js_.velocity[5] = 0.;
          current_js_.velocity[6] = 0.;

          ::ros::NodeHandle nh;
          js_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states",100);
          jt_sub_ = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_traj_cmd", 1000, &KukaLBRiiwaROSPlugin::jt_callback, this);
          jt_pt_sub_ = nh.subscribe<trajectory_msgs::JointTrajectoryPoint>("joint_traj_pt_cmd", 1000, &KukaLBRiiwaROSPlugin::jt_pt_callback, this);
          mode_sub_ = nh.subscribe<std_msgs::String>("interaction_mode", 1000, &KukaLBRiiwaROSPlugin::mode_callback, this);

          dummy_sub_ = nh.subscribe<std_msgs::String>("dummy_topic", 1000, &KukaLBRiiwaROSPlugin::dummy_callback, this);
          cart_imp_sub_ = nh.subscribe<cartesian_impedance_msgs::SetCartesianImpedance>("set_cartesian_impedance_params", 1000, &KukaLBRiiwaROSPlugin::set_cartesian_impedance_callback, this);
          cart_ft_sub_ = nh.subscribe<cartesian_impedance_msgs::SetCartesianForceCtrl>("set_cartesian_ft_params", 1000, &KukaLBRiiwaROSPlugin::set_force_control_callback, this);

          ROS_INFO("done creating subscribers");
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
                //     std::string(std::std::get<LocalHostKukaKoniUDPAddress >        (params)),
                //     std::string(std::std::get<LocalHostKukaKoniUDPPort    >        (params)),
                //     std::string(std::std::get<RemoteHostKukaKoniUDPAddress>        (params)),
                //     std::string(std::std::get<RemoteHostKukaKoniUDPPort   >        (params)),
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
        simJointPosition.clear();
        // boost::copy(current_js_.position,std::back_inserter(simJointPosition));
        // if(simJointPosition.size()) KukaDriverP_->set( simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());

        // simJointPosition.clear();
        // //simJointVelocity
        // simJointVelocity.clear();
        // //simJointForce
        // simJointForce.clear();

        //std::cerr << "Mode command = " << msg->data.c_str() << "\n";
        if (debug) {
          ROS_INFO("Receiving mode command: %s", msg->data.c_str());
        }

        unsigned int ArmStateLen = 9;
        for (unsigned int i = 0; i < ArmStateLen; ++i) {
          if (msg->data == grl::flatbuffer::EnumNamesArmState()[i]) {

            if (debug) {
              std::string info = std::string("Valid grl::flatbuffer::ArmState command received for ") + grl::flatbuffer::EnumNamesArmState()[i] + std::string(" mode");
              ROS_INFO(info.c_str());
            }

            interaction_mode = static_cast<grl::flatbuffer::ArmState>(i);
            KukaDriverP_->set(interaction_mode);
            break;
          }
        }

      }
      void dummy_callback(const std_msgs::StringConstPtr &msg)
      {
        boost::lock_guard<boost::mutex> lock(jt_mutex);

        ROS_INFO("Dummy message set!");
        KukaDriverP_->set(msg->data);
        //KukaDriverP_->set();

      }
      void set_cartesian_impedance_callback(const cartesian_impedance_msgs::SetCartesianImpedanceConstPtr &cartImpedance)
      {
            boost::lock_guard<boost::mutex> lock(jt_mutex);
            ROS_INFO("Changing Cartesian Impedance Parameters");
            //
            // simJointPosition.clear();
            // boost::copy(current_js_.position,std::back_inserter(simJointPosition));
            // if(simJointPosition.size()) KukaDriverP_->set( simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());

            //Cartesian stiffness msg values to the flatbuffers
            grl::flatbuffer::Vector3d cart_stifness_trans(cartImpedance->stiffness.translational.x,cartImpedance->stiffness.translational.y,cartImpedance->stiffness.translational.z);
            grl::flatbuffer::EulerRotation cart_stifness_rot(cartImpedance->stiffness.rotational.x,cartImpedance->stiffness.rotational.y,cartImpedance->stiffness.rotational.z,grl::flatbuffer::EulerOrder_xyz);

            KukaDriverP_->set(cart_stifness_trans,cart_stifness_rot,cart_stiffness_values());


            //Cartesian damping values pass to the flatbuffers
            grl::flatbuffer::Vector3d cart_damping_trans(cartImpedance->damping.translational.x,cartImpedance->damping.translational.y,cartImpedance->damping.translational.z);
            grl::flatbuffer::EulerRotation cart_damping_rot(cartImpedance->damping.rotational.x,cartImpedance->damping.rotational.y,cartImpedance->damping.rotational.z,grl::flatbuffer::EulerOrder_xyz);

            KukaDriverP_->set(cart_damping_trans,cart_damping_rot,cart_damping_values());

            //Max Cartesian Path deviation when in cartesian impedance
            //Cartesian max Cartesian velocities [m] for translation and [rad] for rotation
            //iiwa expects the values to be in mm/s for *cart_max_path_deviation*
            grl::flatbuffer::Vector3d cart_max_path_dev_trans(cartImpedance->max_path_deviation.translation.x*meters_to_mm,cartImpedance->max_path_deviation.translation.y*meters_to_mm,cartImpedance->max_path_deviation.translation.z*meters_to_mm);
            grl::flatbuffer::EulerRotation cart_max_path_dev_vel_rot(cartImpedance->max_path_deviation.rotation.x,cartImpedance->max_path_deviation.rotation.y,cartImpedance->max_path_deviation.rotation.z,grl::flatbuffer::EulerOrder_xyz);


            grl::flatbuffer::EulerPose cart_max_path_deviation(cart_max_path_dev_trans,cart_max_path_dev_vel_rot);
            KukaDriverP_->set(cart_max_path_deviation,max_path_deviation());


            //Max Cartesian velocities [m/s] for translation and [rad/s] for rotation
            // iiwa expect the values to be mm/s for the *cart_max_ctrl_vel_trans*
            grl::flatbuffer::Vector3d cart_max_ctrl_vel_trans(cartImpedance->max_cart_vel.set.linear.x*meters_to_mm,cartImpedance->max_cart_vel.set.linear.y*meters_to_mm,cartImpedance->max_cart_vel.set.linear.z*meters_to_mm);
            grl::flatbuffer::EulerRotation cart_max_ctrl_vel_rot(cartImpedance->max_cart_vel.set.angular.x,cartImpedance->max_cart_vel.set.angular.y,cartImpedance->max_cart_vel.set.angular.z,grl::flatbuffer::EulerOrder_xyz);

            grl::flatbuffer::EulerPose cart_max_ctrl_vel(cart_max_ctrl_vel_trans,cart_max_ctrl_vel_rot);
            KukaDriverP_->set(cart_max_ctrl_vel,max_cart_vel());

            //Max Control Force [N] for translation and [Nm] for rotation
            grl::flatbuffer::Vector3d cart_max_ctrl_force_trans(cartImpedance->max_ctrl_force.set.force.x,cartImpedance->max_ctrl_force.set.force.y,cartImpedance->max_ctrl_force.set.force.z);
            grl::flatbuffer::EulerRotation cart_max_ctrl_force_rot(cartImpedance->max_ctrl_force.set.torque.x,cartImpedance->max_ctrl_force.set.torque.y,cartImpedance->max_ctrl_force.set.torque.z,grl::flatbuffer::EulerOrder_xyz);

            grl::flatbuffer::EulerPose cart_max_ctrl_force(cart_max_ctrl_force_trans,cart_max_ctrl_force_rot);

            // //send position after state change
            // simJointPosition.clear();
            // boost::copy(current_js_.position,std::back_inserter(simJointPosition));
            // if(simJointPosition.size()) KukaDriverP_->set( simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());

            KukaDriverP_->set(cart_max_ctrl_force,max_ctrl_force());

            double nullspaceStiffness;
            double nullspaceDamping;

            nullspaceDamping = cartImpedance->null_space_params.damping[0];
            nullspaceStiffness = cartImpedance->null_space_params.stiffness[0];

            // simJointPosition.clear();
            // boost::copy(current_js_.position,std::back_inserter(simJointPosition));
            // if(simJointPosition.size()) KukaDriverP_->set( simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());

            KukaDriverP_->set(nullspaceStiffness,nullspaceDamping,null_space_params());

            // simJointPosition.clear();
            // boost::copy(current_js_.position,std::back_inserter(simJointPosition));
            // if(simJointPosition.size()) KukaDriverP_->set( simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());



      }
      void set_force_control_callback(const cartesian_impedance_msgs::SetCartesianForceCtrlConstPtr &cartFTCtrl)
      {
        boost::lock_guard<boost::mutex> lock(jt_mutex);
        std::string DOF;
        double force;
        double stiffness;

        // simJointPosition.clear();
        // boost::copy(current_js_.position,std::back_inserter(simJointPosition));
        // if(simJointPosition.size()) KukaDriverP_->set( simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());

        KukaDriverP_->set(cartFTCtrl->DOF.c_str(),cartFTCtrl->force,cartFTCtrl->stiffness,set_const_ctrl_force());


        // simJointPosition.clear();
        // boost::copy(current_js_.position,std::back_inserter(simJointPosition));
        // if(simJointPosition.size()) KukaDriverP_->set( simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());

        //ROS_INFO("Set Cartesian Constant Force/torqe Control; %s",cartFTCtrl->DOF.c_str());
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

          ROS_INFO("Im in Joint trajectory msgs!!");

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


         switch(interaction_mode) {
           case grl::flatbuffer::ArmState_MoveArmJointServo:
             if (debug) {
              ROS_INFO("Arm is in SERVO Mode");
             }
             if(simJointPosition.size()) KukaDriverP_->set( simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());
             /// @todo setting joint position clears joint force in KukaDriverP_. Is this right or should position and force be configurable simultaeously?
             //if(simJointForce.size()) KukaDriverP_->set( simJointForce, grl::revolute_joint_torque_open_chain_command_tag());
             break;
           case grl::flatbuffer::ArmState_TeachArm:
             if (debug) {
              ROS_INFO("Arm is in TEACH mode");
             }
             break;
             break;
           case grl::flatbuffer::ArmState_StopArm:
             break;
           case grl::flatbuffer::ArmState_PauseArm:
             break;
           case grl::flatbuffer::ArmState_StartArm:
             if (debug) {
              ROS_INFO("Sending start!");
             }
             break;
           case grl::flatbuffer::ArmState_ShutdownArm:
             break;
           default:
            if (debug) {
            ROS_INFO("KukaLBRiiwaROSPlugin in unsupported mode!");
            }

         }

         haveNewData = KukaDriverP_->run_one();

         if(haveNewData)
         {

           // We have the real kuka state read from the device now
           // update real joint angle data
           current_js_.position.clear();
           KukaDriverP_->get(std::back_inserter(current_js_.position), grl::revolute_joint_angle_open_chain_state_tag());

           current_js_.effort.clear();
           KukaDriverP_->get(std::back_inserter(current_js_.effort), grl::revolute_joint_torque_open_chain_state_tag());

           //current_js_.velocity.clear();
           //grl::robot::arm::copy(friData_->monitoringMsg, std::back_inserter(current_js_.velocity), grl::revolute_joint_angle_open_chain_state_tag());
           current_js_.header.stamp = ::ros::Time::now();
           current_js_.header.seq += 1;
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

      std::string dummy_message;

    private:

      bool debug;
      grl::flatbuffer::ArmState interaction_mode;
      // grl::flatbuffer::KUKAiiwaArmConfiguration cart_impedance;

      boost::mutex jt_mutex;
      boost::shared_ptr<robot::arm::KukaDriver> KukaDriverP_;
      Params params_;

      ::ros::Subscriber jt_sub_; // subscribes to joint state trajectories and executes them
      ::ros::Subscriber jt_pt_sub_; // subscribes to joint state trajectories and executes them
      ::ros::Subscriber mode_sub_; // subscribes to interaction mode messages (strings for now)
      ::ros::Subscriber dummy_sub_;
      ::ros::Subscriber cart_imp_sub_;
      ::ros::Subscriber cart_ft_sub_;

      ::ros::Publisher js_pub_; // publish true joint states from the KUKA


      sensor_msgs::JointState current_js_;

      ::ros::NodeHandle nh_;


    };
  }
}


#endif
