#ifndef GRL_ROS_BRIDGE_HPP_
#define GRL_ROS_BRIDGE_HPP_

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

namespace grl {
  namespace ros {

    enum RobotMode {
      MODE_TEACH, MODE_SERVO, MODE_IDLE
    };


    /** 
     *
     * This class contains code to offer a simple communication layer between ROS and the KUKA LBR iiwa
     *
     * Initally:
     * 
     *
     */
    class KukaLBRiiwaROSPlugin : public std::enable_shared_from_this<KukaLBRiiwaROSPlugin> {
    public:

      enum ParamIndex {
        Joint1Name, 
        Joint2Name, 
        Joint3Name, 
        Joint4Name, 
        Joint5Name, 
        Joint6Name, 
        Joint7Name,
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
            "LBR_iiwa_14_R820_joint1" , // Joint1Handle, 
            "LBR_iiwa_14_R820_joint2" , // Joint2Handle, 
            "LBR_iiwa_14_R820_joint3" , // Joint3Handle, 
            "LBR_iiwa_14_R820_joint4" , // Joint4Handle, 
            "LBR_iiwa_14_R820_joint5" , // Joint5Handle, 
            "LBR_iiwa_14_R820_joint6" , // Joint6Handle, 
            "LBR_iiwa_14_R820_joint7" , // Joint7Handle,
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
        JointLowerPositionLimit
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

      /// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
      /// @warning getting the ik group is optional, so it does not throw an exception
      void construct() {

        params_ = params;
        // keep driver threads from exiting immediately after creation, because they have work to do!
        device_driver_workP_.reset(new boost::asio::io_service::work(device_driver_io_service));

        /// @todo figure out how to re-enable when .so isn't loaded
        // initHandles();
        if( boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI_ASYNC"))
          )
        {
          kukaFRIThreadSeparatorP.reset(
              new grl::KukaFRIThreadSeparator(
                  device_driver_io_service,
                  std::make_tuple(
                      std::string(std::get<LocalHostKukaKoniUDPAddress >        (params)),
                      std::string(std::get<LocalHostKukaKoniUDPPort    >        (params)),
                      std::string(std::get<RemoteHostKukaKoniUDPAddress>        (params)),
                      std::string(std::get<RemoteHostKukaKoniUDPPort   >        (params)),
                      grl::KukaFRIThreadSeparator::run_automatically
                      )
                  ));
        }
        else if( boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI")))
        {
          kukaFRIClientDataDriverP_.reset(
              new grl::robot::arm::KukaFRIClientDataDriver(
                  device_driver_io_service,
                  std::make_tuple(
                      std::string(std::get<LocalHostKukaKoniUDPAddress >        (params)),
                      std::string(std::get<LocalHostKukaKoniUDPPort    >        (params)),
                      std::string(std::get<RemoteHostKukaKoniUDPAddress>        (params)),
                      std::string(std::get<RemoteHostKukaKoniUDPPort   >        (params)),
                      grl::robot::arm::KukaFRIClientDataDriver::run_automatically
                      )
                  )

              );
        }

        try {
          BOOST_LOG_TRIVIAL(trace) << "KukaLBRiiwaRosPlugin: Connecting ZeroMQ Socket from " <<
            std::get<LocalZMQAddress>             (params_) << " to " <<
            std::get<RemoteZMQAddress>            (params_);
          boost::system::error_code ec;
          azmq::socket socket(device_driver_io_service, ZMQ_DEALER);
          socket.bind(   std::get<LocalZMQAddress>             (params_).c_str()   );
          socket.connect(std::get<RemoteZMQAddress>            (params_).c_str()   );
          kukaJavaDriverP = std::make_shared<AzmqFlatbuffer>(std::move(socket));

          // start up the driver thread
          /// @todo perhaps allow user to control this?
          driver_threadP.reset(new std::thread([&]{ device_driver_io_service.run(); }));
        } catch( boost::exception &e) {
          e << errmsg_info("KukaLBRiiwaRosPlugin: Unable to connect to ZeroMQ Socket from " + 
                           std::get<LocalZMQAddress>             (params_) + " to " + 
                           std::get<RemoteZMQAddress>            (params_));
          throw;
        }
        initHandles();

        std::vector<int> jointHandle;
        getHandleFromParam<JointNames>(params_,std::back_inserter(jointHandle));
        handleParams_ =
          std::make_tuple(
              std::move(jointHandle)                                 //Obtain Joint Handles
              ,getHandleFromParam<RobotTipName>           (params_)	//Obtain RobotTip handle
              ,getHandleFromParam<RobotTargetName>        (params_)
              ,getHandleFromParam<RobotTargetBaseName>    (params_)
              ,simGetIkGroupHandle(std::get<RobotIkGroup> (params_).c_str())
              );

        allHandlesSet  = true;

        js_pub_ = nh.advertise<sensor_msgs::JointState>("joint_state",100);
        jt_sub_ = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_traj_cmd", 1000, &KukaLBRiiwaROSPlugin::jt_callback, this);
      }

      /// @return 0 if ok 1 if problem
      /// @todo handle cyclic joints (see isCyclic below & simGetJointInterval)
      bool getState(State& state){
        if(!allHandlesSet) return false;
        const std::vector<int>& jointHandle = std::get<JointNames>(handleParams_);

        std::get<JointPosition>             (state).resize(jointHandle.size());
        std::get<JointForce>                (state).resize(jointHandle.size());
        std::get<JointTargetPosition>       (state).resize(jointHandle.size());
        std::get<JointMatrix>               (state).resize(jointHandle.size());
        std::get<JointLowerPositionLimit>   (state).resize(jointHandle.size());
        std::get<JointUpperPositionLimit>   (state).resize(jointHandle.size());

        enum limit {
          lower
            ,upper
            ,numLimits
        };

        simBool isCyclic;
        float jointAngleInterval[2]; // min,max

        for (std::size_t i=0 ; i < jointHandle.size() ; i++)
        {	
          int currentJointHandle = jointHandle[i];
          simGetJointPosition(currentJointHandle,&std::get<JointPosition>(state)[i]);  //retrieves the intrinsic position of a joint (Angle for revolute joint)
          simGetJointForce(currentJointHandle,&std::get<JointForce>(state)[i]);	//retrieves the force or torque applied to a joint along/about its active axis. This function retrieves meaningful information only if the joint is prismatic or revolute, and is dynamically enabled.
          simGetJointTargetPosition(currentJointHandle,&std::get<JointTargetPosition>(state)[i]);  //retrieves the target position of a joint
          simGetJointMatrix(currentJointHandle,&std::get<JointMatrix>(state)[i][0]);   //retrieves the intrinsic transformation matrix of a joint (the transformation caused by the joint movement)
          simGetJointInterval(currentJointHandle,&isCyclic,jointAngleInterval);
          std::get<JointLowerPositionLimit>(state)[i] = jointAngleInterval[lower];
          std::get<JointUpperPositionLimit>(state)[i] = jointAngleInterval[upper];

        }

        return false;
      }


      void getRealKukaAngles() {
        /// @todo m_haveReceivedRealData = true is a DANGEROUS HACK!!!! REMOVE ME AFTER DEBUGGING
        m_haveReceivedRealData = true;

        if(kukaFRIThreadSeparatorP)
        {
          boost::system::error_code send_ec,recv_ec;
          std::size_t send_bytes, recv_bytes;
          BOOST_VERIFY(kukaFRIThreadSeparatorP);
          std::shared_ptr<grl::robot::arm::kuka::iiwa::MonitorState> updatedState;
          kukaFRIThreadSeparatorP->async_getLatestState(updatedState,recv_ec,recv_bytes,send_ec,send_bytes);

          if (updatedState && !recv_ec && !send_ec) {
            this->m_haveReceivedRealData = true;
          } else {
            /// @todo should the results of getlatest state even be possible to call without receiving real data? should the library change?
            // if we didn't actually get anything don't try and update
            return;
          }
          // We have the real kuka state read from the device now
          // update real joint angle data
          current_js_.position.clear();
          grl::robot::arm::copy(updatedState->get(), std::back_inserter(realJointPosition), grl::revolute_joint_angle_open_chain_state_tag());


          current_js_.effort.clear();
          grl::robot::arm::copy(updatedState->get(), std::back_inserter(current_js_.effort), grl::revolute_joint_torque_open_chain_state_tag());

          current_js_.velocity.clear();
          grl::robot::arm::copy(updatedState->get(), std::back_inserter(current_js_.velocity), grl::revolute_joint_angle_open_chain_state_tag());



#if BOOST_VERSION < 105900
          // here we expect the simulation to be slightly ahead of the arm
          // so we get the simulation based joint angles and update the arm
          BOOST_LOG_TRIVIAL(trace) << "Real joint angles from FRI: " << realJointPosition << "\n";
#endif


          js_pub.publish(current_js_);


        } else if (kukaFRIClientDataDriverP_)
        {
          /// @todo fix this hack, real data hasn't actually been received because send & receive happen simultaneously
          m_haveReceivedRealData = true;
        }

      }



      bool setState(State& state) { return true; }


      const Params & getParams(){
        return params_;
      }

      /**
       * ROS joint trajectory callback
       * this code needs to execute the joint trajectory on the robot
       */
      void jt_callback(const trajectory_msgs::JointTrajectory &msg) {

      }

      void run_one(){
        return std::make_tuple(
            jointNames                , // JointNames
            "RobotMillTip"            , // RobotTipName,
            "RobotMillTipTarget"      , // RobotTargetName,
            "Robotiiwa"               , // RobotTargetBaseName,
            "IK_Group1_iiwa"            // RobotIkGroup
            );


        ros::Rate rate(60);
        while (ros::ok()) {
          ros::spinOnce();

          rate.sleep();
        }
      }


      if(!allHandlesSet) BOOST_THROW_EXCEPTION(std::runtime_error("KukaRosPlugin: Handles have not been initialized, cannot run updates."));
      getRealKukaAngles();
      bool isError = getStateFromVrep(); // true if there is an error
      allHandlesSet = !isError;
      /// @todo re-enable simulation feedback based on actual kuka state
      //updateVrepFromKuka();
      sendSimulatedJointAnglesToKuka();

    }

    ~KukaLBRiiwaROSPlugin(){
      device_driver_workP_.reset();

      if(driver_threadP){
        device_driver_io_service.stop();
        driver_threadP->join();
      }
    }

    void sendSimulatedJointAnglesToKuka(){

      if(!allHandlesSet || !m_haveReceivedRealData) return;

      /// @todo make this handled by template driver implementations/extensions

      if(kukaJavaDriverP && boost::iequals(std::get<KukaCommandMode>(params_),std::string("JAVA")))
      {
        /////////////////////////////////////////
        // Client sends to server asynchronously!

        /// @todo if allocation is a performance problem use boost::container::static_vector<double,7>
        std::vector<double> joints;

        auto fbbP = kukaJavaDriverP->GetUnusedBufferBuilder();

        /// @todo should we use simJointTargetPosition here?
        joints.clear();
        boost::copy(simJointPosition, std::back_inserter(joints));
        auto jointPos = fbbP->CreateVector(&joints[0], joints.size());

#if BOOST_VERSION < 105900
        BOOST_LOG_TRIVIAL(info) << "sending joint angles: " << joints << " from local zmq: " << std::get<LocalZMQAddress>            (params_) << " to remote zmq: " << std::get<RemoteZMQAddress>            (params_);
#endif

        /// @note we don't have a velocity right now, sending empty!
        joints.clear();
        //boost::copy(simJointVelocity, std::back_inserter(joints));
        auto jointVel = fbbP->CreateVector(&joints[0], joints.size());
        joints.clear();
        boost::copy(simJointForce, std::back_inserter(joints));
        auto jointAccel = fbbP->CreateVector(&joints[0], joints.size());
        auto jointState = grl::flatbuffer::CreateJointState(*fbbP,jointPos,jointVel,jointAccel);
        grl::flatbuffer::FinishJointStateBuffer(*fbbP, jointState);
        kukaJavaDriverP->async_send_flatbuffer(fbbP);

      }
      else if(boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI")))
      {
        // note: this one sends *and* receives the joint data!
        BOOST_VERIFY(kukaFRIClientDataDriverP_.get()!=nullptr);
        /// @todo use runtime calculation of NUM_JOINTS instead of constant
        if(!friData_) friData_ = std::make_shared<KUKA::FRI::ClientData>(7);

        // Set the FRI to the simulated joint positions
        grl::robot::arm::set(friData_->commandMsg, simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());
        grl::robot::arm::set(friData_->commandMsg, simJointForce   , grl::revolute_joint_torque_open_chain_command_tag());



        boost::system::error_code send_ec,recv_ec;
        std::size_t send_bytes, recv_bytes;
        bool haveNewData = !kukaFRIClientDataDriverP_->update_state(friData_,recv_ec,recv_bytes,send_ec,send_bytes);

        if(haveNewData)
        {
          this->m_haveReceivedRealData = true;
        }
        else
        {
          /// @todo should the results of getlatest state even be possible to call without receiving real data? should the library change?
          return;
        }


        // We have the real kuka state read from the device now
        // update real joint angle data
        realJointPosition.clear();
        grl::robot::arm::copy(friData_->monitoringMsg, std::back_inserter(realJointPosition), grl::revolute_joint_angle_open_chain_state_tag());


        realJointForce.clear();
        grl::robot::arm::copy(friData_->monitoringMsg, std::back_inserter(realJointForce), grl::revolute_joint_torque_open_chain_state_tag());

        realJointPosition.clear();
        grl::robot::arm::copy(friData_->monitoringMsg, std::back_inserter(realJointPosition), grl::revolute_joint_angle_open_chain_state_tag());


      }
      else if( boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI_ASYNC")))
      {
        BOOST_VERIFY(kukaFRIThreadSeparatorP);
        // create the command for the FRI
        auto commandP = std::make_shared<grl::robot::arm::kuka::iiwa::CommandState>();
        // Set the FRI to the simulated joint positions
        grl::robot::arm::set(*commandP, simJointPosition, grl::revolute_joint_angle_open_chain_command_tag());
        grl::robot::arm::set(*commandP, simJointForce   , grl::revolute_joint_torque_open_chain_command_tag());
        // send the command
        this->kukaFRIThreadSeparatorP->async_sendCommand(commandP);
      } else {
        BOOST_THROW_EXCEPTION(std::runtime_error(std::string("KukaROSPlugin: Selected KukaCommandMode ")+std::get<KukaCommandMode>(params_)+" does not exist! Options are JAVA, FRI, and FRI_ASYNC"));
      }

    }


    volatile bool allHandlesSet = false;
    volatile bool m_haveReceivedRealData = false;

    boost::asio::io_service device_driver_io_service;
    std::unique_ptr<boost::asio::io_service::work> device_driver_workP_;
    std::unique_ptr<std::thread> driver_threadP;
    std::shared_ptr<grl::robot::arm::KukaFRIClientDataDriver> kukaFRIClientDataDriverP_;
    std::shared_ptr<grl::KukaFRIThreadSeparator> kukaFRIThreadSeparatorP;
    std::shared_ptr<AzmqFlatbuffer> kukaJavaDriverP;
    std::shared_ptr<RosArmDriver> RosArmDriverP_;

    /// @todo replace all these simJoint elements with simple RosArmDriver::State
    std::vector<float_t> simJointPosition = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    std::vector<float_t> simJointForce = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    std::vector<float_t> simJointTargetPosition = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    RosArmDriver::TransformationMatrices simJointTransformationMatrix;

    /// @note loss of precision! kuka sends double values, if you write custom code don't use these float values. Vrep uses floats internally which is why they are used here.
    std::vector<float_t> realJointPosition        = { 0, 0, 0, 0, 0, 0, 0 };
    // does not exist
    std::vector<float_t> realJointForce           = { 0, 0, 0, 0, 0, 0, 0 };



  private:

    ros::Subscriber jt_sub_; // subscribes to joint state trajectories and executes them 
    ros::Publisher js_pub_; // publish true joint states from the KUKA

    sensor_msgs::JointState current_js_;

    ros::NodeHandle nh_;

  };


}

}

#endif
