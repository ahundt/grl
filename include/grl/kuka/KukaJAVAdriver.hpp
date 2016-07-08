#ifndef GRL_KUKA_JAVA_DRIVER
#define GRL_KUKA_JAVA_DRIVER

#include <iostream>
#include <chrono>
#include <ratio>
#include <thread>

#include <tuple>
#include <memory>
#include <thread>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/exception/all.hpp>
#include <boost/config.hpp>

#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/chrono/include.hpp>
#include <boost/chrono/duration.hpp>


#ifdef BOOST_NO_CXX11_ATOMIC_SMART_PTR
#include <boost/thread.hpp>
#endif

#include "grl/tags.hpp"
#include "grl/exception.hpp"
#include "grl/kuka/Kuka.hpp"
#include "grl/AzmqFlatbuffer.hpp"
#include "grl/flatbuffer/JointState_generated.h"
#include "grl/flatbuffer/ArmControlState_generated.h"
#include "grl/flatbuffer/KUKAiiwa_generated.h"




/// @todo move elsewhere, because it will conflict with others' implementations of outputting vectors
template<typename T>
inline std::ostream& operator<<(std::ostream& out,  std::vector<T>& v)
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


/// @todo move elsewhere, because it will conflict with others' implementations of outputting vectors
template<typename T, std::size_t U>
inline std::ostream& operator<<(std::ostream& out,  boost::container::static_vector<T,U>& v)
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


namespace grl { namespace robot { namespace arm {


    /**
     *
     * This class contains code to offer a simple communication layer between ROS and the KUKA LBR iiwa
     *
     * Initally:
     *
     *
     * @todo make sure mutex is locked when appropriate
     *
     */
    class KukaJAVAdriver : public std::enable_shared_from_this<KukaJAVAdriver> {
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


      KukaJAVAdriver(Params params = defaultParams())
        : params_(params), armControlMode_(flatbuffer::ArmState::ArmState_NONE)
      {}

      void construct(){ construct(params_); sequenceNumber = 0;}

      /// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
      /// @warning getting the ik group is optional, so it does not throw an exception
      void construct(Params params) {

        params_ = params;
        // keep driver threads from exiting immediately after creation, because they have work to do!
        device_driver_workP_.reset(new boost::asio::io_service::work(device_driver_io_service));

        try {
          BOOST_LOG_TRIVIAL(trace) << "KukaLBRiiwaRosPlugin: Connecting ZeroMQ Socket from " <<
            std::get<LocalZMQAddress>             (params_) << " to " <<
            std::get<RemoteZMQAddress>            (params_);
          boost::system::error_code ec;
          azmq::socket socket(device_driver_io_service, ZMQ_DEALER);
          socket.bind(   std::get<LocalZMQAddress>             (params_).c_str()   );
          socket.connect(std::get<RemoteZMQAddress>            (params_).c_str()   );
          kukaJavaDriverP = std::make_shared<AzmqFlatbuffer>(std::move(socket));

        } catch( boost::exception &e) {
          e << errmsg_info("KukaLBRiiwaRosPlugin: Unable to connect to ZeroMQ Socket from " +
                           std::get<LocalZMQAddress>             (params_) + " to " +
                           std::get<RemoteZMQAddress>            (params_));
          throw;
        }
      }




      const Params & getParams(){
        return params_;
      }

      /// shuts down the arm
      bool destruct(){

          auto fbbP = kukaJavaDriverP->GetUnusedBufferBuilder();

          boost::lock_guard<boost::mutex> lock(jt_mutex);

          double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();

          /// @todo is this the best string to pass for the full arm's name?
          auto basename = std::get<RobotName>(params_);

          auto bns = fbbP->CreateString(basename);


          auto controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,flatbuffer::ArmState::ArmState_ShutdownArm,flatbuffer::CreateShutdownArm(*fbbP).Union());

//          auto KUKAiiwa = CreateKUKAiiwaState(*fbbP,
//   flatbuffers::Offset<flatbuffers::String> name = 0,
//   flatbuffers::Offset<flatbuffers::String> destination = 0,
//   flatbuffers::Offset<flatbuffers::String> source = 0,
//   double timestamp = 0,
//   uint8_t setArmControlState = 0,
//   flatbuffers::Offset<grl::flatbuffer::ArmControlState> armControlState = 0,
//   uint8_t setArmConfiguration = 0,
//   flatbuffers::Offset<KUKAiiwaArmConfiguration> armConfiguration = 0,
//   uint8_t hasMonitorState = 0,
//   flatbuffers::Offset<KUKAiiwaMonitorState> monitorState = 0,
//   uint8_t hasMonitorConfig = 0,
//   flatbuffers::Offset<KUKAiiwaMonitorConfiguration> monitorConfig = 0)


          auto KUKAiiwa = CreateKUKAiiwaState(*fbbP,0,0,0,0,1,controlState,0,0,0,0,0,0);

          auto iiwaStateVec = fbbP->CreateVector(&KUKAiiwa, 1);

          auto iiwaStates = flatbuffer::CreateKUKAiiwaStates(*fbbP,iiwaStateVec);



          grl::flatbuffer::FinishKUKAiiwaStatesBuffer(*fbbP, iiwaStates);
          kukaJavaDriverP->async_send_flatbuffer(fbbP);

          return true;
      }

      ~KukaJAVAdriver(){
        device_driver_workP_.reset();

        if(driver_threadP){
          device_driver_io_service.stop();
          driver_threadP->join();
        }
      }


      /// @brief SEND COMMAND TO ARM. Call this often
      /// Performs the main update spin once.
      /// @todo ADD SUPPORT FOR READING ARM STATE OVER JAVA INTERFACE
      bool run_one(){

        // @todo CHECK FOR REAL DATA BEFORE SENDING COMMANDS
        //if(!m_haveReceivedRealDataCount) return;

        bool haveNewData = false;

        /// @todo make this handled by template driver implementations/extensions


        if(kukaJavaDriverP)
        {

          auto fbbP = kukaJavaDriverP->GetUnusedBufferBuilder();

            boost::lock_guard<boost::mutex> lock(jt_mutex);

          double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();

          /// @todo is this the best string to pass for the full arm's name?
          auto basename = std::get<RobotName>(params_);

          auto bns = fbbP->CreateString(basename);

          flatbuffers::Offset<flatbuffer::ArmControlState> controlState;


          switch (armControlMode_) {

              case flatbuffer::ArmState::ArmState_StartArm: {
                 controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,flatbuffer::CreateStartArm(*fbbP).Union());
                 break;
              }
              case flatbuffer::ArmState::ArmState_MoveArmJointServo: {

                /// @todo when new
                auto armPositionBuffer = fbbP->CreateVector(armState_.commandedPosition_goal.data(),armState_.commandedPosition_goal.size());
                auto commandedTorque = fbbP->CreateVector(armState_.commandedTorque.data(),armState_.commandedTorque.size());
                auto goalJointState = grl::flatbuffer::CreateJointState(*fbbP,armPositionBuffer,0/*no velocity*/,0/*no acceleration*/,commandedTorque);
                auto moveArmJointServo = grl::flatbuffer::CreateMoveArmJointServo(*fbbP,goalJointState);
                controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,moveArmJointServo.Union());
                std::cout << "\nKukaJAVAdriver sending armposition command:" <<armState_.commandedPosition_goal<<"\n";
                 break;
              }
              case flatbuffer::ArmState::ArmState_TeachArm: {
                 controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,flatbuffer::CreateTeachArm(*fbbP).Union());
                 break;
              }
              case flatbuffer::ArmState::ArmState_PauseArm: {
                 controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,flatbuffer::CreatePauseArm(*fbbP).Union());
                 break;
              }
              case flatbuffer::ArmState::ArmState_StopArm: {
                 controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,flatbuffer::CreateStopArm(*fbbP).Union());
                 break;
              }
              case flatbuffer::ArmState::ArmState_ShutdownArm: {
                 controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,flatbuffer::CreateStopArm(*fbbP).Union());
                 break;
              }
              case flatbuffer::ArmState::ArmState_NONE: {
                 //std::cerr << "Waiting for interation mode... (currently NONE)\n";
                 break;
              }
              default:
                 std::cerr << "KukaJAVAdriver unsupported use case: " << armControlMode_ << "\n";
          }

          auto name = fbbP->CreateString(std::get<RobotName>(params_));
          auto dummy = fbbP->CreateString(dummy_message_);

          // //const grl::flatbuffer::EulerPoseParams* test;
          // grl::flatbuffer::EulerRotation parms(0.3,0.3,0.3,grl::flatbuffer::EulerOrder_xyz);
          // grl::flatbuffer::Vector3d rv(300,400,500);
          //
          // const flatbuffers::Offset<grl::flatbuffer::EulerPoseParams> test;

          // test->add_position(&rv);
          // test->add_rotation(&parms);
          auto stiffnessPose  = flatbuffer::CreateEulerPoseParams(*fbbP,&cart_stifness_trans_,&cart_stifness_rot_);
          auto dampingPose  = flatbuffer::CreateEulerPoseParams(*fbbP,&cart_damping_trans_,&cart_damping_rot_);
          auto cartImpedance = flatbuffer::CreateCartesianImpedenceControlMode(*fbbP,cartImpValuesChanged,stiffnessPose,dampingPose,&cart_max_path_deviation_,&cart_max_ctrl_vel_,&cart_max_ctrl_force_,nullspaceStiffness_,nullspaceDamping_);
          auto cartDOFBuff = fbbP->CreateString(ft_dof_);
          auto cartFTCtrl = flatbuffer::CreateConstantForceControlMode(*fbbP, cartFTControl_,cartDOFBuff, ft_force_,ft_stiffness_);
          auto kukaiiwaArmConfiguration = flatbuffer::CreateKUKAiiwaArmConfiguration(*fbbP,dummy,name,commandInterface_,monitorInterface_,cartImpedance,cartFTCtrl);

          auto kukaiiwastate = flatbuffer::CreateKUKAiiwaState(*fbbP,0,0,0,0,1,controlState,1,kukaiiwaArmConfiguration);

          auto kukaiiwaStateVec = fbbP->CreateVector(&kukaiiwastate, 1);

          auto states = flatbuffer::CreateKUKAiiwaStates(*fbbP,kukaiiwaStateVec);

          grl::flatbuffer::FinishKUKAiiwaStatesBuffer(*fbbP, states);

          flatbuffers::Verifier verifier(fbbP->GetBufferPointer(),fbbP->GetSize());
          BOOST_VERIFY(grl::flatbuffer::VerifyKUKAiiwaStatesBuffer(verifier));


          if(armControlMode_ == flatbuffer::ArmState::ArmState_MoveArmJointServo)
          {
              auto states2 = flatbuffer::GetKUKAiiwaStates(fbbP->GetBufferPointer());
              auto movearm = static_cast<const flatbuffer::MoveArmJointServo*>(states2->states()->Get(0)->armControlState()->state());
              //kukaiiwaArmConfiguration.dummy(dummy_message_);
              std::cout << "re-extracted " << movearm->goal()->position()->size() << " joint angles: ";
              for(std::size_t i = 0; i <  movearm->goal()->position()->size(); ++i)
              {
                std::cout << i << "=" << movearm->goal()->position()->Get(i) << ", ";
              }
              std::cout << "\n";
          }

          kukaJavaDriverP->async_send_flatbuffer(fbbP);
          cartImpValuesChanged = false;
          cartFTControl_       = false;
        }

         return haveNewData;
      }

      volatile std::size_t m_haveReceivedRealDataCount = 0;
      volatile std::size_t m_attemptedCommunicationCount = 0;
      volatile std::size_t m_attemptedCommunicationConsecutiveFailureCount = 0;
      volatile std::size_t m_attemptedCommunicationConsecutiveSuccessCount = 0;

      boost::asio::io_service device_driver_io_service;
      std::unique_ptr<boost::asio::io_service::work> device_driver_workP_;
      std::unique_ptr<std::thread> driver_threadP;
      std::shared_ptr<AzmqFlatbuffer> kukaJavaDriverP;



     /**
      * \brief Set the joint positions for the current interpolation step.
      *
      * This method is only effective when the robot is in a commanding state
      * and the set time point for reaching the destination is in the future.
      * This function sets the goal time point for a motion to the epoch, aka "time 0" (which is in the past) for safety.
      *
      *
      * For position based motion to work, you must set both the position you want and the time you want it to get there.
      * This is required because the robot can move extremely fast, so accidentally setting the velocity to the max is
      * very dangerous. If the time point is in the past, the robot will not move. If the time point is too near in the future
      * to make it, the robot will move at the max speed towards that position.
      *
      * @see KukaFRIdriver::set(TimePoint && time, time_point_command_tag) to set the destination time point in the future so the position motion can start.
      *
      * @param state Object which stores the current state of the robot, including the command to send next
      * @param range Array with the new joint positions (in radians)
      * @param tag identifier object indicating that revolute joint angle commands should be modified
      */
   template<typename Range>
   void set(Range&& range, grl::revolute_joint_angle_open_chain_command_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       armState_.clearCommands();
       boost::copy(range, std::back_inserter(armState_.commandedPosition));
       boost::copy(range, std::back_inserter(armState_.commandedPosition_goal));
    }

    /**
     *  @brief set the interface over which commands are sent (FRI interface, alternately SmartServo/DirectServo == JAVA interface, )
     */
    void set(flatbuffer::KUKAiiwaInterface cif, command_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       commandInterface_ = cif;
    }

    /**
     *  @brief set the interface over which state is monitored (FRI interface, alternately SmartServo/DirectServo == JAVA interface, )
     */
    void set(flatbuffer::KUKAiiwaInterface mif, state_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       commandInterface_ = mif;
    }

    /**
     *  @brief set the interface over which state is monitored (FRI interface, alternately SmartServo/DirectServo == JAVA interface, )
     */
    void set(std::string dummy_message) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       dummy_message_ = dummy_message;
    }

    /**
    *   @brief set the cartesian impedance stiffness
    */
    void set(grl::flatbuffer::Vector3d cart_stifness_trans, grl::flatbuffer::EulerRotation cart_stifness_rot,cart_stiffness_values)
    {
      boost::lock_guard<boost::mutex> lock(jt_mutex);
      cart_stifness_trans_= cart_stifness_trans;
      cart_stifness_rot_ =  cart_stifness_rot;
    }

    void set(grl::flatbuffer::Vector3d cart_damping_trans, grl::flatbuffer::EulerRotation cart_damping_rot,cart_damping_values)
    {
      boost::lock_guard<boost::mutex> lock(jt_mutex);
      cart_damping_trans_ =  cart_damping_trans;
      cart_damping_rot_   =  cart_damping_rot;

      cartImpValuesChanged = true;
    }

    // Set the max cartesian path deviation in the java driver
    void set(grl::flatbuffer::EulerPose cart_max_path_deviation,max_path_deviation)
    {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       cart_max_path_deviation_ =  cart_max_path_deviation;
       cartImpValuesChanged = true;
    }

    // Set the max cartesian Velocity in the java driver
    void set(grl::flatbuffer::EulerPose cart_max_ctrl_vel,max_cart_vel)
    {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       cart_max_ctrl_vel_ =  cart_max_ctrl_vel;
       cartImpValuesChanged = true;
    }

    // Set the max cartesian control force in the java driver
    void set(grl::flatbuffer::EulerPose cart_max_ctrl_force,max_ctrl_force)
    {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       cart_max_ctrl_force_ =  cart_max_ctrl_force;
       cartImpValuesChanged = true;
    }
    // Set the max cartesian control force in the java driver
    void set(double nullspaceStiffness,double nullspaceDamping, null_space_params)
    {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       nullspaceStiffness_ = nullspaceStiffness;
       nullspaceDamping_   = nullspaceDamping;
       cartImpValuesChanged = true;
    }

    // Set the constant ft control
    void set(std::string ft_dof,double ft_force, double ft_stiffness,set_const_ctrl_force)
    {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       ft_dof_         = ft_dof;
       ft_force_       = ft_force;
       ft_stiffness_   = ft_stiffness;
       cartFTControl_  = true;
    }

    /**
     * @brief Set the time duration expected between new position commands
     *
     * The driver will likely be updated every so often, such as every 25ms, and the lowest level of the
     * driver may update even more frequently, such as every 1ms. By providing as accurate an
     * estimate between high level updates the low level driver can smooth out the motion through
     * interpolation (the default), or another algorithm. See LowLevelStepAlgorithmType template parameter
     * in the KukaFRIdriver class if you want to change out the low level algorithm.
     *
     * @see KukaFRIdriver::get(time_duration_command_tag)
     *
     * @param duration_to_goal_command std::chrono time format representing the time duration between updates
     *
     */
    template<typename TimeDuration>
    void set(TimeDuration && duration_to_goal_command, time_duration_command_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       armState_.goal_position_command_time_duration = duration_to_goal_command;
    }

    /**
     * @brief Get the timestamp of the most recent armState
     *
     *
     *
     * @see KukaFRIdriver::set(Range&& range, grl::revolute_joint_angle_open_chain_command_tag)
     *
     */
    KukaState::time_point_type get(time_point_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       return armState_.timestamp;
    }

     /**
      * \brief Set the applied joint torques for the current interpolation step.
      *
      * This method is only effective when the client is in a commanding state.
      * The ControlMode of the robot has to be joint impedance control mode. The
      * Client Command Mode has to be torque.
      *
      * @param state Object which stores the current state of the robot, including the command to send next
      * @param torques Array with the applied torque values (in Nm)
      * @param tag identifier object indicating that the torqe value command should be modified
      */
   template<typename Range>
   void set(Range&& range, grl::revolute_joint_torque_open_chain_command_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       armState_.clearCommands();
      boost::copy(range, armState_.commandedTorque);
    }


     /**
      * \brief Set the applied wrench vector of the current interpolation step.
      *
      * The wrench vector consists of:
      * [F_x, F_y, F_z, tau_A, tau_B, tau_C]
      *
      * F ... forces (in N) applied along the Cartesian axes of the
      * currently used motion center.
      * tau ... torques (in Nm) applied along the orientation angles
      * (Euler angles A, B, C) of the currently used motion center.
      *
      * This method is only effective when the client is in a commanding state.
      * The ControlMode of the robot has to be Cartesian impedance control mode. The
      * Client Command Mode has to be wrench.
      *
      * @param state object storing the command data that will be sent to the physical device
      * @param range wrench Applied Cartesian wrench vector, in x, y, z, roll, pitch, yaw force measurments.
      * @param tag identifier object indicating that the wrench value command should be modified
      *
      * @todo perhaps support some specific more useful data layouts
      */
   template<typename Range>
   void set(Range&& range, grl::cartesian_wrench_command_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       armState_.clearCommands();
      std::copy(range,armState_.commandedCartesianWrenchFeedForward);
    }

   /// @todo should this exist? is it written correctly?
   void get(KukaState & state)
   {
     boost::lock_guard<boost::mutex> lock(jt_mutex);
     state = armState_;
   }

   /// set the mode of the arm. Examples: Teach or MoveArmJointServo
   /// @see grl::flatbuffer::ArmState in ArmControlState_generated.h
   void set(const flatbuffer::ArmState& armControlMode)
   {
        armControlMode_ = armControlMode;
   }

    private:


      Params params_;
      KukaState armState_;
        // armControlMode is the current GRL_Driver.java configuration to which the arm is currently set.
        // Options are:
        //  ArmState_NONE = 0,
        //  ArmState_StartArm = 1,
        //  ArmState_StopArm = 2,
        //  ArmState_PauseArm = 3,
        //  ArmState_ShutdownArm = 4,
        //  ArmState_TeachArm = 5,
        //  ArmState_MoveArmTrajectory = 6,
        //  ArmState_MoveArmJointServo = 7,
        //  ArmState_MoveArmCartesianServo = 8
      flatbuffer::ArmState                 armControlMode_;
      flatbuffer::KUKAiiwaInterface commandInterface_ = flatbuffer::KUKAiiwaInterface_SmartServo;// KUKAiiwaInterface_SmartServo;
      flatbuffer::KUKAiiwaInterface monitorInterface_ = flatbuffer::KUKAiiwaInterface_FRI;
//      flatbuffers::FlatBufferBuilder       builder_;
//
//      flatbuffer::JointStateBuilder        jointStateServoBuilder_;
//      flatbuffer::MoveArmJointServoBuilder moveArmJointServoBuilder_;
//      flatbuffer::TeachArmBuilder          teachArmBuilder_;
//      flatbuffer::ArmControlStateBuilder   armControlStateBuilder_;
//      flatbuffer::KUKAiiwaStateBuilder     iiwaStateBuilder_;
//      flatbuffer::KUKAiiwaStatesBuilder    iiwaStatesBuilder_;
//
//      flatbuffers::Offset<flatbuffer::KUKAiiwaState> iiwaState;

      boost::mutex jt_mutex;

      int64_t sequenceNumber;

      std::string dummy_message_;
      bool cartImpValuesChanged = false;
      bool cartFTControl_ = false;

      //Cartesian Impedance Values
       grl::flatbuffer::Vector3d cart_stifness_trans_ = grl::flatbuffer::Vector3d(500,500,500);
       grl::flatbuffer::EulerRotation cart_stifness_rot_ = grl::flatbuffer::EulerRotation(200,200,200,grl::flatbuffer::EulerOrder_xyz);

       grl::flatbuffer::Vector3d cart_damping_trans_ = grl::flatbuffer::Vector3d(0.3,0.3,0.3);
       grl::flatbuffer::EulerRotation cart_damping_rot_ = grl::flatbuffer::EulerRotation(0.3,0.3,0.3,grl::flatbuffer::EulerOrder_xyz);
       grl::flatbuffer::EulerPose cart_max_path_deviation_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(1000,1000,100), grl::flatbuffer::EulerRotation(5.,5.,5.,grl::flatbuffer::EulerOrder_xyz));
       grl::flatbuffer::EulerPose cart_max_ctrl_vel_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(1000,1000,1000), grl::flatbuffer::EulerRotation(6.3,6.3,6.3,grl::flatbuffer::EulerOrder_xyz));
       grl::flatbuffer::EulerPose cart_max_ctrl_force_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(200,200,200), grl::flatbuffer::EulerRotation(200.,200.,200.,grl::flatbuffer::EulerOrder_xyz));
       double nullspaceStiffness_ = 2.;
       double nullspaceDamping_ = 0.5;

       std::string ft_dof_ = "X";
       double ft_force_ =5.;
       double ft_stiffness_ = 200;




    };

}}}// namespace grl::robot::arm

#endif
