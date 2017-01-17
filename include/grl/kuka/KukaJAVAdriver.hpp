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
#include <boost/lexical_cast.hpp>

#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/chrono/include.hpp>
#include <boost/chrono/duration.hpp>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>

//#ifdef BOOST_NO_CXX11_ATOMIC_SMART_PTR
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
//#endif

#include <spdlog/spdlog.h>

#include "grl/tags.hpp"
#include "grl/exception.hpp"
#include "grl/kuka/Kuka.hpp"

#include "grl/flatbuffer/JointState_generated.h"
#include "grl/flatbuffer/ArmControlState_generated.h"
#include "grl/flatbuffer/KUKAiiwa_generated.h"
#include "grl/vector_ostream.hpp"


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
        LocalUDPAddress,
        LocalUDPPort,
        RemoteUDPAddress,
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
        std::string,
        std::string
          > Params;


      static const Params defaultParams(){
        return std::make_tuple(
            "Robotiiwa"               , // RobotName,
            "KUKA_LBR_IIWA_14_R820"   , // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800)
            "0.0.0.0"                 , // LocalUDPAddress
            "30010"                   , // LocalUDPPort
            "172.31.1.147"            , // RemoteUDPAddress
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
      {logger_ = spdlog::get("console");}

      void construct(){ construct(params_); sequenceNumber = 0; }

      /// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
      /// @warning getting the ik group is optional, so it does not throw an exception
      void construct(Params params) {

        params_ = params;


        try {
          logger_->info("KukaLBRiiwaRosPlugin: Connecting UDP Socket from ",
            std::get<LocalUDPAddress>             (params_), ":", std::get<LocalUDPPort>             (params_), " to ",
            std::get<RemoteUDPAddress>            (params_));

            /// @todo TODO(ahundt) switch from linux socket to boost::asio::ip::udp::socket, see Kuka.hpp and KukaFRIdriver.hpp for examples, and make use of KukaUDP class.
            socket_local = socket(AF_INET, SOCK_DGRAM, 0);
            if (socket_local < 0) {
                 BOOST_THROW_EXCEPTION(std::runtime_error("KukaJAVAdriver Error opening socket. Check that the port is available, and that all the cables are connected tightly. If you have no other options try restarting your computer."));
            }

            port = boost::lexical_cast<int>( std::get<LocalUDPPort>             (params_));
            // convert the string to network presentation value
            inet_pton(AF_INET, std::get<LocalUDPAddress>(params_).c_str(), &(local_sockaddr.sin_addr));
            local_sockaddr.sin_family = AF_INET;
            local_sockaddr.sin_port = htons(port);
        //    local_sockaddr.sin_addr.s_addr = INADDR_ANY;

            /// @todo TODO(ahundt) Consider switching to boost::asio synchronous calls (async has high latency)!
            /// @todo TODO(ahundt) Need to switch back to an appropriate exception rather than exiting so VREP isn't taken down.
            /// @todo TODO(ahundt) switch from linux socket to boost::asio::ip::udp::socket, see Kuka.hpp and KukaFRIdriver.hpp for examples, and make use of KukaUDP class.
            if (bind(socket_local, (struct sockaddr *)&local_sockaddr, sizeof(local_sockaddr)) < 0) {
                printf("Error binding sr_joint!\n");
                BOOST_THROW_EXCEPTION(std::runtime_error("KukaJAVAdriver Error opening socket. Check that the port is available, and that all the cables are connected tightly. If you have no other options try restarting your computer."));
            }

            FD_ZERO(&mask);
            FD_ZERO(&dummy_mask);
            FD_SET(socket_local, &mask);


        } catch( boost::exception &e) {
          e << errmsg_info("KukaLBRiiwaRosPlugin: Unable to connect to UDP Socket from " +
                           std::get<LocalUDPAddress>             (params_) + " to " +
                           std::get<RemoteUDPAddress>            (params_));
          throw;
        }
      }




      const Params & getParams(){
        return params_;
      }

      /// shuts down the arm
      bool destruct(){
          close(socket_local);
          return true;
      }

      ~KukaJAVAdriver(){
         /// @todo TODO(ahundt) switch to asio, remove destruct call from destructor
         destruct();
      }


      /// @brief SEND COMMAND TO ARM. Call this often
      /// Performs the main update spin once.
      /// @todo ADD SUPPORT FOR READING ARM STATE OVER JAVA INTERFACE
      bool run_one(){

        // @todo CHECK FOR REAL DATA BEFORE SENDING COMMANDS
        //if(!m_haveReceivedRealDataCount) return;

        bool haveNewData = false;

        /// @todo make this handled by template driver implementations/extensions





           std::shared_ptr<flatbuffers::FlatBufferBuilder> fbbP;
           fbbP = std::make_shared<flatbuffers::FlatBufferBuilder>();

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

          auto kukaiiwaArmConfiguration = flatbuffer::CreateKUKAiiwaArmConfiguration(*fbbP,name,commandInterface_,monitorInterface_);

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
              std::cout << "re-extracted " << movearm->goal()->position()->size() << " joint angles: ";
              for(std::size_t i = 0; i <  movearm->goal()->position()->size(); ++i)
              {
                std::cout << i << "=" << movearm->goal()->position()->Get(i) << ", ";
              }
              std::cout << "\n";
          }

          int ret;
          // Send UDP packet to Robot
          ret = sendto(socket_local, fbbP->GetBufferPointer(), fbbP->GetSize(), 0, (struct sockaddr *)&dst_sockaddr, sizeof(dst_sockaddr));
          
          if (static_cast<long>(ret) != static_cast<long>(fbbP->GetSize())) printf("Error sending packet to KUKA iiwa: ret = %d, len = %u\n", ret, fbbP->GetSize());


              // Receiving data from Sunrise

              int num;
              temp_mask = mask;

              // tv is the time that you wait for a new message to arrive,
              // since we don't wont to hinder the execution of each cycle, it is set to zero...
              // (The FRI interface read rate is much higher than the data coming for the controller, so we don't wait for controller data)
              struct timeval tv;
              tv.tv_sec = 0;
              tv.tv_usec = 0;

              // Check to see if any packets are available with waiting time of 0
              // Please note that with this configuration some packets may be dropped.
              /// @todo TODO(ahundt) eventually run this in a separate thread so we can receive packets asap and minimize dropping
              num = select(FD_SETSIZE, &temp_mask, &dummy_mask, &dummy_mask, &tv);

                if (num > 0)
                {
                      // packets are available, process them
                      if (FD_ISSET(socket_local, &temp_mask))
                      {
                           static const std::size_t udp_size = 1400;
                           unsigned char recbuf[udp_size];
                           static const int flags = 0;
                      
                           ret = recvfrom(socket_local, recbuf, sizeof(recbuf), flags, (struct sockaddr *)&dst_sockaddr, &dst_sockaddr_len);
                           if (ret <= 0) printf("Receive Error: ret = %d\n", ret);

                           if (ret > 0){

                               if(debug_) std::cout << "received message size: " << ret << "\n";


                               auto rbPstart = static_cast<const uint8_t *>(recbuf);

                               auto verifier = flatbuffers::Verifier(rbPstart, ret);
                               auto bufOK = grl::flatbuffer::VerifyKUKAiiwaStatesBuffer(verifier);
                                
                               // Flatbuffer has been verified as valid
                               if (bufOK) {
                                   // only reading the wrench data currently
                                   auto bufff = static_cast<const void *>(rbPstart);
                                   if(debug_) std::cout << "Succeeded in verification.  " << "\n";

                                   auto fbKUKAiiwaStates = grl::flatbuffer::GetKUKAiiwaStates(bufff);
                                   auto wrench = fbKUKAiiwaStates->states()->Get(0)->monitorState()->CartesianWrench();

                                   armState_.wrenchJava.clear();
                                   armState_.wrenchJava.push_back(wrench->force().x());
                                   armState_.wrenchJava.push_back(wrench->force().y());
                                   armState_.wrenchJava.push_back(wrench->force().z());
                                   armState_.wrenchJava.push_back(wrench->torque().x());
                                   armState_.wrenchJava.push_back(wrench->torque().y());
                                   armState_.wrenchJava.push_back(wrench->torque().z());


                               } else {
                                   std::cout << "Failed verification. bufOk: " << bufOK << "\n";
                               }


                           }

                      }
                }


         return haveNewData;
      }

      volatile std::size_t m_haveReceivedRealDataCount = 0;
      volatile std::size_t m_attemptedCommunicationCount = 0;
      volatile std::size_t m_attemptedCommunicationConsecutiveFailureCount = 0;
      volatile std::size_t m_attemptedCommunicationConsecutiveSuccessCount = 0;


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

   void getWrench(KukaState & state)
   { boost::lock_guard<boost::mutex> lock(jt_mutex);

       if (!armState_.wrenchJava.empty()) {
           state.wrenchJava = armState_.wrenchJava;
       }
   }

   /// set the mode of the arm. Examples: Teach or MoveArmJointServo
   /// @see grl::flatbuffer::ArmState in ArmControlState_generated.h
   void set(const flatbuffer::ArmState& armControlMode)
   {
        armControlMode_ = armControlMode;
   }

    private:

      std::shared_ptr<spdlog::logger> logger_;
      int socket_local;
      int port;
      struct sockaddr_in  dst_sockaddr;
      socklen_t  dst_sockaddr_len = sizeof(dst_sockaddr);
      struct sockaddr_in local_sockaddr;
      long dst_ip;
      fd_set mask, temp_mask, dummy_mask;

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
      
      bool debug_ = false;

    };

}}}// namespace grl::robot::arm

#endif
