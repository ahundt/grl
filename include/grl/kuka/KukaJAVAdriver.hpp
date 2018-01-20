/// KukaJavaDriver.hpp handles communication with the Kuka Sunrise Java software.
/// If you are new to this code base you are most likely looking for KukaDriver.hpp
#ifndef GRL_KUKA_JAVA_DRIVER
#define GRL_KUKA_JAVA_DRIVER

#include <iostream>
#include <chrono>
#include <ratio>
#include <thread>
#include <algorithm>        // Required

#include <tuple>
#include <memory>
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
// C++11, use std::false_type and std::true_type
#include <type_traits>
#include <vector>

#include <iterator>            // Required
#include <queue> // Required

//#ifdef BOOST_NO_CXX11_ATOMIC_SMART_PTR
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
//#endif

#include "grl/tags.hpp"
#include "grl/exception.hpp"
#include "grl/kuka/Kuka.hpp"

#include "grl/flatbuffer/JointState_generated.h"
#include "grl/flatbuffer/ArmControlState_generated.h"
#include "grl/flatbuffer/KUKAiiwa_generated.h"
#include "grl/vector_ostream.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <thirdparty/fbs_tk/fbs_tk.hpp>

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
          : params_(params), armControlMode_(flatbuffer::ArmState::NONE)
        {}

        void construct(){ construct(params_); sequenceNumber = 0; }

        /// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
        /// @warning getting the ik group is optional, so it does not throw an exception
        void construct(Params params) {
            try { logger_ = spdlog::stdout_logger_mt("console"); } catch (spdlog::spdlog_ex ex) { logger_ = spdlog::get("console"); }
            std::cout<< "Start KukaJAVAdriver->construct()..." << std::endl;
            params_ = params;

            try {
                logger_->info("KukaLBRiiwaRosPlugin: Connecting UDP Socket from ",
                std::get<LocalUDPAddress>             (params_), ":", std::get<LocalUDPPort>             (params_), " to ",
                std::get<RemoteUDPAddress>            (params_));

                /// @todo TODO(ahundt) switch from linux socket to boost::asio::ip::udp::socket, see Kuka.hpp and KukaFRIdriver.hpp for examples, and make use of KukaUDP class.
                /// AF_INET (IPv4 protocol) , AF_INET6 (IPv6 protocol)
                /// communication type: SOCK_STREAM: TCP(reliable, connection oriented), SOCK_DGRAM: UDP(unreliable, connectionless)
                /// Protocol value for Internet Protocol(IP), which is 0.
                /// socket_local is the socket descriptor, which is an integer.
                socket_local = socket(AF_INET, SOCK_DGRAM, 0);
                if (socket_local < 0) {
                    BOOST_THROW_EXCEPTION(std::runtime_error("KukaJAVAdriver Error opening socket. Check that the port is available, and that all the cables are connected tightly. If you have no other options try restarting your computer."));
                }
                /// convert string to int
                port = boost::lexical_cast<int>( std::get<LocalUDPPort> (params_));
                /// convert the string to network presentation value
                /// inet_pton() returns 1 on success. It returns -1 if there was an error (errno is set), or 0 if the input isn't a valid IP address.
                inet_pton(AF_INET, std::get<LocalUDPAddress>(params_).c_str(), &(local_sockaddr.sin_addr));
                local_sockaddr.sin_family = AF_INET;
                /// htons: host to network short
                local_sockaddr.sin_port = htons(port);
                ///  local_sockaddr.sin_addr.s_addr = INADDR_ANY;

                /// @todo TODO(ahundt) Consider switching to boost::asio synchronous calls (async has high latency)!
                /// @todo TODO(ahundt) Need to switch back to an appropriate exception rather than exiting so VREP isn't taken down.
                /// @todo TODO(ahundt) switch from linux socket to boost::asio::ip::udp::socket, see Kuka.hpp and KukaFRIdriver.hpp for examples, and make use of KukaUDP class.
                /// After creation of the socket, bind function binds the socket to the address and port number specified in local_sockaddr(custom data structure).
                if (bind(socket_local, (struct sockaddr *)&local_sockaddr, sizeof(local_sockaddr)) < 0) {
                    printf("Error binding sr_joint!\n");
                    BOOST_THROW_EXCEPTION(std::runtime_error("KukaJAVAdriver Error opening socket. Check that the port is available, and that all the cables are connected tightly. If you have no other options try restarting your computer."));
                }
                /// clear the socket set defined by fd_set
                FD_ZERO(&mask);
                FD_ZERO(&dummy_mask);
                /// if valid socket descriptor then add to socket set
                FD_SET(socket_local, &mask);
                // set arm to StartArm mode on initalization
                //set(grl::flatbuffer::ArmState::StartArm);
                set(grl::flatbuffer::ArmState::MoveArmJointServo);
                std::cout<< "End KukaJAVAdriver->construct()..." << std::endl;
            } catch( boost::exception &e) {
                e << errmsg_info("KukaLBRiiwaRosPlugin: Unable to connect to UDP Socket from {}{}{}" +
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

                case flatbuffer::ArmState::StartArm: {
                   controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,flatbuffer::CreateStartArm(*fbbP).Union());
                   break;
                }
                case flatbuffer::ArmState::MoveArmJointServo: {

                  /// @todo when new
                  /// armState_ isn't assigned with any values correctly.

                  auto armPositionBuffer = fbbP->CreateVector(armState_.commandedPosition_goal.data(),armState_.commandedPosition_goal.size());
                  auto commandedTorque = fbbP->CreateVector(armState_.commandedTorque.data(),armState_.commandedTorque.size());
                  auto goalJointState = grl::flatbuffer::CreateJointState(*fbbP,armPositionBuffer,0/*no velocity*/,0/*no acceleration*/,commandedTorque);
                  auto moveArmJointServo = grl::flatbuffer::CreateMoveArmJointServo(*fbbP,goalJointState);
                  controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,moveArmJointServo.Union());
                      logger_->info("C++ KukaJAVAdriver: sending armposition command: {}{}", armState_.commandedPosition_goal);
                   break;
                }
                case flatbuffer::ArmState::TeachArm: {
                   controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,flatbuffer::CreateTeachArm(*fbbP).Union());
                   break;
                }
                case flatbuffer::ArmState::PauseArm: {
                   controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,flatbuffer::CreatePauseArm(*fbbP).Union());
                   break;
                }
                case flatbuffer::ArmState::StopArm: {
                   controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,flatbuffer::CreateStopArm(*fbbP).Union());
                   break;
                }
                case flatbuffer::ArmState::ShutdownArm: {
                   controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,armControlMode_,flatbuffer::CreateStopArm(*fbbP).Union());
                   break;
                }
                case flatbuffer::ArmState::NONE: {
                   //std::cerr << "Waiting for interation mode... (currently NONE)\n";
                   break;
                }
                default:
                   logger_->error("C++ KukaJAVAdriver: unsupported use case: {}", EnumNameArmState(armControlMode_));
            }

            auto name = fbbP->CreateString(std::get<RobotName>(params_));

            auto clientCommandMode = grl::flatbuffer::EClientCommandMode::POSITION;
            auto overlayType =  grl::flatbuffer::EOverlayType::NO_OVERLAY;

            //auto stiffnessPose  = flatbuffer::CreateEulerPoseParams(*fbbP,&cart_stiffness_trans_,&cart_stiffness_rot_);
            //auto dampingPose  = flatbuffer::CreateEulerPoseParams(*fbbP,&cart_damping_trans_,&cart_damping_rot_);

            auto setCartesianImpedance = grl::flatbuffer::CreateCartesianImpedenceControlMode(*fbbP, &cart_stiffness_, &cart_damping_,
                  nullspace_stiffness_, nullspace_damping_, &cart_max_path_deviation_, &cart_max_ctrl_vel_, &cart_max_ctrl_force_, max_control_force_stop_);

            auto jointStiffnessBuffer = fbbP->CreateVector(joint_stiffness_.data(),joint_stiffness_.size());
            auto jointDampingBuffer = fbbP->CreateVector(joint_damping_.data(),joint_damping_.size());

            auto setJointImpedance = grl::flatbuffer::CreateJointImpedenceControlMode(*fbbP, jointStiffnessBuffer, jointDampingBuffer);

            auto kukaiiwaArmConfiguration = flatbuffer::CreateKUKAiiwaArmConfiguration(*fbbP,name,commandInterface_,monitorInterface_, clientCommandMode, overlayType,
                        controlMode_, setCartesianImpedance, setJointImpedance);

            bool setArmControlState = true; // only actually change the arm state when this is true.

            // TODO fill the 0s
            auto kukaiiwastate = flatbuffer::CreateKUKAiiwaState(*fbbP,0,0,0,0,1,controlState,setArmConfiguration_,kukaiiwaArmConfiguration);

            auto kukaiiwaStateVec = fbbP->CreateVector(&kukaiiwastate, 1);

            auto states = flatbuffer::CreateKUKAiiwaStates(*fbbP,kukaiiwaStateVec);

            grl::flatbuffer::FinishKUKAiiwaStatesBuffer(*fbbP, states);

            flatbuffers::Verifier verifier(fbbP->GetBufferPointer(),fbbP->GetSize());
            BOOST_VERIFY(grl::flatbuffer::VerifyKUKAiiwaStatesBuffer(verifier));


            if(armControlMode_ == flatbuffer::ArmState::MoveArmJointServo)
            {
                auto states2 = flatbuffer::GetKUKAiiwaStates(fbbP->GetBufferPointer());
                auto movearm = static_cast<const flatbuffer::MoveArmJointServo*>(states2->states()->Get(0)->armControlState()->state());
                std::vector<double> angles;
                for(std::size_t i = 0; i <  movearm->goal()->position()->size(); ++i)
                {
                  angles.push_back(movearm->goal()->position()->Get(i));
                }
                logger_->info("re-extracted {}{}{}", movearm->goal()->position()->size(), " joint angles: ",angles);
            }

            if(debug_) logger_->info("sending packet to KUKA iiwa: len = {}", fbbP->GetSize());
            int ret;
            // Send UDP packet to Robot
            ret = sendto(socket_local, fbbP->GetBufferPointer(), fbbP->GetSize(), 0, (struct sockaddr *)&dst_sockaddr, sizeof(dst_sockaddr));

            if (static_cast<long>(ret) != static_cast<long>(fbbP->GetSize())) logger_->error("Error sending packet to KUKA iiwa: ret = {}, len = {}", ret, fbbP->GetSize());

            setArmConfiguration_ = false;

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
                        // allocate the buffer, should only happen once
                        if(!java_interface_received_statesP_) {
                          java_interface_received_statesP_ = std::make_shared<fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates>>(fbs_tk::Buffer(udp_size_));
                        }
                        if(!java_interface_next_statesP_) {
                            java_interface_next_statesP_ = std::make_shared<fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates>>(fbs_tk::Buffer(udp_size_));
                        }
                        static const int flags = 0;
                        // get a reference to the buffer object
                        const fbs_tk::Buffer& internal_buffer = java_interface_received_statesP_->get_data();
                        // receive an update from the java driver over UDP
                        std::vector<uint8_t> bufferdata = internal_buffer.get_data();
                        ret = recvfrom(socket_local, reinterpret_cast<void*>(&bufferdata[0]), internal_buffer.size(), flags, (struct sockaddr *)&dst_sockaddr, &dst_sockaddr_len);
                        if (ret <= 0) {
                          bool java_state_received_successfully = false;
                          logger_->error("C++ KukaJAVAdriver Error: Receive failed with ret = {}", ret);
                        } else {
                             if(debug_) logger_->info("C++ KukaJAVAdriver received message size: {}",ret);
                             java_interface_received_statesP_->update_root();
                             // Flatbuffer has been verified as valid
                             if (java_interface_received_statesP_->valid()) {
                                bool java_state_received_successfully = true;
                                std::swap(java_interface_received_statesP_, java_interface_next_statesP_);
                                if(debug_) logger_->info("C++ KukaJAVAdriver: flatbuffer verified successfully");
                             } else {
                                // TODO(ahundt) consider specific error codes for verifier failure vs udp receive failure
                                bool java_state_received_successfully = false;
                                logger_->error("C++ KukaJAVAdriver Error: flatbuff failed verification. bufOk: {}", java_state_received_successfully);
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
        void setPositionControlMode()
        {
          boost::lock_guard<boost::mutex> lock(jt_mutex);
          controlMode_ = grl::flatbuffer::EControlMode::POSITION_CONTROL_MODE;
          setArmConfiguration_ = true;
        }
        bool setJointImpedanceMode(std::vector<double> joint_stiffnes, std::vector<double>joint_damping) {
          boost::lock_guard<boost::mutex> lock(jt_mutex);
          //TODO use tags
          joint_stiffness_ = joint_stiffnes;
          joint_damping_ = joint_damping;
          controlMode_ = grl::flatbuffer::EControlMode::JOINT_IMP_CONTROL_MODE;
          setArmConfiguration_ = true;
        }
        // TODO: define custom flatbuffer for Cartesion Quantities
        void setCartesianImpedanceMode(
            const grl::flatbuffer::EulerPose cart_stiffness,
            const grl::flatbuffer::EulerPose cart_damping,
            const double nullspace_stiffness,
            const double nullspace_damping,
            const grl::flatbuffer::EulerPose cart_max_path_deviation,
            const grl::flatbuffer::EulerPose cart_max_ctrl_vel,
            const grl::flatbuffer::EulerPose cart_max_ctrl_force,
            const bool max_control_force_stop)
        {
            boost::lock_guard<boost::mutex> lock(jt_mutex);

            cart_stiffness_ = cart_stiffness;
            cart_damping_ = cart_damping;

            cart_max_path_deviation_  = cart_max_path_deviation;
            cart_max_ctrl_vel_ = cart_max_ctrl_vel;
            cart_max_ctrl_force_  = cart_max_ctrl_force;

            nullspace_stiffness_ = nullspace_stiffness;
            nullspace_damping_ = nullspace_damping;

            max_control_force_stop_ = max_control_force_stop;

            controlMode_ = grl::flatbuffer::EControlMode::CART_IMP_CONTROL_MODE;
            setArmConfiguration_ = true;
        }

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
    // KukaState::time_point_type get(time_point_tag) {
    //    boost::lock_guard<boost::mutex> lock(jt_mutex);
    //    return armState_.timestamp;
    // }
    cartographer::common::Time get(time_point_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       return armState_.time_event_stamp.device_time;
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

   /// get 6 element wrench entries
   /// [force_x, force_y, force_z, torque_x, torque_y, torque_z]
   /*
    template<typename OutputIterator>
    void getWrench(OutputIterator output)
    {
        boost::lock_guard<boost::mutex> lock(jt_mutex);

       // make sure the object exists and contains data
        if (java_interface_received_statesP_ && java_interface_received_statesP_->valid())
        {
            std::cout<< "eqTypes Test: " << eqTypes<int, double>() << std::endl;
            // T* operator->() overload
            // https://stackoverflow.com/questions/38542766/why-the-t-operator-is-applied-repeatedly-even-if-written-once
            auto wrench = (*java_interface_received_statesP_)->states()->Get(0)->monitorState()->CartesianWrench();
            // insert the values into the output vector
            *output++ = wrench->force().x();  // double
            *output++ = wrench->force().y();
            *output++ = wrench->force().z();
            *output++ = wrench->torque().x();
            *output++ = wrench->torque().y();
            *output++ = wrench->torque().z();
        }
   }
   */
  template<typename Container>
    void getWrench(Container output)
    {
        boost::lock_guard<boost::mutex> lock(jt_mutex);

       // make sure the object exists and contains data
        if (java_interface_received_statesP_ && java_interface_received_statesP_->valid())
        {
            std::cout<< "eqTypes Test: " << eqTypes<int, double>() << std::endl;
            // T* operator->() overload
            // https://stackoverflow.com/questions/38542766/why-the-t-operator-is-applied-repeatedly-even-if-written-once
            auto wrench = (*java_interface_received_statesP_)->states()->Get(0)->monitorState()->CartesianWrench();
            // insert the values into the output vector
            output.push_back(wrench->force().x());
            output.push_back(wrench->force().y());
            output.push_back(wrench->force().z());
            output.push_back(wrench->torque().x());
            output.push_back(wrench->torque().y());
            output.push_back(wrench->torque().z());
        }
   }

   /// set the mode of the arm. Examples: Teach or MoveArmJointServo
   /// @see grl::flatbuffer::ArmState in ArmControlState_generated.h
   void set(const flatbuffer::ArmState& armControlMode)
   {
        boost::lock_guard<boost::mutex> lock(jt_mutex);
        armControlMode_ = armControlMode;
   }

   /// get the mode of the arm. Examples: Teach or MoveArmJointServo
   /// @see grl::flatbuffer::ArmState in ArmControlState_generated.h
   void get(flatbuffer::ArmState& armControlMode)
   {
        boost::lock_guard<boost::mutex> lock(jt_mutex);
        armControlMode = armControlMode_;
   }

   /////////////////////////////////////////////////////////////////////
   /* Helper function for checking type equality in compile-time. */
   /// https://stackoverflow.com/questions/16924168/compile-time-function-for-checking-type-equality
    template<typename T, typename U>
    struct is_same : std::false_type { };

    template<typename T>
    struct is_same<T, T> : std::true_type { };

    template<typename T, typename U>
    constexpr bool eqTypes() { return is_same<T, U>::value; }
   /////////////////////////////////////////////////////////////////////

    private:

      /// @TODO(ahundt) don't assume this fixed size, consider a parameter
      static const std::size_t udp_size_ = 1400;
      std::shared_ptr<spdlog::logger> logger_;
      int socket_local;
      int port;
      struct sockaddr_in  dst_sockaddr;
      socklen_t  dst_sockaddr_len = sizeof(dst_sockaddr);
      struct sockaddr_in local_sockaddr;
      long dst_ip;
      /// An fd_set is a set of sockets to "monitor" for some activity (set of socket descriptors).
      fd_set mask, temp_mask, dummy_mask;

      Params params_;
      KukaState armState_;  // structure defined in Kuka.hpp
      // this is the data that was received from the remote computer
      // with the data from the java interface that gets sent back and forth
      // this one will only contain valid received data
      std::shared_ptr<fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates>> java_interface_received_statesP_;
      /// indicates if the next state was received successfully in java_interface_next_statesP_
      /// and java_interface_received_statesP_ was updated accordingly.
      bool java_state_received_successfully = false;
      /// used for receiving the next buffer over the network.
      /// if some receive calls fail this one may not contain valid data.
      std::shared_ptr<fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates>> java_interface_next_statesP_;
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
      flatbuffer::KUKAiiwaInterface commandInterface_ = flatbuffer::KUKAiiwaInterface::SmartServo;// KUKAiiwaInterface::SmartServo;
      flatbuffer::KUKAiiwaInterface monitorInterface_ = flatbuffer::KUKAiiwaInterface::FRI;
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

      bool setArmConfiguration_ = true; // set the arm config first time

      grl::flatbuffer::EControlMode controlMode_ = grl::flatbuffer::EControlMode::POSITION_CONTROL_MODE;

      //TODO: Custom flatbuffer type. Load defaults from params/config
      //Cartesian Impedance Values
      grl::flatbuffer::Vector3d cart_stiffness_trans_ = grl::flatbuffer::Vector3d(500,500,500);
      grl::flatbuffer::EulerRotation cart_stifness_rot_ = grl::flatbuffer::EulerRotation(200,200,200,grl::flatbuffer::EulerOrder::xyz);

      grl::flatbuffer::Vector3d cart_damping_trans_ = grl::flatbuffer::Vector3d(0.3,0.3,0.3);
      grl::flatbuffer::EulerRotation cart_damping_rot_ = grl::flatbuffer::EulerRotation(0.3,0.3,0.3,grl::flatbuffer::EulerOrder::xyz);

      grl::flatbuffer::EulerPose cart_stiffness_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(500,500,500), grl::flatbuffer::EulerRotation(200,200,200,grl::flatbuffer::EulerOrder::xyz));
      grl::flatbuffer::EulerPose cart_damping_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(0.3,0.3,0.3), grl::flatbuffer::EulerRotation(0.3,0.3,0.3,grl::flatbuffer::EulerOrder::xyz));

      grl::flatbuffer::EulerPose cart_max_path_deviation_  = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(1000,1000,100), grl::flatbuffer::EulerRotation(5.,5.,5., grl::flatbuffer::EulerOrder::xyz));
      grl::flatbuffer::EulerPose cart_max_ctrl_vel_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(1000,1000,1000), grl::flatbuffer::EulerRotation(6.3,6.3,6.3, grl::flatbuffer::EulerOrder::xyz));
      grl::flatbuffer::EulerPose cart_max_ctrl_force_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(200,200,200), grl::flatbuffer::EulerRotation(200.,200.,200., grl::flatbuffer::EulerOrder::xyz));

      double nullspace_stiffness_ = 2.0;
      double nullspace_damping_ = 0.5;

      bool max_control_force_stop_ = false;

      //Joint Impedance Values TODO: set default values?
      std::vector<double> joint_stiffness_;
      std::vector<double> joint_damping_;

    };  // End of class

}}}// namespace grl::robot::arm

#endif
