/// KukaFRIDriver.hpp handles communication with the Kuka over FRI.
/// If you are new to this code base you are most likely looking for KukaDriver.hpp
#ifndef _KUKA_FRI_DRIVER
#define _KUKA_FRI_DRIVER

#include "grl/kuka/KukaFRIClientDataDriver.hpp"

struct KukaState;

namespace grl {
namespace robot {
namespace arm {

/// @brief Primary Kuka FRI driver, only talks over realtime network FRI KONI
/// ethernet port
///
///
/// @note If you aren't sure, see KukaDriver in KukaDriver.hpp.
///
/// @note If you want to change how the lowest level high rate updates are
/// performed, make another version of this class or update so
/// LowLevelStepAlgorithmType, like LinearInterpolation,
/// is configurable. @see KukaFRIdriver
///
///
/// KukaFRIdriver is a low level driver at a slightly "higher level" than the
/// the "lowest level" KukaFRIClientDataDriver
/// to communicate. This is the class you will want to replace if you want to
/// change how low level position
/// updates are changed between FRI update steps, which occur at a configurable
/// duration of 1-5 ms.
///
/// For position based motion to work, you must set both the position you want
/// and the time you want it to get there.
/// This is required because the robot can move extremely fast, so accidentally
/// setting the velocity to the max is
/// very dangerous. If the time point is in the past, the robot will not move.
/// If the time point is too near in the future
/// to make it, the robot will move at the max speed towards that position.
///
///
/// While velocity limits are not provided explicitly by KUKA in their low level
/// C++ API,
/// if you send a command that violates the velocity limits specified in KUKA's
/// documenation
/// the arm stops immediately with an error, even over FRI.
///
/// @todo support generic read/write
/// @todo ensure commands stay within machine limits to avoid lockup
/// @todo reset and resume if lockup occurs whenever possible
/// @todo in classes that use this driver, make the use of this class templated
/// so that the low level update algorithm can change.
/// @todo add getter for number of milliseconds between fri updates (1-5) aka
/// sync_period aka send_period aka ms per tick

/// std::enable_shared_from_this allows an object t that is currently managed by
/// a std::shared_ptr named pt to safely generate additional std::shared_ptr instances pt1, pt2, ...
/// that all share ownership of t with pt.
template <typename LowLevelStepAlgorithmType = LinearInterpolation>
class KukaFRIdriver : public std::enable_shared_from_this<
                          KukaFRIdriver<LowLevelStepAlgorithmType>>,
                      public KukaUDP {

public:
    using KukaUDP::ParamIndex;  // enum, define some connection parameters, such as ip, port...
    using KukaUDP::ThreadingRunMode;
    using KukaUDP::Params;      // std::tuple, contains the information needed to connect to the robot.
    using KukaUDP::defaultParams;  // A method to assign Params with defaut values.

    KukaFRIdriver(Params params = defaultParams()) : params_(params),  m_shouldStop(false) {}

    //      KukaFRIdriver(boost::asio::io_service&
    //      device_driver_io_service__,Params params = defaultParams())
    //        :
    //        device_driver_io_service(device_driver_io_service__),
    //        params_(params)
    //      {}

    void construct() { construct(params_); }

    /// @todo create a function that calls simGetObjectHandle and throws an
    /// exception when it fails
    /// @warning getting the ik group is optional, so it does not throw an
    /// exception
    void construct(Params params) {

        params_ = params;
        // keep driver threads from exiting immediately after creation, because they
        // have work to do!
        // boost::asio::io_service::work: Constructor notifies the io_service that work is starting.
        device_driver_workP_.reset(
            new boost::asio::io_service::work(device_driver_io_service));

        kukaFRIClientDataDriverP_.reset(
            new grl::robot::arm::KukaFRIClientDataDriver<LowLevelStepAlgorithmType>(
                device_driver_io_service,
                std::make_tuple(std::string(std::get<RobotModel>(params)),
                                std::string(std::get<localhost>(params)),
                                std::string(std::get<localport>(params)),
                                std::string(std::get<remotehost>(params)),
                                std::string(std::get<remoteport>(params)),
                                grl::robot::arm::KukaFRIClientDataDriver<
                                    LowLevelStepAlgorithmType>::run_automatically))

                );
        // flatbuffersbuilder does not yet exist
        m_logFileBufferBuilderP = std::make_shared<flatbuffers::FlatBufferBuilder>();
        m_KUKAiiwaStateBufferP = std::make_shared<std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState>>>();
        #ifdef HAVE_spdlog
            loggerP = spdlog::stdout_logger_mt("logs/kukaiiwa_logger.txt");
        #endif // HAVE_spdlog
    }

    const Params &getParams() { return params_; }

    ~KukaFRIdriver() {
        device_driver_workP_.reset();

        if (driver_threadP) {
          device_driver_io_service.stop();
          driver_threadP->join();
        }
    }

    /// gets the number of seconds in one message exchange "tick" aka "cycle",
    /// "time step" of the robot arm's low level controller
    double getSecondsPerTick() {
        return std::chrono::duration_cast<std::chrono::seconds>(
                 std::chrono::milliseconds(grl::robot::arm::get(
                     friData_->monitoringMsg, grl::time_step_tag()))).count();
    }

    /// @todo make this configurable for different specific robots. Currently set
    /// for kuka lbr iiwa 14kg R820
    KukaState::joint_state getMaxVel() {
        KukaState::joint_state maxVel;
        /// get max velocity constraint parameter for this robot model
        copy(std::get<RobotModel>(params_), std::back_inserter(maxVel),
             grl::revolute_joint_velocity_open_chain_state_constraint_tag());

        // scale velocity down to a single timestep. In other words multiply each
        // velocity by the number of seconds in a tick, likely 0.001-0.005
        boost::transform(
            maxVel, maxVel.begin(),
            std::bind2nd(std::multiplies<KukaState::joint_state::value_type>(),
                       getSecondsPerTick()));

        return maxVel;
    }

    /**
     * spin once, this is what you call each time you synchronize the client with
     * the robot over UDP
     * it is expected to be called at least once per send_period_millisec, which
     * is the time between
     * each FRI udp packet.
     *
     */
    bool run_one() {
        grl::TimeEvent time_event_stamp;
        // note: this one sends *and* receives the joint data!
        BOOST_VERIFY(kukaFRIClientDataDriverP_.get() != nullptr);
        /// @todo use runtime calculation of NUM_JOINTS instead of constant
        if (!friData_) {
            friData_ = std::make_shared<KUKA::FRI::ClientData>(KUKA::LBRState::NUM_DOF);
        }

        bool haveNewData = false;

        static const std::size_t minimumConsecutiveSuccessesBeforeSendingCommands = 100;

        std::shared_ptr<typename LowLevelStepAlgorithmType::Params> lowLevelStepAlgorithmCommandParamsP;

        /// @todo probably only need to set this once
        armState.velocity_limits.clear();
        armState.velocity_limits = getMaxVel();

        // This is the key point where the arm's motion goal command is updated and
        // sent to the robot
        // Set the FRI to the simulated joint positions
        if (this->m_haveReceivedRealDataCount > minimumConsecutiveSuccessesBeforeSendingCommands) {
            /// @todo TODO(ahundt) Need to eliminate this allocation
            boost::lock_guard<boost::mutex> lock(jt_mutex);

            boost::container::static_vector<double, 7> jointStateToCommand;
            boost::copy(armState.commandedPosition,std::back_inserter(jointStateToCommand));
            // pass time to reach specified goal for position control
            lowLevelStepAlgorithmCommandParamsP = std::make_shared<grl::robot::arm::LinearInterpolation::Params>(std::make_tuple(jointStateToCommand,armState.goal_position_command_time_duration));
            /// @todo construct new low level command object and pass to
            /// KukaFRIClientDataDriver
            /// this is where we used to setup a new FRI command

            // std::cout << "commandToSend: " << commandToSend << "\n" <<
            // "currentJointPos: " << currentJointPos << "\n" << "amountToMove: " <<
            // amountToMove << "\n" << "maxVel: " << maxvel << "\n";
        } else {
            /// @todo TODO(ahundt) BUG: Need way to pass time to reach specified goal for position control and eliminate this allocation
            lowLevelStepAlgorithmCommandParamsP.reset(new typename LowLevelStepAlgorithmType::Params());
        }

        BOOST_VERIFY(lowLevelStepAlgorithmCommandParamsP != nullptr);
        boost::system::error_code send_ec, recv_ec;
        std::size_t send_bytes, recv_bytes;
        // sync with device over network
        haveNewData = !kukaFRIClientDataDriverP_->update_state(
            lowLevelStepAlgorithmCommandParamsP, friData_, recv_ec, recv_bytes, send_ec,
            send_bytes, time_event_stamp);
        m_attemptedCommunicationCount++;

        if (haveNewData) {
            boost::lock_guard<boost::mutex> lock(jt_mutex);
            // if there were problems sending commands, start by sending the current
            // position
            //            if(this->m_haveReceivedRealDataCount >
            //            minimumConsecutiveSuccessesBeforeSendingCommands-1)
            //            {
            //              boost::lock_guard<boost::mutex> lock(jt_mutex);
            //              // initialize arm commands to current arm position
            //              armState.clearCommands();
            ////              armState.commandedPosition.clear();
            ////              armState.commandedTorque.clear();
            ////              grl::robot::arm::copy(friData_->monitoringMsg,
            /// std::back_inserter(armState.commandedPosition),
            /// grl::revolute_joint_angle_open_chain_command_tag());
            ////              grl::robot::arm::copy(friData_->monitoringMsg,
            /// std::back_inserter(armState.commandedTorque)   ,
            /// grl::revolute_joint_torque_open_chain_command_tag());
            //            }

            m_attemptedCommunicationConsecutiveSuccessCount++;
            this->m_attemptedCommunicationConsecutiveFailureCount = 0;
            this->m_haveReceivedRealDataCount++;

            // We have the real kuka state read from the device now
            // update real joint angle data
            armState.position.clear();
            grl::robot::arm::copy(friData_->monitoringMsg,
                                  std::back_inserter(armState.position),
                                  grl::revolute_joint_angle_open_chain_state_tag());

            armState.torque.clear();
            grl::robot::arm::copy(friData_->monitoringMsg,
                                  std::back_inserter(armState.torque),
                                  grl::revolute_joint_torque_open_chain_state_tag());

            armState.externalTorque.clear();
            grl::robot::arm::copy(
                friData_->monitoringMsg, std::back_inserter(armState.externalTorque),
                grl::revolute_joint_torque_external_open_chain_state_tag());

            // only supported for kuka sunrise OS 1.9
            #ifdef KUKA_SUNRISE_1_9
                  armState.externalForce.clear();
                  grl::robot::arm::copy(friData_->monitoringMsg,
                                        std::back_inserter(armState.externalForce),
                                        grl::cartesian_external_force_tag());
            #endif // KUKA_SUNRISE_1_9
            armState.ipoJointPosition.clear();
            grl::robot::arm::copy(
                friData_->monitoringMsg,
                std::back_inserter(armState.ipoJointPosition),
                grl::revolute_joint_angle_interpolated_open_chain_state_tag());

            armState.sendPeriod = std::chrono::milliseconds(
                grl::robot::arm::get(friData_->monitoringMsg, grl::time_step_tag()));

            armState.time_event_stamp = time_event_stamp;

            std::cout << "Measured Torque: ";
            std::cout << std::setw(6);
            for (float t:armState.torque) {
                std::cout << t << " ";
            }
            std::cout << '\n';
            std::cout << "External Torque: ";
            std::cout << std::setw(6);
            for (float t:armState.externalTorque) {
                std::cout << t << " ";
            }
            std::cout << '\n';
            std::cout << "External Force: ";
            for (float t:armState.externalForce) {
                std::cout << t << " ";
            }
            std::cout << '\n';


            // TODO(chunting) add data to log here, when full write to disk on a separate thread like FusionTrackLogAndTrack
            save_oneKUKAiiwaStateBuffer();
            update();
            // m_driverThread.reset(new std::thread(&KukaFRIdriver::save_recording));
        } else {
            m_attemptedCommunicationConsecutiveFailureCount++;
            std::cerr << "No new FRI data available, is an FRI application running "
                         "on the Kuka arm? \n Total sucessful transfers: "
                      << this->m_haveReceivedRealDataCount
                      << "\n Total attempts: " << m_attemptedCommunicationCount
                      << "\n Consecutive Failures: "
                      << m_attemptedCommunicationConsecutiveFailureCount
                      << "\n Consecutive Successes: "
                      << m_attemptedCommunicationConsecutiveSuccessCount << "\n";
            m_attemptedCommunicationConsecutiveSuccessCount = 0;
            /// @todo TODO(ahundt) Add time information from update_state call here for debugging purposes
            /// @todo TODO(ahundt) should the results of getlatest state even be possible to call
            /// without receiving real data? should the library change?
            /// @todo TODO(ahundt) use spdlog library instead of cerr?
        }
        return haveNewData;
    }

    /**
     * \brief Set the joint positions for the current interpolation step.
     *
     * This method is only effective when the robot is in a commanding state
     * and the set time point for reaching the destination is in the future.
     * This function sets the goal time point for a motion to the epoch, aka "time
     * 0" (which is in the past) for safety.
     *
     *
     * For position based motion to work, you must set both the position you want
     * and the time you want it to get there.
     * This is required because the robot can move extremely fast, so accidentally
     * setting the velocity to the max is
     * very dangerous. If the time point is in the past, the robot will not move.
     * If the time point is too near in the future
     * to make it, the robot will move at the max speed towards that position.
     *
     * @see KukaFRIdriver::set(TimePoint && duration_to_goal_command, time_duration_command_tag) to set
     * the destination time point in the future so the position motion can start.
     *
     * @param state Object which stores the current state of the robot, including
     * the command to send next
     * @param range Array with the new joint positions (in radians)
     * @param tag identifier object indicating that revolute joint angle commands
     * should be modified
     */
    template <typename Range>
    void set(Range &&range, grl::revolute_joint_angle_open_chain_command_tag) {
      boost::lock_guard<boost::mutex> lock(jt_mutex);
      armState.clearCommands();
      boost::copy(range, std::back_inserter(armState.commandedPosition));
      boost::copy(range, std::back_inserter(armState.commandedPosition_goal));
    }

    /**
     * @brief Set the time duration expected between new position commands in ms
     *
     * The driver will likely be updated every so often, such as every 25ms, and
     * the lowest level of the
     * driver may update even more frequently, such as every 1ms. By providing as
     * accurate an
     * estimate between high level updates the low level driver can smooth out the
     * motion through
     * interpolation (the default), or another algorithm. See
     * LowLevelStepAlgorithmType template parameter
     * in the KukaFRIdriver class if you want to change out the low level
     * algorithm.
     *
     * @see KukaFRIdriver::get(time_duration_command_tag)
     *
     * @param duration_to_goal_command std::chrono time format representing the
     * time duration between updates
     *
     */
    void set(double duration_to_goal_command, time_duration_command_tag) {
      boost::lock_guard<boost::mutex> lock(jt_mutex);
      armState.goal_position_command_time_duration = duration_to_goal_command;
    }

    /**
     * @brief Get the timestamp of the most recent armState
     *
     *
     *
     * @see KukaFRIdriver::set(Range&& range,
     * grl::revolute_joint_angle_open_chain_command_tag)
     *
     */
    // KukaState::time_point_type get(time_point_tag) {
    //   boost::lock_guard<boost::mutex> lock(jt_mutex);
    //   return armState.timestamp;
    // }

      cartographer::common::Time get(time_point_tag) {
         boost::lock_guard<boost::mutex> lock(jt_mutex);
         return armState.time_event_stamp.device_time;
      }

    /**
     * \brief Set the applied joint torques for the current interpolation step.
     *
     * This method is only effective when the client is in a commanding state.
     * The ControlMode of the robot has to be joint impedance control mode. The
     * Client Command Mode has to be torque.
     *
     * @param state Object which stores the current state of the robot, including
     * the command to send next
     * @param torques Array with the applied torque values (in Nm)
     * @param tag identifier object indicating that the torqe value command should
     * be modified
     */
    template <typename Range>
    void set(Range &&range, grl::revolute_joint_torque_open_chain_command_tag) {
      boost::lock_guard<boost::mutex> lock(jt_mutex);
      armState.clearCommands();
      boost::copy(range, armState.commandedTorque);
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
     * The ControlMode of the robot has to be Cartesian impedance control mode.
     * The
     * Client Command Mode has to be wrench.
     *
     * @param state object storing the command data that will be sent to the
     * physical device
     * @param range wrench Applied Cartesian wrench vector, in x, y, z, roll,
     * pitch, yaw force measurments.
     * @param tag identifier object indicating that the wrench value command
     * should be modified
     *
     * @todo perhaps support some specific more useful data layouts
     */
    template <typename Range>
    void set(Range &&range, grl::cartesian_wrench_command_tag) {
      boost::lock_guard<boost::mutex> lock(jt_mutex);
      armState.clearCommands();
      std::copy(range, armState.commandedCartesianWrenchFeedForward);
    }

    /// @todo should this exist, is it a good design? is it written correctly?
    void get(KukaState &state) {
      boost::lock_guard<boost::mutex> lock(jt_mutex);
      state = armState;
    }

    /// start recording the fusiontrack frame data in memory
    /// return true on success, false on failure
    bool start_recording()
    {

      m_isRecording = true;
      return m_isRecording;
    }
    /// stop recording the fusiontrack frame data in memory
    /// return true on success, false on failure
    bool stop_recording()
    {
      m_isRecording = false;
      return !m_isRecording;
    }



    bool save_oneKUKAiiwaStateBuffer()
    {


        //boost::lock_guard<boost::mutex> lock(jt_mutex);
        std::string RobotName("Robotiiwa" );
        std::string destination("where this message is going (URI)");
        std::string source("where this message came from (URI)");
        std::string basename = RobotName; //std::get<0>(params);
        bool setArmConfiguration_ = true; // set the arm config first time
        bool max_control_force_stop_ = false;
          // @TODO(Chunting) Put all these parameters in a configuration file.
        std::vector<double> joint_stiffness_(7, 0.2);
        std::vector<double> joint_damping_(7, 0.3);
        std::vector<double> joint_AccelerationRel_(7, 0.5);
        std::vector<double> joint_VelocityRel_(7, 1.0);
        bool updateMinimumTrajectoryExecutionTime = false;
        double minimumTrajectoryExecutionTime = 4;


        //Cartesian Impedance Values
        grl::flatbuffer::Vector3d cart_stiffness_trans_ = grl::flatbuffer::Vector3d(500,500,500);
        grl::flatbuffer::EulerRotation cart_stifness_rot_ = grl::flatbuffer::EulerRotation(200,200,200,grl::flatbuffer::EulerOrder::xyz);
        grl::flatbuffer::Vector3d cart_damping_trans_ = grl::flatbuffer::Vector3d(0.3,0.3,0.3);
        grl::flatbuffer::EulerRotation cart_damping_rot_ = grl::flatbuffer::EulerRotation(0.3,0.3,0.3,grl::flatbuffer::EulerOrder::xyz);
        grl::flatbuffer::EulerPose cart_stiffness_ = grl::flatbuffer::EulerPose(cart_stiffness_trans_, cart_stifness_rot_);
        grl::flatbuffer::EulerPose cart_damping_ = grl::flatbuffer::EulerPose(cart_damping_trans_, cart_damping_rot_);
        grl::flatbuffer::EulerPose cart_max_path_deviation_  = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(1000,1000,100), grl::flatbuffer::EulerRotation(5.,5.,5., grl::flatbuffer::EulerOrder::xyz));
        grl::flatbuffer::EulerPose cart_max_ctrl_vel_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(1000,1000,1000), grl::flatbuffer::EulerRotation(6.3,6.3,6.3, grl::flatbuffer::EulerOrder::xyz));
        grl::flatbuffer::EulerPose cart_max_ctrl_force_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(200,200,200), grl::flatbuffer::EulerRotation(200.,200.,200., grl::flatbuffer::EulerOrder::xyz));
        double nullspace_stiffness_ = 0.1;
        double nullspace_damping_ = 0.1;
        bool updatePortOnRemote = false;
        int16_t portOnRemote = 3501;
        bool updatePortOnController = false;
        int16_t portOnController = 3502;
        // Resolve the data in FRIMonitoringMessage
        // Don't match the FRIMessage.pb.h
        grl::flatbuffer::ArmState armControlMode = grl::flatbuffer::ArmState::StartArm;
        grl::flatbuffer::KUKAiiwaInterface commandInterface = grl::flatbuffer::KUKAiiwaInterface::SmartServo;// KUKAiiwaInterface::SmartServo;
        grl::flatbuffer::KUKAiiwaInterface monitorInterface = grl::flatbuffer::KUKAiiwaInterface::FRI;

        FRIMonitoringMessage monitoringMsg = friData_->monitoringMsg;
        // RobotInfo
        // how to use pb_callback_t driveState in RobotInfo?
        ::RobotInfo robotInfo = monitoringMsg.robotInfo;
        int NUM_DOF = robotInfo.has_numberOfJoints?robotInfo.numberOfJoints:7;
        ::ControlMode controlMode = robotInfo.controlMode;
        ::SafetyState safetyState = robotInfo.safetyState;
        ::OperationMode operationMode = robotInfo.operationMode;

        // ConnectionInfo
        // how to use uint32_t receiveMultiplier
        ::ConnectionInfo connectionInfo = monitoringMsg.connectionInfo;
        ::FRISessionState friSessionState = connectionInfo.sessionState;
        ::FRIConnectionQuality friConnectionQuality = connectionInfo.quality;
        //  MessageIpoData
        // JointValues jointPosition; double trackingPerformance;
        ::MessageIpoData ipoData = monitoringMsg.ipoData;
        ::ClientCommandMode clientCommandMode = ipoData.clientCommandMode;
        ::OverlayType overlayType = ipoData.overlayType;

        // No MessageHeader in flatbuffer objects
        ::MessageHeader messageHeader = monitoringMsg.header;

        ::MessageMonitorData messageMonitorData = monitoringMsg.monitorData;

        ::Transformation *transformation = new ::Transformation[5];

        for (int i=0; i<5; i++) {
            transformation[i] = monitoringMsg.requestedTransformations[i];
        }
        // MessageEndOf exists in FRIMessage.pb.h, but it's never used in the FlatBuffer objects
        ::MessageEndOf endOfMessageData = monitoringMsg.endOfMessageData;

        auto bns = m_logFileBufferBuilderP->CreateString(basename);
        double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();


        bool OK = true;

        int64_t sequenceNumber = 0;

        copy(monitoringMsg, armState);
        armState.sessionState = grl::toFlatBuffer(friSessionState);
        armState.connectionQuality = grl::toFlatBuffer(friConnectionQuality);
        armState.safetyState = grl::toFlatBuffer(safetyState);
        armState.operationMode = grl::toFlatBuffer(operationMode);
        // This parameter is never used.
        armState.driveState = grl::toFlatBuffer(::DriveState::DriveState_OFF);
        flatbuffers::Offset<grl::flatbuffer::ArmControlState> controlState = grl::toFlatBuffer(*m_logFileBufferBuilderP, basename, sequenceNumber++, duration, armState, armControlMode);
        flatbuffers::Offset<grl::flatbuffer::CartesianImpedenceControlMode> setCartesianImpedance = grl::toFlatBuffer(*m_logFileBufferBuilderP, cart_stiffness_, cart_damping_,
                nullspace_stiffness_, nullspace_damping_, cart_max_path_deviation_, cart_max_ctrl_vel_, cart_max_ctrl_force_, max_control_force_stop_);
        flatbuffers::Offset<grl::flatbuffer::JointImpedenceControlMode> setJointImpedance = grl::toFlatBuffer(*m_logFileBufferBuilderP, joint_stiffness_, joint_damping_);
        // normalized joint accelerations/velocities from 0 to 1 relative to system capabilities
        // how to get the acceleration of the robot? There is no acceleration information in KukaState (armState).
        flatbuffers::Offset<grl::flatbuffer::SmartServo> setSmartServo = grl::toFlatBuffer(*m_logFileBufferBuilderP, joint_AccelerationRel_, joint_VelocityRel_, updateMinimumTrajectoryExecutionTime, minimumTrajectoryExecutionTime);
        // FRI configuration parameters
        flatbuffers::Offset<grl::flatbuffer::FRI> FRIConfig = grl::toFlatBuffer(*m_logFileBufferBuilderP, overlayType, connectionInfo, updatePortOnRemote, portOnRemote, updatePortOnController, portOnController);
        std::vector<flatbuffers::Offset<grl::flatbuffer::LinkObject>> tools;
        std::vector<flatbuffers::Offset<grl::flatbuffer::ProcessData>> processData;
        // @TODO(Chunting) Initialize Pose with 0, we ought to calculate it later with ::Transformation;
        // Also assign the configuration parameters with random values, we can figure them out later.
        for(int i=0; i<7; i++) {
              std::string linkname = "Link" + std::to_string(i);
              std::string parent = i==0?"Base":("Link" + std::to_string(i-1));
              // Compute pose later with transformation matrix
              grl::flatbuffer::Pose pose = grl::flatbuffer::Pose(grl::flatbuffer::Vector3d(0,0,0), grl::flatbuffer::Quaternion(0,0,0,0));
              grl::flatbuffer::Inertia inertia = grl::flatbuffer::Inertia(1, pose, 1, 2, 3, 4, 5, 6);
              flatbuffers::Offset<grl::flatbuffer::LinkObject> linkObject = grl::toFlatBuffer(*m_logFileBufferBuilderP, linkname, parent, pose, inertia);
              tools.push_back(linkObject);
              processData.push_back(
                  grl::toFlatBuffer(*m_logFileBufferBuilderP,
                  "dataType"+ std::to_string(i),
                  "defaultValue"+ std::to_string(i),
                  "displayName"+ std::to_string(i),
                  "id"+ std::to_string(i),
                  "min"+ std::to_string(i),
                  "max"+ std::to_string(i),
                  "unit"+ std::to_string(i),
                  "value"+ std::to_string(i)));
        }
        // Set the configuration of the Kuka iiwa
        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaArmConfiguration> kukaiiwaArmConfiguration = grl::toFlatBuffer(
              *m_logFileBufferBuilderP,
              RobotName,
              commandInterface,
              monitorInterface,
              clientCommandMode,
              overlayType,
              controlMode,
              setCartesianImpedance,
              setJointImpedance,
              setSmartServo,
              FRIConfig,
              tools,
              processData,
              "currentMotionCenter",
              true);
        grl::TimeEvent time_event_stamp;
        flatbuffers::Offset<grl::flatbuffer::FRIMessageLog> friMessageLog = grl::toFlatBuffer(
             *m_logFileBufferBuilderP,
             friSessionState,
             friConnectionQuality,
             controlMode,
             monitoringMsg,
             time_event_stamp);
        // getWrench() is availble in KukaJAVAdriver, so maybe it's better to log data in KukaDriver where user can access both KukaFRIDriver and KukaJAVADriver?
        grl::flatbuffer::Wrench cartesianWrench{grl::flatbuffer::Vector3d(1,1,1),grl::flatbuffer::Vector3d(1,1,1),grl::flatbuffer::Vector3d(1,1,1)};
        // In armState there is neither joint velocity nor acceleration
        std::vector<double> position(7,0);
        std::vector<double> velocity(7,0);
        std::vector<double> acceleration(7,0);
        std::vector<double> torque(7,0);
        for(int i = 0; i<7; i++) {
            position.push_back(armState.position[i]);
            torque.push_back(armState.torque[i]);
        }
        flatbuffers::Offset<grl::flatbuffer::JointState> jointStatetab = grl::toFlatBuffer(*m_logFileBufferBuilderP, position, velocity, acceleration, torque);
        // Calculate the data later.
        // Cartesian pose of the flange relative to the base of the arm
        grl::flatbuffer::Pose cartesianFlangePose = grl::flatbuffer::Pose(grl::flatbuffer::Vector3d(1,1,1), grl::flatbuffer::Quaternion(2,3,4,5));
        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorState> kukaiiwaMonitorState = grl::toFlatBuffer(
            *m_logFileBufferBuilderP,
            jointStatetab, // flatbuffers::Offset<grl::flatbuffer::JointState> &measuredState,
            cartesianFlangePose,
            jointStatetab, // flatbuffers::Offset<grl::flatbuffer::JointState> &jointStateReal,
            jointStatetab, // flatbuffers::Offset<grl::flatbuffer::JointState> &jointStateInterpolated,
            jointStatetab, // flatbuffers::Offset<grl::flatbuffer::JointState> &externalState,
            friSessionState,
            robotInfo.operationMode,
            cartesianWrench);
         // Set it up in the configuration file
        std::vector<double> torqueSensorLimits(7,0.5);
        std::string hardwareVersion( "hardvareVersion");
        bool isReadyToMove = true;
        bool isMastered = true;
        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorConfiguration> monitorConfig = grl::toFlatBuffer(
            *m_logFileBufferBuilderP,
            hardwareVersion,
            torqueSensorLimits,
            isReadyToMove,
            isMastered,
            processData);
        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState> KUKAiiwaState = grl::toFlatBuffer(
            *m_logFileBufferBuilderP,
            RobotName,
            destination,
            source,
            duration,
            true, controlState,
            true, kukaiiwaArmConfiguration,
            true, kukaiiwaMonitorState,
            false, monitorConfig,
            friMessageLog);
        m_KUKAiiwaStateBufferP->push_back(KUKAiiwaState);

        return true;
    }


    // bool startRecordingDataToFlatBuffer(flatbuffers::FlatBufferBuilder &*m_logFileBufferBuilderP, std::shared_ptr<KUKA::FRI::ClientData> &friData)
    bool save_recording(std::string filename = std::string())
    {

         loggerP->info("Here is in save_recording/n ");
        if(filename.empty())
        {
          /// TODO(ahundt) Saving the file twice in one second will overwrite!!!!
          filename = current_date_and_time_string() + "_Kukaiiwa.iiwa";
        }
        #ifdef HAVE_spdlog
            loggerP->info("Save Recording as: ", filename);
        #else // HAVE_spdlog
            std::cout << "Save Recording as: " << filename << std::endl;
        #endif // HAVE_spdlog
        /// lock mutex before accessing file
        // boost::lock_guard<boost::mutex> lock(jt_mutex);

        auto saveLambdaFunction = [
          save_fbbP = std::move(m_logFileBufferBuilderP)
          ,save_KUKAiiwaBufferP = std::move(m_KUKAiiwaStateBufferP)
          ,filename
        #ifdef HAVE_spdlog
          ,lambdaLoggerP = loggerP
        #endif // HAVE_spdlog
        ]() mutable
        {
            bool success = grl::FinishAndVerifyBuffer(*save_fbbP, *save_KUKAiiwaBufferP);
            bool write_binary_stream = true;
            success = success && flatbuffers::SaveFile(filename.c_str(), reinterpret_cast<const char*>(save_fbbP->GetBufferPointer()), save_fbbP->GetSize(), write_binary_stream);
            /// TODO(ahFusionTrackLogAndTrackundt) replace cout with proper spdlog and vrep banner notification
            #ifdef HAVE_spdlog
                lambdaLoggerP->info("filename: ", filename, " verifier success: ", success);
            #else // HAVE_spdlog
                std::cout << "filename: " << filename << " verifier success: " << success << std::endl;
            #endif // HAVE_spdlog
        };

          // save the recording to a file in a separate thread, memory will be freed up when file finishes saving
        std::shared_ptr<std::thread> saveLogThread(std::make_shared<std::thread>(saveLambdaFunction));
        m_saveRecordingThreads.push_back(saveLogThread);
        // flatbuffersbuilder does not yet exist
        m_logFileBufferBuilderP = std::make_shared<flatbuffers::FlatBufferBuilder>();
        m_KUKAiiwaStateBufferP = std::make_shared<std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState>>>();
        std::cout << "End of the program" << std::endl;

        return true;
    }


    // clear the recording buffer from memory immediately to start fresh
    void clear_recording()
    {
      boost::lock_guard<boost::mutex> lock(jt_mutex);
      m_logFileBufferBuilderP.reset();
      m_KUKAiiwaStateBufferP.reset();
    }


    // The total number of times the FRI interface has successfully received a UDP
    // packet
    // from the robot since this class was initialized.
    volatile std::size_t m_haveReceivedRealDataCount = 0;
    // The total number of times the FRI interface has attempted to receive a UDP
    // packet
    // from the robot since this class was initialized, regardless of if it was
    // successful or not.
    volatile std::size_t m_attemptedCommunicationCount = 0;
    // The number of consecutive FRI receive calls that have failed to get data
    // successfully, resets to 0 on a single success.
    volatile std::size_t m_attemptedCommunicationConsecutiveFailureCount = 0;
    // The number of consecutive FRI receive calls that have received data
    // successfully, resets to 0 on a single failure.
    volatile std::size_t m_attemptedCommunicationConsecutiveSuccessCount = 0;

    boost::asio::io_service device_driver_io_service;
    // The work class is used to inform the io_service when work starts and finishes.
    std::unique_ptr<boost::asio::io_service::work> device_driver_workP_;
    std::unique_ptr<std::thread> driver_threadP;
    std::shared_ptr<grl::robot::arm::KukaFRIClientDataDriver<LowLevelStepAlgorithmType>> kukaFRIClientDataDriverP_;
private:
void update()
  {
    const std::size_t MegaByte = 1024*1024;
    // If we write too large a flatbuffer
    const std::size_t single_buffer_limit_bytes = 0.1*MegaByte;

    // run the primary update loop in a separate thread
    bool saveFileNow = false;

    // boost::lock_guard<boost::mutex> lock(jt_mutex);
    /// Temporarily set m_isRecording true.
    m_isRecording = true;
    if (m_isRecording)
    {
        // There is a flatbuffers file size limit of 2GB, but we use a conservative 512MB
        int buffsize = m_logFileBufferBuilderP->GetSize();
        std::cout << "Buffersize:" << buffsize << std::endl;
        if( buffsize > single_buffer_limit_bytes)
        {
            // save the file if we are over the limit
            saveFileNow = true;
        }
    } // end recording steps
    /// TODO(ahundt) Let the user specify the filenames, or provide a way to check the flatbuffer size and know single_buffer_limit_bytes.
    if(saveFileNow)
    {
      save_recording();
      saveFileNow = false;
    }

  }
private:
    KukaState armState;
    boost::mutex jt_mutex;

    Params params_;
    std::shared_ptr<KUKA::FRI::ClientData> friData_;
    std::atomic<bool> m_shouldStop;
    /// is data currently being recorded
    std::atomic<bool> m_isRecording;
    /// builds up the file log in memory as data is received
    /// @todo TODO(ahundt) once using C++14 use unique_ptr https://stackoverflow.com/questions/8640393/move-capture-in-lambda
    std::shared_ptr<flatbuffers::FlatBufferBuilder> m_logFileBufferBuilderP;
    /// this is the current log data stored in memory
    /// @todo TODO(ahundt) once using C++14 use unique_ptr https://stackoverflow.com/questions/8640393/move-capture-in-lambda
    std::shared_ptr<std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState>>> m_KUKAiiwaStateBufferP;
    /// thread that polls the driver for new data and puts the data into the recording
    std::unique_ptr<std::thread> m_driverThread;
    /// @todo TODO(ahundt) the threads that saved files will build up forever, figure out how they can clear themselves out
    std::vector<std::shared_ptr<std::thread>> m_saveRecordingThreads;

    #ifdef HAVE_spdlog
        std::shared_ptr<spdlog::logger> loggerP;
    #endif // HAVE_spdlog
};  /// End of KukaFRIdriver.hpp

/// @brief nonmember wrapper function to help integrate KukaFRIdriver objects
/// with generic programming interface
template <typename LowLevelStepAlgorithmType = LinearInterpolation,
          typename Range, typename T>
static inline void set(KukaFRIdriver<LowLevelStepAlgorithmType> &state,
                       Range &&range, T t) {
  state.set(range, t);
}
}
}
} // namespace grl::robot::arm

#endif
