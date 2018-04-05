/// KukaFRIDriver.hpp handles communication with the Kuka over FRI.
/// If you are new to this code base you are most likely looking for KukaDriver.hpp
#ifndef _KUKA_FRI_DRIVER
#define _KUKA_FRI_DRIVER
#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE

#include "grl/kuka/KukaFRIClientDataDriver.hpp"
#include <cstdio>
#include <cinttypes>
//#include "cartographer/common/time.h"

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
class KukaFRIdriver : public std::enable_shared_from_this<KukaFRIdriver<LowLevelStepAlgorithmType>>, public KukaUDP
{
public:
    using KukaUDP::ParamIndex;  // enum, define some connection parameters, such as ip, port...
    using KukaUDP::ThreadingRunMode;
    using KukaUDP::Params;      // std::tuple, contains the information needed to connect to the robot.
    using KukaUDP::defaultParams;  // A method to assign Params with defaut values.

    KukaFRIdriver(Params params = defaultParams()) : params_(params),  m_shouldStop(false) {}

    void construct() { construct(params_); }

    /// @todo create a function that calls simGetObjectHandle and throws an
    /// exception when it fails
    /// @warning getting the ik group is optional, so it does not throw an
    /// exception
    void construct(Params params) {
        params_ = params;
        // keep driver threads from exiting immediately after creation, because they have work to do!
        // boost::asio::io_service::work: Constructor notifies the io_service that work is starting.
        device_driver_workP_.reset(new boost::asio::io_service::work(device_driver_io_service));

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
    
    /// Saves any log files and shuts down the driver
    /// Called by the destructor, but since this may take a while
    /// a separate funtion is provided so the process can be started in parallel.
    void destruct()
    {
        m_shouldStop = true;
        device_driver_workP_.reset();
        if (m_driverThread)
        {
        m_driverThread->join();
        }

        for(auto saveThreadP : m_saveRecordingThreads)
        {
        saveThreadP->join();
        }
    }

    ~KukaFRIdriver() {
        destruct();
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
        boost::transform( maxVel, maxVel.begin(), std::bind2nd(std::multiplies<KukaState::joint_state::value_type>(),
                       getSecondsPerTick()));
        return maxVel;
    }

    struct MicrosecondClock
    {
        using rep = int64_t;
        /// 1 microsecond
        using period = std::ratio<1, 1000000>;
        using duration = std::chrono::duration<rep, period>;
        using time_point = std::chrono::time_point<MicrosecondClock>;

        static constexpr bool is_steady = true;

        /// TODO(ahundt) currently assuming the FusionTrack timestamp is from the unix time epoch
        static time_point now() noexcept
        {
            using namespace std::chrono;
            return time_point(duration_cast<duration>(system_clock::now().time_since_epoch()));
        }
    };

     /// TODO(ahundt) currently assuming the FusionTrack timestamp is from the unix time epoch
    cartographer::common::Time KukaTimeToCommonTime(typename MicrosecondClock::time_point Kukatime)
    {
        return cartographer::common::Time(
            std::chrono::duration_cast<cartographer::common::UniversalTimeScaleClock::duration>(Kukatime.time_since_epoch()));
    }

     /// TODO(ahundt) currently assuming the FusionTrack timestamp is from the unix time epoch
    cartographer::common::Time KukaTimeToCommonTime( std::chrono::time_point<std::chrono::high_resolution_clock> Kukatime)
    {
        return cartographer::common::Time(
            // std::chrono::duration_cast<cartographer::common::UniversalTimeScaleClock::duration>(Kukatime.time_since_epoch()));
            std::chrono::duration_cast<typename MicrosecondClock::duration>(Kukatime.time_since_epoch()));
    }

    cartographer::common::Time FRITimeStampToCommonTime(const ::TimeStamp &friTimeStamp)
    {
        // Convert the time to microseconds

        std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
        timestamp += std::chrono::seconds(friTimeStamp.sec) + std::chrono::nanoseconds(friTimeStamp.nanosec);

        // std::chrono::seconds sec(friTimeStamp.sec);
        // int64_t seconds = std::chrono::seconds(sec).count();
        // std::chrono::nanoseconds nanosec(friTimeStamp.nanosec);
        // int64_t nanosecs = friTimeStamp.nanosec;
        // int64_t microseconds_sec = std::chrono::microseconds(sec).count();


        // int64_t microseconds = microseconds + nanosecs/1000;
        // if(INT64_MAX - microseconds_sec < nanosecs/1000) {
        //     throw std::runtime_error("signed overflow has occured");
        // }
        // std::cout<<"secondes: "<<seconds <<"  "<<friTimeStamp.sec<<"\n"
        //          <<"secondes in microseconds: "<<microseconds_sec<<"\n"
        //          << "nanosecs: "<< nanosecs  <<"  "<< friTimeStamp.nanosec <<"\n"
        //          <<"microseconds: "<< microseconds<<std::endl;


        // typename MicrosecondClock::time_point fritp = typename MicrosecondClock::time_point(typename MicrosecondClock::duration(microseconds));
        return KukaTimeToCommonTime( timestamp);
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
        // This is the key point where the arm's motion goal command is updated and sent to the robot
        // Set the FRI to the simulated joint positions
        // Why set this check?
        if (this->m_haveReceivedRealDataCount > minimumConsecutiveSuccessesBeforeSendingCommands) {
            /// @todo TODO(ahundt) Need to eliminate this allocation
            boost::lock_guard<boost::mutex> lock(jt_mutex);
            boost::container::static_vector<double, KUKA::LBRState::NUM_DOF> jointStateToCommand;
            boost::copy(armState.commandedPosition,std::back_inserter(jointStateToCommand));
            // pass time to reach specified goal for position control
            lowLevelStepAlgorithmCommandParamsP = std::make_shared<grl::robot::arm::LinearInterpolation::Params>(std::make_tuple(jointStateToCommand,armState.goal_position_command_time_duration));
            /// @todo construct new low level command object and pass to
            /// KukaFRIClientDataDriver
            /// this is where we used to setup a new FRI command
        } else {
            /// @todo TODO(ahundt) BUG: Need way to pass time to reach specified goal for position control and eliminate this allocation
            lowLevelStepAlgorithmCommandParamsP.reset(new typename LowLevelStepAlgorithmType::Params());
        }

        BOOST_VERIFY(lowLevelStepAlgorithmCommandParamsP != nullptr);
        boost::system::error_code send_ec, recv_ec;
        std::size_t send_bytes, recv_bytes;
        // sync with device over network
        haveNewData = !kukaFRIClientDataDriverP_->update_state(
            lowLevelStepAlgorithmCommandParamsP,
            friData_,
            recv_ec,
            recv_bytes,
            send_ec,
            send_bytes,
            time_event_stamp);

        m_attemptedCommunicationCount++;

        if (haveNewData) {
            boost::lock_guard<boost::mutex> lock(jt_mutex);

            m_attemptedCommunicationConsecutiveSuccessCount++;
            this->m_attemptedCommunicationConsecutiveFailureCount = 0;
            this->m_haveReceivedRealDataCount++;

            oneKUKAiiwaStateBuffer();

            saveToDisk();

        } else {
            m_attemptedCommunicationConsecutiveFailureCount++;
            // std::cerr << "No new FRI data available, is an FRI application running "
            //              "on the Kuka arm? \n Total sucessful transfers: "
            //           << this->m_haveReceivedRealDataCount
            //           << "\n Total attempts: " << m_attemptedCommunicationCount
            //           << "\n Consecutive Failures: "
            //           << m_attemptedCommunicationConsecutiveFailureCount
            //           << "\n Consecutive Successes: "
            //           << m_attemptedCommunicationConsecutiveSuccessCount << "\n";
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

    /// start recording the kuka state data in memory
    /// return true on success, false on failure
    bool start_recording(int _single_buffer_limit_bytes)
    {

        m_isRecording = true;
        single_buffer_limit_bytes = _single_buffer_limit_bytes*MegaByte;
        std::cout<< "m_isRecording is set to " << m_isRecording << std::endl;
        std::cout<< "Kuka single_buffer_limit_bytes:  " << _single_buffer_limit_bytes << " MB" << std::endl;
        return m_isRecording;
    }
    /// stop recording the kuka state data in memory
    /// return true on success, false on failure
    bool stop_recording()
    {
        m_isRecording = false;
        return !m_isRecording;
    }

    bool is_recording()
    {
        return m_isRecording;
    }


    bool oneKUKAiiwaStateBuffer()
    {
        // We have the real kuka state read from the device now
        // update real joint angle data
        std::string RobotName = std::string(std::get<RobotModel>(params_));
        std:: string destination = std::string(std::get<remotehost>(params_));
        std::string source =  std::string(std::get<localhost>(params_));
        int16_t portOnRemote = std::stoi(std::string(std::get<remoteport>(params_)));
        int16_t portOnController  = std::stoi(std::string(std::get<localport>(params_)));
        std::string basename = RobotName;

        // bool max_control_force_stop_ = false;
        // std::vector<double> joint_stiffness_(KUKA::LBRState::NUM_DOF, 0);
        // std::vector<double> joint_damping_(KUKA::LBRState::NUM_DOF, 0);
        // std::vector<double> joint_AccelerationRel_(KUKA::LBRState::NUM_DOF, 0);
        // std::vector<double> joint_VelocityRel_(KUKA::LBRState::NUM_DOF, 0);
        // bool updateMinimumTrajectoryExecutionTime = false;
        // double minimumTrajectoryExecutionTime = 4;

        //Cartesian Impedance Values
        // grl::flatbuffer::Vector3d cart_stiffness_trans_ = grl::flatbuffer::Vector3d(0,0,0);
        // grl::flatbuffer::EulerRotation cart_stifness_rot_ = grl::flatbuffer::EulerRotation(0,0,0,grl::flatbuffer::EulerOrder::xyz);
        // grl::flatbuffer::Vector3d cart_damping_trans_ = grl::flatbuffer::Vector3d(0,0,0);
        // grl::flatbuffer::EulerRotation cart_damping_rot_ = grl::flatbuffer::EulerRotation(0,0,0,grl::flatbuffer::EulerOrder::xyz);
        // grl::flatbuffer::EulerPose cart_stiffness_ = grl::flatbuffer::EulerPose(cart_stiffness_trans_, cart_stifness_rot_);
        // grl::flatbuffer::EulerPose cart_damping_ = grl::flatbuffer::EulerPose(cart_damping_trans_, cart_damping_rot_);
        // grl::flatbuffer::EulerPose cart_max_path_deviation_  = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(0,0,0), grl::flatbuffer::EulerRotation(0,0,0, grl::flatbuffer::EulerOrder::xyz));
        // grl::flatbuffer::EulerPose cart_max_ctrl_vel_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(0,0,0), grl::flatbuffer::EulerRotation(0,0,0, grl::flatbuffer::EulerOrder::xyz));
        // grl::flatbuffer::EulerPose cart_max_ctrl_force_ = grl::flatbuffer::EulerPose(grl::flatbuffer::Vector3d(0,0,0), grl::flatbuffer::EulerRotation(0.,0.,0., grl::flatbuffer::EulerOrder::xyz));
        // double nullspace_stiffness_ = 0;
        // double nullspace_damping_ = 0;
        // bool updatePortOnRemote = false;

        // bool updatePortOnController = false;

        // Resolve the data in FRIMonitoringMessage
        // Don't match the FRIMessage.pb.h
        grl::flatbuffer::ArmState armControlMode = grl::flatbuffer::ArmState::StartArm;
        grl::flatbuffer::KUKAiiwaInterface commandInterface = grl::flatbuffer::KUKAiiwaInterface::SmartServo;// KUKAiiwaInterface::SmartServo;
        grl::flatbuffer::KUKAiiwaInterface monitorInterface = grl::flatbuffer::KUKAiiwaInterface::FRI;

        FRIMonitoringMessage monitoringMsg = friData_->monitoringMsg;
        std::size_t NUM_DOF = grl::robot::arm::get(monitoringMsg, KUKA::LBRState::NUM_DOF);;
        ::ControlMode controlMode = grl::robot::arm::get(monitoringMsg, ::ControlMode());
        ::SafetyState safetyState = grl::robot::arm::get(monitoringMsg, ::SafetyState());
        ::OperationMode operationMode = grl::robot::arm::get(monitoringMsg, ::OperationMode());
        ::ConnectionInfo connectionInfo = monitoringMsg.connectionInfo;
        ::FRISessionState friSessionState = grl::robot::arm::get(monitoringMsg, ::FRISessionState());
        ::FRIConnectionQuality friConnectionQuality = grl::robot::arm::get(monitoringMsg, ::FRIConnectionQuality());
        ::ClientCommandMode clientCommandMode = grl::robot::arm::get(monitoringMsg, ::ClientCommandMode());

        ::OverlayType overlayType = grl::robot::arm::get(monitoringMsg, ::OverlayType());

        ::MessageMonitorData messageMonitorData = monitoringMsg.monitorData;

        std::string s_event_name = RobotName + "/state";
        std::string device_clock_id_str = s_event_name + "/device/clock";
        std::string local_clock_id_str = s_event_name + "/control_computer/clock/steady";

        TimeEvent::UnsignedCharArray event_name;
        std::size_t length = s_event_name.copy(event_name.begin(), std::min(s_event_name.size(), event_name.size()));
        event_name[length] = '\0';
        time_event_stamp.event_name = event_name;

        TimeEvent::UnsignedCharArray device_clock_id;
        length = device_clock_id_str.copy(device_clock_id.begin(), std::min(device_clock_id_str.size(), device_clock_id.size()));
        device_clock_id[length] = '\0';
        time_event_stamp.device_clock_id = device_clock_id;

        TimeEvent::UnsignedCharArray local_clock_name_arr;
        length = local_clock_id_str.copy(local_clock_name_arr.begin(), std::min(local_clock_id_str.size(),local_clock_name_arr.size()));
        local_clock_name_arr[length] = '\0';
        time_event_stamp.local_clock_id = local_clock_name_arr;
        time_event_stamp.device_time = FRITimeStampToCommonTime(messageMonitorData.timestamp);
        armState.time_event_stamp = time_event_stamp;

        //std::cout<< time_event_stamp.event_name << std::endl << time_event_stamp.device_clock_id << std::endl << time_event_stamp.local_clock_id <<std::endl;
        ::Transformation *transformation = new ::Transformation[5];

        for (int i=0; i<5; i++) {
            transformation[i] = monitoringMsg.requestedTransformations[i];
            // for(int j=0; j<12; ++j){
            //     std::cout<< transformation[i].matrix[j] << " ";
            // }
            // std::cout<<std::endl;
        }
        // std::cout<<std::endl;
        // MessageEndOf exists in FRIMessage.pb.h, but it's never used in the FlatBuffer objects
        ::MessageEndOf endOfMessageData = monitoringMsg.endOfMessageData;

        auto bns = m_logFileBufferBuilderP->CreateString(basename);
        double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();
        bool OK = true;
        int64_t sequenceNumber = 0;

        copy(monitoringMsg, armState);

        flatbuffers::Offset<grl::flatbuffer::ArmControlState> controlState = grl::toFlatBuffer(*m_logFileBufferBuilderP, basename, sequenceNumber++, duration, armState, armControlMode);
        // flatbuffers::Offset<grl::flatbuffer::CartesianImpedenceControlMode> setCartesianImpedance = grl::toFlatBuffer(*m_logFileBufferBuilderP, cart_stiffness_, cart_damping_,
        //        nullspace_stiffness_, nullspace_damping_, cart_max_path_deviation_, cart_max_ctrl_vel_, cart_max_ctrl_force_, max_control_force_stop_);
        flatbuffers::Offset<grl::flatbuffer::CartesianImpedenceControlMode> setCartesianImpedance = 0;
        // flatbuffers::Offset<grl::flatbuffer::JointImpedenceControlMode> setJointImpedance = grl::toFlatBuffer(*m_logFileBufferBuilderP, joint_stiffness_, joint_damping_);
        flatbuffers::Offset<grl::flatbuffer::JointImpedenceControlMode> setJointImpedance = 0;
        // normalized joint accelerations/velocities from 0 to 1 relative to system capabilities
        // how to get the acceleration of the robot? There is no acceleration information in KukaState (armState).
        flatbuffers::Offset<grl::flatbuffer::SmartServo> setSmartServo = 0; //grl::toFlatBuffer(*m_logFileBufferBuilderP, joint_AccelerationRel_, joint_VelocityRel_, updateMinimumTrajectoryExecutionTime, minimumTrajectoryExecutionTime);
        // FRI configuration parameters
        flatbuffers::Offset<grl::flatbuffer::FRI> FRIConfig = 0; //grl::toFlatBuffer(*m_logFileBufferBuilderP, overlayType, connectionInfo, updatePortOnRemote, portOnRemote, updatePortOnController, portOnController);
        // std::vector<flatbuffers::Offset<grl::flatbuffer::LinkObject>> tools;
        // std::vector<flatbuffers::Offset<grl::flatbuffer::ProcessData>> processData_vec;
        // @TODO(Chunting) Initialize Pose with 0, we ought to calculate it later with ::Transformation;
        // Also assign the configuration parameters with random values, we can figure them out later.
        // for(int i=0; i<KUKA::LBRState::NUM_DOF; i++) {
        //       std::string linkname = "Link" + std::to_string(i);
        //       std::string parent = i==0?"Base":("Link" + std::to_string(i-1));
        //       // Compute pose later with transformation matrix
        //       grl::flatbuffer::Pose pose = grl::flatbuffer::Pose(grl::flatbuffer::Vector3d(0,0,0), grl::flatbuffer::Quaternion(0,0,0,0));
        //       grl::flatbuffer::Inertia inertia = grl::flatbuffer::Inertia(1, pose, 0, 0, 0, 0, 0, 0);
        //       flatbuffers::Offset<grl::flatbuffer::LinkObject> linkObject = grl::toFlatBuffer(*m_logFileBufferBuilderP, linkname, parent, pose, inertia);
        //       auto singleprocessdata =  grl::toFlatBuffer(
        //           *m_logFileBufferBuilderP,
        //           "dataType"+ std::to_string(i),
        //           "defaultValue"+ std::to_string(i),
        //           "displayName"+ std::to_string(i),
        //           "id"+ std::to_string(i),
        //           "min"+ std::to_string(i),
        //           "max"+ std::to_string(i),
        //           "unit"+ std::to_string(i),
        //           "value"+ std::to_string(i));
        //       tools.push_back(linkObject);
        //       processData_vec.push_back(singleprocessdata);
        // }
        // Set the configuration of the Kuka iiwa
        // flatbuffers::Offset<grl::flatbuffer::KUKAiiwaArmConfiguration> kukaiiwaArmConfiguration = grl::toFlatBuffer(
        //       *m_logFileBufferBuilderP,
        //       RobotName,
        //       commandInterface,
        //       monitorInterface,
        //       clientCommandMode,
        //       overlayType,
        //       controlMode,
        //       setCartesianImpedance,
        //       setJointImpedance,
        //       setSmartServo,
        //       FRIConfig,
        //       tools,
        //       processData_vec,
        //       "currentMotionCenter",
        //       true);
        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaArmConfiguration> kukaiiwaArmConfiguration;

        flatbuffers::Offset<grl::flatbuffer::FRIMessageLog> friMessageLog = grl::toFlatBuffer(
             *m_logFileBufferBuilderP,
             friSessionState,
             friConnectionQuality,
             controlMode,
             monitoringMsg,
             armState.time_event_stamp);
        // getWrench() is availble in KukaJAVAdriver, so maybe it's better to log data in KukaDriver where user can access both KukaFRIDriver and KukaJAVADriver?
        // grl::flatbuffer::Wrench cartesianWrench{grl::flatbuffer::Vector3d(0, 0, 0),grl::flatbuffer::Vector3d(0, 0, 0),grl::flatbuffer::Vector3d(0, 0, 0)};
        grl::flatbuffer::Wrench cartesianWrench;
        // In armState there is neither joint velocity nor acceleration
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> acceleration;
        std::vector<double> torque;
        std::vector<double> jointIpoPostion;
        std::vector<double> externalTorque;
        for(int i = 0; i<KUKA::LBRState::NUM_DOF; i++) {
            position.push_back(armState.position[i]);
            torque.push_back(armState.torque[i]);
            if(armState.ipoJointPosition.size() > i){
                jointIpoPostion.push_back(armState.ipoJointPosition[i]);
            }
            externalTorque.push_back(armState.externalTorque[i]);
        }
        flatbuffers::Offset<grl::flatbuffer::JointState> jointStatetab = grl::toFlatBuffer(*m_logFileBufferBuilderP, position, velocity, acceleration, torque);
        torque.clear();
        flatbuffers::Offset<grl::flatbuffer::JointState> jointIpoState = grl::toFlatBuffer(*m_logFileBufferBuilderP, jointIpoPostion, velocity, acceleration, torque);
        position.clear();
        flatbuffers::Offset<grl::flatbuffer::JointState> externalState = grl::toFlatBuffer(*m_logFileBufferBuilderP, position, velocity, acceleration, externalTorque);
        // Calculate the data later.
        // Cartesian pose of the flange relative to the base of the arm
        // grl::flatbuffer::Pose cartesianFlangePose = grl::flatbuffer::Pose(grl::flatbuffer::Vector3d(0, 0, 0), grl::flatbuffer::Quaternion(0,0,0,0));
        grl::flatbuffer::Pose cartesianFlangePose = grl::flatbuffer::Pose();
        // flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorState> kukaiiwaMonitorState = 0;
        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorState> kukaiiwaMonitorState = grl::toFlatBuffer(
            *m_logFileBufferBuilderP,
            jointStatetab, // flatbuffers::Offset<grl::flatbuffer::JointState> &measuredState,
            cartesianFlangePose,
            jointStatetab, // flatbuffers::Offset<grl::flatbuffer::JointState> &jointStateReal,
            jointIpoState, // flatbuffers::Offset<grl::flatbuffer::JointState> &jointStateInterpolated,
            externalState, // flatbuffers::Offset<grl::flatbuffer::JointState> &externalState,
            friSessionState,
            operationMode,
            cartesianWrench);
        // Set it up in the configuration file
        // std::vector<double> torqueSensorLimits(KUKA::LBRState::NUM_DOF,0.1);
        // std::string hardwareVersion("hardvareVersion");
        // bool isReadyToMove = true;
        // bool isMastered = true;
        // flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorConfiguration> monitorConfig = grl::toFlatBuffer(
        //     *m_logFileBufferBuilderP,
        //     hardwareVersion,
        //     torqueSensorLimits,
        //     isReadyToMove,
        //     isMastered,
        //     processData_vec);
        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaMonitorConfiguration> monitorConfig = 0;
        flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState> KUKAiiwaState = grl::toFlatBuffer(
            *m_logFileBufferBuilderP,
            RobotName,
            destination,
            source,
            armState.time_event_stamp,
            false, controlState,  // if false, then don't record the value
            false, kukaiiwaArmConfiguration,
            false, kukaiiwaMonitorState,
            false, monitorConfig,
            friMessageLog);
        m_KUKAiiwaStateBufferP->push_back(KUKAiiwaState);

        return true;
    }

    bool save_recording(std::string filename = std::string())
    {
        if(filename.empty())
        {
            /// TODO(ahundt) Saving the file twice in one second will overwrite!!!!
            filename = current_date_and_time_string() + "_Kukaiiwa.iiwa";
        }
        #ifdef HAVE_spdlog
            loggerP->info("Save Recording as in Kuka: {}", filename);
        #else // HAVE_spdlog
            std::cout << "Save Recording as: " << filename << std::endl;
        #endif // HAVE_spdlog
        auto saveLambdaFunction = [
            save_fbbP = std::move(m_logFileBufferBuilderP)
            ,save_KUKAiiwaBufferP = std::move(m_KUKAiiwaStateBufferP)
            ,filename
        #ifdef HAVE_spdlog
            ,lambdaLoggerP = loggerP
        #endif // HAVE_spdlog
        ]() mutable
        {

            std::string currentWorkingDir = boost::filesystem::current_path().string();
            lambdaLoggerP->info("currentWorkingDir ...: {}", currentWorkingDir);
            if(save_fbbP != nullptr && save_KUKAiiwaBufferP != nullptr) {
                bool success = grl::FinishAndVerifyBuffer(*save_fbbP, *save_KUKAiiwaBufferP);
                bool write_binary_stream = true;
                success = success && flatbuffers::SaveFile(filename.c_str(), reinterpret_cast<const char*>(save_fbbP->GetBufferPointer()), save_fbbP->GetSize(), write_binary_stream);
                // assert(success);
                /// TODO(ahFusionTrackLogAndTrackundt) replace cout with proper spdlog and vrep banner notification
                #ifdef HAVE_spdlog
                    lambdaLoggerP->info("For KUKA filename: {},  verifier success:{}", filename,success);
                #else // HAVE_spdlog
                    std::cout << "filename: " << filename << " verifier success: " << success << std::endl;
                #endif // HAVE_spdlog
            }else{
               lambdaLoggerP->error("pointer is nullptr...");
            }
        };

        // save the recording to a file in a separate thread, memory will be freed up when file finishes saving
        std::shared_ptr<std::thread> saveLogThread(std::make_shared<std::thread>(saveLambdaFunction));
        m_saveRecordingThreads.push_back(saveLogThread);
        // flatbuffersbuilder does not yet exist
        m_logFileBufferBuilderP = std::make_shared<flatbuffers::FlatBufferBuilder>();
        m_KUKAiiwaStateBufferP = std::make_shared<std::vector<flatbuffers::Offset<grl::flatbuffer::KUKAiiwaState>>>();

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
/// Check the size of the buffer, when it hit the limit, save it to disk.
void saveToDisk()
  {
   

    // run the primary update loop in a separate thread
    bool saveFileNow = false;
    /// Temporarily set m_isRecording true.
    /// m_isRecording = true;
    if (m_isRecording)
    {
        // There is a flatbuffers file size limit of 2GB, but we use a conservative 512MB
        int buffsize = m_logFileBufferBuilderP->GetSize();
        int statessize = m_KUKAiiwaStateBufferP->size();
        // if( buffsize > single_buffer_limit_bytes || statessize > single_buffer_limit_states)
         if( buffsize > single_buffer_limit_bytes)
        {
            // save the file if we are over the limit
            saveFileNow = true;
            std::cout << "Buffersize:" << buffsize << std::endl;
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

    grl::TimeEvent time_event_stamp;

    const std::size_t MegaByte = 1024*1024;
    // If we write too large a flatbuffer
    std::size_t single_buffer_limit_bytes = 20*MegaByte;
    const std::size_t single_buffer_limit_states = 1350000000;

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
