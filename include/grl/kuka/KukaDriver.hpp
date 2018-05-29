/// KukaDriver.hpp defines the main Kuka driver class most people implementing new libraries or integrations should use
/// See the class grl::robot::arm::KukaDriver in this file and the grl documentation.
#ifndef GRL_KUKA_DRIVER
#define GRL_KUKA_DRIVER


#include <tuple>
#include <memory>
#include <thread>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/exception/all.hpp>
#include <boost/config.hpp>

#include <boost/make_shared.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/transform.hpp>

#include <boost/make_shared.hpp>

#ifdef BOOST_NO_CXX11_ATOMIC_SMART_PTR
#include <boost/thread.hpp>
#endif

#include "Kuka.hpp"
#include "grl/kuka/KukaJAVAdriver.hpp"
#include "grl/kuka/KukaFRIdriver.hpp"
#include "grl/tags.hpp"


namespace grl { namespace robot { namespace arm {
    ///
    ///
    /// @brief Kuka LBR iiwa Primary Multi Mode Driver, supports communication over FRI and JAVA interfaces
    ///
    /// The most effective configuration of this driver is to command the KUKA in JAVA mode
    /// and receive robot status updates via FRI mode.
    ///
    /// @todo enable commanding and monitoring to be independently configured for both FRI and JAVA interface.
    ///
    class KukaDriver : public std::enable_shared_from_this<KukaDriver> {
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

        KukaDriver(Params params = defaultParams())
          : params_(params), debug(false)
        {}

        void construct(){ construct(params_);}

        bool destruct(){ return JAVAdriverP_->destruct(); }


        /// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
        /// @warning getting the ik group is optional, so it does not throw an exception
        void construct(Params params ) {
            params_ = params;
            // keep driver threads from exiting immediately after creation, because they have work to do!
            //device_driver_workP_.reset(new boost::asio::io_service::work(device_driver_io_service));

            /// @todo figure out how to re-enable when .so isn't loaded
            if(    boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI"))
                || boost::iequals(std::get<KukaMonitorMode>(params_),std::string("FRI")))
            {
                FRIdriverP_.reset(
                    new grl::robot::arm::KukaFRIdriver<LinearInterpolation>(
                        //device_driver_io_service,
                        std::make_tuple(
                            std::string(std::get<RobotModel                  >        (params)),
                            std::string(std::get<LocalHostKukaKoniUDPAddress >        (params)),
                            std::string(std::get<LocalHostKukaKoniUDPPort    >        (params)),
                            std::string(std::get<RemoteHostKukaKoniUDPAddress>        (params)),
                            std::string(std::get<RemoteHostKukaKoniUDPPort   >        (params)),
                            grl::robot::arm::KukaFRIClientDataDriver<LinearInterpolation>::run_automatically
                            )
                        )
                    );
                    FRIdriverP_->construct();
            }

            /// @todo implement reading configuration in both FRI and JAVA mode from JAVA interface
            if(    boost::iequals(std::get<KukaCommandMode>(params_),std::string("JAVA"))
                || boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI")))
            {
                try {
                    JAVAdriverP_ = boost::make_shared<KukaJAVAdriver>(params_);
                    JAVAdriverP_->construct();

                    // start up the driver thread
                    /// @todo perhaps allow user to control this?
                    //driver_threadP.reset(new std::thread([&]{ device_driver_io_service.run(); }));
                } catch( boost::exception &e) {
                    e << errmsg_info("KukaDriver: Unable to connect to ZeroMQ Socket from " +
                                   std::get<LocalUDPAddress>             (params_) + " to " +
                                   std::get<RemoteUDPAddress>            (params_));
                    throw;
                }
            }
        }

        const Params & getParams(){
            return params_;
        }

        ~KukaDriver(){
            device_driver_workP_.reset();

        if(driver_threadP){
            device_driver_io_service.stop();
            driver_threadP->join();
        }
      }

     /**
      * @brief spin once - call this function as frequently as possible, it drives the system
      *
      */
      bool run_one(){

        // @todo CHECK FOR REAL DATA BEFORE SENDING COMMANDS
        //if(!m_haveReceivedRealDataCount) return;
        bool haveNewData = false;
        /// @todo make this handled by template driver implementations/extensions

        if(JAVAdriverP_.get() != nullptr)
        {
            if (debug) {
                std::cout << "commandedpos:" << armState_.commandedPosition << "\n";
            }


            /////////////////////////////////////////
            // Do some configuration
            if(boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI")))
            {
                // configure to send commands over FRI interface
                JAVAdriverP_->set(flatbuffer::KUKAiiwaInterface::FRI,command_tag());
            }

            if(boost::iequals(std::get<KukaMonitorMode>(params_),std::string("FRI")))
            {
                // configure to send commands over FRI interface
                JAVAdriverP_->set(flatbuffer::KUKAiiwaInterface::FRI,state_tag());
            }


            /////////////////////////////////////////
            // set new destination

            if( boost::iequals(std::get<KukaCommandMode>(params_),std::string("JAVA")))
            {
                JAVAdriverP_->set(armState_.commandedPosition,revolute_joint_angle_open_chain_command_tag());

                // configure to send commands over JAVA interface
                JAVAdriverP_->set(flatbuffer::KUKAiiwaInterface::SmartServo,command_tag());

            }

            // sync JAVA driver with the robot, note client sends to server asynchronously!
            haveNewData = JAVAdriverP_->run_one();

            if( boost::iequals(std::get<KukaMonitorMode>(params_),std::string("JAVA")))
            {
                JAVAdriverP_->get(armState_);
                JAVAdriverP_->set(flatbuffer::KUKAiiwaInterface::SmartServo,state_tag());

            }
        }

        if(FRIdriverP_.get() != nullptr)
        {
            if( boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI")))
            {
                FRIdriverP_->set(armState_.commandedPosition,revolute_joint_angle_open_chain_command_tag());
            }

            haveNewData = FRIdriverP_->run_one();

            if( boost::iequals(std::get<KukaMonitorMode>(params_),std::string("FRI")))
            {
                FRIdriverP_->get(armState_);
                //JAVAdriverP_->getWrench(armState_);
            }
        }
        return haveNewData;
      }

    /// start recording the kuka state data in memory
    /// return true on success, false on failure
    bool start_recording(int single_buffer_limit_bytes)
    {
        if(FRIdriverP_.get() != nullptr) {
            return FRIdriverP_->start_recording(single_buffer_limit_bytes);
        }
        return false;
    }
    /// stop recording the kuka state data in memory
    /// return true on success, false on failure
    bool stop_recording()
    {
        if(FRIdriverP_.get() != nullptr) {
            return FRIdriverP_->stop_recording();
        }
        return false;

    }
    bool save_recording(std::string filename = std::string()) {
        if(FRIdriverP_.get() != nullptr) {
            return FRIdriverP_->save_recording(filename);
        }
        return false;
    }
    void clear_recording()
    {
        if(FRIdriverP_.get() != nullptr) {
            FRIdriverP_->clear_recording();
        }
    }
    bool is_recording()
    {   if(FRIdriverP_.get() != nullptr) {
            // std::cout<< "In KukaDriver is_recording(): " << FRIdriverP_->is_recording() <<std::endl;
            return FRIdriverP_->is_recording();
        }
        return false;
    }
   /// set the mode of the arm. Examples: Teach or MoveArmJointServo
   /// @see grl::flatbuffer::ArmState in ArmControlState_generated.h
   void set(const flatbuffer::ArmState & armControlMode)
   {
        if(JAVAdriverP_)
        {
            JAVAdriverP_->set(armControlMode);
        }
   }

   /// get the mode of the arm. Examples: Teach or MoveArmJointServo
   /// @see grl::flatbuffer::ArmState in ArmControlState_generated.h
   void get(flatbuffer::ArmState & armControlMode)
   {
        if(JAVAdriverP_)
        {
            JAVAdriverP_->get(armControlMode);
        }
   }

    bool setPositionControlMode()
    {
      if(JAVAdriverP_)
      {
        JAVAdriverP_->setPositionControlMode();
        return true;
      }
      else
        return false;
    }


    bool setJointImpedanceMode(std::vector<double> joint_stiffnes, std::vector<double>joint_damping)
    {
      if(JAVAdriverP_)
      {
        JAVAdriverP_->setJointImpedanceMode(joint_stiffnes, joint_damping);
        return true;
      }
      else
        return false;
    }

    // TODO: define custom flatbuffer for Cartesion Quantities, currently flatbuffer::CartesianImpedenceControlMode
    bool setCartesianImpedanceMode(
        grl::flatbuffer::EulerPose cart_stiffness, grl::flatbuffer::EulerPose cart_damping,
        double nullspace_stiffness, double nullspace_damping,
        grl::flatbuffer::EulerPose cart_max_path_deviation,
        grl::flatbuffer::EulerPose cart_max_ctrl_vel,
        grl::flatbuffer::EulerPose cart_max_ctrl_force,
        bool max_control_force_stop)
    {
        if(JAVAdriverP_)
        {
          JAVAdriverP_->setCartesianImpedanceMode(cart_stiffness, cart_damping,
              nullspace_stiffness, nullspace_damping,
              cart_max_path_deviation, cart_max_ctrl_vel, cart_max_ctrl_force,
              max_control_force_stop);
            return true;
        }
        else
            return false;
    }

     /**
      * \brief Set the joint positions for the current interpolation step.
      *
      * This method is only effective when the client is in a commanding state.
      * @param state Object which stores the current state of the robot, including the command to send next
      * @param range Array with the new joint positions (in radians)
      * @param tag identifier object indicating that revolute joint angle commands should be modified
      */
   template<typename Range>
   void set(Range&& range, grl::revolute_joint_angle_open_chain_command_tag) {
       boost::unique_lock<boost::mutex> lock(jt_mutex);
       armState_.clearCommands();
       boost::copy(range, std::back_inserter(armState_.commandedPosition));
       boost::copy(range, std::back_inserter(armState_.commandedPosition_goal));
       // std::cout << "set commandedpos:" << armState_.commandedPosition;
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
       boost::unique_lock<boost::mutex> lock(jt_mutex);
       armState_.clearCommands();
       boost::copy(range, std::back_inserter(armState_.commandedTorque));
    }


    /**
     * @brief Set the time duration expected between new position commands in ms
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
    void set(double duration_to_goal_command, time_duration_command_tag tag) {
        boost::lock_guard<boost::mutex> lock(jt_mutex);
        armState_.goal_position_command_time_duration = duration_to_goal_command;
        if(FRIdriverP_)
        {
          FRIdriverP_->set(duration_to_goal_command,tag);
        }
        if(JAVAdriverP_)
        {
          JAVAdriverP_->set(duration_to_goal_command,tag);
        }
    }


    /**
     * @brief Get the timestamp of the most recent armState
     *
     *
     *
     * @see KukaFRIdriver::set(Range&& range, grl::revolute_joint_angle_open_chain_command_tag)
     *
     */
    cartographer::common::Time get(time_point_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       return armState_.time_event_stamp.device_time;
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
       boost::unique_lock<boost::mutex> lock(jt_mutex);
       armState_.clearCommands();
       std::copy(range,armState_.commandedCartesianWrenchFeedForward);
    }

    /// @todo implement get function
    template<typename OutputIterator>
    void get(OutputIterator output, grl::revolute_joint_angle_open_chain_state_tag)
    {
       boost::unique_lock<boost::mutex> lock(jt_mutex);
        boost::copy(armState_.position,output);
    }

    /// @todo implement get function
    template<typename OutputIterator>
    void get(OutputIterator output, grl::revolute_joint_torque_open_chain_state_tag)
    {
       boost::unique_lock<boost::mutex> lock(jt_mutex);
       boost::copy(armState_.torque,output);

    }

    template<typename OutputIterator>
    void get(OutputIterator output, grl::revolute_joint_torque_external_open_chain_state_tag)
    {
        boost::unique_lock<boost::mutex> lock(jt_mutex);
        boost::copy(armState_.externalTorque,output);

    }

    template<typename OutputIterator>
    void get(OutputIterator output, grl::cartesian_external_force_tag)
    {
        boost::unique_lock<boost::mutex> lock(jt_mutex);
        boost::copy(armState_.externalForce,output);

    }

    /// @todo(ahundt) replace with standard get() call with tag dispatch like above
   /// get 6 element wrench entries
   /// [force_x, force_y, force_z, torque_x, torque_y, torque_z]
   /// @param output a C++ OutputIterator which adds the values to your output object,
   ///        see http://en.cppreference.com/w/cpp/iterator/back_inserter
    // template<typename OutputIterator>
    // void getWrench(OutputIterator output)
    // {
    //     boost::unique_lock<boost::mutex> lock(jt_mutex);
    //     JAVAdriverP_->getWrench(output);
    // }

        template<typename Container>
        void getWrench(Container output)
       {
          boost::unique_lock<boost::mutex> lock(jt_mutex);
          JAVAdriverP_->getWrench(output);
       }

      volatile std::size_t m_haveReceivedRealDataCount = 0;
      volatile std::size_t m_attemptedCommunicationCount = 0;
      volatile std::size_t m_attemptedCommunicationConsecutiveFailureCount = 0;
      volatile std::size_t m_attemptedCommunicationConsecutiveSuccessCount = 0;

      boost::asio::io_service device_driver_io_service;
      std::unique_ptr<boost::asio::io_service::work> device_driver_workP_;
      std::unique_ptr<std::thread> driver_threadP;



    private:

      KukaState armState_;

      boost::mutex jt_mutex;
      boost::shared_ptr<KukaFRIdriver<LinearInterpolation>> FRIdriverP_;
      boost::shared_ptr<KukaJAVAdriver> JAVAdriverP_;

      Params params_;

      bool debug;

    };

}}}// namespace grl::robot::arm

#endif
