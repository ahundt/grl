#ifndef GRL_KUKA_DRIVER
#define GRL_KUKA_DRIVER


#include <tuple>
#include <memory>
#include <thread>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/exception/all.hpp>
#include <boost/config.hpp>

#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/transform.hpp>


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
    /// @todo enable commanding and monitoring to be independently configured for both FRI and JAVA interface.
    ///
    class KukaDriver : public std::enable_shared_from_this<KukaDriver> {
    public:

      enum ParamIndex {
        RobotTipName,
        RobotTargetName,
        RobotTargetBaseName,
        LocalZMQAddress,
        RemoteZMQAddress,
        LocalZMQConfigAddress,
        RemoteZMQConfigAddress,
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
            "tcp://0.0.0.0:30011"     , // LocalZMQConfigAddress
            "tcp://172.31.1.147:30011", // RemoteZMQConfigAddress
            "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
            "30200"                   , // LocalHostKukaKoniUDPPort,
            "192.170.10.2"            , // RemoteHostKukaKoniUDPAddress,
            "30200"                   , // RemoteHostKukaKoniUDPPort
            "JAVA"                     , // KukaCommandMode (options are FRI, JAVA)
            "JAVA"                     , // KukaMonitorMode (options are FRI, JAVA)
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


      KukaDriver(Params params = defaultParams())
        : params_(params)
      {}

      void construct(){ construct(params_);}
      
      bool destruct(){ return JAVAdriverP_->destruct(); }
      
      bool startArm(){ return JAVAdriverP_->startArm(); }
      
      bool teachMode(){ return JAVAdriverP_->teachArm(); }
      
      bool pauseArm(){ return JAVAdriverP_->pauseArm(); }
      
      bool stopArm(){ return JAVAdriverP_->stopArm(); }
      
      bool sendJointPositions(){ return JAVAdriverP_->sendJointPositions(); }
      

      /// @todo create a function that calls simGetObjectHandle and throws an exception when it fails
      /// @warning getting the ik group is optional, so it does not throw an exception
      void construct(Params params) {

        params_ = params;
        // keep driver threads from exiting immediately after creation, because they have work to do!
        //device_driver_workP_.reset(new boost::asio::io_service::work(device_driver_io_service));

        /// @todo figure out how to re-enable when .so isn't loaded
        if(    boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI"))
            || boost::iequals(std::get<KukaMonitorMode>(params_),std::string("FRI")))
        {
          FRIdriverP_.reset(
              new grl::robot::arm::KukaFRIdriver(
                  //device_driver_io_service,
                  std::make_tuple(
                      std::string(std::get<LocalHostKukaKoniUDPAddress >        (params)),
                      std::string(std::get<LocalHostKukaKoniUDPPort    >        (params)),
                      std::string(std::get<RemoteHostKukaKoniUDPAddress>        (params)),
                      std::string(std::get<RemoteHostKukaKoniUDPPort   >        (params)),
                      ms_per_tick ,
                      grl::robot::arm::KukaFRIClientDataDriver::run_automatically
                      )
                  )

              );
              FRIdriverP_->construct();
        }

        /// @todo implement reading configuration in both FRI and JAVA mode from JAVA interface
        if(    boost::iequals(std::get<KukaCommandMode>(params_),std::string("JAVA"))){
        try {
          JAVAdriverP_ = boost::make_shared<KukaJAVAdriver>(params_);
          JAVAdriverP_->construct();

          // start up the driver thread
          /// @todo perhaps allow user to control this?
          //driver_threadP.reset(new std::thread([&]{ device_driver_io_service.run(); }));
        } catch( boost::exception &e) {
          e << errmsg_info("KukaDriver: Unable to connect to ZeroMQ Socket from " + 
                           std::get<LocalZMQAddress>             (params_) + " to " + 
                           std::get<RemoteZMQAddress>            (params_));
          throw;
        }
        
        }
      }




      bool setState(State& state) { return true; }


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
      * spin once 
      *
      */
      bool run_one(){

        // @todo CHECK FOR REAL DATA BEFORE SENDING COMMANDS
        //if(!m_haveReceivedRealDataCount) return;
        bool haveNewData = false;

        /// @todo make this handled by template driver implementations/extensions

        if(JAVAdriverP_.get() != nullptr)
        {
          /////////////////////////////////////////
          // Client sends to server asynchronously!
          if( boost::iequals(std::get<KukaCommandMode>(params_),std::string("JAVA")))
          {
            JAVAdriverP_->set(armState.commandedPosition,revolute_joint_angle_open_chain_command_tag());
          }
        
          haveNewData = JAVAdriverP_->run_one();
        
          if( boost::iequals(std::get<KukaMonitorMode>(params_),std::string("JAVA")))
          {
            JAVAdriverP_->get(armState);
          }
        }
        
        if(FRIdriverP_.get() != nullptr)
        {
          if( boost::iequals(std::get<KukaCommandMode>(params_),std::string("FRI")))
          {
            FRIdriverP_->set(armState.commandedPosition,revolute_joint_angle_open_chain_command_tag());
          }
        
          haveNewData = FRIdriverP_->run_one();
        
          if( boost::iequals(std::get<KukaMonitorMode>(params_),std::string("FRI")))
          {
            FRIdriverP_->get(armState);
          }
        }
        
        return haveNewData;
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
       armState.clearCommands();
       boost::copy(range, std::back_inserter(armState.commandedPosition));
       boost::copy(range, std::back_inserter(armState.commandedPosition_goal));
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
       armState.clearCommands();
       boost::copy(range, std::back_inserter(armState.commandedTorque));
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
       armState.clearCommands();
       std::copy(range,armState.commandedCartesianWrenchFeedForward);
    }
    
    /// @todo implement get function
    template<typename OutputIterator>
    void get(OutputIterator output, grl::revolute_joint_angle_open_chain_state_tag)
    {
       boost::unique_lock<boost::mutex> lock(jt_mutex);
        boost::copy(armState.position,output);
    }
    
    /// @todo implement get function
    template<typename OutputIterator>
    void get(OutputIterator output, grl::revolute_joint_torque_open_chain_state_tag)
    {
       boost::unique_lock<boost::mutex> lock(jt_mutex);
        boost::copy(armState.torque,output);
    }
      volatile std::size_t m_haveReceivedRealDataCount = 0;
      volatile std::size_t m_attemptedCommunicationCount = 0;
      volatile std::size_t m_attemptedCommunicationConsecutiveFailureCount = 0;
      volatile std::size_t m_attemptedCommunicationConsecutiveSuccessCount = 0;

      boost::asio::io_service device_driver_io_service;
      std::unique_ptr<boost::asio::io_service::work> device_driver_workP_;
      std::unique_ptr<std::thread> driver_threadP;
 
    

    private:

      /// @todo read ms_per_tick from JAVA interface
      std::size_t ms_per_tick = 1;
      KukaState armState;

      boost::mutex jt_mutex;
      boost::shared_ptr<KukaFRIdriver> FRIdriverP_;
      boost::shared_ptr<KukaJAVAdriver> JAVAdriverP_;

      Params params_;

    };    

}}}// namespace grl::robot::arm

#endif
