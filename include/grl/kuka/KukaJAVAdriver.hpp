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
        std::string
          > Params;


      static const Params defaultParams(){
        return std::make_tuple(
            "Robotiiwa"               , // RobotName,
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
        : params_(params), armControlMode(flatbuffer::ArmState::ArmState_NONE)
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
      
      bool startArm(){
        
          auto fbbP = kukaJavaDriverP->GetUnusedBufferBuilder();
          
            boost::lock_guard<boost::mutex> lock(jt_mutex);
          
          double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();
          
          /// @todo is this the best string to pass for the full arm's name?
          auto basename = std::get<RobotName>(params_);
          
          auto bns = fbbP->CreateString(basename);
          
          auto controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,flatbuffer::ArmState::ArmState_StartArm,flatbuffer::CreateStartArm(*fbbP).Union());
          
          
          auto jointPos = fbbP->CreateVector(&controlState, 1);
          
          auto armSeries = flatbuffer::CreateArmControlSeries(*fbbP,jointPos);
          
          
          
            grl::flatbuffer::FinishArmControlSeriesBuffer(*fbbP, armSeries);
            kukaJavaDriverP->async_send_flatbuffer(fbbP);
          
          return false;
      }
      
      
      bool teachArm(){
        
          auto fbbP = kukaJavaDriverP->GetUnusedBufferBuilder();
          
            boost::lock_guard<boost::mutex> lock(jt_mutex);
          
          double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();
          
          /// @todo is this the best string to pass for the full arm's name?
          auto basename = std::get<RobotName>(params_);
          
          auto bns = fbbP->CreateString(basename);
          
          auto controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,flatbuffer::ArmState::ArmState_TeachArm,flatbuffer::CreateTeachArm(*fbbP).Union());
          
          
          auto jointPos = fbbP->CreateVector(&controlState, 1);
          
          auto armSeries = flatbuffer::CreateArmControlSeries(*fbbP,jointPos);
          
          auto kukaiiwastate = flatbuffer::CreateKUKAiiwaState(*fbbP,0,0,0,0,1,controlState);
          
          auto kukaiiwaStateVec = fbbP->CreateVector(&kukaiiwastate, 1);
          
          auto states = flatbuffer::CreateKUKAiiwaStates(*fbbP,kukaiiwaStateVec);
          
          grl::flatbuffer::FinishKUKAiiwaStatesBuffer(*fbbP, states);
          kukaJavaDriverP->async_send_flatbuffer(fbbP);
          
          return false;
      }

      
      
      bool pauseArm(){
        
          auto fbbP = kukaJavaDriverP->GetUnusedBufferBuilder();
          
            boost::lock_guard<boost::mutex> lock(jt_mutex);
          
          double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();
          
          /// @todo is this the best string to pass for the full arm's name?
          auto basename = std::get<RobotName>(params_);
          
          auto bns = fbbP->CreateString(basename);
          
          auto controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,flatbuffer::ArmState::ArmState_PauseArm,flatbuffer::CreateTeachArm(*fbbP).Union());
          
          
          auto jointPos = fbbP->CreateVector(&controlState, 1);
          
          auto armSeries = flatbuffer::CreateArmControlSeries(*fbbP,jointPos);
          
          
          
            grl::flatbuffer::FinishArmControlSeriesBuffer(*fbbP, armSeries);
            kukaJavaDriverP->async_send_flatbuffer(fbbP);
          
          return false;
      }

      
      
      bool stopArm(){
        
          auto fbbP = kukaJavaDriverP->GetUnusedBufferBuilder();
          
            boost::lock_guard<boost::mutex> lock(jt_mutex);
          
          double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();
          
          /// @todo is this the best string to pass for the full arm's name?
          auto basename = std::get<RobotName>(params_);
          
          auto bns = fbbP->CreateString(basename);
          
          auto controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,flatbuffer::ArmState::ArmState_StopArm,flatbuffer::CreateTeachArm(*fbbP).Union());
          
          
          auto jointPos = fbbP->CreateVector(&controlState, 1);
          
          auto armSeries = flatbuffer::CreateArmControlSeries(*fbbP,jointPos);
          
          
          
            grl::flatbuffer::FinishArmControlSeriesBuffer(*fbbP, armSeries);
            kukaJavaDriverP->async_send_flatbuffer(fbbP);
          
          return false;
      }
      
      
      /// @todo add optional parameter? possibly additional config?
      bool sendJointPositions(){
      
          /////////////////////////////////////////
          // Client sends to server asynchronously!

          /// @todo if allocation is a performance problem use boost::container::static_vector<double,7>
          std::vector<double> joints;

          auto fbbP = kukaJavaDriverP->GetUnusedBufferBuilder();

          /// @todo should we use simJointTargetPosition here?
          joints.clear();
          {
            boost::lock_guard<boost::mutex> lock(jt_mutex);
            boost::copy(armState.commandedPosition, std::back_inserter(joints));
          }
          auto jointPos = fbbP->CreateVector(&joints[0], joints.size());

#if 0 //BOOST_VERSION < 105900
          BOOST_LOG_TRIVIAL(info) << "sending joint angles: " << joints << " from local zmq: " << std::get<LocalZMQAddress>            (params_) << " to remote zmq: " << std::get<RemoteZMQAddress>            (params_);
#endif

          /// @note we don't have a velocity right now, sending empty!
          joints.clear();
          //boost::copy(simJointVelocity, std::back_inserter(joints));
          {
            boost::lock_guard<boost::mutex> lock(jt_mutex);
            // no velocity data available directly in this arm at time of writing
            //boost::copy(simJointVelocity, std::back_inserter(joints));
            auto jointVel = fbbP->CreateVector(&joints[0], joints.size());
            joints.clear();
            boost::copy(armState.torque, std::back_inserter(joints));
            auto jointAccel = fbbP->CreateVector(&joints[0], joints.size());
            auto jointState = grl::flatbuffer::CreateJointState(*fbbP,jointPos,jointVel,jointAccel);
          
          
            
        
              auto fbbP = kukaJavaDriverP->GetUnusedBufferBuilder();
              
              double duration = boost::chrono::high_resolution_clock::now().time_since_epoch().count();
              
              /// @todo is this the best string to pass for the full arm's name?
              auto basename = std::get<RobotName>(params_);
              
              auto bns = fbbP->CreateString(basename);
              
              auto controlState = flatbuffer::CreateArmControlState(*fbbP,bns,sequenceNumber++,duration,flatbuffer::ArmState::ArmState_TeachArm,flatbuffer::CreateTeachArm(*fbbP).Union());
              
              
              auto jointPos = fbbP->CreateVector(&controlState, 1);
              
              auto armSeries = flatbuffer::CreateArmControlSeries(*fbbP,jointPos);
          
          
          
            grl::flatbuffer::FinishArmControlSeriesBuffer(*fbbP, armSeries);
            kukaJavaDriverP->async_send_flatbuffer(fbbP);
          
            grl::flatbuffer::FinishJointStateBuffer(*fbbP, jointState);
            kukaJavaDriverP->async_send_flatbuffer(fbbP);
          }
          
          return false;
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
          switch (armControlMode) {
          
              case flatbuffer::ArmState::ArmState_StartArm:
                 sendJointPositions();
                 break;
              case flatbuffer::ArmState::ArmState_MoveArmJointServo:
                 sendJointPositions();
                 break;
              case flatbuffer::ArmState::ArmState_TeachArm:
                 teachArm();
                 break;
              case flatbuffer::ArmState::ArmState_PauseArm:
                 pauseArm();
                 break;
              case flatbuffer::ArmState::ArmState_StopArm:
                 stopArm();
                 break;
              case flatbuffer::ArmState::ArmState_ShutdownArm:
                 destruct();
                 break;
              case flatbuffer::ArmState::ArmState_NONE:
                 //std::cerr << "Waiting for interation mode... (currently NONE)\n";
                 break;
              default:
                 std::cerr << "KukaJAVAdriver unsupported use case: " << armControlMode << "\n";
          }

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
      * This method is only effective when the client is in a commanding state.
      * @param state Object which stores the current state of the robot, including the command to send next
      * @param range Array with the new joint positions (in radians)
      * @param tag identifier object indicating that revolute joint angle commands should be modified
      */
   template<typename Range>
   void set(Range&& range, grl::revolute_joint_angle_open_chain_command_tag) {
       boost::lock_guard<boost::mutex> lock(jt_mutex);
       armState.commandedPosition.clear();
      boost::copy(range, std::back_inserter(armState.commandedPosition));
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
       armState.commandedTorque.clear();
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
       boost::lock_guard<boost::mutex> lock(jt_mutex);
      std::copy(range,armState.commandedCartesianWrenchFeedForward);
    }
    
    /// @todo implement get function
    template<typename OutputIterator>
    void get(OutputIterator /*output*/, grl::revolute_joint_angle_open_chain_state_tag)
    {
        BOOST_VERIFY(false); // not implemented yet
    }
    
   /// @todo should this exist? is it written correctly?
   void get(KukaState & state)
   {
     boost::lock_guard<boost::mutex> lock(jt_mutex);
     state = armState;
   }

    private:

      flatbuffer::ArmState armControlMode;
      KukaState armState;

      boost::mutex jt_mutex;

      Params params_;
      
      int64_t sequenceNumber;

    };    

}}}// namespace grl::robot::arm

#endif
