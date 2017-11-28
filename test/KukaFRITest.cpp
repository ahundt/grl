
// Library includes
#include <string>
#include <ostream>
#include <iostream>
#include <memory>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "grl/periodic.hpp"
#include "grl/kuka/KukaFRIdriver.hpp"
#include "grl/vector_ostream.hpp"
#include "grl/kuka/KukaDriver.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include <boost/asio.hpp>



using boost::asio::ip::udp;


enum { max_length = 1024 };

enum class HowToMove
{
   remain_stationary,
   absolute_position,
   relative_position,
   absolute_position_with_relative_rotation
};

enum class DriverToUse
{
   low_level_fri_function,
   low_level_fri_class,
   kuka_driver_high_level_class
};

int main(int argc, char* argv[])
{
  bool debug = true;
  int print_every_n = 100;
  std::size_t q_size = 4096; //queue size must be power of 2
  spdlog::set_async_mode(q_size);
  std::shared_ptr<spdlog::logger>                  loggerPG;
  try {
    loggerPG = spdlog::stdout_logger_mt("console");
  }
  catch (spdlog::spdlog_ex ex) 	{
      loggerPG = spdlog::get("console");
  }

  grl::periodic<> callIfMinPeriodPassed;
  HowToMove howToMove = HowToMove::absolute_position_with_relative_rotation;//HowToMove::absolute_position; HowToMove::relative_position;
  DriverToUse driverToUse = DriverToUse::kuka_driver_high_level_class;

  try
  {
    std::string localhost("192.170.10.100");
    std::string localport("30200");
    std::string remotehost("192.170.10.2");
    std::string remoteport("30200");

    std::cout << "argc: " << argc << "\n";
    if (argc !=5 && argc !=1)
    {
      loggerPG->error("Usage: ", argv[0], " <localip> <localport> <remoteip> <remoteport>\n");
      return 1;
    }

    if(argc ==5){
      localhost = std::string(argv[1]);
      localport = std::string(argv[2]);
      remotehost = std::string(argv[3]);
      remoteport = std::string(argv[4]);
    }

      std::cout << "using: "  << argv[0] << " " <<  localhost << " " << localport << " " <<  remotehost << " " << remoteport << "\n";
    // A single class for an I/O service object.
    boost::asio::io_service io_service;

	std::shared_ptr<KUKA::FRI::ClientData> friData(std::make_shared<KUKA::FRI::ClientData>(7));
	/// std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    cartographer::common::Time startTime;

    BOOST_VERIFY(friData);

    double delta = -0.0005;
    double delta_sum = 0;
    /// consider moving joint angles based on time
    int joint_to_move = 6;
    loggerPG->warn("WARNING: YOU COULD DAMAGE OR DESTROY YOUR KUKA ROBOT {}{}{}{}{}{}",
                      "if joint angle delta variable is too large with respect to ",
                      "the time it takes to go around the loop and change it. ",
                      "Current delta (radians/update): ", delta, " Joint to move: ", joint_to_move);

    std::vector<double> ipoJointPos(7,0);
    std::vector<double> jointOffset(7,0); // length 7, value 0
    boost::container::static_vector<double, 7> jointStateToCommand(7,0);

    // Absolute goal position to travel to in some modes of HowToMove
    // Set all 7 joints to go to a position 1 radian from the center
    std::vector<double> absoluteGoalPos(7,0.2);

    /// TODO(ahundt) remove deprecated arm state from here and implementation
    grl::robot::arm::KukaState armState;
    std::unique_ptr<grl::robot::arm::LinearInterpolation> lowLevelStepAlgorithmP;

    // Need to tell the system how long in milliseconds it has to reach the goal
    // or it will never move!
    std::size_t goal_position_command_time_duration = 4;
    lowLevelStepAlgorithmP.reset(new grl::robot::arm::LinearInterpolation());
    // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800) see grl::robot::arm::KukaState::KUKA_LBR_IIWA_14_R820

    std::shared_ptr<grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation>> highLevelDriverClassP;

    if(driverToUse == DriverToUse::low_level_fri_class)
    {
      /// @todo TODO(ahundt) BUG: Need way to supply time to reach specified goal for position control and eliminate this allocation internally in the kuka driver. See similar comment in KukaFRIDriver.hpp
      /// IDEA: PASS A LOW LEVEL STEP ALGORITHM PARAMS OBJECT ON EACH UPDATE AND ONLY ONE INSTANCE OF THE ALGORITHM OBJECT ITSELF
      highLevelDriverClassP = std::make_shared<grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation>>(io_service,
        std::make_tuple("KUKA_LBR_IIWA_14_R820",
                        localhost,
                        localport,
                        remotehost,
                        remoteport/*,4 ms per tick*/,
                        grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation>::run_automatically));

    }

    std::shared_ptr<boost::asio::ip::udp::socket> socketP;

    if(driverToUse == DriverToUse::low_level_fri_function)
    {

        socketP = std::make_shared<boost::asio::ip::udp::socket>(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(localhost), boost::lexical_cast<short>(localport)));

        boost::asio::ip::udp::resolver resolver(io_service);
        boost::asio::ip::udp::endpoint endpoint = *resolver.resolve({boost::asio::ip::udp::v4(), remotehost, remoteport});
        socketP->connect(endpoint);

        /// @todo maybe there is a more convienient way to set this that is easier for users? perhaps initializeClientDataForiiwa()?
        friData->expectedMonitorMsgID = KUKA::LBRState::LBRMONITORMESSAGEID;
    }

    std::shared_ptr<grl::robot::arm::KukaDriver> kukaDriverP;

    if(driverToUse == DriverToUse::kuka_driver_high_level_class)
    {
        grl::robot::arm::KukaDriver::Params params = std::make_tuple(
                "Robotiiwa"               , // RobotName,
                "KUKA_LBR_IIWA_14_R820"   , // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800)
                "0.0.0.0"                 , // LocalUDPAddress
                "30010"                   , // LocalUDPPort
                "172.31.1.147"            , // RemoteUDPAddress
                "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
                "30200"                   , // LocalHostKukaKoniUDPPort,
                remotehost                , // RemoteHostKukaKoniUDPAddress,
                remoteport                , // RemoteHostKukaKoniUDPPort
                "FRI"                     , // KukaCommandMode (options are FRI, JAVA)
                "FRI"                       // KukaMonitorMode (options are FRI, JAVA)
                );
        /// @todo TODO(ahundt) Currently assumes ip address
        kukaDriverP=std::make_shared<grl::robot::arm::KukaDriver>(params);
        kukaDriverP->construct();
        // Default to joint servo mode for commanding motion
        kukaDriverP->set(grl::flatbuffer::ArmState::MoveArmJointServo);
        kukaDriverP->set(goal_position_command_time_duration,grl::time_duration_command_tag());
        std::cout << "KUKA COMMAND MODE: " << std::get<grl::robot::arm::KukaDriver::KukaCommandMode>(params) << "\n";

    }


    unsigned int num_missed = 0;

	for (std::size_t i = 0;;++i) {

        /// Save the interpolated joint position from the previous update as the base for some motions
        /// The interpolated position is where the JAVA side is commanding,
        /// Specifically in the case of the GRL drivers this the fixed starting position set
        /// with a hold position command on the java side.
        if(i!=0 && friData) grl::robot::arm::copy(friData->monitoringMsg,ipoJointPos.begin(),grl::revolute_joint_angle_interpolated_open_chain_state_tag());

        /// perform the update step, receiving and sending data to/from the arm
        boost::system::error_code send_ec, recv_ec;
        std::size_t send_bytes_transferred = 0, recv_bytes_transferred = 0;
        bool haveNewData = false;
        grl::TimeEvent time_event_stamp;
        if(driverToUse == DriverToUse::low_level_fri_class)
        {
            auto step_commandP = std::make_shared<grl::robot::arm::LinearInterpolation::Params>(std::make_tuple(jointStateToCommand, goal_position_command_time_duration));

            haveNewData = !highLevelDriverClassP->update_state(step_commandP,
                                                               friData,
                                                               recv_ec,
                                                               recv_bytes_transferred,
                                                               send_ec,
                                                               send_bytes_transferred,
                                                               time_event_stamp);
        }

        if(driverToUse == DriverToUse::low_level_fri_function)
        {
            /// This update_state function is different from the above one.
            /// They both are defined in KukaFRIdriver.hpp.
            /// Also should the argument time_event_stam be the same with above one?
            grl::robot::arm::update_state(*socketP,
                                          *lowLevelStepAlgorithmP,
                                          *friData,
                                          send_ec,
                                          send_bytes_transferred,
                                          recv_ec,
                                          recv_bytes_transferred,
                                          time_event_stamp);
        }

        if(driverToUse == DriverToUse::kuka_driver_high_level_class)
        {

            kukaDriverP->set( jointStateToCommand, grl::revolute_joint_angle_open_chain_command_tag());
            kukaDriverP->run_one();
        }

        // if data didn't arrive correctly, skip and try again
        if(send_ec || recv_ec )
        {

           loggerPG->error("receive error: ", recv_ec, "receive bytes: ", recv_bytes_transferred, " send error: ", send_ec, " send bytes: ", send_bytes_transferred,  " iteration: ", i);
           if(driverToUse == DriverToUse::low_level_fri_class) std::this_thread::sleep_for(std::chrono::milliseconds(1));
           continue;
        }

        // If we didn't receive anything new that is normal behavior,
        // but we can't process the new data so try updating again immediately.
        if(!haveNewData && !recv_bytes_transferred)
        {
          if(driverToUse == DriverToUse::low_level_fri_class) std::this_thread::sleep_for(std::chrono::milliseconds(1));
          ++num_missed;
          if(num_missed>10000) {
            loggerPG->warn("No new data for ", num_missed, " milliseconds.");
            break;
          } else {
            continue;
          }
        } else {
          num_missed = 0;
        }

        /// use the interpolated joint position from the previous update as the base
        /// The interpolated position is where the java side is commanding,
        /// or the fixed starting position with a hold position command on the java side.
        if(i!=0 && friData) grl::robot::arm::copy(friData->monitoringMsg,ipoJointPos.begin(),grl::revolute_joint_angle_interpolated_open_chain_state_tag());


        // setting howToMove to HowToMove::remain_stationary block causes the robot to simply sit in place, which seems to work correctly. Enabling it causes the joint to rotate.
        if (howToMove != HowToMove::remain_stationary &&
            grl::robot::arm::get(friData->monitoringMsg,KUKA::FRI::ESessionState()) == KUKA::FRI::COMMANDING_ACTIVE)
        {
            callIfMinPeriodPassed.execution( [&armState,&jointOffset,&delta,&delta_sum,joint_to_move]()
            {
                    // Here we are using the time step defined in the FRI communication frequency but larger values are ok.
                    jointOffset[joint_to_move]+=delta;
                    delta_sum+=delta;
                    // swap directions when a half circle was completed
                    if (
                         (delta_sum >  0.2 && delta > 0) ||
                         (delta_sum < -0.2 && delta < 0)
                       )
                    {
                       delta *=-1;
                    }
            });
        }

        KUKA::FRI::ESessionState sessionState = grl::robot::arm::get(friData->monitoringMsg,KUKA::FRI::ESessionState());
        // copy current joint position to commanded position
        if (sessionState == KUKA::FRI::COMMANDING_WAIT || sessionState == KUKA::FRI::COMMANDING_ACTIVE)
        {
            if (howToMove == HowToMove::relative_position) {
                // go to a position relative to the current position
                boost::transform ( ipoJointPos, jointOffset, jointStateToCommand.begin(), std::plus<double>());
            } else if (howToMove == HowToMove::absolute_position) {
                // go to an absolute position
                boost::copy ( absoluteGoalPos, jointStateToCommand.begin());
            } else if (howToMove == HowToMove::absolute_position_with_relative_rotation) {
                // go to a position relative to the current position

                boost::transform ( absoluteGoalPos, jointOffset, jointStateToCommand.begin(), std::plus<double>());
            }
        }

        // copy the state data into a more accessible object
        /// TODO(ahundt) switch from this copy to a non-deprecated call
        grl::robot::arm::copy(friData->monitoringMsg,armState);
        if(debug && (i % print_every_n) ==0 ) {
            loggerPG->info(
                "position: {}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}", armState.position,
                " commanded Position: ", jointStateToCommand,
                /// " us: ", std::chrono::duration_cast<std::chrono::microseconds>(armState.timestamp - startTime).count(),
                " us: ", std::chrono::duration_cast<std::chrono::microseconds>(armState.time_event_stamp.device_time - startTime).count(),
                " connectionQuality: ", EnumNameEConnectionQuality(armState.connectionQuality),
                " operationMode: ", EnumNameEOperationMode(armState.operationMode),
                " sessionState: ", EnumNameESessionState(armState.sessionState),
                " driveState: ", EnumNameEDriveState(armState.driveState),
                " ipoJointPosition: ", armState.ipoJointPosition,
                " jointOffset: ", jointOffset);
        }

	}
  }
  catch (boost::exception &e)
  {
    std::string errmsg("If you get an error 'std::exception::what: bind: Can't assign requested address', check your network connection.\n\nKukaFRITest Main Test Loop Stopped:\n" + boost::diagnostic_information(e));
    loggerPG->error(errmsg);
    spdlog::drop_all();
    //
    return 1;
  }

  // Release and close all loggers
  spdlog::drop_all();
  return 0;
}
