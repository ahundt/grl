
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

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include <boost/asio.hpp>



using boost::asio::ip::udp;


enum { max_length = 1024 };

enum class HowToMove
{
   absolute_position,
   relative_position
};

int main(int argc, char* argv[])
{
  bool debug = true;
  int print_every_n = 100;
  std::size_t q_size = 4096; //queue size must be power of 2
  spdlog::set_async_mode(q_size);
  std::shared_ptr<spdlog::logger>                  loggerPG;
	try 	{ 		 loggerPG = spdlog::stdout_logger_mt("console"); 	} 	catch (spdlog::spdlog_ex ex) 	{ 		loggerPG = spdlog::get("console"); 	}

  grl::periodic<> callIfMinPeriodPassed;
  HowToMove howToMove = HowToMove::relative_position;

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

    boost::asio::io_service io_service;

	std::shared_ptr<KUKA::FRI::ClientData> friData(std::make_shared<KUKA::FRI::ClientData>(7));
	std::chrono::time_point<std::chrono::high_resolution_clock> startTime;

    BOOST_VERIFY(friData);

    double delta = -0.0005;
    double delta_sum = 0;
    /// consider moving joint angles based on time
    int joint_to_move = 6;
    loggerPG->warn("WARNING: YOU COULD DAMAGE OR DESTROY YOUR KUKA ROBOT ",
                      "if joint angle delta variable is too large with respect to ",
                      "the time it takes to go around the loop and change it. ",
                      "Current delta (radians/update): ", delta, " Joint to move: ", joint_to_move);

    std::vector<double> ipoJointPos(7,0);
    std::vector<double> jointOffset(7,0); // length 7, value 0
    std::vector<double> jointStateToCommand(7,0);
    std::vector<double> absoluteGoalPos(7,0);

    /// TODO(ahundt) remove deprecated arm state from here and implementation
    grl::robot::arm::KukaState armState;
    std::unique_ptr<grl::robot::arm::LinearInterpolation> lowLevelStepAlgorithmP;
    armState.goal_position_command_time_duration = 4;
    lowLevelStepAlgorithmP.reset(new grl::robot::arm::LinearInterpolation(armState));
    // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800) see grl::robot::arm::KukaState::KUKA_LBR_IIWA_14_R820

      /// @todo TODO(ahundt) BUG: Need way to supply time to reach specified goal for position control and eliminate this allocation internally in the kuka driver. See similar comment in KukaFRIDriver.hpp
      /// IDEA: PASS A LOW LEVEL STEP ALGORITHM PARAMS OBJECT ON EACH UPDATE AND ONLY ONE INSTANCE OF THE ALGORITHM OBJECT ITSELF
    grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation> driver(io_service,
        std::make_tuple("KUKA_LBR_IIWA_14_R820",localhost,localport,remotehost,remoteport/*,4 ms per tick*/,grl::robot::arm::KukaFRIClientDataDriver<grl::robot::arm::LinearInterpolation>::run_automatically)
    );

    //
    unsigned int num_missed = 0;
    if(howToMove == HowToMove::absolute_position)
    {
        /// @todo TODO(ahundt) BUG: Need way to supply time to reach specified goal for position control
        // Execute a single move to the absolute goal position
        // For example you can say you want to make your move over 5000 ms
        armState.goal_position_command_time_duration = 5000; // ms
        absoluteGoalPos = std::vector<double>(7,0.1);
    }

	for (std::size_t i = 0;;++i) {

        /// use the interpolated joint position from the previous update as the base
        /// @todo why is this?
        if(i!=0 && friData) grl::robot::arm::copy(friData->monitoringMsg,ipoJointPos.begin(),grl::revolute_joint_angle_interpolated_open_chain_state_tag());

        /// perform the update step, receiving and sending data to/from the arm
        boost::system::error_code send_ec, recv_ec;
        std::size_t send_bytes_transferred = 0, recv_bytes_transferred = 0;
        bool haveNewData = !driver.update_state(*lowLevelStepAlgorithmP, friData, recv_ec, recv_bytes_transferred, send_ec, send_bytes_transferred);

        // if data didn't arrive correctly, skip and try again
        if(send_ec || recv_ec )
        {
           loggerPG->error("receive error: ", recv_ec, "receive bytes: ", recv_bytes_transferred, " send error: ", send_ec, " send bytes: ", send_bytes_transferred,  " iteration: ", i);
           std::this_thread::sleep_for(std::chrono::milliseconds(1));
           continue;
        }

        // If we didn't receive anything new that is normal behavior,
        // but we can't process the new data so try updating again immediately.
        if(!haveNewData)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
        /// @todo why is this?
        if(i!=0 && friData) grl::robot::arm::copy(friData->monitoringMsg,ipoJointPos.begin(),grl::revolute_joint_angle_interpolated_open_chain_state_tag());



        if (grl::robot::arm::get(friData->monitoringMsg,KUKA::FRI::ESessionState()) == KUKA::FRI::COMMANDING_ACTIVE)
        {
#if 1 // disabling this block causes the robot to simply sit in place, which seems to work correctly. Enabling it causes the joint to rotate.
            callIfMinPeriodPassed.execution( [&howToMove,&friData,&armState,&jointOffset,&delta,&delta_sum,joint_to_move]()
            {
                    // Need to tell the system how long in milliseconds it has to reach the goal or it will never move!
                    // Here we are using the time step defined in the FRI communication frequency but larger values are ok.
                    //armState.goal_position_command_time_duration = grl::robot::arm::get(friData->monitoringMsg, grl::time_step_tag()); // ms
                    //armState.goal_position_command_time_duration = 4;
                    // increment relative goal position
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

#endif
        }

        KUKA::FRI::ESessionState sessionState = grl::robot::arm::get(friData->monitoringMsg,KUKA::FRI::ESessionState());
        // copy current joint position to commanded position
        if (sessionState == KUKA::FRI::COMMANDING_WAIT || sessionState == KUKA::FRI::COMMANDING_ACTIVE)
        {
            if (howToMove == HowToMove::relative_position) {
                // go to a position relative to the current position
                boost::transform ( ipoJointPos, jointOffset, jointStateToCommand.begin(), std::plus<double>());
            } else if (howToMove == HowToMove::absolute_position) {
                // go to a position relative to the current position
                boost::transform ( absoluteGoalPos, jointOffset, jointStateToCommand.begin(), std::plus<double>());
            }
            grl::robot::arm::set(friData->commandMsg, jointStateToCommand, grl::revolute_joint_angle_open_chain_command_tag());
        }

        // copy the state data into a more accessible object
        /// TODO(ahundt) switch from this copy to a non-deprecated call
        grl::robot::arm::copy(friData->monitoringMsg,armState);
        if(debug && (i % print_every_n) ==0 ) loggerPG->info("position: {}{}{}{}{}{}{}{}{}{}{}{}{}{}{}", armState.position, " us: ", std::chrono::duration_cast<std::chrono::microseconds>(armState.timestamp - startTime).count(), " connectionQuality: ", armState.connectionQuality, " operationMode: ", armState.operationMode, " sessionState: ", armState.sessionState, " driveState: ", armState.driveState, " ipoJointPosition: ", armState.ipoJointPosition, " jointOffset: ", jointOffset);

	}
  }
  catch (boost::exception &e)
  {
    std::string errmsg("If you get an error 'std::exception::what: bind: Can't assign requested address', check your network connection.\n\nKukaFRIClientDataDriverTest Main Test Loop Stopped:\n" + boost::diagnostic_information(e));
    loggerPG->error(errmsg);
    spdlog::drop_all();
    //
    return 1;
  }

  // Release and close all loggers
  spdlog::drop_all();
  return 0;
}
