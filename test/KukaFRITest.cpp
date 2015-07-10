
// Library includes
#include <string>
#include <ostream>
#include <iostream>
#include <memory>

#include "grl/KukaFRIThreadSeparator.hpp"
#include "grl/KukaFRI.hpp"
#include "grl/KukaFriClientData.hpp"
#include "grl/stattimer.hpp"
#include "grl/realtime.hpp"

#include <boost/asio.hpp>
#include <boost/log/trivial.hpp>



template<typename T>
inline boost::log::formatting_ostream& operator<<(boost::log::formatting_ostream& out,  std::vector<T>& v)
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

using boost::asio::ip::udp;

enum { max_length = 1024 };

std::atomic<bool> stop;
std::atomic<bool> changed;
std::mutex printm;
std::vector<double> jointAnglesG;
KUKA::FRI::EConnectionQuality connectionQG;
KUKA::FRI::EOperationMode operationModeG;
enum TimerAccess {
  TimeLoop,
  TimeHandler,
  TimeDriver,
  TimerCount
};
STimerList timers(TimerCount);
std::atomic<uint64_t> timerLoops;

void printState(){

    std::vector<double> jointAngles;
    KUKA::FRI::EConnectionQuality connectionQ;
    KUKA::FRI::EOperationMode operationMode;
    changed = false;
    bool shouldprint = false;
    
    while (!stop) {
       
        if(changed){
            {
              std::lock_guard<std::mutex> lock(printm);
              if(connectionQG!=connectionQ) shouldprint = true;
              jointAngles = jointAnglesG;
              connectionQ = connectionQG;
              operationMode = operationModeG;
              changed = false;
            }
            
           if (timerLoops%1000 == 0) shouldprint = true;
           
            if(shouldprint)  {
              BOOST_LOG_TRIVIAL(trace) << "jointAngles: " << jointAngles << " connectionQuality: " << connectionQ << " operationMode: " << operationMode << "\n";
              BOOST_LOG_TRIVIAL(trace) << "\n" << timers.report();
            }
            shouldprint = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
   }
}


int main(int argc, char* argv[])
{
//  try
//  {
    std::string localhost("192.170.10.100");
    std::string localport("30200");
    std::string remotehost("192.170.10.2");
    std::string remoteport("30200");
  
    std::cout << "argc: " << argc << "\n";
	  /// @todo add default localhost/localport
    if (argc !=5 && argc !=1)
    {
      std::cerr << "Usage: " << argv[0] << " <localip> <localport> <remoteip> <remoteport>\n";
      return 1;
    }
  
    if(argc ==5){
      localhost = std::string(argv[1]);
      localport = std::string(argv[2]);
      remotehost = std::string(argv[3]);
      remoteport = std::string(argv[4]);
    }
    
      std::cout << "using: "  << argv[0] << " " <<  localhost << " " << localport << " " <<  remotehost << " " << remoteport << "\n";

    stop = false;
    std::thread printstateThread(printState);
    
    boost::asio::io_service io_service;
    auto kukaFRIThreadSeparator =
      std::make_shared<grl::KukaFRIThreadSeparator>(
        io_service,
	    std::make_tuple(localhost, localport, remotehost, remoteport,grl::KukaFRIThreadSeparator::run_manually)
      );
    
    //std::thread secondDriverThread([&io_service](){io_service.run();});


    double delta = 0.001;
    BOOST_LOG_TRIVIAL(warning) << "WARNING: YOU COULD DAMAGE OR DESTROY YOUR KUKA ROBOT "
                               << "if joint angle delta variable is too large with respect to "
                               << "the time it takes to go around the loop and change it. "
                               << "Current delta (radians/update): " << delta << "\n";
  
    std::function<void(std::shared_ptr<grl::robot::arm::kuka::iiwa::MonitorState>,
                       boost::system::error_code,
                       std::size_t,
                       boost::system::error_code,
                       std::size_t
                      )
                  > update_fn;
    
    // this implementation is a hack, not recommended for real code.
    update_fn =
      [&delta,kukaFRIThreadSeparator,&update_fn]
      (
          std::shared_ptr<grl::robot::arm::kuka::iiwa::MonitorState> updatedStateP,
          boost::system::error_code receive_ec,
          std::size_t receive_bytes_transferred,
          boost::system::error_code send_ec,
          std::size_t send_bytes_transferred
       ){
            // this is a hack so the test doesn't go crazy, not recommended for real code.
            if(updatedStateP.get()==nullptr || receive_ec || !receive_bytes_transferred){
              // If this is null there are already some calls waiting on results
              // Use the direct kuka driver if you don't have special thread requirements
              // If you need to get data to a "special" thread, like the primary GUI thread
              // in Qt, or VREP then use the kukaFRIThreadSeparator, and replace the 1ms sleep
              // with running the special stuff that thread really needs to do.
              //std::this_thread::sleep_for(std::chrono::milliseconds(1));
              kukaFRIThreadSeparator->async_getLatestState(update_fn);
              return;
            }
            {
              std::lock_guard<std::mutex> lock(printm);
              timers.stop(TimeDriver);
              timers.start(TimeHandler);
            }
        
            // ask for a new update right away to minimize delays
            kukaFRIThreadSeparator->async_getLatestState(update_fn);
        
            // get the joint angles from the monitorstate
            std::vector<double> jointAngles;
            grl::robot::arm::copy(*updatedStateP, std::back_inserter(jointAngles), grl::revolute_joint_angle_open_chain_state_tag());
          
            if(jointAngles.size()==7){
                // move the joint angles to test FRI commands
                /// @todo consider moving joint angles based on time
                jointAngles[6]+=delta;
                if (jointAngles[6] >  1.5 && delta > 0) delta *=-1;
                if (jointAngles[6] < -1.5 && delta < 0) delta *=-1;
            
                /// @todo need to avoid reallocating the CommandState every time, probably with async_MakeCommandState
                auto commandP = kukaFRIThreadSeparator->makeCommandState();
                grl::robot::arm::set(*commandP, jointAngles, grl::revolute_joint_angle_open_chain_command_tag());
                kukaFRIThreadSeparator->async_sendCommand(commandP);
                auto connectionQ = grl::robot::arm::get(*updatedStateP,  KUKA::FRI::EConnectionQuality());
                auto operationMode = grl::robot::arm::get(*updatedStateP,  KUKA::FRI::EOperationMode());
            
            
               {
                  std::lock_guard<std::mutex> lock(printm);
                  timers.laptime(TimeLoop);
                  jointAnglesG = jointAngles;
                  connectionQG = connectionQ;
                  operationModeG = operationMode;
               }
               timerLoops++;
               changed = true;
           }
            //BOOST_LOG_TRIVIAL(trace) << "jointAngles: " << jointAngles << " connectionQuality: " << connectionQ << " operationMode: " << operationMode << "\n";;
        
            // return the state to the system
            kukaFRIThreadSeparator->async_addMonitorState(updatedStateP);
            {
              std::lock_guard<std::mutex> lock(printm);
              timers.stop(TimeHandler);
              timers.start(TimeDriver);
            }
        };
    
        kukaFRIThreadSeparator->async_getLatestState(update_fn);

#ifdef HAVE_REALTIME
#ifdef __APPLE__
        int msToNS = 1000000, period = 1*msToNS, computation = 0.1*msToNS, constraint = 0.5*msToNS;
         bool preemptible = false;
        if(set_realtime( period,  computation,  constraint,  preemptible)){
           std::cout << "realtime mode successfully enabled!\n";
        };
#endif // __APPLE__
#endif // HAVE_REALTIME
    
  
        // create work so it will block forever but not chew cpu or memory
        //auto work = boost::asio::io_service::work(kukaFRIThreadSeparator->get_io_service());
        kukaFRIThreadSeparator->run();
        
//        // create work so it will block forever but not chew cpu or memory
//        auto work = boost::asio::io_service::work(kukaFRIThreadSeparator->get_user_io_service());
//        kukaFRIThreadSeparator->run_user();

        stop = true;
		
//  }
//  catch (std::exception& e)
//  {
//    std::cerr << "Exception: " << e.what() << "\n";
//  }

  return 0;
}
