
// Library includes
#include <string>
#include <ostream>
#include <iostream>
#include <memory>

#include "grl/KukaFRIThreadSeparator.hpp"
#include "grl/KukaFRI.hpp"
#include "grl/KukaFriClientData.hpp"

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

    boost::asio::io_service io_service;
    auto kukaFRIThreadSeparator = std::make_shared<grl::KukaFRIThreadSeparator>(io_service, 
	std::make_tuple(localhost, localport, remotehost, remoteport));


    double delta = 0.0001;
    BOOST_LOG_TRIVIAL(warning) << "WARNING: YOU COULD DAMAGE OR DESTROY YOUR KUKA ROBOT "
                               << "if joint angle delta variable is too large with respect to "
                               << "the time it takes to go around the loop and change it. "
                               << "Current delta (radians/update): " << delta << "\n";
  
    std::function<void(std::shared_ptr<grl::robot::arm::kuka::iiwa::MonitorState>)> update_fn;
    
    // this implementation is a hack, not recommended for real code.
    update_fn = [&delta,kukaFRIThreadSeparator,&update_fn](std::shared_ptr<grl::robot::arm::kuka::iiwa::MonitorState> updatedState){
            // this is a hack so the test doesn't go crazy, not recommended for real code.
            if(updatedState.get()==nullptr){
              // If this is null there are already some calls waiting on results
              // Use the direct kuka driver if you don't have special thread requirements
              // If you need to get data to a "special" thread, like the primary GUI thread
              // in Qt, or VREP then use the kukaFRIThreadSeparator, and replace the 1ms sleep
              // with running the special stuff that thread really needs to do.
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
              kukaFRIThreadSeparator->async_getLatestState(update_fn);
              return;
            }
        
            // ask for a new update right away to minimize delays
            kukaFRIThreadSeparator->async_getLatestState(update_fn);
        
            // get the joint angles from the monitorstate
            std::vector<double> jointAngles;
            grl::robot::arm::copy(updatedState->get(), std::back_inserter(jointAngles), grl::revolute_joint_angle_open_chain_state_tag());
            
            // move the joint angles to test FRI commands
            /// @todo consider moving joint angles based on time
            jointAngles[6]+=delta;
            if (jointAngles[6] >  1.5 && delta > 0) delta *=-1;
            if (jointAngles[6] < -1.5 && delta < 0) delta *=-1;
        
            /// @todo need to avoid reallocating the CommandState every time, probably with async_MakeCommandState
            auto commandP = std::make_shared<grl::robot::arm::kuka::iiwa::CommandState>();
            grl::robot::arm::set(*commandP, jointAngles, grl::revolute_joint_angle_open_chain_command_tag());
            kukaFRIThreadSeparator->async_sendCommand(commandP);
            auto connectionQ = grl::robot::arm::get(*updatedState,  KUKA::FRI::EConnectionQuality());
            auto operationMode = grl::robot::arm::get(*updatedState,  KUKA::FRI::EOperationMode());
            BOOST_LOG_TRIVIAL(trace) << "jointAngles: " << jointAngles << " connectionQuality: " << connectionQ << " operationMode: " << operationMode << "\n";;
        
            // return the state to the system
            kukaFRIThreadSeparator->async_addMonitorState(updatedState);
        };
    
        kukaFRIThreadSeparator->async_getLatestState(update_fn);
  
        // create work so it will block forever but not chew cpu or memory
        auto work = boost::asio::io_service::work(kukaFRIThreadSeparator->get_user_io_service());
        kukaFRIThreadSeparator->run_user();
  
		
//  }
//  catch (std::exception& e)
//  {
//    std::cerr << "Exception: " << e.what() << "\n";
//  }

  return 0;
}
