
// Library includes
#include <string>
#include <ostream>
#include <iostream>
#include <memory>

#include "robone/KukaFRIThreadSeparator.hpp"
#include "robone/KukaFRI.hpp"
#include "robone/KukaFriClientData.hpp"

#include <boost/asio.hpp>


using boost::asio::ip::udp;

enum { max_length = 1024 };

int main(int argc, char* argv[])
{
  try
  {
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

    robone::KukaFRIThreadSeparator kukaFRIThreadSeparator(io_service);


    double delta = 0.0001;
	for (std::size_t i = 0;;++i) {
        kukaFRIThreadSeparator.async_getLatestState([&delta,&kukaFRIThreadSeparator](std::shared_ptr<robone::robot::arm::kuka::iiwa::MonitorState> updatedState){
            boost::container::static_vector<double,KUKA::LBRState::NUM_DOF> jointAngles;
            robone::robot::arm::copy(updatedState->get(), std::back_inserter(jointAngles), robone::revolute_joint_angle_open_chain_state_tag());
            
            /// consider moving joint angles based on time
            jointAngles[6]+=delta;
            if (jointAngles[6] >  1.5 && delta > 0) delta *=-1;
            if (jointAngles[6] < -1.5 && delta < 0) delta *=-1;
            
            auto commandP = std::make_shared<robone::robot::arm::kuka::iiwa::CommandState>();
            robone::robot::arm::set(*commandP, jointAngles, robone::revolute_joint_angle_open_chain_command_tag());
            kukaFRIThreadSeparator.async_sendCommand(commandP);
            
            
        });
        
            kukaFRIThreadSeparator.run_user();
		
	}
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}