
// Library includes
#include <string>
#include <ostream>
#include <iostream>
#include <memory>

#include "grl/KukaFRI.hpp"
#include "grl/KukaFriClientData.hpp"
#include <boost/log/trivial.hpp>


#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include <iostream>



//
//template<typename T,typename V>
//inline T& printseq(T& out, V& v){
//    out << "[";
//    size_t last = v.size() - 1;
//    for(size_t i = 0; i < v.size(); ++i) {
//        out << v[i];
//        if (i != last) 
//            out << ", ";
//    }
//    out << "]";
//    return out;
//}
//
//template<typename T,size_t N>
//inline boost::log::formatting_ostream& operator<< (boost::log::formatting_ostream& out, const boost::container::static_vector<T,N>& v) {
//  return printseq(out,v);
//}
//
//
//template<typename T,size_t N>
//ostream& operator<< (ostream& out, const boost::container::static_vector<T,N>& v) {
//  return printseq(out,v);
//}
//
//
//template<typename T>
//inline boost::log::formatting_ostream& operator<<(boost::log::formatting_ostream& out,  std::vector<T>& v)
//{
//    return printseq(out, v);
//}
//template<typename T>
//inline std::ostream& operator<<(std::ostream& out,  std::vector<T>& v)
//{
//    return printseq(out,v);
//}

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

    boost::asio::ip::udp::socket s(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(localhost), boost::lexical_cast<short>(localport)));

    boost::asio::ip::udp::resolver resolver(io_service);
    boost::asio::ip::udp::endpoint endpoint = *resolver.resolve({boost::asio::ip::udp::v4(), remotehost, remoteport});
	s.connect(endpoint);
	
	KUKA::FRI::ClientData friData(7);
    /// @todo maybe there is a more convienient way to set this that is easier for users? perhaps initializeClientDataForiiwa()?
    friData.expectedMonitorMsgID = KUKA::LBRState::LBRMONITORMESSAGEID;

	std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
  
    
    double delta = -0.001;
    /// consider moving joint angles based on time
    int joint_to_move = 6;
    BOOST_LOG_TRIVIAL(warning) << "WARNING: YOU COULD DAMAGE OR DESTROY YOUR KUKA ROBOT "
                               << "if joint angle delta variable is too large with respect to "
                               << "the time it takes to go around the loop and change it. "
                               << "Current delta (radians/update): " << delta << " Joint to move: " << joint_to_move << "\n";
  
    std::vector<double> ipoJointPos(7,0);
    std::vector<double> offsetFromipoJointPos(7,0); // length 7, value 0
    std::vector<double> jointStateToCommand(7,0);

	for (std::size_t i = 0;;++i) {
    
        /// use the interpolated joint position from the previous update as the base
        /// @todo why is this?
        grl::robot::arm::copy(friData.monitoringMsg,ipoJointPos.begin(),grl::revolute_joint_angle_interpolated_open_chain_state_tag());
        
        /// perform the update step, receiving and sending data to/from the arm
        boost::system::error_code send_ec, recv_ec;
        std::size_t send_bytes_transferred, recv_bytes_transferred;
        grl::robot::arm::update_state(s,friData,send_ec,send_bytes_transferred, recv_ec, recv_bytes_transferred);
        
        // if data didn't arrive correctly, skip and try again
        if(send_ec || recv_ec) continue;
        
        
        if
        (
            grl::robot::arm::get(friData.monitoringMsg,KUKA::FRI::ESessionState()) == KUKA::FRI::COMMANDING_ACTIVE
        )
        {
            offsetFromipoJointPos[joint_to_move]+=delta;
            // swap directions when a half circle was completed
            if (
                 (offsetFromipoJointPos[joint_to_move] >  0.2 && delta > 0) ||
                 (offsetFromipoJointPos[joint_to_move] < -0.2 && delta < 0)
               )
            {
               delta *=-1;
            }
        }
        
            KUKA::FRI::ESessionState sessionState = grl::robot::arm::get(friData.monitoringMsg,KUKA::FRI::ESessionState());
        // copy current joint position to commanded position
        if (sessionState == KUKA::FRI::COMMANDING_WAIT || sessionState == KUKA::FRI::COMMANDING_ACTIVE)
        {
            boost::transform ( ipoJointPos, offsetFromipoJointPos, jointStateToCommand.begin(), std::plus<double>());
            grl::robot::arm::set(friData.commandMsg, jointStateToCommand, grl::revolute_joint_angle_open_chain_command_tag());
        }
        
        // vector addition between ipoJointPosition and ipoJointPositionOffsets, copying the result into jointStateToCommand
        /// @todo should we take the current joint state into consideration?
        
		//BOOST_LOG_TRIVIAL(trace) << "position: " << state.position << " us: " << std::chrono::duration_cast<std::chrono::microseconds>(state.timestamp - startTime).count() << " connectionQuality: " << state.connectionQuality << " operationMode: " << state.operationMode << " sessionState: " << state.sessionState << " driveState: " << state.driveState << " ipoJointPosition: " << state.ipoJointPosition << " ipoJointPositionOffsets: " << state.ipoJointPositionOffsets << "\n";
	}
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
