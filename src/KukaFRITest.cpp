
// Library includes
#include <string>
#include <ostream>
#include <iostream>
#include <memory>

#include "robone/KukaFRI.hpp"
#include "robone/KukaFriClientData.hpp"

//
// blocking_udp_echo_client.cpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2014 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <vector>
using std::vector;
#include <iostream>
using std::ostream;

template<typename T,size_t N>
ostream& operator<< (ostream& out, const boost::container::static_vector<T,N>& v) {
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
  try
  {
    std::string host("");
    std::string port("30200");
  
	  /// @todo add default host/port
    if (argc !=3 || argc !=0)
    {
      std::cerr << "Usage: " << argv[0] << " <host> <port>\n";
      return 1;
    }
  
    if(argc ==3){
      host = std::string(argv[1]);
      port = std::string(argv[2]);
    }
    
      std::cout << "using: "  << argv[0] << " " <<  host << " " << port;

    boost::asio::io_service io_service;

    boost::asio::ip::udp::socket s(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0));

    boost::asio::ip::udp::resolver resolver(io_service);
    boost::asio::ip::udp::endpoint endpoint = *resolver.resolve({boost::asio::ip::udp::v4(), host, port});
	s.connect(endpoint);
	
	KUKA::FRI::ClientData friData(7);
	robone::robot::arm::KukaState state;

	std::chrono::time_point<std::chrono::high_resolution_clock> startTime;

	for (std::size_t i = 0;;++i) {
		update_state(s,friData,state);
		if (i==0) {
			startTime = state.timestamp;
		}
		std::cout << "position: " << state.position << " us: " << std::chrono::duration_cast<std::chrono::microseconds>(state.timestamp - startTime).count() << "\n";
		
	}
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}