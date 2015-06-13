//
// echo_server.cpp
// ~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2014 Christopher M. Kohlhoff (chris at kohlhoff dot com)
// Copyright (c) 2015 Andrew Hundt
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

//#include "grl/AzmqFlatbuffer.hpp"
//#include "grl/flatbuffer/Geometry_generated.h"
//#include "grl/flatbuffer/VrepControlPoint_generated.h"
//#include "grl/flatbuffer/VrepPath_generated.h"


#include "flatbuffers/flatbuffers.h"


#include <mutex>
#include <iostream>
#include <memory>
#include <thread>

#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/container/static_vector.hpp>


#include <azmq/socket.hpp>

/// Send messages between a client and server asynchronously. 
///
/// @see overview of zmq socket types https://sachabarbs.wordpress.com/2014/08/21/zeromq-2-the-socket-types-2/
/// @see bounce is based on https://github.com/zeromq/azmq/blob/master/test/socket/main.cpp
/// @see flatbuffers https://google.github.io/flatbuffers/md__cpp_usage.html
void bounce(std::shared_ptr<azmq::socket> sendP, std::shared_ptr<azmq::socket> receiveP, bool shouldReceive = true) {
            std::atomic<int> recv_count(0);
            std::atomic<int> send_count(0);
    
            constexpr int messagesToSend = 1000;
    
	for (int x = 0; x<messagesToSend; ++x) {
		
		/////////////////////////////////////////
		// Client sends to server asynchronously!
		{
             sendP->async_send(boost::asio::buffer(&x, 4), [x,&sendP,&send_count] (boost::system::error_code const& ec, size_t bytes_transferred) {
                    if(ec) std::cout << "SendFlatBuffer error! todo: figure out how to handle this\n";
                    else {
                      std::cout << "sent: " << x << "\n";
                      send_count++;
                    }

                });
		}
		
		//////////////////////////////////////////////
		// Server receives from client asynchronously!
		while (shouldReceive && recv_count < send_count ) {
			std::shared_ptr<int> recvBufP(std::make_shared<int>(0));
            BOOST_VERIFY(*recvBufP == 0);
            receiveP->async_receive(boost::asio::buffer(recvBufP.get(),sizeof(*recvBufP)), [recvBufP,receiveP,&recv_count, messagesToSend](boost::system::error_code const ec, size_t bytes_transferred) {
            
                if(ec) std::cout << "start_async_receive_buffers error! todo: figure out how to handle this\n";
                else std::cout << "received: " << *recvBufP << " recv_count:" << recv_count << "\n";
                // make rbp the size of the actual amount of data read
                recv_count++;
            });
			
		}
		
		//std::this_thread::sleep_for( std::chrono::milliseconds(1) );
    }
    sendP.reset();
    receiveP.reset();
}


int main(int argc, char* argv[])
{
//  try
//  {
  
    std::string localhost("127.0.0.1");
    std::string localport("9998");
    std::string remotehost("127.0.0.1");
    std::string remoteport("9998");
  
    std::cout << "argc: " << argc << "\n";
	  /// @todo add default localhost/localport
    if (argc != 5 && argc != 1 && argc != 3)
    {
      std::cerr << "Usage: " << argv[0] << " <localip> <localport> <remoteip> <remoteport>\n";
      return 1;
    }
    
    bool shouldReceive = true;
  
    if(argc == 3){
      remotehost = std::string(argv[1]);
      remoteport = std::string(argv[2]);
      shouldReceive = false;
    }
    
    if(argc == 5){
      localhost = std::string(argv[1]);
      localport = std::string(argv[2]);
      remotehost = std::string(argv[3]);
      remoteport = std::string(argv[4]);
      shouldReceive = true;
    }
    
    std::cout << "using: "  << argv[0] << " ";
    if(shouldReceive) std::cout <<  localhost << " " << localport << " ";
    std::cout <<  remotehost << " " << remoteport << "\n";
    boost::asio::io_service io_service;
    std::thread thr;
	
    // Register signal handlers so that the daemon may be shut down when a signal is received.
    boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);
    signals.async_wait( std::bind(&boost::asio::io_service::stop, &io_service));

    std::shared_ptr<azmq::socket> socket = std::make_shared<azmq::socket>(io_service, ZMQ_DEALER);
    socket->bind("tcp://" + localhost + ":" + localport);
    socket->connect("tcp://"+ remotehost + ":" + remoteport);

    // Will run until signal is received, using one object for both send and receive
    thr = std::thread(bounce,socket,socket,shouldReceive);
	
	io_service.run();
    
    thr.join();
	
//  }
//  catch (std::exception& e)
//  {
//    std::cerr << "Exception: " << e.what() << "\n";
//  }

  return 0;
}