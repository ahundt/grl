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



#include <mutex>
#include <iostream>
#include <memory>
#include <thread>


#include <azmq/socket.hpp>

/// Send messages between a client and server asynchronously. 
///
/// @see overview of zmq socket types https://sachabarbs.wordpress.com/2014/08/21/zeromq-2-the-socket-types-2/
/// @see bounce is based on https://github.com/zeromq/azmq/blob/master/test/socket/main.cpp
/// @see flatbuffers https://google.github.io/flatbuffers/md__cpp_usage.html
void bounce(std::shared_ptr<azmq::socket> sendP, std::shared_ptr<azmq::socket> receiveP, bool shouldReceive = true) {
            std::shared_ptr<std::atomic<int> > recv_countP(std::make_shared<std::atomic<int>>(0));
            std::shared_ptr<std::atomic<int> > send_countP(std::make_shared<std::atomic<int>>(0));
    
            constexpr int messagesToSend = 1000;
    int receiveAttempts = 0;
    
	for (int x = 0; (x<messagesToSend) || ((*recv_countP) < messagesToSend); ++x) {
		
		/////////////////////////////////////////
		// Client sends to server asynchronously!
		{
             sendP->async_send(boost::asio::buffer(&x, 4), [x,sendP,send_countP] (boost::system::error_code const& ec, size_t bytes_transferred) {
                    if(ec) std::cout << "SendFlatBuffer error! todo: figure out how to handle this\n";
                    else {
                      std::cout << "sent: " << x << "\n";
                      (*send_countP)++;
                    }

                });
		}
		
		//////////////////////////////////////////////
		// Server receives from client asynchronously!
		while
        (
                  shouldReceive && (*recv_countP < *send_countP)
               && (*recv_countP < messagesToSend)
               && (receiveAttempts < (*send_countP)-(*recv_countP))
        )
        {
			std::shared_ptr<int> recvBufP(std::make_shared<int>(0));
            BOOST_VERIFY(*recvBufP == 0);
            receiveP->async_receive(boost::asio::buffer(recvBufP.get(),sizeof(*recvBufP)), [recvBufP,receiveP,recv_countP, messagesToSend](boost::system::error_code const ec, size_t bytes_transferred) {
            
                if(ec) std::cout << "start_async_receive_buffers error! todo: figure out how to handle this\n";
                else std::cout << "received: " << *recvBufP << " recv_count:" << *recv_countP << "\n";
                // make rbp the size of the actual amount of data read
                (*recv_countP)++;
            });
			receiveAttempts++;
		}
        receiveAttempts = 0;
		
		//std::this_thread::sleep_for( std::chrono::milliseconds(1) );
    }
    sendP->get_io_service().stop();
    receiveP->get_io_service().stop();
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
    {
    boost::asio::io_service io_service;
    // Register signal handlers so that the daemon may be shut down when a signal is received.
//    boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);
//    signals.async_wait( std::bind(&boost::asio::io_service::stop, &io_service));

    std::shared_ptr<azmq::socket> sendsocket = std::make_shared<azmq::socket>(io_service, ZMQ_DEALER);
    std::shared_ptr<azmq::socket> recvsocket = std::make_shared<azmq::socket>(io_service, ZMQ_DEALER);
    recvsocket->bind("tcp://" + localhost + ":" + localport);
    sendsocket->connect("tcp://"+ remotehost + ":" + remoteport);

    
    
    // Will run until signal is received, using one object for both send and receive
    std::thread ios_t;
    
    {
        boost::asio::io_service::work work(io_service);
        ios_t = std::thread([&] {
            io_service.run();
        });
    
        bounce(sendsocket, recvsocket,shouldReceive);
        sendsocket.reset();
        recvsocket.reset();
    }
    
    io_service.stop();
    ios_t.join();
    }
	
//  }
//  catch (std::exception& e)
//  {
//    std::cerr << "Exception: " << e.what() << "\n";
//  }

  return 0;
}