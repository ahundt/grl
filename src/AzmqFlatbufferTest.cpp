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

#include "robone/AzmqFlatbuffer.hpp"

/// Send messages between a client and server asynchronously. 
///
/// @see overview of zmq socket types https://sachabarbs.wordpress.com/2014/08/21/zeromq-2-the-socket-types-2/
/// @see bounce is based on https://github.com/zeromq/azmq/blob/master/test/socket/main.cpp
/// @see flatbuffers https://google.github.io/flatbuffers/md__cpp_usage.html
void bounce(std::shared_ptr<AzmqFlatbuffer> sendP, std::shared_ptr<AzmqFlatbuffer> receiveP, bool shouldReceive = true) {
	
	receiveP->start_async_receive_buffers();
	
	for (int x = 0; x<1000; ++x) {
		
		/////////////////////////////////////////
		// Client sends to server asynchronously!
		{
			auto fbbP = sendP->GetUnusedBufferBuilder();
			
			robone::Vector3d rv(x,0,0);
			auto controlPoint = robone::CreateVrepControlPoint(*fbbP,&rv);
			robone::FinishVrepControlPointBuffer(*fbbP, controlPoint);
			sendP->async_send_flatbuffer(fbbP);
		}
		
		//////////////////////////////////////////////
		// Server receives from client asynchronously!
		while (shouldReceive && !receiveP->receive_buffers_empty()) {
			auto rbP = receiveP->get_back_receive_buffer_with_data();
			auto rbPstart = &(rbP->begin()[0]);
			auto verifier = flatbuffers::Verifier(rbPstart,rbP->size());
			auto bufOK = robone::VerifyVrepControlPointBuffer(verifier);
		
			if(bufOK){
				const robone::VrepControlPoint* VCPin = robone::GetVrepControlPoint(rbPstart);
				std::cout << "received: " << VCPin->position()->x() << "\n";
			} else {
				std::cout << "Failed verification. bufOk: " <<bufOK << "\n";
			}
			
		}
		
		std::this_thread::sleep_for( std::chrono::milliseconds(1) );
    }
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
    
    bool shouldReceive = false;
  
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
	
	
    // Register signal handlers so that the daemon may be shut down when a signal is received.
    boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);
    signals.async_wait( std::bind(&boost::asio::io_service::stop, &io_service));
#if 0
	std::shared_ptr<AzmqFlatbuffer> sendP;
	{
		boost::system::error_code ec;
		azmq::socket socket(io_service, ZMQ_DEALER);
		socket.connect("tcp://"+ remotehost + ":" + remoteport);
		sendP = std::make_shared<AzmqFlatbuffer>(std::move(socket));
	}
	std::shared_ptr<AzmqFlatbuffer> receiveP;
	{
		boost::system::error_code ec;
		azmq::socket socket(io_service, ZMQ_DEALER);
		if (shouldReceive) socket.bind("tcp://" + localhost + ":" + localport);
		receiveP = std::make_shared<AzmqFlatbuffer>(std::move(socket));
	}

	// Will run until signal is received, using separate objects for send and receive
    std::thread t(bounce,sendP,receiveP);
#else

	std::shared_ptr<AzmqFlatbuffer> receiveP;
	{
		boost::system::error_code ec;
		azmq::socket socket(io_service, ZMQ_DEALER);
		if(shouldReceive) socket.bind("tcp://" + localhost + ":" + localport);
		socket.connect("tcp://"+ remotehost + ":" + remoteport);
		receiveP = std::make_shared<AzmqFlatbuffer>(std::move(socket));
	}

	// Will run until signal is received, using one object for both send and receive
    std::thread t(bounce,receiveP,receiveP,shouldReceive);
#endif
	
	io_service.run();
	
//  }
//  catch (std::exception& e)
//  {
//    std::cerr << "Exception: " << e.what() << "\n";
//  }

  return 0;
}