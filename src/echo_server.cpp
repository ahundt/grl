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

#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/container/static_vector.hpp>
#include <iostream>
#include <memory>
#include <thread>


#include <azmq/socket.hpp>
#include <flatbuffers/flatbuffers.h>


#include "robone/flatbuffer/Geometry_generated.h"
#include "robone/flatbuffer/VrepControlPoint_generated.h"
#include "robone/flatbuffer/VrepPath_generated.h"
#include "robone/AzmqFlatbuffer.hpp"



#include <boost/asio/yield.hpp>
/// @brief sends and receives flatbuffer data via AZMQ implementation of ZeroMQ plus manages the relevant buffers
///
/// Sending: This class provides a mechanism to asynchronously send google flatbuffers.
/// It also stores a pool of these buffers so that they don't need to be reallocated.
///
/// Rationale: FlatBufferBuilders are much faster if they are reused
/// so we create a small pool of them that you can get from this object
/// after you send a flat buffer, the builder is put into the pool
class ZMQFlatbufferSession : public std::enable_shared_from_this<ZMQFlatbufferSession>
{
public:

static const int default_circular_buffer_size = 10;
/// @todo Consider making this a simple std::vector so it is runtime configurable
typedef std::shared_ptr<boost::container::static_vector<uint8_t,256>> receive_buffer_type;


  explicit ZMQFlatbufferSession(azmq::socket socket)
    : socket_(std::move(socket)),
      strand_(socket_.get_io_service()),
	  unusedFlatBufferBuilders_(default_circular_buffer_size),
	  unusedReceiveBuffers_(default_circular_buffer_size),
	  receiveBuffersWithData_(default_circular_buffer_size),
	  doneReceiving_(true)
  {
     for (int i = 0; i<default_circular_buffer_size; ++i) {
        unusedFlatBufferBuilders_.push_back(std::make_shared<flatbuffers::FlatBufferBuilder>());
		unusedReceiveBuffers_.push_back(std::make_shared<receive_buffer_type::element_type>());
	 }
  }

/// @todo make it so FlatBufferBuilders can be used directly
  void async_send_flatbuffer(std::shared_ptr<flatbuffers::FlatBufferBuilder> fbbP)
  {
    auto self(shared_from_this());
	
	
	 socket_.async_send(boost::asio::buffer(fbbP->GetBufferPointer(), fbbP->GetSize()), [this,self,fbbP] (boost::system::error_code const& ec, size_t bytes_transferred) {
			if(ec) std::cout << "SendFlatBuffer error! todo: figure out how to handle this\n";
			std::lock_guard<std::mutex> lock(this->unusedFlatBufferBuildersLock_);
			fbbP->Clear();
			this->unusedFlatBufferBuilders_.push_back(fbbP);
        });
		
	/// commented until this is supported
//    boost::asio::spawn(strand_,
//        [this, self, fbbP](boost::asio::yield_context yield)
//        {
//          try
//          {
//		      boost::system::error_code ec;
//			  // todo: check if error code no_buffer_space
//              socket_.async_send(boost::asio::buffer(fbbP->GetBufferPointer(), fbbP->GetSize()), yield[ec]);
//          }
//          catch (std::exception& e)
//          {
//            //socket_.close();
//            //timer_.cancel();
//          }
//        });

  }

	std::shared_ptr<flatbuffers::FlatBufferBuilder> GetUnusedBufferBuilder(){
			std::lock_guard<std::mutex> lock(this->unusedFlatBufferBuildersLock_);
			std::shared_ptr<flatbuffers::FlatBufferBuilder> back;
			if (!unusedFlatBufferBuilders_.empty()) {
				back = unusedFlatBufferBuilders_.back();
				unusedFlatBufferBuilders_.pop_back();
			} else {
			/// @todo eliminate the need for this extra allocation
				back = std::make_shared<flatbuffers::FlatBufferBuilder>();
			}
		
			return back;
	}
	
	/// @todo this only works once... make it keep receiving repeatedly
	void start_async_receive_buffers(){
		if(doneReceiving_){
			doneReceiving_=false;
			next_async_receive_buffers();
		}
	}
	
private:
	void next_async_receive_buffers(){
	    receive_buffer_type rbP;
		{ // this bracket is important so the lock is released ASAP
			std::lock_guard<std::mutex> lock(receiveBuffersLock_);
			if (!unusedReceiveBuffers_.empty()) {
				rbP = unusedReceiveBuffers_.back();
				unusedReceiveBuffers_.pop_back();
			} else {
			/// @todo eliminate the need for this extra allocation
				rbP = std::make_shared<receive_buffer_type::element_type>();
			}
			
		}
		// use the full capacity of the buffer
		rbP->resize(rbP->capacity());
		
		auto self(shared_from_this());
		
		socket_.async_receive(boost::asio::buffer(&(rbP->begin()[0]),rbP->size()), [this,self,rbP](boost::system::error_code const& ec, size_t bytes_transferred) {
			if(ec) std::cout << "start_async_receive_buffers error! todo: figure out how to handle this\n";
			// make rbp the size of the actual amount of data read
			rbP->resize(std::min(bytes_transferred,rbP->capacity()));
			std::lock_guard<std::mutex> lock(receiveBuffersLock_);
			self->receiveBuffersWithData_.push_back(rbP);
			
			// run this function again *after* it returns
			if(!doneReceiving_) socket_.get_io_service().post(std::bind(&ZMQFlatbufferSession::next_async_receive_buffers,this));
		});
	}
public:
	void stop_async_receive_buffers(){
		doneReceiving_ = true;
	}
	
#if 0
	//void start_async_receive_buffers(){}
	void start_async_receive_buffers(){
	
	    receive_buffer_type rbP;
		// use the full capacity of the buffer
		
			
		{ // this bracket is important so the lock is released ASAP
			std::lock_guard<std::mutex> lock(receiveBuffersLock_);
			if (!unusedFlatBufferBuilders_.empty()) {
				rbP = unusedReceiveBuffers_.back();
				unusedReceiveBuffers_.pop_back();
			}
			
		}
		rbP->resize(rbP->capacity());
		
		auto self(shared_from_this());
		
		socket_.async_receive(boost::asio::buffer(&(rbP->begin()[0]),rbP->size()), std::bind(&operator(),this, boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred,self, rbP));
	}
private:

void operator()(boost::system::error_code ec, std::size_t bytes_transferred, std::shared_ptr<ZMQFlatbufferSession> self, receive_buffer_type rbP){

			if(ec) std::cout << "start_async_receive_buffers error! todo: figure out how to handle this\n";
			// make rbp the size of the actual amount of data read
			rbP->resize(std::min(bytes_transferred,rbP->capacity()));
			std::lock_guard<std::mutex> lock(receiveBuffersLock_);
			receiveBuffersWithData_.push_back(rbP);
			if(!doneReceiving_) start_async_receive_buffers();
}
#endif

//  void operator()(boost::system::error_code ec = boost::system::error_code(), std::size_t n = 0){
//  
//		
//	auto self(shared_from_this());
//	receive_buffer_type rbP;
//	
//    if (!ec) reenter (this)
//    {
//      for (;;)
//      {
//		{ // this bracket is important so the lock is released ASAP
//			std::lock_guard<std::mutex> lock(receiveBuffersLock_);
//			if (!unusedFlatBufferBuilders_.empty()) {
//				rbP = unusedReceiveBuffers_.back();
//				unusedReceiveBuffers_.pop_back();
//			}
//			
//		}
//		// use the full capacity of the buffer
//		rbP->resize(rbP->capacity());
//		
//		yield socket_.async_receive(boost::asio::buffer(&(rbP->begin()[0]),rbP->size()),*this);
//		
//		if(ec) std::cout << "operator() and start_async_receive_buffers error! todo: figure out how to handle this\n";
//		// make rbp the size of the actual amount of data read
//		rbP->resize(std::min(bytes_transferred,rbP->capacity()));
//		{
//			std::lock_guard<std::mutex> lock(receiveBuffersLock_);
//			receiveBuffersWithData_.push_back(std::move(rbP));
//		}
//
//	  }
//	}
//  }
	
public:
	bool receive_buffers_empty(){
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
		return receiveBuffersWithData_.empty();
	}
	
	std::size_t receive_buffers_size(){
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
		return receiveBuffersWithData_.size();
	}
	
	std::size_t receive_buffers_capacity(){
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
		return receiveBuffersWithData_.capacity();
	}

	/// Inserts a range of values into the available receive buffers
	///
	///  @pre p must be a valid iterator of *this in range [begin(), end()].
	///  @pre distance(first, last) <= capacity()
	///  @pre Iterator must meet the ForwardTraversalIterator concept.
	template<typename T>
	void insert_unused_receive_buffers(T& range){
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
		unusedReceiveBuffers_.insert(unusedFlatBufferBuilders_.end(), range.begin(), range.end());
	}
	
	void push_back_unused_receive_buffer(receive_buffer_type rb){
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
		unusedReceiveBuffers_.push_back(rb);
	}
	
	receive_buffer_type get_back_receive_buffer_with_data(){
	    receive_buffer_type rbP;
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
			if (!receiveBuffersWithData_.empty()) {
				rbP = receiveBuffersWithData_.back();
				receiveBuffersWithData_.pop_back();
			}
		return rbP;
	}
	
	
	receive_buffer_type get_front_receive_buffer_with_data(){
	    receive_buffer_type rbP;
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
			if (!receiveBuffersWithData_.empty()) {
				rbP = unusedReceiveBuffers_.front();
				receiveBuffersWithData_.pop_front();
			}
		return rbP;
	}
	
	
	/// @brief Copies all receive buffers that have received messages into the output iterator and clears the internal container
	template<typename OutputIterator>
	void get_all_receive_buffers_with_data(OutputIterator it){
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
		std::copy(receiveBuffersWithData_.begin(), receiveBuffersWithData_.end(), it);
		receiveBuffersWithData_.clear();
	}

private:
/// @todo may need to create io_service::work object here to keep io_service from exiting run() call
  azmq::socket socket_;
  boost::asio::io_service::strand strand_;
/// @todo it is a bit unsafe to use shared pointers, but I'm not sure if it is possible to std::move FlatBufferBuilders themselves, or unique ptrs into lambda functions
  boost::circular_buffer<std::shared_ptr<flatbuffers::FlatBufferBuilder>> unusedFlatBufferBuilders_;
  std::mutex unusedFlatBufferBuildersLock_;


  boost::circular_buffer<receive_buffer_type> unusedReceiveBuffers_;
  boost::circular_buffer<receive_buffer_type> receiveBuffersWithData_;
  std::mutex receiveBuffersLock_;
  std::atomic<bool> doneReceiving_;
};

#include <boost/asio/unyield.hpp>

/// Send messages between a client and server synchronously. 
///
/// @see overview of zmq socket types https://sachabarbs.wordpress.com/2014/08/21/zeromq-2-the-socket-types-2/
/// @see bounce is based on https://github.com/zeromq/azmq/blob/master/test/socket/main.cpp
/// @see flatbuffers https://google.github.io/flatbuffers/md__cpp_usage.html
void bounce(std::shared_ptr<ZMQFlatbufferSession> sendP, std::shared_ptr<ZMQFlatbufferSession> receiveP) {
	
	receiveP->start_async_receive_buffers();
	
	for (int x = 0; x<100; ++x) {
		
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
		while (!receiveP->receive_buffers_empty()) {
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
  try
  {
    boost::asio::io_service io_service;
	
	
    // Register signal handlers so that the daemon may be shut down when a signal is received.
    boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);
    signals.async_wait( std::bind(&boost::asio::io_service::stop, &io_service));
#if 0
	std::shared_ptr<ZMQFlatbufferSession> sendP;
	{
		boost::system::error_code ec;
		azmq::socket socket(io_service, ZMQ_DEALER);
		socket.connect("tcp://127.0.0.1:9998");
		sendP = std::make_shared<ZMQFlatbufferSession>(std::move(socket));
	}
	std::shared_ptr<ZMQFlatbufferSession> receiveP;
	{
		boost::system::error_code ec;
		azmq::socket socket(io_service, ZMQ_DEALER);
		socket.bind("tcp://127.0.0.1:9998");
		receiveP = std::make_shared<ZMQFlatbufferSession>(std::move(socket));
	}

	// Will run until signal is received, using separate objects for send and receive
    std::thread t(bounce,sendP,receiveP);
#else

	std::shared_ptr<ZMQFlatbufferSession> receiveP;
	{
		boost::system::error_code ec;
		azmq::socket socket(io_service, ZMQ_DEALER);
		socket.bind("tcp://127.0.0.1:9998");
		socket.connect("tcp://127.0.0.1:9998");
		receiveP = std::make_shared<ZMQFlatbufferSession>(std::move(socket));
	}

	// Will run until signal is received, using one object for both send and receive
    std::thread t(bounce,receiveP,receiveP);
#endif
	
	io_service.run();
	
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}