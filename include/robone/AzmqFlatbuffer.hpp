#ifndef _AZMQ_FLATBUFFER_
#define _AZMQ_FLATBUFFER_



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



/// @brief sends and receives flatbuffer data via AZMQ implementation of ZeroMQ plus manages the relevant buffers
///
/// Sending: This class provides a mechanism to asynchronously send google flatbuffers.
/// It also stores a pool of these buffers so that they don't need to be reallocated.
///
/// Rationale: FlatBufferBuilders are much faster if they are reused
/// so we create a small pool of them that you can get from this object
/// after you send a flat buffer, the builder is put into the pool
///
/// @todo the buffer pools may be a premature optimization. Evaluate this.
class AzmqFlatbuffer : public std::enable_shared_from_this<AzmqFlatbuffer>
{
public:

static const int default_circular_buffer_size = 10;
/// @todo Consider making this a simple std::vector so it is runtime configurable
typedef std::shared_ptr<boost::container::static_vector<uint8_t,256>> receive_buffer_type;

  /// Initialize AzmqFlatbuffer with a socket.
  /// The socket should be fully configured 
  /// and ready to use when it is passed to this object.
  /// We also recommend the user utilizes AzmqFlatbuffer(std::move(socket)) 
  /// when calling this constructor.
  /// @see AzmqFlatbufferTest for an example of usage.
  ///
  /// @todo consider making default_circular_buffer_size and receive_buffer_type configurable
  explicit AzmqFlatbuffer(azmq::socket socket)
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

  /// Send a FlatbufferBuilder to the destination specified in the socket.
  /// @todo make it so FlatBufferBuilders can be used directly without shared_ptr overhead.
  ///
  /// @pre there are no other instances of fbbP (reference count of the shared_ptr should be 1)
  ///
  /// @note the FlatBufferBuilder is automatically put back into the pool internal to this class after sending
  void async_send_flatbuffer(std::shared_ptr<flatbuffers::FlatBufferBuilder> fbbP)
  {
    auto self(shared_from_this());
	
	
	 socket_.async_send(boost::asio::buffer(fbbP->GetBufferPointer(), fbbP->GetSize()), [this,self,fbbP] (boost::system::error_code const& ec, size_t bytes_transferred) {
			if(ec) std::cout << "SendFlatBuffer error! todo: figure out how to handle this\n";
			std::lock_guard<std::mutex> lock(this->unusedFlatBufferBuildersLock_);
			fbbP->Clear();
			this->unusedFlatBufferBuilders_.push_back(fbbP);
        });
		
  }

  /// get a google FlatBufferBuilder object from the pool of unused objects
	std::shared_ptr<flatbuffers::FlatBufferBuilder> GetUnusedBufferBuilder(){
			std::lock_guard<std::mutex> lock(this->unusedFlatBufferBuildersLock_);
			std::shared_ptr<flatbuffers::FlatBufferBuilder> back;
			if (!unusedFlatBufferBuilders_.empty()) {
				back = unusedFlatBufferBuilders_.back();
				unusedFlatBufferBuilders_.pop_back();
			} else {
			/// @todo eliminate the need for this extra allocation, though this may be premature optimization
				back = std::make_shared<flatbuffers::FlatBufferBuilder>();
			}
		
			return back;
	}
	
	/// Initializes the process of receiving 
	/// buffers from the source specified
	/// by the azmq::socket taht was provided.
	/// This initializes a loop that will continuously
	/// read data and fill out the internal ring buffer
	/// for users to extract. This allows this class to
	/// run asynchronously while interacting with
	/// synchronous users.
	void start_async_receive_buffers(){
		if(doneReceiving_){
			doneReceiving_=false;
			next_async_receive_buffers();
		}
	}
	
private:
	/// read the next set of data from the zeromq interface in a loop.
	/// This relies on the io_service for the loop, so that the stack
	/// doesn't get used up.
	///
	/// @todo When the receiveBuffersWithData buffer is full, consider moving the oldest buffer to the "unused" buffer to save allocations.
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
			if(!doneReceiving_) socket_.get_io_service().post(std::bind(&AzmqFlatbuffer::next_async_receive_buffers,this));
		});
	}
public:
	
	/// Stop receiving buffers
	void stop_async_receive_buffers(){
		doneReceiving_ = true;
	}
	
	
	/// @return true if there are no buffers that have been received via the socket, false otherwise
	bool receive_buffers_empty(){
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
		return receiveBuffersWithData_.empty();
	}
	
	/// @return The number of incoming buffers stored
	std::size_t receive_buffers_size(){
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
		return receiveBuffersWithData_.size();
	}
	
	/// @return The number of possible incoming buffers that can be stored
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
	
	/// Put an unused buffer, or one that is no longer needed, back into the pool
	void push_back_unused_receive_buffer(receive_buffer_type rb){
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
		unusedReceiveBuffers_.push_back(rb);
	}
	
	/// get the last, aka most chronologically recent, incoming buffer from the pool
	receive_buffer_type get_back_receive_buffer_with_data(){
	    receive_buffer_type rbP;
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
			if (!receiveBuffersWithData_.empty()) {
				rbP = receiveBuffersWithData_.back();
				receiveBuffersWithData_.pop_back();
			}
		return rbP;
	}
	
	
	/// get the first, aka the chronologically oldest, incoming buffer from the pool
	receive_buffer_type get_front_receive_buffer_with_data(){
	    receive_buffer_type rbP;
		std::lock_guard<std::mutex> lock(receiveBuffersLock_);
			if (!receiveBuffersWithData_.empty()) {
				rbP = unusedReceiveBuffers_.front();
				receiveBuffersWithData_.pop_front();
			}
		return rbP;
	}
	
	
	/// @brief Get all buffers from the pool at once and put them in an OutputIterator. This reduces locking/unlocking substantially.
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


#endif