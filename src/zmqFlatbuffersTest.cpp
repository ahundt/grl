
// Library includes
#include <string>
#include <ostream>
#include <iostream>
#include <memory>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/tokenizer.hpp>
#include <boost/math/special_functions/binomial.hpp>
#include <boost/filesystem.hpp>
#include <cmath>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "robone/flatbuffer/VrepControlPoint_generated.h"
#include "robone/flatbuffer/VrepPath_generated.h"
#include "robone/AzmqFlatbuffer.hpp"


namespace po = boost::program_options;


/// @todo what does this help accomplish?
struct monitor_handler {

#if defined BOOST_MSVC
#pragma pack(push, 1)
    struct event_t
    {
        uint16_t e;
        uint32_t i;
    };
#pragma pack(pop)
#else
    struct event_t
    {
        uint16_t e;
        uint32_t i;
    } __attribute__((packed));
#endif

    azmq::socket socket_;
    std::string role_;
    std::vector<event_t> events_;

    monitor_handler(boost::asio::io_service & ios, azmq::socket& s, std::string role)
        : socket_(s.monitor(ios, ZMQ_EVENT_ALL))
        , role_(std::move(role))
    { }

    void start()
    {
        socket_.async_receive([this](boost::system::error_code const& ec,
                                     azmq::message & msg, size_t) {
                if (ec)
                    return;
                event_t event;
                msg.buffer_copy(boost::asio::buffer(&event, sizeof(event)));
                events_.push_back(event);
                socket_.flush();
                start();
            });
    }

    void cancel()
    {
        socket_.cancel();
    }
};

/// Send messages between a client and server synchronously. 
///
/// @see overview of zmq socket types https://sachabarbs.wordpress.com/2014/08/21/zeromq-2-the-socket-types-2/
/// @see bounce is based on https://github.com/zeromq/azmq/blob/master/test/socket/main.cpp
/// @see flatbuffers https://google.github.io/flatbuffers/md__cpp_usage.html
void bounce(azmq::socket & server, azmq::socket & client) {
	
	std::array<uint8_t, 512> buf;
	for (int x = 0; x<100; ++x) {
		
		/////////////////////////
		// Client sends to server
		
		flatbuffers::FlatBufferBuilder fbb;
		robone::Vector3d rv(x,0,0);
	    auto controlPoint = robone::CreateVrepControlPoint(fbb,&rv);
		robone::FinishVrepControlPointBuffer(fbb, controlPoint);
        client.send(boost::asio::buffer(fbb.GetBufferPointer(), fbb.GetSize()));
		
		
		//////////////////////////////
		// Server receives from client
		
        auto size = server.receive(boost::asio::buffer(buf));
		auto verifier = flatbuffers::Verifier(buf.begin(),buf.size());
		auto bufOK = robone::VerifyVrepControlPointBuffer(verifier);
		
		if(size == fbb.GetSize() && bufOK){
		    const robone::VrepControlPoint* VCPin = robone::GetVrepControlPoint(buf.begin());
		    std::cout << "received: " << VCPin->position()->x() << "\n";
		} else {
			std::cout << "wrong size or failed verification. size: "<< size <<" bufOk: " <<bufOK << "\n";
		}
    }
}


void RunSocketMonitor() {
    boost::asio::io_service ios;
    boost::asio::io_service ios_m;

    using socket_ptr = std::unique_ptr<azmq::socket>;
    socket_ptr client(new azmq::socket(ios, ZMQ_DEALER));
    socket_ptr server(new azmq::socket(ios, ZMQ_DEALER));

    monitor_handler client_monitor(ios_m, *client, "client");
    monitor_handler server_monitor(ios_m, *server, "server");

    client_monitor.start();
    server_monitor.start();

    std::thread t([&] {
        ios_m.run();
    });

    server->bind("tcp://127.0.0.1:9998");
    client->connect("tcp://127.0.0.1:9998");

    bounce(*client, *server);

    // On Windows monitored sockets must be closed before their monitors,
    // otherwise ZMQ crashes or deadlocks during the context termination.
    // ZMQ's bug?
    client.reset();
    server.reset();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    ios_m.stop();
    t.join();

//    BOOST_REQUIRE(client_monitor.events_.size() == 3);
//    CHECK(client_monitor.events_[0].e == ZMQ_EVENT_CONNECT_DELAYED);
//    CHECK(client_monitor.events_[1].e == ZMQ_EVENT_CONNECTED);
//    CHECK(client_monitor.events_[2].e == ZMQ_EVENT_MONITOR_STOPPED);
//
//    REQUIRE(server_monitor.events_.size() == 4);
//    CHECK(server_monitor.events_[0].e == ZMQ_EVENT_LISTENING);
//    CHECK(server_monitor.events_[1].e == ZMQ_EVENT_ACCEPTED);
//    CHECK(server_monitor.events_[2].e == ZMQ_EVENT_CLOSED);
//    CHECK(server_monitor.events_[3].e == ZMQ_EVENT_MONITOR_STOPPED);
}


/**************************************************************************/
/**
 * @brief Main function
 *
 * @param argc  Number of input arguments
 * @param argv  Pointer to input arguments
 *
 * @return int
 */
int main(int argc,char**argv) {

	RunSocketMonitor();

	
    return 0;
}



