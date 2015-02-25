
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


int x = 0;
namespace po = boost::program_options;

/// @todo this is not ready for use!
void receiveVCP(boost::system::error_code ec,azmq::sub_socket&& subscriber,azmq::pub_socket&& publisher, flatbuffers::FlatBufferBuilder&& fbb){
	
        std::array<char, 256> buf;
        auto size = subscriber.receive(boost::asio::buffer(buf));
		auto VrepControlPointIn = robone::GetVrepControlPoint(buf.begin());
		const robone::VrepControlPoint* VCPin = robone::GetVrepControlPoint(buf.begin());
		
		std::cout << "received: " << VCPin->position()->x() << "\n";
		
		x++;
		auto rv = robone::Vector3d(x,0,0);
        publisher.send(boost::asio::buffer(fbb.GetBufferPointer(), fbb.GetSize()));
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

    boost::asio::io_service ios;
    azmq::sub_socket subscriber(ios);
    subscriber.connect("tcp://127.0.0.1:5556");
    //subscriber.connect("tcp://192.168.55.201:7721");
    subscriber.set_option(azmq::socket::subscribe("CUT_PATH"));

    azmq::pub_socket publisher(ios);
    publisher.bind("tcp://127.0.0.1:5556");
	
	flatbuffers::FlatBufferBuilder fbb;
	
	robone::Vector3d rv(0,0,0);
    std::array<char, 256> buf;

	//std::bind(receiveVCP(std::placeholders::_1,subscriber, publisher,fbb));

    for (;;) {
	
	
		x++;
		rv = robone::Vector3d(x,0,0);
	    auto controlPoint = robone::CreateVrepControlPoint(fbb,&rv);
        publisher.send(boost::asio::buffer(fbb.GetBufferPointer(), fbb.GetSize()));
	
        auto size = subscriber.receive(boost::asio::buffer(buf));
		auto VrepControlPointIn = robone::GetVrepControlPoint(buf.begin());
		const robone::VrepControlPoint* VCPin = robone::GetVrepControlPoint(buf.begin());
		
		std::cout << "received: " << VCPin->position()->x() << "\n";
		
    }

	
    return 0;
}



