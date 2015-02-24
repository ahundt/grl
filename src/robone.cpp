
// Library includes
#include <string>
#include <ostream>
#include <iostream>
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

#include "robone/flatbuffer/vrepPath_generated.h"
#include "robone/AzmqFlatbuffer.hpp"


namespace po = boost::program_options;



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
    subscriber.connect("tcp://192.168.55.112:5556");
    subscriber.connect("tcp://192.168.55.201:7721");
    subscriber.set_option(azmq::socket::subscribe("NASDAQ"));

    azmq::pub_socket publisher(ios);
    publisher.bind("ipc://nasdaq-feed");
	
	flatbuffers::FlatBufferBuilder fbb;
	
	auto controlPoint = robone::CreateVrepControlPoint(fbb);

    for (;;) {
        auto size = subscriber.receive(boost::asio::buffer(fbb.GetBufferPointer(), fbb.GetSize()));
        publisher.send(boost::asio::buffer(fbb.GetBufferPointer(), fbb.GetSize()));
    }
    return 0;
}



