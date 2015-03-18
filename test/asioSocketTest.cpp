
// Library includes
#include <string>
#include <ostream>
#include <iostream>
#include <memory>
#include <vector>
#include <iostream>

// Boost includes
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>


enum { max_length = 1024 };

struct dummy{};

struct SocketTest : std::enable_shared_from_this<SocketTest> {


  SocketTest(boost::asio::ip::udp::socket socket):socket_(std::move(socket)),strand_(socket_.get_io_service()){}

  template<typename Handler>
  void async_receive(boost::system::error_code ec, std::size_t bytes_transferred, dummy dummy1,Handler handler, dummy dummy2){
    auto self(shared_from_this());
  
    socket_.async_receive(boost::asio::buffer(buf_), strand_.wrap([this,self,handler](boost::system::error_code ec, std::size_t bytes_transferred){
       handler(ec,bytes_transferred);
        
    }));
  }

  std::array<char,max_length> buf_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::io_service::strand strand_;
};


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
	
    SocketTest socket_test(std::move(s));

    socket_test.async_receive(boost::system::error_code(),0,dummy(), []( boost::system::error_code ec, std::size_t bytes_transferred){
        // many cool things accomplished!
    },dummy());
  
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}