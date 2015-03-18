#ifndef _KUKA_FRI_THREAD_SEPARATOR_
#define _KUKA_FRI_THREAD_SEPARATOR_

#include <tuple>
#include <memory>
#include <thread>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include "robone/KukaFRI.hpp"

namespace robone {

/// @brief Allows a thread to get data from a kuka device asynchronously without capturing the thread.
///
/// Interface for threads that have special status so cannot be mixed with other threads.
/// Examples of this problem can be found in Qt and V-Rep plugins, for example.
/// all async calls are done in a separate user thread
/// you must call the async calls then call run_user()
/// to call the corresponding handlers
/// @todo Either rename this to be FRI specific, or maybe have one FRIKuka class, one JavaKuka class, and one new class that should have the KukaFRIThreadSeparator name combining the two.
/// @todo generalize this if possible to work for multiple devices beyond the kuka
//template<template <typename> Allocator = std::allocator>
/// @todo consider making this an asio io_service, see asio logger C++03 example https://github.com/boostorg/asio/tree/master/example/cpp03/services
class KukaFRIThreadSeparator : std::enable_shared_from_this<KukaFRIThreadSeparator> {
    
public:
    enum ParamIndex {
        localhost,  // 192.170.10.100
        localport,  // 30200
        remotehost, // 192.170.10.2
        remoteport  // 30200
    };
    
    /// @todo allow default params
    typedef std::tuple<std::string,std::string,std::string,std::string> Params;
    
    static const Params defaultParams(){
        return std::make_tuple(std::string("192.170.10.100"),std::string("30200"),std::string("192.170.10.2"),std::string("30200"));
    }
    
    /// @todo move this to params
    static const int default_circular_buffer_size = 3;
    
	KukaFRIThreadSeparator(boost::asio::io_service& ios, Params params = defaultParams())
        :
        io_service_(ios),
        strand_(ios)
    {
      construct(params);
	}
        
	KukaFRIThreadSeparator(Params params = defaultParams())
        :
        optional_internal_io_service_P(new boost::asio::io_service),
        io_service_(*optional_internal_io_service_P),
        strand_(*optional_internal_io_service_P)
    {
      construct(params);
	}
    
    
    void construct(Params params = defaultParams()){

        boost::asio::ip::udp::socket s(io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(std::get<localhost>(params)), boost::lexical_cast<short>(std::get<localport>(params))));

        boost::asio::ip::udp::resolver resolver(io_service_);
        boost::asio::ip::udp::endpoint endpoint = *resolver.resolve({boost::asio::ip::udp::v4(), std::get<remotehost>(params), std::get<remoteport>(params)});
    	s.connect(endpoint);
        
        iiwaP.reset(new robot::arm::kuka::iiwa(std::move(s)));
        
        
        for (int i = 0; i<default_circular_buffer_size; ++i) {
           monitorStates.push_back(std::make_shared<robot::arm::kuka::iiwa::MonitorState>());
   	       commandStates.push_back(std::make_shared<robot::arm::kuka::iiwa::CommandState>());
   	     }
    
        // start running the
        update_state();
        
        // start up the driver thread
        /// @todo perhaps allow user to control this?
        driver_threadP.reset(new std::thread([&]{ io_service_.run(); }));
    }
	
	~KukaFRIThreadSeparator(){
		workP.reset();
	}
	
	void sendControlPointToJava(){
		
	}
	
    
    /// pass a handler with the signature void f(std::shared_ptr<robot::arm::kuka::iiwa::MonitorState>)
    /// note that if no states are available, an empty sharedptr will be posted,
    /// so be sure to check if it is valid
    /// @todo instead of posting an empty shared ptr consider also posting the corresponding error code
    template<typename Handler>
	void async_getLatestState(Handler handler){
        auto self(shared_from_this());
        
		io_service_.post(strand_.wrap([this,self,handler](){
		    // now in io_service_ thread, can get latest state
            std::shared_ptr<robot::arm::kuka::iiwa::MonitorState> monitorP;
            if(!monitorStates.empty()){
                monitorP = monitorStates.front();
                monitorStates.pop_front();
            } else {
               // @todo allocation here, make sure the user knows something is wrong!
               monitorP = std::make_shared<robot::arm::kuka::iiwa::MonitorState>();
            }

            // don't need to wrap this one because it happens in the user thread
            user_io_service_.post([monitorP,handler](){
                // now in user_io_service_thread, can pass the final result to the user
                handler(monitorP);
            });
		}));
	}
    
    /// @todo maybe allow a handler so the user can get their commands back?
    /// @todo change this to accept a unique_ptr (may be easier with C++14
    void async_sendCommand(std::shared_ptr<robot::arm::kuka::iiwa::CommandState> commandStateP){
        auto self (shared_from_this());
        
		io_service_.post(strand_.wrap([this,self, commandStateP](){
		    // now in io_service_ thread, can set latest state
            commandStates.push_front(commandStateP);
		}));
        
    }
    
    
    /// If you received a MonitorState previously, you can return it to the pool
    /// this is only useful for performance reasons
    void async_addMonitorState(std::shared_ptr<robot::arm::kuka::iiwa::MonitorState> monitorP){
    
        auto self (shared_from_this());
        
		io_service_.post(strand_.wrap([this,self, monitorP](){
		    // now in io_service_ thread, can return monitorstate to the buffer
            monitorStates.push_back(monitorP);
		}));
    }
    
    
    /// @brief get a command state from the internal pool
    /// pass a handler with the signature void f(std::shared_ptr<robot::arm::kuka::iiwa::MonitorState>)
    /// @note IMPORTANT: WRITE ONLY! The command you get may contain no data or old data
    /// @todo this function may do an allocation, consider alternatives
    template<typename Handler>
    void async_MakeCommandState(Handler handler){
        auto self(shared_from_this());
        
		io_service_.post(strand_.wrap([this,self,handler](){
		    // now in io_service_ thread, can get latest state
            std::shared_ptr<robot::arm::kuka::iiwa::CommandState> commandStateP;
            // the most recent command should stay there
            if(monitorStates.size()>1){
                // old commands are in the back
                commandStateP = commandStates.back();
                commandStates.pop_front();
            } else {
               // @todo allocation here, make sure the user knows something is wrong!
               commandStateP = std::make_shared<robot::arm::kuka::iiwa::CommandState>();
            }

            // don't need to wrap this one because it happens in the user thread
            user_io_service_.post([commandStateP,handler](){
                // now in user_io_service_thread, can pass the final result to the user
                handler(commandStateP);
            });
		}));
      
    }
    
  /// The io_service::run() call will block until all asynchronous operations
  /// have finished.
    /// @todo not implemented
    void run(){
    }
    
    /// run_user will only block briefly as all the requested callbacks that have been completed are made
    void run_user(){
        user_io_service_.run();
    }

private:

     /// @todo how to handle command state? If the user doesn't provide one what do we do? Copy the monitor state? This may be handled already in KukaFRI
    void update_state(){
        if(!io_service_.stopped()) {
            
            BOOST_VERIFY(!monitorStates.empty()); // we should always keep at least one state available
            BOOST_VERIFY(!commandStates.empty()); // we should always keep at least one state available
            std::shared_ptr<robot::arm::kuka::iiwa::MonitorState> ms = monitorStates.front();
            monitorStates.pop_front();
            std::shared_ptr<robot::arm::kuka::iiwa::CommandState> cs = commandStates.front();
            commandStates.pop_front();
            
            auto self(shared_from_this());
            
            iiwaP->async_update(*ms,*cs,//strand_.wrap( /// @todo FIXME
                [this,self,ms,cs](boost::system::error_code ec, std::size_t bytes_transferred){
                    // new data available

                    // put the latest state on the front and get the oldest from the back
                    monitorStates.push_front(ms);
                    // this command state is old, put it in back of the line
                    commandStates.push_back(cs);
                     
                    // run this function again *after* it returns
                    io_service_.post(std::bind(&KukaFRIThreadSeparator::update_state,this));
                }
             //) // wrap call ends here, needs to be fixed
             ); // end async_update call
        }
    }
    
    /// may be null, allows the user to choose if they want to provide an io_service
    std::unique_ptr<boost::asio::io_service> optional_internal_io_service_P;
    
	// other things to do somewhere:
	// - get list of control points
	// - get the control point in the arm base coordinate system
	// - load up a configuration file with ip address to send to, etc.
	boost::asio::io_service& io_service_;
    boost::asio::io_service user_io_service_;
    boost::asio::io_service::strand strand_;
    std::unique_ptr<robot::arm::kuka::iiwa> iiwaP;
    std::unique_ptr<boost::asio::io_service::work> workP;
    std::unique_ptr<std::thread> driver_threadP;
    
    /// @todo have separate sets for full/empty states?
    /// @todo replace with unique_ptr
    
    /// the front is the most recent state, the back is the oldest
    boost::circular_buffer<std::shared_ptr<robot::arm::kuka::iiwa::MonitorState>> monitorStates;
    /// the front is the most recent state, the back is the oldest
    boost::circular_buffer<std::shared_ptr<robot::arm::kuka::iiwa::CommandState>> commandStates;
    
};

} // namespace robone

#endif
