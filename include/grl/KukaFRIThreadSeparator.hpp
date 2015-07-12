#ifndef _KUKA_FRI_THREAD_SEPARATOR_
#define _KUKA_FRI_THREAD_SEPARATOR_

#include <tuple>
#include <memory>
#include <thread>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/exception/all.hpp>

#include "grl/KukaFRI.hpp"
#include "grl/exception.hpp"

namespace grl {

class KukaFRI {
    
public:

    enum ParamIndex {
        localhost,  // 192.170.10.100
        localport,  // 30200
        remotehost, // 192.170.10.2
        remoteport,  // 30200
        is_running_automatically // true by default, this means that an internal thread will be created to run the driver.
    };
    
    enum ThreadingRunMode {
      run_manually = 0,
      run_automatically = 1
    };
    
    typedef std::tuple<std::string,std::string,std::string,std::string,ThreadingRunMode> Params;
    
    
    static const Params defaultParams(){
        return std::make_tuple(std::string("192.170.10.100"),std::string("30200"),std::string("192.170.10.2"),std::string("30200"),run_automatically);
    }
    
    
    /// Advanced functionality, do not use without a great reason
    static boost::asio::ip::udp::socket connect(Params& params, boost::asio::io_service& io_service_){
            boost::asio::ip::udp::socket s(io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(std::get<localhost>(params)), boost::lexical_cast<short>(std::get<localport>(params))));

            boost::asio::ip::udp::resolver resolver(io_service_);
            boost::asio::ip::udp::endpoint endpoint = *resolver.resolve({boost::asio::ip::udp::v4(), std::get<remotehost>(params), std::get<remoteport>(params)});
            s.connect(endpoint);
        
            return std::move(s);
    }
    
    static void add_details_to_connection_error(boost::exception& e, Params& params){
                e << errmsg_info("KukaFRIThreadSeparator: Unable to connect to Kuka FRI Koni UDP device using boost::asio::udp::socket configured with localhost:localport @ " +
                                   std::get<localhost>(params) + ":" + std::get<localport>(params) + " and remotehost:remoteport @ " +
                                   std::get<remotehost>(params) + ":" + std::get<remoteport>(params) + "\n");
    }

};

/// @brief Allows a thread to get data from a kuka device asynchronously without capturing the thread.
///
/// Interface for threads that have special status so cannot be mixed with other threads.
/// Examples of this problem can be found in Qt and V-Rep plugins, for example.
/// all async calls are done in a separate user thread
/// you must call the async calls then call run_user()
/// to call the corresponding handlers
/// @todo generalize this if possible to work for multiple devices beyond the kuka
//template<template <typename> Allocator = std::allocator>
/// @todo consider making this an asio io_service, see asio logger C++03 example https://github.com/boostorg/asio/tree/master/example/cpp03/services
///
/// One important aspect of this design is the is_running_automatically flag. If you are unsure,
/// the suggested default is run_automatically (true/enabled). When it is enabled,
/// the driver will create a thread internally and run the event loop (io_service) itself.
/// If run manually, you are expected to call io_service.run() on the io_service you provide,
/// or on the run() member function. When running manually you are also expected to call
/// async_getLatestState(handler) frequently enought that the 5ms response requirement of the KUKA
/// FRI interface is met.
class KukaFRIThreadSeparator : public std::enable_shared_from_this<KukaFRIThreadSeparator>, public KukaFRI {
    
public:

    using KukaFRI::ParamIndex;
    using KukaFRI::ThreadingRunMode;
    using KukaFRI::Params;
    using KukaFRI::defaultParams;
    
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
      
        // start up the driver thread since the io_service_ is internal only
        if(std::get<is_running_automatically>(params))
        {
          driver_threadP_.reset(new std::thread([&]{ io_service_.run(); }));
        }
	}
    
    /// Call this to initialize the object
    void construct(Params params = defaultParams()){

        try {
        
            iiwaP = std::make_shared<robot::arm::kuka::iiwa>(KukaFRI::connect(params,io_service_));
            
            latest_commandState_ = std::make_shared<robot::arm::kuka::iiwa::CommandState>();
        
            // start running the driver
            io_service_.post(std::bind(&KukaFRIThreadSeparator::update_state,this));
            
        } catch( boost::exception &e) {
            add_details_to_connection_error(e,params);
            throw;
        }
        
    }
	
    /// @todo properly stop the sockets and other state so that the object can be destroyed cleanly. https://github.com/zeromq/azmq/issues/73
	~KukaFRIThreadSeparator(){
        if(driver_threadP_){
		  //workP.reset();
          io_service_.stop();
          driver_threadP_->join();
        }
	}
	
    /// @brief Advanced: Get the latest state update from the driver when new data is ready.
    ///
    /// @param {
    ///    handler pass a handler with the function signature
    ///      void f(
    ///          std::shared_ptr<grl::robot::arm::kuka::iiwa::MonitorState> updatedStateP,
    ///          boost::system::error_code receive_ec,
    ///          std::size_t receive_bytes_transferred,
    ///          boost::system::error_code send_ec,
    ///          std::size_t send_bytes_transferred
    ///       )
    /// }
    ///
    /// If no states are available, an empty sharedptr will be posted so be sure to check if it is valid.
    /// Once the handler is called, you are responsible for the lifetime of the shared_ptr<MonitorState>,
    /// and it is no longer stored within this class.
    /// @note Only call this version if you are running with is_running_automatically set to run_manually (false/disabled)
    template<typename Handler>
	void async_getLatestState(Handler handler){
        auto self(shared_from_this());
        
        std::shared_ptr<robot::arm::kuka::iiwa::CommandState> cs;
        std::shared_ptr<robot::arm::kuka::iiwa::MonitorState> ms;
        get_cs_ms(cs,ms);
        
            auto update_handler = [this,self,ms,cs,handler](boost::system::error_code receive_ec, std::size_t receive_bytes_transferred,boost::system::error_code send_ec, std::size_t send_bytes_transferred){
                
                    // cs, ms, and self must exist
                BOOST_VERIFY(self);
                BOOST_VERIFY(cs);
                BOOST_VERIFY(ms);
                
                // don't think the state should be saved in this case, because it is already being given to the user
                // save_latest_state(ms,receive_ec,receive_bytes_transferred,send_ec,send_bytes_transferred);
                
                handler(ms,receive_ec,receive_bytes_transferred,send_ec,send_bytes_transferred);

            };
            
            iiwaP->async_update(*ms,*cs,//strand_.wrap( /// @todo FIXME
                     update_handler
                   //) // wrap call ends here, needs to be fixed
                   ); // end async_update call
	}
    
    /// @brief gets the latest state stored in the object, returning immediately
    ///
    /// This will give you robot::arm::kuka::iiwa::MonitorState data, returning an empty
    /// shared_ptr if there is no new data available.
    /// Once this function is called, you are responsible for the lifetime of the shared_ptr<MonitorState>
    /// and it is no longer stored within this class.
    void async_getLatestState(std::shared_ptr<robot::arm::kuka::iiwa::MonitorState>& latestState,
                    boost::system::error_code& recv_ec,std::size_t& recv_bytes_transferred,
                    boost::system::error_code& send_ec,std::size_t& send_bytes_transferred){
                    
        std::shared_ptr<LatestState> latestStateP;
        std::atomic_exchange(&latestStateP,latest_state_);
        
          if(latestStateP.get()!=nullptr) {
            std::tie(latestState,recv_ec,recv_bytes_transferred,send_ec,send_bytes_transferred) = *latestStateP;
          } else {
            std::tie(latestState,recv_ec,recv_bytes_transferred,send_ec,send_bytes_transferred) = LatestState();
          }
    }
    
    
    /// Send a command to the arm.
    /// @todo change this to accept a unique_ptr (may be easier with C++14
    void async_sendCommand(std::shared_ptr<robot::arm::kuka::iiwa::CommandState> commandStateP){
            std::atomic_exchange(&commandStateP,latest_commandState_);
            std::atomic_exchange(&commandStateP,spare_commandState_);
    }
    
    
    /// If you received a MonitorState previously, you can return it to the pool
    /// this is only useful for performance reasons
    void async_addMonitorState(std::shared_ptr<robot::arm::kuka::iiwa::MonitorState> monitorP){
        spare_monitorState_ = monitorP;
    }
    
    
    /// @brief get a command state from the internal pool
    /// pass a handler with the signature void f(std::shared_ptr<robot::arm::kuka::iiwa::CommandState>)
    /// @note IMPORTANT: WRITE ONLY! The command you get may contain no data or old data
    template<typename Handler>
    void async_MakeCommandState(Handler handler){
            std::shared_ptr<robot::arm::kuka::iiwa::CommandState> commandStateP = makeCommandState();
            /// @todo verify io_service choice.
            // don't need to wrap this one because it happens in the user thread
            // was user_io_service_
            io_service_.post([commandStateP,handler](){
                // now in user_io_service_thread, can pass the final result to the user
                 // now in io_service_ thread, can get latest state
                handler(commandStateP);
                
            });
    }
    
    /// @brief get a command state from the internal pool
    /// pass a handler with the signature void f(std::shared_ptr<robot::arm::kuka::iiwa::CommandState>)
    /// @note IMPORTANT: WRITE ONLY! The command you get may contain no data or old data
    std::shared_ptr<robot::arm::kuka::iiwa::CommandState> makeCommandState(){
            std::shared_ptr<robot::arm::kuka::iiwa::CommandState> commandStateP;
            std::atomic_exchange(&commandStateP,spare_commandState_);
            if(commandStateP.get()==nullptr){
              commandStateP = std::make_shared<robot::arm::kuka::iiwa::CommandState>();
            }
            return commandStateP;
    }
    
  /// The io_service::run() call will block until all asynchronous operations
  /// have finished.
    void run(){
        io_service_.run();
    }
    
    boost::asio::io_service& get_io_service(){
        return io_service_;
    }
    
//    /// run_user will only block briefly as all the requested callbacks that have been completed are made
//    void run_user(){
//        user_io_service_.run();
//    }
//    
//    boost::asio::io_service& get_user_io_service(){
//        return user_io_service_;
//    }

private:

    /// gets the commandstate and the monitorstate utilizing existing data if possible
    void get_cs_ms(std::shared_ptr<robot::arm::kuka::iiwa::CommandState>& cs,std::shared_ptr<robot::arm::kuka::iiwa::MonitorState>& ms){
            std::atomic_exchange(&ms,spare_monitorState_);
            if (ms.get()==nullptr) {
                ms = std::make_shared<robot::arm::kuka::iiwa::MonitorState>();
            }
            cs = latest_commandState_;
        
            // cs and ms must exist
            BOOST_VERIFY(cs);
            BOOST_VERIFY(ms);
    }
    
    /// saves the results of a read from the wire to the object, saving data spares if possible to avoid extra allocations
    void save_latest_state(
            const std::shared_ptr<robot::arm::kuka::iiwa::MonitorState>& ms,
            const boost::system::error_code& recv_ec,
            const std::size_t& receive_bytes_transferred,
            const boost::system::error_code& send_ec,
            const std::size_t& send_bytes_transferred
          )
    {
                    
                    // new data available
                    /// @todo if there is a spare use that
                    std::shared_ptr<LatestState> latestState = std::make_shared<LatestState>(std::make_tuple(ms, recv_ec,  receive_bytes_transferred, send_ec,  send_bytes_transferred));
                    std::atomic_exchange(&latest_state_,latestState);

                    // save the monitor state as a spare if it is possible
                    if(
                       latestState.get()!=nullptr &&
                       std::get<latest_receive_monitor_state>(*latestState).get()!=nullptr &&
                       spare_monitorState_.get() == nullptr
                      )
                    {
                      std::atomic_exchange(&spare_monitorState_,std::get<latest_receive_monitor_state>(*latestState));
                    }
    
    }

     /// @todo how to handle command state? If the user doesn't provide one what do we do? Copy the monitor state? This may be handled already in KukaFRI
    void update_state(){
        if(!io_service_.stopped()) {
        
            std::shared_ptr<robot::arm::kuka::iiwa::CommandState> cs;
            std::shared_ptr<robot::arm::kuka::iiwa::MonitorState> ms;
            get_cs_ms(cs,ms);
            
            
            auto self(shared_from_this());
            auto update_handler = [this,self,ms,cs](boost::system::error_code recv_ec, std::size_t receive_bytes_transferred,boost::system::error_code send_ec, std::size_t send_bytes_transferred){
                
                    // cs, ms, and self must exist
                    BOOST_VERIFY(self);
                    BOOST_VERIFY(cs);
                    BOOST_VERIFY(ms);
                
                    save_latest_state(ms,recv_ec,receive_bytes_transferred,send_ec,send_bytes_transferred);
                
                    // run this function again *after* it returns
                    io_service_.post(std::bind(&KukaFRIThreadSeparator::update_state,this));

            };
            
            iiwaP->async_update(*ms,*cs,//strand_.wrap( /// @todo FIXME
                     update_handler
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
    //boost::asio::io_service user_io_service_;
    boost::asio::io_service::strand strand_;
    std::shared_ptr<robot::arm::kuka::iiwa> iiwaP;
    //std::unique_ptr<boost::asio::io_service::work> workP;
    std::unique_ptr<std::thread> driver_threadP_;
    
    
    enum LatestStateIndex{
     latest_receive_monitor_state,
     latest_receive_ec,
     latest_receive_bytes_transferred,
     latest_send_ec,
     latest_send_bytes_transferred
    };
    
    typedef std::tuple<std::shared_ptr<robot::arm::kuka::iiwa::MonitorState>,boost::system::error_code, std::size_t,boost::system::error_code, std::size_t>
    LatestState;
    
    
    /// @todo replace with unique_ptr
    /// the front is the most recent state, the back is the oldest
    std::shared_ptr<LatestState>        latest_state_;
    std::shared_ptr<robot::arm::kuka::iiwa::MonitorState> spare_monitorState_;
    /// the front is the most recent state, the back is the oldest
    std::shared_ptr<robot::arm::kuka::iiwa::CommandState> latest_commandState_;
    std::shared_ptr<robot::arm::kuka::iiwa::CommandState> spare_commandState_;
    
};

} // namespace grl

#endif
