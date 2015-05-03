#ifndef GRL_VREP_HPP_
#define GRL_VREP_HPP_

template<std::size_t I, typename Param>
int getHandleFromParam(const Param& params_){
    int handle = simGetObjectHandle(std::get<I>              (params_).c_str()   );
    if(handle == -1){
        BOOST_THROW_EXCEPTION(std::runtime_error("getHandleFromParam: Handles are not valid. Make sure the object you are looking for actually exists."));
    }
    return handle;
};

#endif