#ifndef GRL_VREP_HPP_
#define GRL_VREP_HPP_

#include <string>
#include "v_repLib.h"

template<std::size_t I, typename Param>
int getHandleFromParam(const Param& params_){
    std::string param(std::get<I>              (params_).c_str());
    int handle = simGetObjectHandle( param.c_str()  );
    if(handle == -1){
        BOOST_THROW_EXCEPTION(std::runtime_error(std::string("getHandleFromParam: Handle ") + param + " is not valid. Make sure the object you are looking for actually exists."));
    }
    return handle;
};

#endif