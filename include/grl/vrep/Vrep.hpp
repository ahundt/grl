/// @file a set of v-rep utility functions for accessing simulation state.
#ifndef GRL_VREP_HPP_
#define GRL_VREP_HPP_

#include <boost/exception/all.hpp>
#include <string>
#include <boost/range/algorithm/transform.hpp>
#include "v_repLib.h"


namespace grl { namespace vrep {

/// @brief Get the handle of a single object, otherwise throw an exception
///
/// The handle is an integer that uniquely identifies a visible object in
/// a V-REP simulation.
int getHandle(const std::string& param)
{
    int handle = simGetObjectHandle( param.c_str()  );
    if(handle == -1){
        BOOST_THROW_EXCEPTION(std::runtime_error(std::string("getHandleFromParam: Handle ") + param + " is not valid. Make sure the object you are looking for actually exists."));
    }
    return handle;
}

/// @brief Get the v-rep handle from a range of object strings, such as std::vector<std::string>.
///
/// The handle is an integer that uniquely identifies a visible object in
/// a V-REP simulation.
template<typename SinglePassRange, typename OutputIterator>
OutputIterator getHandles(const SinglePassRange inputRange, OutputIterator out)
{
return boost::range::transform(
                          inputRange,
                          out,
                          [](const std::string& element)
                          {
                            return getHandle(element);
                          }
                         );
}

/// @brief Get the handle from a tuple object used for params.
/// an example can be found in AtracsysFusionTrackVrepPlugin.hpp
template<std::size_t I, typename Param>
int getHandleFromParam(const Param& params_){
    std::string param(std::get<I>              (params_).c_str());
    return getHandle(param);
};

/// @brief Get the handle from a range of entries in a tuple object used for params.
///
/// The handle is an integer that uniquely identifies a visible object in
/// a V-REP simulation.
///  @param params_ Params range of handles such as std::vector<std::String>
template<std::size_t I, typename Params, typename OutputIterator>
OutputIterator getHandleFromParam(const Params params_, OutputIterator out){
  auto& inputRange = std::get<I>(params_);
  return getHandles(inputRange,out);
}

}} // grl::vrep

#endif