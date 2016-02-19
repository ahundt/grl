# ============================================================================
# Copyright (c) 2015 <provider-name>
# All rights reserved.
#
# See COPYING file for license information.
# ============================================================================

##############################################################################
# @file  Settings.cmake
# @brief Non-default project settings.
#
# This file is included by basis_project_impl(), after it looked for the
# required and optional dependencies and the CMake variables related to the
# project directory structure were defined (see Directories.cmake file in
# @c BINARY_CONFIG_DIR). It is also included before the BasisSettings.cmake
# file.
#
# In particular build options should be added in this file using CMake's
# <a href="http://www.cmake.org/cmake/help/cmake-2-8-docs.html#command:option">
# option()</a> command. Further, any common settings related to using a found
# dependency can be set here if the basis_use_package() command was enable
# to import the required configuration of a particular external package.
#
# @ingroup BasisSettings
##############################################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR})

# Enable C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support, or our tests failed to detect it correctly. Please use a different C++ compiler or report this problem to the developers.")
endif()

if(UNIX AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64") 
	ADD_DEFINITIONS(-fPIC)
endif(UNIX AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")


# Link the boost.log library
# @todo consider an alternative to always linking boost log
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

# OSX RPATH
if(APPLE)
   set(CMAKE_MACOSX_RPATH ON)
   #set(CMAKE_SKIP_BUILD_RPATH FALSE)
   #set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
   #set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
   #set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
   #list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
   #if("${isSystemDir}" STREQUAL "-1")
   #    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
   #endif()
endif()

if(OpenCV_FOUND)
    if(NOT ${OpenCV_VERSION} VERSION_LESS 3.0.0)
        add_definitions(-DHAVE_OPENCV3)
    endif()
endif()

# list flatbuffer headers
set(RFB ArmControlState.fbs Geometry.fbs JointState.fbs KUKAiiwaConfiguration.fbs LinkObject.fbs Time.fbs VrepControlPoint.fbs VrepPath.fbs )
# directory to include flatbuffers
set(GRL_FLATBUFFERS_INCLUDE_DIR ${CMAKE_BINARY_DIR}/include)
# Generate flatbuffer message C++ headers
flatbuffers_generate_c_headers(GRL_FLATBUFFERS include/grl/flatbuffer/  ${GRL_FLATBUFFERS_INCLUDE_DIR}/grl/flatbuffer ${RFB})
add_custom_target(grlflatbuffers DEPENDS ${GRL_FLATBUFFERS_OUTPUTS})
basis_include_directories(${GRL_FLATBUFFERS_INCLUDE_DIR} )

# workaround so that compilation will work if FindFRI-Client-SDK_Cpp is a module
# it may also be an external, see config/FindFRI-Client-SDK_Cpp 
if(MODULE_FRI-Client-SDK_Cpp)
    set(FRI-Client-SDK_Cpp_LIBRARIES  KukaFRIClient)

    if(NOT Nanopb_FOUND)
        set(Nanopb_LIBRARIES nanopb)
    endif()
endif()


if(CISSTNETLIB_FOUND)
    set(CISSTNETLIB_INCLUDE_DIRS ${CISSTNETLIB_INCLUDE_DIR})
endif()

