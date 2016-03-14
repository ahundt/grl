# ============================================================================
# Copyright (c) 2015 <provider-name>
# All rights reserved.
#
# See COPYING file for license information.
# ============================================================================

##############################################################################
# @file  CMakeLists.txt
# @brief Build configuration of software.
##############################################################################

# ============================================================================
# library target(s)
# ============================================================================

# Add library target for each library using basis_add_library().
#
# This command can not only be used to build libraries from C/C++, but also source
# code written in other programming languages such as Java, Python, Perl,
# MATLAB, and Bash. Note that here we consider modules written in a scripting
# language which are no executables but to be included by other scripts written
# in the particular language as libraries.
#
# Note: Public modules written in a scripting language such as Python, Perl,
#       MATLAB, or Bash which are intended for use by other packages should
#       be placed in the lib/[<lang>/]grl/ directory,
#       where <lang> is the language name in lowercase and is optional.
#       BASIS will automatically add a library build target for these modules.


# ============================================================================
# executable target(s)
# ============================================================================

# Add executable target for each executable program using basis_add_executable().
#
# This command can not only be used to build executables from C/C++, but also
# source code written in other programming languages such as Java, Python, Perl,
# MATLAB, and Bash.


find_package(Boost COMPONENTS program_options filesystem unit_test_framework system regex coroutine log chrono)
if(ROS_FOUND AND FRI-Client-SDK_Cpp_FOUND AND AZMQ_FOUND  AND Boost_FOUND AND Boost_LOG_FOUND AND Boost_CHRONO_FOUND)
  basis_include_directories(${FRI-Client-SDK_Cpp_PROJECT_INCLUDE_DIRS} ${FRI-Client-SDK_Cpp_INCLUDE_DIRS} ${ROS_INCLUDE_DIR})
  basis_add_executable(grl_kuka_ros_driver grl_kuka_ros_driver.cpp)
  basis_add_dependencies(grl_kuka_ros_driver grlflatbuffers ${FRI-Client-SDK_Cpp_LIBRARIES})
  basis_target_link_libraries(grl_kuka_ros_driver
    ${Boost_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Boost_LOG_LIBRARY}
    ${Boost_CHRONO_LIBRARY}
    ${Boost_REGEX_LIBRARY}
    ${ZeroMQ_LIBRARIES}
    ${FRI-Client-SDK_Cpp_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Nanopb_LIBRARIES}
    ${ROS_LIBRARIES})
endif()
