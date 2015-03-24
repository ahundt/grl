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


# list flatbuffer headers
set(RFB Geometry.fbs VrepControlPoint.fbs VrepPath.fbs JointState.fbs)
# directory to include flatbuffers
set(ROBONE_FLATBUFFERS_INCLUDE_DIR ${CMAKE_BINARY_DIR}/include/robone/flatbuffer)
# Generate flatbuffer message C++ headers
flatbuffers_generate_c_headers(ROBONE_FLATBUFFERS ../include/robone/flatbuffer/  ${ROBONE_FLATBUFFERS_INCLUDE_DIR} ${RFB})
basis_include_directories(${ROBONE_FLATBUFFERS_INCLUDE_DIR} )
basis_set_property(SOURCE zmqFlatbuffersTest.cpp APPEND PROPERTY OBJECT_DEPENDS ${ROBONE_FLATBUFFERS_OUTPUTS})


basis_add_executable(zmqFlatbuffersTest.cpp ${ROBONE_FLATBUFFERS_OUTPUTS})
basis_target_link_libraries(zmqFlatbuffersTest ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ZeroMQ_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} )


basis_add_executable(AzmqFlatbufferTest.cpp ${ROBONE_FLATBUFFERS_OUTPUTS})
basis_target_link_libraries(AzmqFlatbufferTest ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ZeroMQ_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

#message(STATUS "kuka_lwr_iiwa_fri_INCLUDE_DIRS:${kuka_lwr_iiwa_fri_INCLUDE_DIRS}")
#message(STATUS "KUKA_LWR_IIWA_FRI_INCLUDE_DIRS:${KUKA_LWR_IIWA_FRI_INCLUDE_DIRS}")
#message(STATUS "kuka_lwr_iiwa_fri_INCLUDE_DIR:${kuka_lwr_iiwa_fri_INCLUDE_DIR}")
#message(STATUS "KUKA_LWR_IIWA_FRI_INCLUDE_DIR:${KUKA_LWR_IIWA_FRI_INCLUDE_DIR}")
#message(STATUS "MODULE_kuka_lwr_iiwa_fri_INCLUDE_DIRS:${kuka_lwr_iiwa_fri_INCLUDE_DIRS}")
#message(STATUS "MODULE_KUKA_LWR_IIWA_FRI_INCLUDE_DIRS:${KUKA_LWR_IIWA_FRI_INCLUDE_DIRS}")
#message(STATUS "MODULE_kuka_lwr_iiwa_fri_INCLUDE_DIR:${kuka_lwr_iiwa_fri_INCLUDE_DIR}")
#message(STATUS "MODULE_KUKA_LWR_IIWA_FRI_INCLUDE_DIR:${KUKA_LWR_IIWA_FRI_INCLUDE_DIR}")
#message(STATUS "KUKA_LWR_IIWA_FRI_PROJECT_INCLUDE_DIR:${KUKA_LWR_IIWA_FRI_PROJECT_INCLUDE_DIR}")
#message(STATUS "kuka_lwr_iiwa_fri_PROJECT_INCLUDE_DIR:${kuka_lwr_iiwa_fri_PROJECT_INCLUDE_DIR}")
#message(STATUS "MODULE_INCLUDE_DIRS:${MODULE_INCLUDE_DIRS}")

#TODO: THIS IS A HACK, fix it!
if(MODULE_kuka_lwr_iiwa_fri)
	# These directories are hard coded
	# replace with relative reconfigurable directories
	basis_include_directories(
	   ../modules/kuka_lwr_iiwa_fri/include
       ../modules/kuka_lwr_iiwa_fri/src/base
       ../modules/kuka_lwr_iiwa_fri/src/protobuf
       ../modules/kuka_lwr_iiwa_fri/src/protobuf_gen
	   ../modules/kuka_lwr_iiwa_fri/src/nanopb-0.2.8
	   #${NANOPB_INCLUDE_DIRS}
    )
	#basis_include_directories(${kuka_lwr_iiwa_fri_PROJECT_INCLUDE_DIRS})
	basis_add_executable(KukaFRITest.cpp ${ROBONE_FLATBUFFERS_OUTPUTS})
	basis_target_link_libraries(KukaFRITest ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ZeroMQ_LIBRARIES} FRIClient nanopb ${CMAKE_THREAD_LIBS_INIT})

	basis_add_executable(KukaFRIClientDataTest.cpp ${ROBONE_FLATBUFFERS_OUTPUTS})
	basis_target_link_libraries(KukaFRIClientDataTest ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ZeroMQ_LIBRARIES} FRIClient nanopb ${CMAKE_THREAD_LIBS_INIT})
endif()

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
#       be placed in the lib/[<lang>/]Robone/ directory,
#       where <lang> is the language name in lowercase and is optional.
#       BASIS will automatically add a library build target for these modules.

add_subdirectory(v_repExtRobonePlugin)

# ============================================================================
# executable target(s)
# ============================================================================

# Add executable target for each executable program using basis_add_executable().
#
# This command can not only be used to build executables from C/C++, but also
# source code written in other programming languages such as Java, Python, Perl,
# MATLAB, and Bash.


