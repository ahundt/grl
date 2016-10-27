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


#TODO: HACK: find_backage(Boost) shouldn't be necessary, boost.log isn't found properly... remove when possible
find_package(Boost COMPONENTS program_options filesystem unit_test_framework system regex coroutine log chrono)
message(STATUS <<<<<<<<<<<<<IK<<<<<<<<<<<<<<SpaceVecAlg_FOUND:${SpaceVecAlg_FOUND}<<<sch-core_FOUND:${sch-core_FOUND}<<<<RBDyn_FOUND:${RBDyn_FOUND}<<<<Tasks_FOUND:${Tasks_FOUND})
    # SpaceVecAlg        # Implementation of Roy Featherstone's Spatial Vector Algebra https://github.com/jrl-umi3218/SpaceVecAlg
    # sch-core           # Algorithms for convex hulls https://github.com/jrl-umi3218/sch-core used for inverse kinematics
    # RBDyn              # Models the dynamics of rigid body system https://github.com/jrl-umi3218/RBDyn used for inverse kinematics
    # Tasks              # Real time control of Kinematic Trees https://github.com/jrl-umi3218/Tasks used for inverse kinematics
if( SpaceVecAlg_FOUND AND sch-core_FOUND AND RBDyn_FOUND AND Tasks_FOUND AND Boost_FOUND AND Boost_LOG_FOUND)

message(STATUS <<<<<<<<<<<<<IK_BUILDING!!!)
    #TODO: #WARNING: INSTALLER WILL BE BROKEN UNLESS THIS IS CHANGED TO USE basis_target_link_libraries, etc. 
    #@see https://github.com/schuhschuh/cmake-basis/issues/442
    basis_include_directories(
      ${PROJECT_INCLUDE_DIR}/thirdparty/vrep/include 
      ${EIGEN3_INCLUDE_DIR}
      # TODO(ahundt): include directories from Tasks & its deps
      #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mylib>
      #$<INSTALL_INTERFACE:include/mylib>  # <prefix>/include/mylib
    )
    # ${CISST_SAW_INCLUDE_DIRS}
    basis_add_library(v_repExtGrlInverseKinematics SHARED v_repExtGrlInverseKinematics.cpp )
    basis_target_link_libraries(v_repExtGrlInverseKinematics 
                    ${Boost_LIBRARIES}  
                    ${Boost_LOG_LIBRARY}
                    ${CMAKE_THREAD_LIBS_INIT}
                    SpaceVecAlg
                    sch-core
                    RBDyn
                    Tasks
                    v_repLib
                )
endif()


# ============================================================================
# executable target(s)
# ============================================================================

# Add executable target for each executable program using basis_add_executable().
#
# This command can not only be used to build executables from C/C++, but also
# source code written in other programming languages such as Java, Python, Perl,
# MATLAB, and Bash.


