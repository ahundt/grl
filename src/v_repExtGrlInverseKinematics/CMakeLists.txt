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


#TODO: add cisst-saw found flag and make it work correctly for independent grl https://github.com/jhu-cisst/cisst-saw
if(CISST_SAW_FOUND)
basis_include_directories(${PROJECT_INCLUDE_DIR}/thirdparty/vrep/include ${CISST_SAW_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
basis_add_library(v_repExtGrlInverseKinematics SHARED v_repExtGrlInverseKinematics.cpp )
basis_target_link_libraries(v_repExtGrlInverseKinematics ${Boost_LIBRARIES}  ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT}  v_repLib camodocal_calib)
endif()


# ============================================================================
# executable target(s)
# ============================================================================

# Add executable target for each executable program using basis_add_executable().
#
# This command can not only be used to build executables from C/C++, but also
# source code written in other programming languages such as Java, Python, Perl,
# MATLAB, and Bash.


