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
#       be placed in the lib/[<lang>/]Robone/ directory,
#       where <lang> is the language name in lowercase and is optional.
#       BASIS will automatically add a library build target for these modules.



# @todo THIS IF STATEMENT IS A HACK, FIX IT!
if(MODULE_kuka_lwr_iiwa_fri)
basis_add_library(v_repExtRobonePlugin SHARED v_repExtRobonePlugin.cpp v_repLib.cpp)
basis_target_link_libraries(v_repExtRobonePlugin ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${ZeroMQ_LIBRARIES} FRIClient nanopb)
target_include_directories(v_repExtRobonePlugin SYSTEM PRIVATE "${PROJECT_INCLUDE_DIR}/thirdparty/vrep/include")
endif()



# ============================================================================
# executable target(s)
# ============================================================================

# Add executable target for each executable program using basis_add_executable().
#
# This command can not only be used to build executables from C/C++, but also
# source code written in other programming languages such as Java, Python, Perl,
# MATLAB, and Bash.


