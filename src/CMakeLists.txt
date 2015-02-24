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
set(RFB vrepPath.fbs)
# directory to include flatbuffers
set(ROBONE_FLATBUFFERS_INCLUDE_DIR ${CMAKE_BINARY_DIR}/include/robone/flatbuffer)
# Generate flatbuffer message C++ headers
flatbuffers_generate_c_headers(ROBONE_FLATBUFFERS ../include/robone/flatbuffer/  ${ROBONE_FLATBUFFERS_INCLUDE_DIR} ${RFB})
basis_include_directories(${ROBONE_FLATBUFFERS_INCLUDE_DIR})
basis_set_property(SOURCE robone.cpp APPEND PROPERTY OBJECT_DEPENDS ${ROBONE_FLATBUFFERS_OUTPUTS})


basis_add_executable(robone.cpp)
basis_target_link_libraries(robone ${PCL_LIBRARIES} ${Boost_LIBRARIES})

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



# ============================================================================
# executable target(s)
# ============================================================================

# Add executable target for each executable program using basis_add_executable().
#
# This command can not only be used to build executables from C/C++, but also
# source code written in other programming languages such as Java, Python, Perl,
# MATLAB, and Bash.


