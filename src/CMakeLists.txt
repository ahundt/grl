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

add_subdirectory(v_repExtKukaLBRiiwaPlugin)

# ============================================================================
# executable target(s)
# ============================================================================

# Add executable target for each executable program using basis_add_executable().
#
# This command can not only be used to build executables from C/C++, but also
# source code written in other programming languages such as Java, Python, Perl,
# MATLAB, and Bash.


