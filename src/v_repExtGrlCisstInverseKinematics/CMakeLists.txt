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
if(cisst_cisstNumerical_FOUND AND CISSTNETLIB_FOUND AND sawConstraintController_FOUND AND Boost_FOUND)
    #TODO: #WARNING: INSTALLER WILL BE BROKEN UNLESS THIS IS CHANGED TO USE basis_target_link_libraries, etc.
    #@see https://github.com/schuhschuh/cmake-basis/issues/442
    basis_include_directories(
      ${PROJECT_INCLUDE_DIR}/thirdparty/vrep/include
      ${CISSTNETLIB_INCLUDE_DIRS}
      ${CISST_INCLUDE_DIR}
      ${EIGEN3_INCLUDE_DIR}
      ${sawConstraintController_INCLUDE_DIR}
      ${FLATBUFFERS_INCLUDE_DIRS}  ## remove it after calibration debug
    )
    # ${CISST_SAW_INCLUDE_DIRS}
    basis_add_library(v_repExtGrlCisstInverseKinematics SHARED v_repExtGrlInverseKinematics.cpp )
    basis_target_link_libraries(v_repExtGrlCisstInverseKinematics
                    ${Boost_LIBRARIES}
                    ${CMAKE_THREAD_LIBS_INIT}
                    ${Boost_REGEX_LIBRARY}
                    # ${cisstNetlib_LIBRARY_cisstNetlib}
                    # ${cisstNetlib_LIBRARY_cisstNetlib_gcc}
                    # ${cisstNetlib_LIBRARY_cisstNetlib_blas}
                    # ${cisstNetlib_LIBRARY_cisstNetlib_lapack}
                    # ${cisstNetlib_LIBRARY_cisstNetlib_gfortran}
                    ${CISSTNETLIB_LIBRARIES}
                    ${CISST_LIBRARIES}
                    ##${CISST_SAW_LIBRARIES}
                    #${CISSTNETLIB_LIBRARIES}
                    ${sawConstraintController_LIBRARIES}
                    sawConstraintController
                    ${Nanopb_LIBRARIES}
                    ${CMAKE_THREAD_LIBS_INIT}
                    ${FLATBUFFERS_STATIC_LIB}
                    -lboost_iostreams -lboost_system -lboost_filesystem
                    -ldl
                    KukaFRIClient
                    mc_rbdyn_urdf
                    SpaceVecAlg #TODO: re-enable when exported targets are correct again
                    sch-core
                    RBDyn
                    Tasks ## remove it after calibration work
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


