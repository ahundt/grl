# ============================================================================
# Copyright (c) 2015 <provider-name>
# All rights reserved.
#
# See COPYING file for license information.
# ============================================================================

##############################################################################
# @file  CMakeLists.txt
# @brief Root build configuration file.
#
# @note This package utilizes <a href="http://opensource.andreasschuh.com/cmake-basis">CMake BASIS</a>.
#
##############################################################################

# ----------------------------------------------------------------------------
# minimum required CMake version
cmake_minimum_required (VERSION 2.8.4)

# ----------------------------------------------------------------------------
# include BASIS policies, settings, macros, and functions

# circumvent issue with CMake's find_package() interpreting these variables
# relative to the current binary directory instead of the top-level directory
if (BASIS_DIR AND NOT IS_ABSOLUTE "${BASIS_DIR}")
  set (BASIS_DIR "${CMAKE_BINARY_DIR}/${BASIS_DIR}")
  get_filename_component (BASIS_DIR "${BASIS_DIR}" ABSOLUTE)
endif ()

# users tend to specify the directory where BASIS was installed
# rather than the directory containing a BASISConfig.cmake,
# so add a workaround to allow that to work as well
if (IS_DIRECTORY "${BASIS_DIR}")
  list (INSERT CMAKE_PREFIX_PATH 0 "${BASIS_DIR}")
endif ()

# look for an existing CMake BASIS installation and use it if found
# otherwise, attempt to download and install it locally
find_package (BASIS QUIET)
if (NOT BASIS_FOUND)
  if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/BasisBootstrapping.cmake")
    include ("${CMAKE_CURRENT_SOURCE_DIR}/BasisBootstrapping.cmake")
  else ()
    message (FATAL_ERROR "Could not find an existing CMake BASIS installation!\n"
                           "This project uses CMake BASIS for the build configuration."
                           " Visit http://opensource.andreasschuh.com/cmake-basis for"
                           " more information about the CMake BASIS package.\n")
  endif ()
  # download and install BASIS in build tree of project
  basis_bootstrap(
    VERSION 3.1.0          # CMake BASIS version to download
    USE_MATLAB       FALSE # Enable/disable Matlab support
    USE_PythonInterp FALSE # Enable/disable Python support
    USE_JythonInterp FALSE # Enable/disable Jython support
    USE_Perl         FALSE # Enable/disalbe Perl   support
    USE_BASH         FALSE # Enable/disable Bash   support
    USE_Doxygen      TRUE  # Enable/disable documentation generation using Doxygen
    USE_Sphinx       TRUE  # Enable/disable documentation generation using Sphinx
    USE_ITK          FALSE # Enable/disable image processing regression testing
    INFORM_USER            # Inform user during first configure step
                           # that BASIS needs to be bootstrapped or installed manually
  )
  # look for local installation
  find_package (BASIS QUIET)
  if (NOT BASIS_FOUND)
    message (FATAL_ERROR "CMake BASIS setup failed. Please take the following steps:\n"
                         "\t1. Clear the CMake cache and try from scratch\n"
                         "\t2. Check the CMake BASIS website:\n"
                         "\t\thttp://opensource.andreasschuh.com/cmake-basis/\n"
                         "\t3. Search for an existing issue:\n"
                         "\t\thttps://github.com/schuhschuh/cmake-basis/issues/\n"
                         "\t4. If this did not resolve the issue, please report your problem:\n"
                         "\t\thttps://github.com/schuhschuh/cmake-basis/issues/new\n"
                         "\t5. Try a manual build of CMake BASIS following the instructions at:\n"
                         "\t\thttp://opensource.andreasschuh.com/cmake-basis/quickstart.html"
    )
  endif ()
else ()
  set (BASIS_INSTALL_PREFIX "${BASIS_DIR}")
endif ()

# ----------------------------------------------------------------------------
# configure build system
basis_project_impl ()

# raise error if project uses the BASIS Utilities,
# but BASIS was not installed as part of the bootstrapping
if (DEFINED BASIS_INSTALL_PREFIX_CONFIGURED AND NOT BASIS_INSTALL_PREFIX_CONFIGURED)
  basis_get_project_uses_utilities (PROJECT_USES_BASIS_UTILITIES)
  if (PROJECT_USES_BASIS_UTILITIES)
    message (FATAL_ERROR "This project uses the BASIS Utilities. Therefore CMake BASIS"
                         " must be installed permanently. Please specify the location"
                         " where to install BASIS using the BASIS_INSTALL_PREFIX"
                         " variable and re-run CMake in order to first install BASIS"
                         " and then re-configure this project to use this installation.")
  endif ()
endif ()
