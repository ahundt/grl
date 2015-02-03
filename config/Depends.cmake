# ============================================================================
# Copyright (c) 2015 <provider-name>
# All rights reserved.
#
# See COPYING file for license information.
# ============================================================================

##############################################################################
# @file  Depends.cmake
# @brief Find additional dependencies.
#
# This file is included by basis_project_impl() after it included the
# BasisProject.cmake file of the project and collected information about its
# modules. Furthermore, it is included before it looks for the packages
# specified as arguments to the basis_project() command. At this point, the
# CMake project is not initialized yet and most BASIS variables are not set.
#
# Use this file to find additional dependencies or dependencies which are
# only required by a project if certain modules are enabled.
#
# Use case: If this project has a module which is a Slicer Extension
# and the project itself shall be build as Slicer Extension only if the
# module is enabled, the Slicer package configuration file has to be
# included here such that the Slicer settings are imported before any
# module is configured. This is done by using the command
#
# Another use case would be that you want to specify advanced options to
# the basis_find_package() function which you cannot specify as part of
# the dependencies arguments of the basis_project() function.
#
# Example:
# @code
# if (SlicerExtension_ENABLED)
#   # requires Slicer if the SlicerExtension module is enabled
#   basis_find_package (Slicer REQUIRED)
#   basis_use_package (Slicer)
# endif ()
# @endcode
#
# @ingroup BasisSettings
##############################################################################

