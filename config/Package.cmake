# ============================================================================
# Copyright (c) 2015 <provider-name>
# All rights reserved.
#
# See COPYING file for license information.
# ============================================================================

##############################################################################
# @file  Package.cmake
# @brief Package configuration.
#
# This file is included by the BasisPack module prior to the CPack module.
# It can be used to overwrite or extend the default package configuration.
#
# @ingroup BasisSettings
##############################################################################

# ============================================================================
# package information/general settings
# ============================================================================

# overwrite default package information here.
#
# See http://www.vtk.org/Wiki/CMake:Packaging_With_CPack

# ============================================================================
# source package
# ============================================================================

# Pattern of files in the source tree that will not be packaged when building
# a source package. This is a list of patterns, e.g., "/CVS/", "/\\.svn/",
# ".swp$", ".#", "/#", "*~", and "cscope*", which are ignored by default.
# Moreover, the directory PROJECT_TESTING_DIR/internal/ is excluded.
# Ignore additional patterns by appending to CPACK_SOURCE_IGNORE_FILE.
#
# Example: list (APPEND CPACK_SOURCE_IGNORE_FILE "<exclude_regex>")
