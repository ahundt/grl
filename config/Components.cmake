# ============================================================================
# Copyright (c) 2015 <provider-name>
# All rights reserved.
#
# See COPYING file for license information.
# ============================================================================

##############################################################################
# @file  Components.cmake
# @brief Configuration of component-based installers.
#
# This file is included by the BasisPack.cmake module if found in the
# @c PROJECT_CONFIG_DIR after the CPack module was included. It is used to
# configure component-based installers.
#
# Use the functions basis_add_component() and basis_add_component_group()
# to add a component or component group, respectively. See documentation of
# these functions for details.
#
# @sa basis_add_component()
# @sa basis_add_component_group()
#
# For CPack generators which generate several packages, the default behavior
# is to generate one package per component group. However, one can modify this
# default behavior by setting CPACK_COMPONENTS_GROUPING to one of the
# following values:
#
#   - ALL_GROUPS_IN_ONE       Generate separate package for each component group.
#   - IGNORE                  Generate separate package for each component.
#   - ALL_COMPONENTS_IN_ONE   Generate single package for all components.
#
# @sa http://www.vtk.org/Wiki/CMake:Component_Install_With_CPack#Controlling_Differents_Ways_of_packaging_components
#
# @ingroup BasisSettings
##############################################################################

