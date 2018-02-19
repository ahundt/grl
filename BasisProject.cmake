# ===============================================================================
# Copyright (c) 2015-2016 Andrew Hundt
# All rights reserved.
#
# See COPYING file for license information.
# ===============================================================================

#################################################################################
# @file  BasisProject.cmake
# @brief Sets basic information about a BASIS Project and calls basis_project().
#
# This file defines basic information about a project by calling
# the basis_project() function. This basic information, also known as metadata,
# is used by BASIS to setup the project. Moreover, if the project is a module
# of another BASIS project, the dependencies to other modules have to be specified
# here such that the top-level project can analyze the inter-module dependencies.
#
# @sa http://opensource.andreasschuh.com/cmake-basis/standard/modules.html
#
# However, not only dependencies to other modules can be specified here,
# but also dependencies on external packages. A more flexible alternative to
# resolve external dependencies is to add the corresponding basis_find_package()
# statements to the Depends.cmake file. This should, however, only be done
# if specifying the dependencies as arguments to the basis_project() function
# cannot be used to resolve the dependencies properly. If you only need to
# make use of additional variables set by the package configuration file
# of the external package or the corresponding Find<Package>.cmake module,
# add the related CMake code to the Settings.cmake file instead.
#
# Example:
# @code
# basis_project (
#   # ------------------------------------------------------------------------
#   # meta-data
#   NAME              MyProject
#   PACKAGE_VENDOR    shortvname     # Note: PACKAGE_VENDOR will also be part of the default installation path
#   VERSION           1.1.5
#   DESCRIPTION       "This is the description of the project, which is useful for this"
#                     " important thing and that important thing."
#                     " MyProject follows the BASIS implementation standard."
#   AUTHOR            "Ima Nauthor"
#   PROVIDER_NAME     "Great Product Co"
#   PROVIDER_WEBSITE  "http://www.greatproductcompany.com"
#   PROVIDER_LOGO     "${PROJECT_SOURCE_DIR}/doc/logo.png"
#   DIVISION_NAME     "Awesome App Division"
#   DIVISION_WEBSITE  "http://www.awesomeapp.greatproductcompany.com"
#   DIVISION_LOGO     ""${PROJECT_SOURCE_DIR}/doc/division_logo.png""
#   COPYRIGHT         "Copyright (c) 2013 Great Product Co"
#   LICENSE           "See COPYING file."
#   CONTACT           "Contact <info@greatproductcompany.com>"
#   # ------------------------------------------------------------------------
#   # dependencies
#   DEPENDS
#      NiftiCLib
#      PythonInterp
#   OPTIONAL_DEPENDS
#     PythonInterp
#     JythonInterp
#     Perl
#     MATLAB{matlab}
#     BASH
#     Doxygen
#     Sphinx{build}
#     ITK # TODO required by basistest-driver, get rid of this dependency
#   TEST_DEPENDS
#      Perl
#   OPTIONAL_TEST_DEPENDS
#     MATLAB{mex}
#     MATLAB{mcc}
# )
# @endcode
#
# @ingroup BasisSettings
##############################################################################

# Note: The #<*> dependency patterns are required by the basisproject tool and
#       should be kept on a separate line as last commented argument of the
#       corresponding options of the basis_project() command. The TEMPLATE
#       option and set argument are also required by this tool and should not
#       be changed manually. The argument is updated by basisproject --update.

basis_project (
  # --------------------------------------------------------------------------
  # meta-data
  NAME             "grl"
  VERSION          "4.1.0"
  AUTHORS          "Andrew Hundt"
                   #<author>
  DESCRIPTION      "Generic Robotics Library - Robotics software and drivers aiming to be implemented using generic programming"
  # Note: VENDOR will also be part of the default installation path
  VENDOR           #<vendor>
  PROVIDER_NAME    #<provider-name>
  PROVIDER_LOGO    #<provider-logo>
  PROVIDER_WEBSITE "http://www.github.com/ahundt/grl"
  DIVISION_NAME    #<division-name>
  DIVISION_LOGO    #<division-logo>
  DIVISION_WEBSITE "http://www.github.com/ahundt/grl"
  COPYRIGHT        "2015-2017 Andrew Hundt et al"
  LICENSE          "BSD license, See COPYING file for license information."
  CONTACT          "Andrew Hundt <ATHundt@gmail.com>"
  TEMPLATE         "basis/1.1"

  # --------------------------------------------------------------------------
  # dependencies
  DEPENDS
    FlatBuffers # google flatbuffers https://github.com/google/flatbuffers
    Boost{program_options,filesystem,unit_test_framework,system,regex,coroutine,chrono}
    #<dependency>
  OPTIONAL_DEPENDS
    #<optional-dependency>
    Threads            # pthreads, see CMake documentation
    Eigen3             # Linear Algebra eigen.tuxfamily.com
    Nanopb             # Used in Kuka Fast Robot Interface for serializing and deserializing protobufs
    Ceres              # http://ceres-solver.org/ used in arm hand eye calibration
    #CAMODOCAL         # used for hand eye calibration plugin, files included directly https://github.com/hengli/camodocal
    ROS                # Robot Operating System http://ROS.org
    TRTK               # Transform Registration Toolkit, for pivot calibration https://github.com/RWTHmediTEC/TRTK
    SpaceVecAlg        # Implementation of Roy Featherstone's Spatial Vector Algebra https://github.com/jrl-umi3218/SpaceVecAlg
    sch-core           # Algorithms for convex hulls https://github.com/jrl-umi3218/sch-core used for inverse kinematics
    RBDyn              # Models the dynamics of rigid body system https://github.com/jrl-umi3218/RBDyn used for inverse kinematics
    Tasks              # Real time control of Kinematic Trees https://github.com/jrl-umi3218/Tasks used for inverse kinematics
    mc_rbdyn_urdf      # This library allows to parse an URDF file and create RBDyn structure from it. https://github.com/jrl-umi3218/mc_rbdyn_urdf
    spdlog             # fast logging library https://github.com/gabime/spdlog
    LibDL              # Linux Dynamic Loader library, linux only https://refspecs.linuxfoundation.org/LSB_2.0.1/LSB-Core/LSB-Core/libdl.html
  TEST_DEPENDS
    #<test-dependency>
  OPTIONAL_TEST_DEPENDS
    #<optional-test-dependency>
  INCLUDE_DIRS
    include
    include/thirdparty/vrep/include
    include/thirdparty/cartographer/include
    include/thirdparty/camodocal/include
)
