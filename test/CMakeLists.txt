
# ============================================================================
# Copyright (c) 2015 <provider-name>
# All rights reserved.
#
# See COPYING file for license information.
# ============================================================================

##############################################################################
# @file  CMakeLists.txt
# @brief Build configuration of software testing.
#
# Unit tests test only single functions, classes, and modules of the software.
# System tests, on the other side, execute the programs of this package with
# given test input data and compare the output of the programs to the expected
# results. For the definition of system tests which perform image regression
# tests, i.e., compare the images written by the test to given baseline images,
# the basis.testdriver executable target imported from BASIS can be used to
# execute the executable to test and perform the regression tests.
#
# Example:
# @code
# basis_add_test (
#  COMMAND basis.testdriver -v
#    --intensity-tolerance 2
#    --compare "t1_bc.nii.gz"
#              "${PROJECT_TESTING_DIR}/baseline/t1_bc.nii.gz"
#    --
#    $<TARGET_FILE:biascorrect>
#      "${PROJECT_TESTING_DIR}/input/t1.nii.gz"
#      t1_bc.nii.gz
# )
# @endcode
#
# If the tests are implemented in C/C++ as supported by CMake's
# create_test_sourcelist() function, a custom test driver can be generated
# using the basis_add_test_driver() function and this test driver then be used
# to execute the tests and perform the regression tests.
#
# Example:
# @code
# basis_add_test_driver (testdriver test1.cxx test2.cxx test3.cxx)
#
# basis_add_test (
#   COMMAND testdriver -v
#     --intensity-tolerance 2
#     --compare "t1_bc.nii.gz"
#               "${PROJECT_TESTING_DIR}/baseline/t1_bc.nii.gz"
#     --
#     biascorrect # test1.cxx defines int biascorrect(int argc, char* argv[])
#      "${PROJECT_TESTING_DIR}/input/t1.nii.gz"
#      t1_bc.nii.gz
# )
#
# # ...
# @endcomsg->#
# @sa http://www.cmake.org/cmake/help/cmake-2-8-docs.html#command:create_test_sourcelist
#
# The built test executables can be found in @c TESTING_RUNTIME_DIR.
# The test output shall be written to @c TESTING_OUTPUT_DIR. In most cases,
# a subdirectory for each test should be created under this output directory.
# Therefore, the basis_add_test() function creates such subdirectory and sets
# the working directory of the test to this output directory.
##############################################################################

# ============================================================================
# internal tests
# ============================================================================

if (IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/internal")
  add_subdirectory ("internal")
endif ()

# ============================================================================
# public tests
# ============================================================================

# For KUKA IIWA FRI Libraries
if(TARGET KukaFRIClient OR FRI_Client_SDK_Cpp_FOUND)

    basis_include_directories(${FRI_Client_SDK_Cpp_PROJECT_INCLUDE_DIRS} ${Boost_REGEX_LIBRARY} ${FRI_Client_SDK_Cpp_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR}/thirdparty/vrep/include
                              ${FLATBUFFERS_INCLUDE_DIRS} ${FUSIONTRACK_INCLUDE_DIRS})

	basis_add_executable(KukaFRITest.cpp)# ${GRL_FLATBUFFERS_OUTPUTS})
	basis_target_link_libraries(KukaFRITest ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${FRI_Client_SDK_Cpp_LIBRARIES}  ${Nanopb_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${FLATBUFFERS_STATIC_LIB} KukaFRIClient )

    if(UNIX AND NOT APPLE)
      set(LINUX_ONLY_LIBS ${LIBDL_LIBRARIES})
    endif()

    basis_add_test(KukaLBRiiwaVrepPluginTest.cpp)
	basis_target_link_libraries(KukaLBRiiwaVrepPluginTest ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} ${FRI_Client_SDK_Cpp_LIBRARIES}  ${Nanopb_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT}
        ${FLATBUFFERS_STATIC_LIB} v_repLib KukaFRIClient ${LINUX_ONLY_LIBS} )

	basis_add_executable(FlatbuffersExampleWithKukaIiwaTest.cpp)# ${GRL_FLATBUFFERS_OUTPUTS})
	basis_target_link_libraries(FlatbuffersExampleWithKukaIiwaTest ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${FRI_Client_SDK_Cpp_LIBRARIES}  ${Nanopb_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} KukaFRIClient ${FLATBUFFERS_STATIC_LIB})

    basis_add_executable(readFRIFBTest.cpp)# ${GRL_FLATBUFFERS_OUTPUTS})
	basis_target_link_libraries(readFRIFBTest ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${FRI_Client_SDK_Cpp_LIBRARIES}  ${Nanopb_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} KukaFRIClient ${FLATBUFFERS_STATIC_LIB} -lboost_iostreams -lboost_system -lboost_filesystem)
endif()


if(CERES_FOUND OR USE_INTERNAL_CERES)
    basis_include_directories(${PROJECT_INCLUDE_DIR}/thirdparty/vrep/include ${PROJECT_INCLUDE_DIR}/thirdparty/camodocal/include ${EIGEN3_INCLUDE_DIR})
    basis_add_test(HandEyeCalibration_test.cpp)
    basis_target_link_libraries(HandEyeCalibration_test ${Boost_LIBRARIES}  ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT}  v_repExtHandEyeCalibration)
endif()


if(FUSIONTRACK_FOUND)
    basis_include_directories(${GRL_INCLUDE_DIRS}/thirdparty/vrep/include ${FUSIONTRACK_INCLUDE_DIRS} ${grl_INCLUDE_DIRS}/thirdparty/vrep/include)
    basis_add_executable(fusionTrackTest.cpp)
    basis_target_link_libraries(fusionTrackTest ${Boost_LIBRARIES}   ${CMAKE_THREAD_LIBS_INIT} ${FUSIONTRACK_LIBRARIES}  v_repLib)
endif()