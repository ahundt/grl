
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
# @endcode
#
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


basis_add_executable(zmqFlatbuffersTest.cpp)# ${GRL_FLATBUFFERS_OUTPUTS})
basis_target_link_libraries(zmqFlatbuffersTest ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY} ${ZeroMQ_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} )
basis_add_dependencies(zmqFlatbuffersTest grlflatbuffers)

basis_add_executable(AzmqFlatbufferTest.cpp)# ${GRL_FLATBUFFERS_OUTPUTS})
basis_target_link_libraries(AzmqFlatbufferTest ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY} ${ZeroMQ_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
basis_add_dependencies(AzmqFlatbufferTest grlflatbuffers)

basis_add_executable(AzmqTest.cpp)
basis_target_link_libraries(AzmqTest ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY} ${ZeroMQ_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

basis_add_executable(asioSocketTest.cpp)
basis_target_link_libraries(asioSocketTest ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

basis_find_package( OpenCV REQUIRED )
basis_include_directories( ${OpenCV_INCLUDE_DIRS})
basis_add_executable( ppf_load_match ppf_load_match.cpp)
basis_target_link_libraries( ppf_load_match ${OpenCV_LIBS} )

# For KUKA IIWA FRI Libraries
if(FRI-Client-SDK_Cpp_FOUND)
    basis_include_directories(${FRI-Client-SDK_Cpp_PROJECT_INCLUDE_DIRS} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY} ${FRI-Client-SDK_Cpp_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR}/thirdparty/vrep/include)
	basis_add_executable(KukaFRITest.cpp)# ${GRL_FLATBUFFERS_OUTPUTS})
	basis_target_link_libraries(KukaFRITest ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY} ${ZeroMQ_LIBRARIES} ${FRI-Client-SDK_Cpp_LIBRARIES} ${Nanopb_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

	basis_add_executable(KukaFRIClientDataTest.cpp)# ${GRL_FLATBUFFERS_OUTPUTS})
	basis_target_link_libraries(KukaFRIClientDataTest ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY} ${ZeroMQ_LIBRARIES} ${FRI-Client-SDK_Cpp_LIBRARIES}  ${Nanopb_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

	basis_add_executable(KukaFRIClientDataDriverTest.cpp)# ${GRL_FLATBUFFERS_OUTPUTS})
	basis_target_link_libraries(KukaFRIClientDataDriverTest ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY} ${ZeroMQ_LIBRARIES} ${FRI-Client-SDK_Cpp_LIBRARIES}  ${Nanopb_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
    
    basis_add_test(KukaLBRiiwaVrepPluginTest.cpp)
	basis_target_link_libraries(KukaLBRiiwaVrepPluginTest ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} ${ZeroMQ_LIBRARIES} ${FRI-Client-SDK_Cpp_LIBRARIES}  ${Nanopb_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} v_repExtKukaLBRiiwa)
endif()



if(PCL_FOUND AND freenect2_LIBRARIES)
    pkg_search_module(GLFW REQUIRED glfw3)

    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    basis_include_directories(${OpenCV_INCLUDE_DIRS})
    basis_include_directories(${GLEWF_INCLUDE_DIRS})
    basis_include_directories(${PCL_INCLUDE_DIRS})
    basis_include_directories(${GLFW_INCLUDE_DIRS})

    basis_add_executable(kinect2grabber kinect2pclgrabber.cpp)
    basis_add_executable(pcl_grabbing_and_matching pcl_grabbing_and_matching.cpp pcl_grabbing_and_matching_class.cpp)
    basis_add_executable(kinect2pclObjRecRansac kinect2pclObjRecRansac.cpp)
    basis_add_executable(homography2d homography2d.cpp)


    basis_target_link_libraries(kinect2grabber ${OpenCV_LIBS} ${GLFW_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY})
    basis_target_link_libraries(pcl_grabbing_and_matching ${OpenCV_LIBS} ${GLFW_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY})
    basis_target_link_libraries(kinect2pclObjRecRansac ${OpenCV_LIBS} ${GLFW_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY})
    basis_target_link_libraries(homography2d ${OpenCV_LIBS} ${GLFW_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${Boost_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_LOG_LIBRARY})
endif()
