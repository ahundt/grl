# ============================================================================
# Copyright (c) 2015 <provider-name>
# All rights reserved.
#
# See COPYING file for license information.
# ============================================================================

##############################################################################
# @file  CMakeLists.txt
# @brief Build configuration of example application of software.
#
# By default, all content of the example folder is simply copied to the
# example directory of the installation tree.
##############################################################################
basis_add_subdirectory(v_repExtPluginSkeleton)

basis_install_directory ("." "${INSTALL_EXAMPLE_DIR}")

if(FUSIONTRACK_FOUND)
    basis_include_directories(${GRL_INCLUDE_DIRS}/thirdparty/vrep/include ${FUSIONTRACK_INCLUDE_DIRS} ${grl_INCLUDE_DIRS}/thirdparty/vrep/include
                              ${FLATBUFFERS_INCLUDE_DIRS})
    basis_add_executable(fusionTrackExample.cpp)
    basis_target_link_libraries(fusionTrackExample ${Boost_LIBRARIES}   ${CMAKE_THREAD_LIBS_INIT} ${FUSIONTRACK_LIBRARIES}  v_repLib ${FLATBUFFERS_STATIC_LIB})
    basis_add_dependencies(fusionTrackExample grlflatbuffers)
    # need to copy the flatbuffers schema so config files can be loaded
    # http://stackoverflow.com/a/13429998/99379
    # CMAKE_CURRENT_SOURCE_DIR 
    #    this is the directory where the currently processed CMakeLists.txt is located in
	add_custom_command(TARGET fusionTrackExample PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                      ${GRL_FLATBUFFERS_INCLUDE_DIR}/grl/flatbuffer/ $<TARGET_FILE_DIR:fusionTrackExample>)
endif()


