# Find AZMQ Headers/Libs

# Variables
# AZMQROOT - set this to a location where ZeroMQ may be found
#
# AZMQ_FOUND - True of ZeroMQ found
# AZMQ_INCLUDE_DIRS - Location of ZeroMQ includes
# AZMQ_LIBRARIS - ZeroMQ libraries

include(FindPackageHandleStandardArgs)

if ("$ENV{AZMQ_ROOT}" STREQUAL "")
    find_path(_AZMQ_ROOT NAMES include/azmq/actor.hpp)
else()
    set(_AZMQ_ROOT "$ENV{AZMQ_ROOT}")
endif()

find_path(AZMQ_INCLUDE_DIRS NAMES azmq/actor.hpp HINTS ${_AZMQ_ROOT}/include)


find_package_handle_standard_args(AZMQ FOUND_VAR AZMQ_FOUND
                                      REQUIRED_VARS AZMQ_INCLUDE_DIRS)

if (AZMQ_FOUND)
    mark_as_advanced(AZMQ_FIND_VERSION_EXACT AZMQ_INCLUDE_DIRS )
endif()
