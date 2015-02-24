# Find AzioMQ Headers/Libs

# Variables
# AZIOMQROOT - set this to a location where ZeroMQ may be found
#
# AzioMQ_FOUND - True of ZeroMQ found
# AzioMQ_INCLUDE_DIRS - Location of ZeroMQ includes
# AzioMQ_LIBRARIS - ZeroMQ libraries

include(FindPackageHandleStandardArgs)

if ("$ENV{AZIOMQ_ROOT}" STREQUAL "")
    find_path(_AzioMQ_ROOT NAMES include/aziomq/io_service.hpp)
else()
    set(_AzioMQ_ROOT "$ENV{AZIOMQ_ROOT}")
endif()

find_path(AzioMQ_INCLUDE_DIRS NAMES aziomq/version.hpp HINTS ${_AzioMQ_ROOT}/include)
set(_AzioMQVersion_HPP ${AzioMQ_INCLUDE_DIRS}/aziomq/version.hpp)

find_library(AzioMQ_LIBRARIES NAMES aziomq HINTS ${_AzioMQ_ROOT}/lib)

function(_azmqver_EXTRACT _AzioMQ_VER_COMPONENT _AzioMQ_VER_OUTPUT)
    execute_process(
        COMMAND grep "#define ${_AzioMQ_VER_COMPONENT}"
        COMMAND cut -d\  -f3
        RESULT_VARIABLE _azmqver_RESULT
        OUTPUT_VARIABLE _azmqver_OUTPUT
        INPUT_FILE ${_AzioMQVersion_HPP}
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(${_AzioMQ_VER_OUTPUT} ${_azmqver_OUTPUT} PARENT_SCOPE)
endfunction()

_azmqver_EXTRACT("AZIOMQ_VERSION_MAJOR" AzioMQ_VERSION_MAJOR)
_azmqver_EXTRACT("AZIOMQ_VERSION_MINOR" AzioMQ_VERSION_MINOR)

set(AzioMQ_FIND_VERSION_EXACT "${AzioMQ_VERSION_MAJOR}.${AzioMQ_VERSION_MINOR}")
find_package_handle_standard_args(AzioMQ FOUND_VAR AzioMQ_FOUND
                                      REQUIRED_VARS AzioMQ_INCLUDE_DIRS AzioMQ_LIBRARIES
                                      VERSION_VAR AzioMQ_VERSION)

if (AzioMQ_FOUND)
    mark_as_advanced(AzioMQ_FIND_VERSION_EXACT AzioMQ_VERSION AzioMQ_INCLUDE_DIRS AzioMQ_LIBRARIES)
    message(STATUS "AzioMQ version: ${AzioMQ_VERSION_MAJOR}.${AzioMQ_VERSION_MINOR}")
endif()
