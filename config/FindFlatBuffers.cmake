#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Tries to find Flatbuffers headers and libraries.
#
# Usage of this module as follows:
#
#  find_package(Flatbuffers)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
#  Flatbuffers_HOME -
#   When set, this path is inspected instead of standard library locations as
#   the root of the Flatbuffers installation.  The environment variable
#   FLATBUFFERS_HOME overrides this veriable.
#
# This module defines
#  FLATBUFFERS_INCLUDE_DIR, directory containing headers
#  FLATBUFFERS_LIBS, directory containing flatbuffers libraries
#  FLATBUFFERS_STATIC_LIB, path to libflatbuffers.a
#  FLATBUFFERS_FOUND, whether flatbuffers has been found

if( NOT "$ENV{FLATBUFFERS_HOME}" STREQUAL "")
file( TO_CMAKE_PATH "$ENV{FLATBUFFERS_HOME}" _native_path )
list( APPEND _flatbuffers_roots ${_native_path} )
elseif ( Flatbuffers_HOME )
list( APPEND _flatbuffers_roots ${Flatbuffers_HOME} )
endif()

# Try the parameterized roots, if they exist
if ( _flatbuffers_roots )
find_path( FLATBUFFERS_INCLUDE_DIR NAMES flatbuffers/flatbuffers.h
    PATHS ${_flatbuffers_roots} NO_DEFAULT_PATH
    PATH_SUFFIXES "include" )
find_library( FLATBUFFERS_LIBRARIES NAMES flatbuffers
    PATHS ${_flatbuffers_roots} NO_DEFAULT_PATH
    PATH_SUFFIXES "lib" )
else ()
find_path( FLATBUFFERS_INCLUDE_DIR NAMES flatbuffers/flatbuffers.h )
find_library( FLATBUFFERS_LIBRARIES NAMES flatbuffers )
endif ()

find_program(FLATBUFFERS_COMPILER flatc
$ENV{FLATBUFFERS_HOME}/bin
${_flatbuffers_roots}/bin
/usr/local/bin
/usr/bin
NO_DEFAULT_PATH
)

if (FLATBUFFERS_INCLUDE_DIR AND FLATBUFFERS_LIBRARIES)
set(FLATBUFFERS_FOUND TRUE)
get_filename_component( FLATBUFFERS_LIBS ${FLATBUFFERS_LIBRARIES} PATH )
set(FLATBUFFERS_LIB_NAME libflatbuffers)
set(FLATBUFFERS_STATIC_LIB ${FLATBUFFERS_LIBS}/${FLATBUFFERS_LIB_NAME}.a)
else ()
set(FLATBUFFERS_FOUND FALSE)
endif ()

if (FLATBUFFERS_FOUND)
if (NOT Flatbuffers_FIND_QUIETLY)
message(STATUS "Found the Flatbuffers library: ${FLATBUFFERS_LIBRARIES}")
endif ()
else ()
if (NOT Flatbuffers_FIND_QUIETLY)
set(FLATBUFFERS_ERR_MSG "Could not find the Flatbuffers library. Looked in ")
if ( _flatbuffers_roots )
  set(FLATBUFFERS_ERR_MSG "${FLATBUFFERS_ERR_MSG} in ${_flatbuffers_roots}.")
else ()
  set(FLATBUFFERS_ERR_MSG "${FLATBUFFERS_ERR_MSG} system search paths.")
endif ()
if (Flatbuffers_FIND_REQUIRED)
  message(FATAL_ERROR "${FLATBUFFERS_ERR_MSG}")
else (Flatbuffers_FIND_REQUIRED)
  message(STATUS "${FLATBUFFERS_ERR_MSG}")
endif (Flatbuffers_FIND_REQUIRED)
endif ()
endif ()

mark_as_advanced(
FLATBUFFERS_INCLUDE_DIR
FLATBUFFERS_LIBS
FLATBUFFERS_STATIC_LIB
FLATBUFFERS_COMPILER
)

if(FLATBUFFERS_FOUND)
  function(FLATBUFFERS_GENERATE_C_HEADERS Name FLATBUFFERS_DIR OUTPUT_DIR)
  # Name is the name of the user defined variable that will be created by this function
  #     Another variable that will be set is ${NAME}_OUTPUTS which will be set to the names
  #     of all output files that have been generated.
  # FLATBUFFERS_DIR is the directory in which to look for the .fbs files
  # OUTPUT_DIR is the directory in which all output files should be placed
    set(FLATC_OUTPUTS)
    foreach(FILE ${ARGN})
      get_filename_component(FLATC_OUTPUT ${FILE} NAME_WE)
      # create a target for the specific flatbuffers file
      set(FBS_FILE_COPY_INCLUDE copy_${FLATC_OUTPUT}_to_include)
      set(FBS_FILE_COPY_BIN copy_${FLATC_OUTPUT}_to_bin)
      # create a target for the generated output cpp file
      set(FLATC_OUTPUT
        "${OUTPUT_DIR}/${FLATC_OUTPUT}_generated.h")
        list(APPEND FLATC_OUTPUTS ${FLATC_OUTPUT} ${FBS_FILE_COPY_INCLUDE} ${FBS_FILE_COPY_BIN})

      # this is the absolute path to the actual filename.fbs file
      set(ABSOLUTE_FBS_FILE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${FLATBUFFERS_DIR}/${FILE})

      add_custom_command(OUTPUT ${FLATC_OUTPUT}
        COMMAND ${FLATBUFFERS_FLATC_EXECUTABLE}
        # Note: We are setting several custom parameters here to make life easier.
        # see flatbuffers documentation for details.
        # flatc --gen-name-strings --scoped-enums --gen-object-api -c -j -p -o
        # see https://google.github.io/flatbuffers/flatbuffers_guide_using_schema_compiler.html
        ARGS --gen-name-strings --scoped-enums --gen-object-api -c -j -p -o "${OUTPUT_DIR}" ${FILE}
		    MAIN_DEPENDENCY ${ABSOLUTE_FBS_FILE_PATH}
        COMMENT "Building C++, Java, and Python header for ${FILE}"
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${FLATBUFFERS_DIR})

         # need to copy the flatbuffers schema so config files can be loaded
         # http://stackoverflow.com/a/13429998/99379
         # CMAKE_CURRENT_SOURCE_DIR 
         #    this is the directory where the currently processed CMakeLists.txt is located in
      # terminal copy commands change between OS versions, so we use CMake's built in file
      # copy command which runs with "cmake -E copy file_to_copy file_destination"
      # we use some variables here so the path is reproducible.
	     add_custom_command(OUTPUT ${FBS_FILE_COPY_INCLUDE}
          COMMAND ${CMAKE_COMMAND} ARGS -E copy ${ABSOLUTE_FBS_FILE_PATH} ${OUTPUT_DIR}
          MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/${FLATBUFFERS_DIR}/${FILE}
          COMMENT "Copying fbs file ${FILE} to ${OUTPUT_DIR}"
          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${FLATBUFFERS_DIR}
       )

       add_custom_command(OUTPUT ${FBS_FILE_COPY_BIN}
       # TODO(ahundt) remove hacky /bin manually set path, this will break for some IDEs
       COMMAND ${CMAKE_COMMAND} ARGS -E copy ${ABSOLUTE_FBS_FILE_PATH} ${CMAKE_BINARY_DIR}/bin
       MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/${FLATBUFFERS_DIR}/${FILE}
       COMMENT "Copying fbs file ${FILE} to ${CMAKE_BINARY_DIR}"
       WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${FLATBUFFERS_DIR}
       )
    endforeach()
    set(${Name}_OUTPUTS ${FLATC_OUTPUTS} PARENT_SCOPE)
  endfunction()

  set(FLATBUFFERS_INCLUDE_DIRS ${FLATBUFFERS_INCLUDE_DIR})
  include_directories(${CMAKE_BINARY_DIR})
else()
  set(FLATBUFFERS_INCLUDE_DIR)
endif()
