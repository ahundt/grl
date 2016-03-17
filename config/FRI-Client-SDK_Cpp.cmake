# Sets up KUKA Sunrise FRI Cpp SDK with KUKA connectivity suite

# Variables
# FRI_Client_SDK_Cpp_zip_PATH - Path to folder containing FRI-Client-SDK_Cpp.zip
#
# Targets
# KukaFRIClient     - KukaFRIClient Libraries you should link to to use Kuka FRI APIs
# nanopb            - nanopb library used to communicate with FRI on Kuka controller
# friUdpConnection  - UDP components for demo code (not needed by github.com/ahundt/grl)
#
# Author: Andrew Hundt <ATHundt@gmail.com>

# Sets up KUKA Sunrise FRI Cpp SDK with KUKA connectivity suite
message(STATUS "Looking for File Path to FRI-Client-SDK_Cpp.zip KUKA FRI C++ API zip file from sunrise connectivity suite v1.9")
set(FRI_Client_SDK_Cpp_zip_PATH "${CMAKE_SOURCE_DIR}/data/" CACHE FILEPATH "File Path to FRI-Client-SDK_Cpp.zip KUKA FRI C++ API zip file from sunrise connectivity suite v1.9")

find_path(FRI_Client_SDK_Cpp_zip_FILEPATH FRI-Client-SDK_Cpp.zip HINTS ${FRI_Client_SDK_Cpp_zip_PATH} ${CMAKE_PROJECT_DIR}/data/ ${CMAKE_CURRENT_PROJECT_DIR}/data/ )

if(EXISTS "${FRI_Client_SDK_Cpp_zip_FILEPATH}")
    message(STATUS "Found FRI-Client-SDK_Cpp.zip in ${FRI_Client_SDK_Cpp_zip_FILEPATH}/FRI-Client-SDK_Cpp.zip")
    # directory where fri code will be extracted
    set(FRI_DIR ${CMAKE_BINARY_DIR}/FRI_Client_SDK_Cpp)
    set(FRI_SRC_DIR ${FRI_DIR}/src)
    
    message(STATUS "extracting FRI C++ SDK... ${FRI_Client_SDK_Cpp_zip_FILEPATH} to ${CMAKE_CURRENT_BINARY_DIR}")
    file(MAKE_DIRECTORY ${FRI_DIR})
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${FRI_Client_SDK_Cpp_zip_FILEPATH}/FRI-Client-SDK_Cpp.zip
      WORKING_DIRECTORY ${FRI_DIR}
      RESULT_VARIABLE rv)

    if(NOT rv EQUAL 0)
      message(STATUS "extracting... FRI-Client-SDK_Cpp [error clean up]")
      file(REMOVE_RECURSE "${CMAKE_BINARY_DIR}/FRI-Client-SDK_Cpp")
      message(FATAL_ERROR "error: extract of '${FRI_Client_SDK_Cpp_zip_FILEPATH}' failed")
    else()
        
        # TODO: DONT HARDCODE DEFINITIONS, Use cmake CHECK_INCLUDE_FILE() to dynamically configure compile definitions
        set(FRI_Client_SDK_Cpp_COMPILE_DEFINITIONS
			"-DHAVE_SOCKLEN_T;-DPB_SYSTEM_HEADER=\"pb_syshdr.h\";-DPB_FIELD_16BIT;-DHAVE_STDINT_H;-DHAVE_STDDEF_H;-DHAVE_STDBOOL_H;-DHAVE_STDLIB_H;-DHAVE_STRING_H"
        )

        if(Nanopb_FOUND)
            basis_include_directories(${NANOPB_INCLUDE_DIRS})
        else()
            message(STATUS "Using internal nanopb, this should be ok. If you also use nanopb and there are linker errors set NANOPB_SRC_ROOT_FOLDER. Consult FRI-Client-SDK_Cpp/config/FindNanopb.cmake for details.")
    
            basis_include_directories(nanopb-0.2.8)
            basis_add_library(nanopb 
                ${FRI_SRC_DIR}/nanopb-0.2.8/pb_encode.c
                ${FRI_SRC_DIR}/nanopb-0.2.8/pb_decode.c
            )
            target_compile_definitions(nanopb PUBLIC ${FRI_Client_SDK_Cpp_COMPILE_DEFINITIONS})
            
            target_include_directories(nanopb PUBLIC $<BUILD_INTERFACE:${FRI_SRC_DIR}/nanopb-0.2.8/>)
    
            set(Nanopb_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIRS/nanopb-0.2.8})
            set(Nanopb_LIBRARIES nanopb)
            basis_install_directory(nanopb-0.2.8)
        endif()

        basis_include_directories(${FRI_DIR}/include ${FRI_SRC_DIR}/base ${FRI_SRC_DIR}/protobuf ${FRI_SRC_DIR}/protobuf_gen)
        basis_install_directory(FRI_Client_SDK_Cpp/src/protobuf)
        basis_install_directory(FRI_Client_SDK_Cpp/src/protobuf_gen)

        basis_add_library(KukaFRIClient 
                    ${FRI_SRC_DIR}/protobuf/friCommandMessageEncoder.cpp 
                    ${FRI_SRC_DIR}/protobuf/friMonitoringMessageDecoder.cpp
                    ${FRI_SRC_DIR}/protobuf/pb_frimessages_callbacks.c
                    ${FRI_SRC_DIR}/protobuf_gen/FRIMessages.pb.c
                    ${FRI_SRC_DIR}/client_lbr/friLBRClient.cpp  
                    ${FRI_SRC_DIR}/client_lbr/friLBRCommand.cpp 
                    ${FRI_SRC_DIR}/client_lbr/friLBRState.cpp
                    ${FRI_SRC_DIR}/base/friClientApplication.cpp
                    ${FRI_SRC_DIR}/client_trafo/friTransformationClient.cpp
                    ${FRI_SRC_DIR}/client_trafo/friTransformationContainer.cpp
                )

        target_compile_definitions(KukaFRIClient PUBLIC ${FRI_Client_SDK_Cpp_COMPILE_DEFINITIONS})

        target_include_directories(KukaFRIClient PUBLIC 
            $<BUILD_INTERFACE:${FRI_DIR}/include>
            $<BUILD_INTERFACE:${FRI_SRC_DIR}/base>
            $<BUILD_INTERFACE:${FRI_SRC_DIR}/client_lbr>
            $<BUILD_INTERFACE:${FRI_SRC_DIR}/client_trafo>
            $<BUILD_INTERFACE:${FRI_SRC_DIR}/connection>
            $<BUILD_INTERFACE:${FRI_SRC_DIR}/protobuf>
            $<BUILD_INTERFACE:${FRI_SRC_DIR}/protobuf_gen>
        )

        if(CMAKE_SYSTEM_NAME EQUAL "LINUX")
            basis_add_library(friUdpConnection ${FRI_SRC_DIR}/connection/friUdpConnection.cpp)
            target_compile_definitions(friUdpConnection ${FRI_Client_SDK_Cpp_COMPILE_DEFINITIONS})
        endif()
        
        basis_target_link_libraries(KukaFRIClient nanopb)
    endif()
    set(FRI_Client_SDK_Cpp_FOUND TRUE PARENT_SCOPE)
else()
    message(AUTHOR_WARNING "FRI-Client-SDK_Cpp.zip NOT_FOUND, skipping...")
endif()
