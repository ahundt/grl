# For building the Kuka iiwa FRI Client library

# @TODO: Making this a find script is a little bit strange, because not all of the project state, such as C++ build parameters are set. It may be better to place this in Settings.cmake. The advantage of being here is that it may work as both a module and as a zip

include (ExternalProject)
# BASIS
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/data/FRI-Client-SDK_Cpp.zip")
  set (FRI-Client-SDK_Cpp_URL "${CMAKE_CURRENT_SOURCE_DIR}/data/FRI-Client-SDK_Cpp.zip")
else ()
  set (FRI-Client-SDK_Cpp_URL "INSERT DOWNLOAD URL OR LOCAL PATH TO FRI-Client-SDK_Cpp.zip HERE, or place zip in ${CMAKE_CURRENT_SOURCE_DIR}/data/FRI-Client-SDK_Cpp.zip")
  message(STATUS "Call cmake with -DFRI_CLIENT_SDK_CPP_URL=/path/to/FRI-Client-SDK_Cpp.zip to build KUKA FRI components")
endif ()

if(EXISTS FRI-Client-SDK_Cpp_URL)
ExternalProject_Add (
  FRI-Client-SDK_Cpp
  URL              "${FRI-Client-SDK_Cpp_URL}"
  URL_MD5          f00aa8257f89016c42026d88d4a3471c
  PATCH_COMMAND    patch -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/data/FRI-Client-SDK_Cpp.patch
  CMAKE_CACHE_ARGS #”-DBUILD_SHARED_LIBS:BOOL=OFF"
                   "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
                   "-DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}"
                   "-DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}" 
                   -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
)

ExternalProject_Get_Property(FRI-Client-SDK_Cpp install_dir)

set(FRI-Client-SDK_Cpp_INCLUDE_DIRS 
${install_dir}/include
${install_dir}/src/base
${install_dir}/src/protobuf
${install_dir}/src/protobuf_gen
)

add_library(KukaFRIClient STATIC IMPORTED )
set_target_properties(KukaFRIClient PROPERTIES IMPORTED_LOCATION ${install_dir}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}KukaFRIClient${CMAKE_STATIC_LIBRARY_SUFFIX} )

set(FRI-Client-SDK_Cpp_LIBRARIES KukaFRIClient)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FRI-Client-SDK_Cpp DEFAULT_MSG FRI-Client-SDK_Cpp_LIBRARIES FRI-Client-SDK_Cpp_INCLUDE_DIRS)

if(NOT Nanopb_FOUND)

    add_library(nanopb STATIC IMPORTED )
    set_target_properties(nanopb PROPERTIES IMPORTED_LOCATION ${install_dir}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}nanopb${CMAKE_STATIC_LIBRARY_SUFFIX} )
    set(Nanopb_LIBRARIES nanopb)
    set(Nanopb_INCLUDE_DIRS     ${install_dir}/src/nanopb-0.2.8)
    find_package_handle_standard_args(Nanopb DEFAULT_MSG Nanopb_LIBRARIES Nanopb_INCLUDE_DIRS)
endif(NOT Nanopb_FOUND)

endif(EXISTS FRI-Client-SDK_Cpp_URL)

