cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD_REQUIRED 17)
set(CMAKE_CXX_STANDARD 17)

project(MQSim)

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -std=c++17 -DDEBUG -Wall -g -ggdb $ENV{CXXFLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++17 -Wall $ENV{CXXFLAGS}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# lib
aux_source_directory(${PROJECT_SOURCE_DIR}/src/exec lib_srcs)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/host lib_srcs)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/nvm_chip/flash_memory lib_srcs)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/sim lib_srcs)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/ssd lib_srcs)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/utils lib_srcs)
add_library(MQSimLib ${lib_srcs})
target_include_directories(MQSimLib PUBLIC ${PROJECT_SOURCE_DIR}/src)

# exe
add_executable(MQSim ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(MQSim PUBLIC MQSimLib)
