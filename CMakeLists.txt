cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp")

if (EXISTS ${CMAKE_BINARY_DIR}/conan_paths.cmake)
  include(${CMAKE_BINARY_DIR}/conan_paths.cmake)
endif()

######
#### Build Proto Options
######

find_package(is-msgs REQUIRED)
find_package(Protobuf REQUIRED)
get_target_property(Protobuf_IMPORT_DIRS is-msgs::is-msgs INTERFACE_INCLUDE_DIRECTORIES)

PROTOBUF_GENERATE_CPP(src hdr skeletons.proto skeletons_group.proto)
list(APPEND sources ${src} ${hdr})
add_library(skeletons ${sources})
set_property(TARGET skeletons PROPERTY CXX_STANDARD 11)
# link dependencies
target_link_libraries(
  skeletons
 PUBLIC
  ${PROTOBUF_LIBRARIES} 
)
# header dependencies
target_include_directories(
  skeletons
 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}> # for headers when building
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # for generated files in build mode
 PUBLIC
  ${PROTOBUF_INCLUDE_DIRS} 
)

######
#### Build Vision library
######
find_package(is-wire REQUIRED)
find_package(armadillo REQUIRED)

add_library(vision vision.cpp)
# link dependencies
target_link_libraries(
  vision
 PUBLIC
  is-msgs::is-msgs
  is-wire::is-wire
  armadillo::armadillo
)
set_property(TARGET vision PROPERTY CXX_STANDARD 14)

######
#### Build debugger binary
######

add_executable(debugger.bin debugger.cpp stream_pb.cpp)

target_link_libraries(
  debugger.bin
 PRIVATE
  vision
  skeletons
  is-msgs::is-msgs # must be placed before is-wire
  armadillo::armadillo
)

set_property(TARGET debugger.bin PROPERTY CXX_STANDARD 14)