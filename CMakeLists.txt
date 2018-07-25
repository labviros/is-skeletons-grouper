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
#### Build Options protobuf
######

find_package(Protobuf REQUIRED)
PROTOBUF_GENERATE_CPP(options_src options_hdr options.proto)

# ######
# #### Build Vision library
# ######


find_package(is-wire REQUIRED)
find_package(armadillo REQUIRED)

add_library(vision vision.cpp)

target_link_libraries(
  vision
 PUBLIC
  armadillo::armadillo
 PRIVATE
  is-wire::is-wire
)
set_property(TARGET vision PROPERTY CXX_STANDARD 14)

# ######
# #### Build Skeletons library
# ######


add_library(skeletons skeletons_grouper.hpp skeletons_grouper.cpp)

target_link_libraries(
  skeletons
 PUBLIC
  is-msgs::is-msgs
 PRIVATE
  armadillo::armadillo
)

set_property(TARGET skeletons PROPERTY CXX_STANDARD 14)

# # ######
# # #### Build service binary
# # ######

find_package(zipkin-cpp-opentracing REQUIRED)
add_executable(service.bin service.cpp ${options_hdr} ${options_src})

target_include_directories(
  service.bin 
 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # for generated files in build mode
)

target_link_libraries(
  service.bin
 PRIVATE
  is-wire::is-wire
  zipkin-cpp-opentracing::zipkin-cpp-opentracing
  skeletons
  vision
)

set_property(TARGET service.bin PROPERTY CXX_STANDARD 14)