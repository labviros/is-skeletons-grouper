cmake_minimum_required(VERSION 3.10.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

######
#### Build Vision library
######

find_package(is-msgs REQUIRED)
find_package(is-wire REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(armadillo REQUIRED)

add_library(vision vision.cpp)

target_link_libraries(
  vision
 PUBLIC
  armadillo::armadillo
  Boost::filesystem
  Boost::system
  is-msgs::is-msgs
 PRIVATE
  is-wire::is-wire
)

target_include_directories(
  vision
 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/..> # for headers when building
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # for generated files in build mode
)

set_property(TARGET vision PROPERTY CXX_STANDARD 14)
add_library(vision::vision ALIAS vision)

######
#### Build Skeletons library
######

add_library(skeletons skeletons-grouper.hpp skeletons-grouper.cpp)

target_link_libraries(
  skeletons
 PUBLIC
  is-msgs::is-msgs
 PRIVATE
  armadillo::armadillo
)

target_include_directories(
  skeletons
 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/..> # for headers when building
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # for generated files in build mode
)

set_property(TARGET skeletons PROPERTY CXX_STANDARD 14)
add_library(skeletons::skeletons ALIAS skeletons)

######
#### Build utils library
######

find_package(zipkin-cpp-opentracing REQUIRED)
add_library(utils utils.hpp utils.cpp)

target_link_libraries(
  utils
 PUBLIC
  options::options
  zipkin-cpp-opentracing::zipkin-cpp-opentracing
  is-msgs::is-msgs
 PRIVATE
  is-wire::is-wire
)

target_include_directories(
  utils
 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/..> # for headers when building
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # for generated files in build mode
)

set_property(TARGET utils PROPERTY CXX_STANDARD 14)
add_library(utils::utils ALIAS utils)