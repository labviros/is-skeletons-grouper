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

add_subdirectory(utils)

# # ######
# # #### Build service binary
# # ######

find_package(is-msgs REQUIRED)
find_package(is-wire REQUIRED)
add_executable(service.bin service.cpp)

target_link_libraries(
  service.bin
 PRIVATE
  is-wire::is-wire
  is-msgs::is-msgs
  utils # must be place here due to libcurl linking error
  skeletons
  vision
)

set_property(TARGET service.bin PROPERTY CXX_STANDARD 14)