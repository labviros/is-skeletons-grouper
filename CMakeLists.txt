cmake_minimum_required(VERSION 3.10.0)
include(GNUInstallDirs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp")

include(${CMAKE_BINARY_DIR}/conan_paths.cmake)

add_subdirectory("./src/is/conf")
add_subdirectory("./src/is/utils")
add_subdirectory("./src/is/skeletons-grouper")