cmake_minimum_required(VERSION 3.10.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

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
  utils::utils # must be place here due to libcurl linking error
  skeletons::skeletons
  vision::vision
)

set_property(TARGET service.bin PROPERTY CXX_STANDARD 14)