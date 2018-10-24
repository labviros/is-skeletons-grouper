cmake_minimum_required(VERSION 3.10.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

######
#### Build grouper request message
######

find_package(Protobuf REQUIRED)
find_package(is-msgs REQUIRED)

get_target_property(Protobuf_IMPORT_DIRS is-msgs::is-msgs INTERFACE_INCLUDE_DIRECTORIES)
protobuf_generate_cpp(grouper_request_SOURCE grouper_request_HEADER grouper_request.proto)

######
#### Build service binary
######

find_package(is-msgs REQUIRED)
find_package(is-wire REQUIRED)

list(APPEND libraries
  is-wire::is-wire
  is-msgs::is-msgs
  utils::utils # must be place here due to libcurl linking error
  skeletons::skeletons
  vision::vision
)

add_executable(service.bin service.cpp)
target_link_libraries(service.bin PRIVATE ${libraries})
set_property(TARGET service.bin PROPERTY CXX_STANDARD 14)

add_executable(rpc.bin rpc.cpp ${grouper_request_SOURCE} ${grouper_request_HEADER})
target_link_libraries(rpc.bin PRIVATE ${libraries})
target_include_directories(rpc.bin PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_property(TARGET rpc.bin PROPERTY CXX_STANDARD 14)