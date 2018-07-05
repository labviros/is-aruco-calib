cmake_minimum_required(VERSION 3.10.0)
include(GNUInstallDirs)

include(${CMAKE_BINARY_DIR}/conan_paths.cmake)

find_package(Protobuf REQUIRED)
find_package(is-wire REQUIRED)
find_package(opencv REQUIRED)

get_target_property(Protobuf_IMPORT_DIRS is-msgs::is-msgs INTERFACE_INCLUDE_DIRECTORIES)
protobuf_generate_cpp(options_SOURCE options_HEADER options.proto)

add_library(options ${options_SOURCE} ${options_HEADER})
target_link_libraries(
  options 
 PUBLIC 
  is-msgs::is-msgs
)
target_include_directories(
  options 
 PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)
set_property(TARGET options PROPERTY CXX_STANDARD 11)
add_library(options::options ALIAS options)

add_executable(intrinsic-calib calibration/intrinsic-calib.cpp)
target_link_libraries(
  intrinsic-calib 
 PRIVATE 
  options::options
  is-wire::is-wire
  opencv::opencv
)
set_property(TARGET intrinsic-calib PROPERTY CXX_STANDARD 11)

add_executable(extrinsic-calib calibration/extrinsic-calib.cpp ${options_SOURCE} ${options_HEADER})
target_link_libraries(
  extrinsic-calib
 PRIVATE 
  options::options
  is-wire::is-wire
  opencv::opencv
)
set_property(TARGET extrinsic-calib PROPERTY CXX_STANDARD 11)

add_executable(create-marker create/create-marker.cpp)
target_link_libraries(
  create-marker 
 PRIVATE 
  options::options
  opencv::opencv
)
set_property(TARGET create-marker PROPERTY CXX_STANDARD 11)

