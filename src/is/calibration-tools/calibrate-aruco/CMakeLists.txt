cmake_minimum_required(VERSION 3.10.0)
include(GNUInstallDirs)

find_package(is-wire REQUIRED)
find_package(opencv REQUIRED)

add_executable(calibrate-intrinsic calibrate-intrinsic.cpp)
target_link_libraries(
  calibrate-intrinsic 
 PRIVATE 
  options::options
  is-wire::is-wire
  opencv::opencv
)
set_property(TARGET calibrate-intrinsic PROPERTY CXX_STANDARD 11)

add_executable(calibrate-extrinsic calibrate-extrinsic.cpp)
target_link_libraries(
 calibrate-extrinsic 
 PRIVATE 
  options::options
  is-wire::is-wire
  opencv::opencv
)
set_property(TARGET calibrate-extrinsic PROPERTY CXX_STANDARD 11)
