cmake_minimum_required(VERSION 3.10.0)
include(GNUInstallDirs)

find_package(opencv REQUIRED)

add_executable(create-marker create-marker.cpp)
target_link_libraries(
  create-marker 
 PRIVATE 
  options::options
  opencv::opencv
)
set_property(TARGET create-marker PROPERTY CXX_STANDARD 11)
