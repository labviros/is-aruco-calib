cmake_minimum_required(VERSION 3.10.0)
include(GNUInstallDirs)

find_package(Protobuf REQUIRED)
find_package(is-msgs REQUIRED)

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
