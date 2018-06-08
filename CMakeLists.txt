cmake_minimum_required(VERSION 3.10.0)
include(GNUInstallDirs)

if (EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  conan_basic_setup()
endif()

find_package(Protobuf REQUIRED)
find_package(is-msgs REQUIRED)

find_library(FFMPEG_avformat_LIBRARY avformat)
find_library(FFMPEG_avcodec_LIBRARY avcodec)
find_library(FFMPEG_avdevice_LIBRARY avdevice)
find_library(FFMPEG_avresample_LIBRARY avresample)
find_library(FFMPEG_avutil_LIBRARY avutil)
find_library(FFMPEG_swscale_LIBRARY swscale)

set(FFMPEG_LIBRARIES ${FFMPEG_avformat_LIBRARY} ${FFMPEG_avcodec_LIBRARY} ${FFMPEG_avdevice_LIBRARY}
  ${FFMPEG_avresample_LIBRARY} ${FFMPEG_avutil_LIBRARY} ${FFMPEG_swscale_LIBRARY})

get_target_property(Protobuf_IMPORT_DIRS is-msgs::is-msgs INTERFACE_INCLUDE_DIRECTORIES)
protobuf_generate_cpp(options_SOURCE options_HEADER options.proto)

add_executable(intrinsic-calib calibration/intrinsic-calib.cpp ${options_SOURCE} ${options_HEADER})
target_include_directories(intrinsic-calib PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_link_libraries(intrinsic-calib PRIVATE ${CONAN_LIBS} ${FFMPEG_LIBRARIES})
set_property(TARGET intrinsic-calib PROPERTY CXX_STANDARD 11)

add_executable(extrinsic-calib calibration/extrinsic-calib.cpp ${options_SOURCE} ${options_HEADER})
target_include_directories(extrinsic-calib PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_link_libraries(extrinsic-calib PRIVATE ${CONAN_LIBS})
set_property(TARGET extrinsic-calib PROPERTY CXX_STANDARD 11)

add_executable(create-marker create/create-marker.cpp ${options_SOURCE} ${options_HEADER})
target_include_directories(create-marker PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_link_libraries(create-marker PRIVATE ${CONAN_LIBS})
set_property(TARGET create-marker PROPERTY CXX_STANDARD 11)

