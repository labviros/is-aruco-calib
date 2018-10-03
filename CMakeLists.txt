
cmake_minimum_required(VERSION 3.10.0)

include(${CMAKE_BINARY_DIR}/conan_paths.cmake)

set(enable_tests true CACHE BOOL "enable testing")
if (enable_tests)
  enable_testing()
endif()

add_subdirectory("./src/is/calibration-tools/conf")
add_subdirectory("./src/is/calibration-tools/create-aruco")
add_subdirectory("./src/is/calibration-tools/calibrate-aruco")