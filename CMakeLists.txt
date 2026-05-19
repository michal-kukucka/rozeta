cmake_minimum_required(VERSION 3.16)
project(rozeta VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(ROZETA_BUILD_SHARED "Build shared library" ON)
option(ROZETA_BUILD_EXAMPLES "Build examples" ON)
option(ROZETA_BUILD_TESTS "Build tests" ON)
option(ROZETA_WITH_OPENCV "Enable optional OpenCV camera backend hooks" OFF)

add_library(rozeta_static STATIC
    src/core.cpp
    src/logging.cpp
    src/motors.cpp
    src/gps.cpp
    src/odometry.cpp
    src/lidar.cpp
    src/navigation.cpp
    src/obstacle_detection.cpp
)
target_include_directories(rozeta_static PUBLIC include)
target_compile_options(rozeta_static PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(rozeta_static PROPERTIES OUTPUT_NAME rozeta)

if(ROZETA_BUILD_SHARED)
  add_library(rozeta_shared SHARED
      src/core.cpp
      src/logging.cpp
      src/motors.cpp
      src/gps.cpp
      src/odometry.cpp
      src/lidar.cpp
      src/navigation.cpp
      src/obstacle_detection.cpp
  )
  target_include_directories(rozeta_shared PUBLIC include)
  target_compile_options(rozeta_shared PRIVATE -Wall -Wextra -Wpedantic)
  set_target_properties(rozeta_shared PROPERTIES OUTPUT_NAME rozeta)
endif()

if(ROZETA_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(ROZETA_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
