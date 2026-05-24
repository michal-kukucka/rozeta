cmake_minimum_required(VERSION 3.16)
project(rozeta VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(ROZETA_BUILD_SHARED "Build shared library" ON)
option(ROZETA_BUILD_EXAMPLES "Build examples" ON)
option(ROZETA_BUILD_TESTS "Build tests" ON)
option(ROZETA_WITH_OPENCV "Enable optional OpenCV camera backend hooks" OFF)
option(ROZETA_WITH_SERIAL_MOTORS "Enable optional serial motor backend" OFF)
option(ROZETA_WITH_YDLIDAR "Enable optional YDLIDAR serial LiDAR backend" OFF)

find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
  add_custom_target(rozeta_docs
      COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      COMMENT "Generate Rozeta code-based API documentation with Doxygen"
      VERBATIM)
endif()

set(ROZETA_SOURCES
    src/core.cpp
    src/logging.cpp
    src/motors.cpp
    src/gps.cpp
    src/odometry.cpp
    src/lidar.cpp
    src/imu.cpp
    src/maps.cpp
    src/navigation.cpp
    src/obstacle_detection.cpp
    src/internal/serial_port.cpp
)

if(ROZETA_WITH_SERIAL_MOTORS)
  list(APPEND ROZETA_SOURCES src/motors_serial.cpp)
endif()
if(ROZETA_WITH_YDLIDAR)
  list(APPEND ROZETA_SOURCES src/internal/ydlidar_parser.cpp src/lidar_ydlidar.cpp)
endif()

add_library(rozeta_static STATIC
    ${ROZETA_SOURCES}
)
target_include_directories(rozeta_static PUBLIC include PRIVATE src)
target_compile_options(rozeta_static PRIVATE -Wall -Wextra -Wpedantic)
if(ROZETA_WITH_SERIAL_MOTORS)
  target_compile_definitions(rozeta_static PUBLIC ROZETA_WITH_SERIAL_MOTORS)
endif()
if(ROZETA_WITH_YDLIDAR)
  target_compile_definitions(rozeta_static PUBLIC ROZETA_WITH_YDLIDAR)
endif()
set_target_properties(rozeta_static PROPERTIES OUTPUT_NAME rozeta)

if(ROZETA_BUILD_SHARED)
  add_library(rozeta_shared SHARED
      ${ROZETA_SOURCES}
  )
  target_include_directories(rozeta_shared PUBLIC include PRIVATE src)
  target_compile_options(rozeta_shared PRIVATE -Wall -Wextra -Wpedantic)
  if(ROZETA_WITH_SERIAL_MOTORS)
    target_compile_definitions(rozeta_shared PUBLIC ROZETA_WITH_SERIAL_MOTORS)
  endif()
  if(ROZETA_WITH_YDLIDAR)
    target_compile_definitions(rozeta_shared PUBLIC ROZETA_WITH_YDLIDAR)
  endif()
  set_target_properties(rozeta_shared PROPERTIES OUTPUT_NAME rozeta)
endif()

if(ROZETA_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(ROZETA_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
