cmake_minimum_required(VERSION 3.16)
project(rozeta VERSION 0.1.0 LANGUAGES C CXX)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(ROZETA_BUILD_SHARED "Build shared library" ON)
option(ROZETA_BUILD_EXAMPLES "Build examples" ON)
option(ROZETA_BUILD_TESTS "Build tests" ON)
option(ROZETA_WITH_OPENCV "Enable optional OpenCV camera backend hooks" OFF)
option(ROZETA_WITH_SERIAL_MOTORS "Enable optional serial motor backend" OFF)
option(ROZETA_WITH_YDLIDAR "Enable optional YDLIDAR serial LiDAR backend" OFF)
option(ROZETA_WITH_LDROBOT_LIDAR "Enable optional LDROBOT LD06/LD19-compatible serial LiDAR backend" OFF)
option(ROZETA_WITH_KINECT "Enable optional libfreenect Kinect backend" OFF)

find_package(Doxygen QUIET)

if(ROZETA_WITH_OPENCV)
  find_package(OpenCV QUIET COMPONENTS core imgproc videoio objdetect)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR
        "ROZETA_WITH_OPENCV=ON requires OpenCV with core, imgproc, videoio "
        "and objdetect components. Install OpenCV development packages or configure "
        "with -DROZETA_WITH_OPENCV=OFF.")
  endif()
endif()

if(ROZETA_WITH_KINECT)
  find_path(FREENECT_INCLUDE_DIR libfreenect.h)
  find_library(FREENECT_LIBRARY freenect)
  if(NOT FREENECT_INCLUDE_DIR OR NOT FREENECT_LIBRARY)
    message(FATAL_ERROR
        "ROZETA_WITH_KINECT=ON requires libfreenect development headers and "
        "library. Install libfreenect-dev or configure with "
        "-DROZETA_WITH_KINECT=OFF.")
  endif()
endif()

if(DOXYGEN_FOUND)
  add_custom_target(rozeta_docs
      COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      COMMENT "Generate Rozeta code-based API documentation with Doxygen"
      VERBATIM)
endif()

set(ROZETA_SOURCES
    src/c_api.cpp
    src/core.cpp
    src/logging.cpp
    src/telemetry.cpp
    src/motors.cpp
    src/gps.cpp
    src/odometry.cpp
    src/lidar.cpp
    src/imu.cpp
    src/camera.cpp
    src/calibration.cpp
    src/perception.cpp
    src/kinect.cpp
    src/maps.cpp
    src/navigation.cpp
    src/runtime.cpp
    src/safety.cpp
    src/field_runner.cpp
    src/mission.cpp
    src/ui.cpp
    src/obstacle_detection.cpp
    src/obstacle_behavior.cpp
    src/operator_io.cpp
    src/robotour_config.cpp
    src/internal/serial_port.cpp
)

if(ROZETA_WITH_SERIAL_MOTORS)
  list(APPEND ROZETA_SOURCES src/motors_serial.cpp)
endif()
if(ROZETA_WITH_YDLIDAR)
  list(APPEND ROZETA_SOURCES src/internal/ydlidar_parser.cpp src/lidar_ydlidar.cpp)
endif()
if(ROZETA_WITH_LDROBOT_LIDAR)
  list(APPEND ROZETA_SOURCES src/internal/ldrobot_lidar_parser.cpp src/lidar_ldrobot.cpp)
endif()
if(ROZETA_WITH_OPENCV)
  list(APPEND ROZETA_SOURCES src/camera_opencv.cpp)
endif()
if(ROZETA_WITH_KINECT)
  list(APPEND ROZETA_SOURCES src/kinect_freenect.cpp)
endif()

add_library(rozeta_static STATIC
    ${ROZETA_SOURCES}
)
target_include_directories(rozeta_static
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE src)
target_compile_options(rozeta_static PRIVATE -Wall -Wextra -Wpedantic)
target_compile_features(rozeta_static PUBLIC cxx_std_17)
target_compile_definitions(rozeta_static PRIVATE ROZETA_VERSION_STRING="${PROJECT_VERSION}")
if(UNIX AND NOT APPLE)
  target_link_libraries(rozeta_static PUBLIC m stdc++)
endif()
if(ROZETA_WITH_SERIAL_MOTORS)
  target_compile_definitions(rozeta_static PUBLIC ROZETA_WITH_SERIAL_MOTORS)
endif()
if(ROZETA_WITH_YDLIDAR)
  target_compile_definitions(rozeta_static PUBLIC ROZETA_WITH_YDLIDAR)
endif()
if(ROZETA_WITH_LDROBOT_LIDAR)
  target_compile_definitions(rozeta_static PUBLIC ROZETA_WITH_LDROBOT_LIDAR)
endif()
if(ROZETA_WITH_OPENCV)
  target_compile_definitions(rozeta_static PUBLIC ROZETA_WITH_OPENCV)
  target_include_directories(rozeta_static PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(rozeta_static PUBLIC ${OpenCV_LIBS})
endif()
if(ROZETA_WITH_KINECT)
  target_compile_definitions(rozeta_static PUBLIC ROZETA_WITH_KINECT)
  target_include_directories(rozeta_static PRIVATE ${FREENECT_INCLUDE_DIR})
  target_link_libraries(rozeta_static PUBLIC ${FREENECT_LIBRARY})
endif()
set_target_properties(rozeta_static PROPERTIES OUTPUT_NAME rozeta EXPORT_NAME rozeta)

if(ROZETA_BUILD_SHARED)
  add_library(rozeta_shared SHARED
      ${ROZETA_SOURCES}
  )
  target_include_directories(rozeta_shared
      PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      PRIVATE src)
  target_compile_options(rozeta_shared PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_features(rozeta_shared PUBLIC cxx_std_17)
  target_compile_definitions(rozeta_shared PRIVATE ROZETA_VERSION_STRING="${PROJECT_VERSION}")
  if(ROZETA_WITH_SERIAL_MOTORS)
    target_compile_definitions(rozeta_shared PUBLIC ROZETA_WITH_SERIAL_MOTORS)
  endif()
  if(ROZETA_WITH_YDLIDAR)
    target_compile_definitions(rozeta_shared PUBLIC ROZETA_WITH_YDLIDAR)
  endif()
  if(ROZETA_WITH_LDROBOT_LIDAR)
    target_compile_definitions(rozeta_shared PUBLIC ROZETA_WITH_LDROBOT_LIDAR)
  endif()
  if(ROZETA_WITH_OPENCV)
    target_compile_definitions(rozeta_shared PUBLIC ROZETA_WITH_OPENCV)
    target_include_directories(rozeta_shared PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rozeta_shared PUBLIC ${OpenCV_LIBS})
  endif()
  if(ROZETA_WITH_KINECT)
    target_compile_definitions(rozeta_shared PUBLIC ROZETA_WITH_KINECT)
    target_include_directories(rozeta_shared PRIVATE ${FREENECT_INCLUDE_DIR})
    target_link_libraries(rozeta_shared PUBLIC ${FREENECT_LIBRARY})
  endif()
  set_target_properties(rozeta_shared PROPERTIES
      OUTPUT_NAME rozeta
      EXPORT_NAME rozeta
      VERSION ${PROJECT_VERSION}
      SOVERSION ${PROJECT_VERSION_MAJOR})
endif()

if(ROZETA_BUILD_SHARED)
  set(ROZETA_INSTALL_TARGET rozeta_shared)
else()
  set(ROZETA_INSTALL_TARGET rozeta_static)
endif()

install(TARGETS ${ROZETA_INSTALL_TARGET}
    EXPORT rozetaTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/rozetaConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/rozetaConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/rozetaConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rozeta)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/rozetaConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/rozetaConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rozeta)
install(EXPORT rozetaTargets
    NAMESPACE rozeta::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rozeta)

if(ROZETA_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(ROZETA_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
