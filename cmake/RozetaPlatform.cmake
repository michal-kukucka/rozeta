# Rozeta normalized platform detection.
#
# Keep platform conditionals centralized so top-level CMake, examples and tests do
# not grow ad-hoc WIN32/UNIX checks as Windows support is added.

set(ROZETA_PLATFORM_WINDOWS OFF)
set(ROZETA_PLATFORM_POSIX OFF)
set(ROZETA_PLATFORM_LINUX OFF)
set(ROZETA_PLATFORM_MACOS OFF)

if(WIN32)
  set(ROZETA_PLATFORM_WINDOWS ON)
endif()

if(UNIX)
  set(ROZETA_PLATFORM_POSIX ON)
endif()

if(APPLE)
  set(ROZETA_PLATFORM_MACOS ON)
elseif(UNIX AND NOT WIN32)
  set(ROZETA_PLATFORM_LINUX ON)
endif()

message(STATUS
    "Rozeta platform: "
    "windows=${ROZETA_PLATFORM_WINDOWS}, "
    "posix=${ROZETA_PLATFORM_POSIX}, "
    "linux=${ROZETA_PLATFORM_LINUX}, "
    "macos=${ROZETA_PLATFORM_MACOS}")
