cmake_minimum_required(VERSION 3.5)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)

set(BUILD_TYPE_RELEASE TRUE)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(robot)
add_subdirectory(third-party)
add_subdirectory(common)
# add_subdirectory(user)
#add_subdirectory(rc_test)
