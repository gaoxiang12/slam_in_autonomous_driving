cmake_minimum_required(VERSION 3.5)
project(slam_in_auto_driving)

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

add_definitions("-DCATKIN_ENABLE_TESTING=0")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

option(BUILD_WITH_UBUNTU1804 OFF)

include(cmake/packages.cmake)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/build/devel/include)

add_subdirectory(src)
