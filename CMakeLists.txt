cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# project(adapter)
include (FindPkgConfig)
include (FindBoost)

add_definitions(--std=c++11 -DROBOSUB_AUV -Wall -O3)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/msgs)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/msgs)

add_subdirectory(msgs)
add_subdirectory(ipc-gazebo-adapter)
add_subdirectory(gazebo-plugins)