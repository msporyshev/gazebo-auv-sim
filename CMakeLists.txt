cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
include (FindBoost)

add_definitions(--std=c++11 -DROBOSUB_AUV -Wall -fPIC)

set(PROJECT_INSTALL_DIR ${PROJECT_BINARY_DIR})

include_directories(${PROJECT_BINARY_DIR}/msgs)
link_directories(${PROJECT_BINARY_DIR}/msgs)

add_subdirectory(msgs)
add_subdirectory(ipc-gazebo-adapter)
add_subdirectory(gazebo-plugins)
