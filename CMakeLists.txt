cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(adapter)

add_definitions(--std=c++11 -DBOOST_LOG_DYN_LINK -DROBOSUB_AUV -Wall -O3)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Protobuf REQUIRED)
find_package(Boost COMPONENTS log)
find_package(Boost REQUIRED COMPONENTS system program_options)

if (Boost_LOG_FOUND)
    add_definitions(-DUSING_BOOST_LOG)
else()
    message("Without boost log")
endif()

add_subdirectory(msgs)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    $ENV{IPC_MSG_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/msgs
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
    ${Protobuf_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(
    ${PROJECT_NAME}
    adapter_msgs
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Boost_LOG_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ipc
)

add_dependencies(${PROJECT_NAME} adapter_msgs)