cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(adapter)

add_definitions(--std=c++11 -DBOOST_LOG_DYN_LINK -DROBOSUB_AUV -Wall -O3)


pkg_check_modules(GAZEBO gazebo)

find_package(Protobuf REQUIRED)
find_package(Boost COMPONENTS log)
find_package(Boost REQUIRED COMPONENTS system program_options)
find_package(OpenCV REQUIRED)

if (Boost_LOG_FOUND)
    add_definitions(-DUSING_BOOST_LOG)
else()
    message("Without boost log")
endif()

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    $ENV{IPC_MSG_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
    ${Protobuf_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME} main.cpp policy.cpp convert.cpp)
target_link_libraries(
    ${PROJECT_NAME}
    adapter_msgs
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Boost_LOG_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ipc
    pthread
)

add_dependencies(${PROJECT_NAME} adapter_msgs)

if(!PROJECT_INSTALL_DIR)
    set(PROJECT_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR})
endif()

install(TARGETS adapter DESTINATION ${PROJECT_INSTALL_DIR}/bin)
