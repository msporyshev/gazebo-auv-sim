cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(--std=c++11 -DBOOST_LOG_DYN_LINK -DBOOST_TEST_DYN_LINK -DROBOSUB_AUV -Wall -O3)


pkg_check_modules(GAZEBO gazebo)

find_package(Protobuf REQUIRED)
find_package(Boost COMPONENTS log)
find_package(Boost REQUIRED COMPONENTS system program_options unit_test_framework)
find_package(OpenCV REQUIRED)

if (Boost_LOG_FOUND)
    add_definitions(-DUSING_BOOST_LOG)
else()
    message("Without boost log")
endif()

add_executable(test test.cpp)
target_link_libraries(
    test
    adapter_msgs
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Boost_LOG_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ipc
    pthread
)

add_executable(regul_test regul_test.cpp)
target_link_libraries(
    regul_test
    adapter_msgs
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Boost_LOG_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    pthread
)


if(!PROJECT_INSTALL_DIR)
    set(PROJECT_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR})
endif()

install(TARGETS test regul_test DESTINATION ${PROJECT_INSTALL_DIR}/bin)
install(FILES regul.log DESTINATION ${PROJECT_INSTALL_DIR}/bin)
