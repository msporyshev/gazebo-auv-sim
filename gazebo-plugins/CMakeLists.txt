cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(--std=c++11)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(CEGUI CEGUI)
  pkg_check_modules(CEGUI-OGRE CEGUI-OGRE)
endif()

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}
    ${OGRE-Terrain_INCLUDE_DIRS} ${CEGUI_INCLUDE_DIRS} ${CEGUI-OGRE_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS}
    ${OGRE-Terrain_LIBRARY_DIRS} ${CEGUI_LIBRARY_DIRS} ${CEGUI-OGRE_LIBRARY_DIRS}
)

find_package(Boost REQUIRED  system)
find_package(Protobuf REQUIRED)

add_library(robosub_auv_plugin SHARED robosub_auv_plugin.cc)
target_link_libraries(robosub_auv_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} adapter_msgs)

add_library(camera_plugin SHARED camera_plugin.cc)
target_link_libraries(camera_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} CameraPlugin adapter_msgs)

add_library(imu_plugin SHARED imu_plugin.cc)
target_link_libraries(imu_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} adapter_msgs)

if(!PROJECT_INSTALL_DIR)
    set(PROJECT_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR})
endif()

file(GLOB CONFIGS "*.sdf")
file(GLOB MESH_FILES "*.dae")

set(GAZEBO_MODEL_PATH $ENV{HOME}/.gazebo/models)
if (DEFINED ENV{GAZEBO_MODEL_PATH})
    message("Using ${GAZEBO_MODEL_PATH} to install models")
    set(GAZEBO_MODEL_PATH $ENV{GAZEBO_MODEL_PATH})
endif()

install(FILES ${CONFIGS} DESTINATION ${PROJECT_INSTALL_DIR})
install(FILES ${MESH_FILES} DESTINATION ${PROJECT_INSTALL_DIR})
install(DIRECTORY orange_stripe torpedo auv_body gripper DESTINATION ${GAZEBO_MODEL_PATH})
