pkg_check_modules(GAZEBO gazebo)

find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(msgs
  regul.proto
  navig.proto
  camera.proto
  compass.proto
  shoot.proto
  gripper.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/image.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/msgs)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
set(MSG_SRCS gripper.pb.cc shoot.pb.cc compass.pb.cc regul.pb.cc navig.pb.cc camera.pb.cc vector3d.pb.h time.pb.h header.pb.h image.pb.h)
add_library(adapter_msgs STATIC ${MSG_SRCS})
target_link_libraries(adapter_msgs ${PROTOBUF_LIBRARIES} ${GAZEBO_LIBRARIES})

if(!PROJECT_INSTALL_DIR)
    set(PROJECT_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR})
endif()

install(TARGETS adapter_msgs DESTINATION ${PROJECT_INSTALL_DIR}/lib)