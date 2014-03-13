find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(msgs
  regul.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/msgs)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(adapter_msgs SHARED regul.pb.cc)
target_link_libraries(adapter_msgs ${PROTOBUF_LIBRARIES} ${GAZEBO_LIBRARIES})