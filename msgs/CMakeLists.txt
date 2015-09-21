find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  CommandAttitudeThrust.proto
  CommandMotorSpeed.proto
  CommandRateThrust.proto
  CommandRollPitchYawrateThrust.proto
  CommandTrajectory.proto
  CommandVelocityTrajectory.proto
  MotorSpeed.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/vector2d.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(mav_msgs SHARED ${PROTO_SRCS})
target_link_libraries(mav_msgs ${PROTOBUF_LIBRARY})
