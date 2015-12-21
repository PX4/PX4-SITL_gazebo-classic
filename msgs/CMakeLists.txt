find_package(Protobuf REQUIRED)

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
#

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  HilGps.proto
  HilSensor.proto
  HilControl.proto
  SensorImu.proto
  Float.proto
  opticalFlow.proto
  lidar.proto
  CommandAttitudeThrust.proto
  CommandMotorSpeed.proto
  CommandRateThrust.proto
  CommandRollPitchYawrateThrust.proto
  CommandTrajectory.proto
  CommandVelocityTrajectory.proto
  MotorSpeed.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(mav_msgs SHARED ${PROTO_SRCS})
target_link_libraries(mav_msgs ${PROTOBUF_LIBRARY} ${GAZEBO_LIBRARIES})
