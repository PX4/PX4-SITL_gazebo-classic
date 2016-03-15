cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

find_package( PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)
pkg_check_modules( EIGEN3 REQUIRED eigen3)
pkg_check_modules( protobuf REQUIRED protobuf )

FIND_PACKAGE( Boost 1.40 COMPONENTS system thread REQUIRED )
pkg_check_modules(GAZEBO REQUIRED gazebo>=6.0)
pkg_check_modules(SDF REQUIRED sdformat)
add_definitions(-std=c++11)

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  ${Boost_INCLUDE_DIR}
  # Hack to work around pkg_check_modules not setting the Eigen dir
  ${Boost_INCLUDE_DIR}/eigen3
  # Hack to work around pkg_check_modules not setting the OGRE dirs correctly
  /usr/local/include/OGRE
  /usr/local/include/OGRE/Paging
  )

set(enable_mavlink_interface "true")
set(enable_ground_truth "false")
set(enable_logging "false")
set(enable_camera "false")
set(rotors_description_dir "${CMAKE_SOURCE_DIR}/models/rotors_description")
set(scripts_dir "${CMAKE_SOURCE_DIR}/scripts")
add_custom_target(sdf
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	COMMAND rm -f ${CMAKE_SOURCE_DIR}/models/iris/iris.sdf
	COMMAND python ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/iris_base.urdf  ${rotors_description_dir}/urdf/iris_base.xacro enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir} 
	COMMAND gz sdf -p  ${rotors_description_dir}/urdf/iris_base.urdf >> ${CMAKE_SOURCE_DIR}/models/iris/iris.sdf
	COMMAND rm -f ${rotors_description_dir}/urdf/iris_base.urdf
	)

link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(hello_world SHARED src/hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})

add_library(rotors_gazebo_controller_interface SHARED src/gazebo_controller_interface.cpp)
target_link_libraries(rotors_gazebo_controller_interface ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE} ${PROTOBUF_LIBRARY} mav_msgs)
add_dependencies(rotors_gazebo_controller_interface mav_msgs)

add_library(rotors_gazebo_motor_model SHARED src/gazebo_motor_model.cpp)
target_link_libraries(rotors_gazebo_motor_model mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(rotors_gazebo_motor_model mav_msgs)

add_library(rotors_gazebo_multirotor_base_plugin SHARED src/gazebo_multirotor_base_plugin.cpp)
target_link_libraries(rotors_gazebo_multirotor_base_plugin mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(rotors_gazebo_multirotor_base_plugin mav_msgs)

add_library(rotors_gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)
target_link_libraries(rotors_gazebo_imu_plugin mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(rotors_gazebo_imu_plugin mav_msgs)

add_library(gazebo_opticalFlow_plugin SHARED src/gazebo_opticalFlow_plugin.cpp)
target_link_libraries(gazebo_opticalFlow_plugin mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE} ${OpenCV_LIBS})
add_dependencies(gazebo_opticalFlow_plugin mav_msgs)

add_library(gazebo_lidar_plugin SHARED src/gazebo_lidar_plugin.cpp)
target_link_libraries(gazebo_lidar_plugin mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(gazebo_lidar_plugin mav_msgs)

add_library(rotors_gazebo_mavlink_interface SHARED src/gazebo_mavlink_interface.cpp src/geo_mag_declination.cpp)
target_link_libraries(rotors_gazebo_mavlink_interface mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(rotors_gazebo_mavlink_interface mav_msgs)

add_library(LiftDragPlugin SHARED src/liftdrag_plugin/liftdrag_plugin.cpp)
target_link_libraries(LiftDragPlugin mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(LiftDragPlugin mav_msgs)
