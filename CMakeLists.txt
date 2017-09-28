cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0054 NEW)

if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "install prefix" FORCE)
endif()

message(STATUS "install-prefix: ${CMAKE_INSTALL_PREFIX}")

project(mavlink_sitl_gazebo VERSION 1.0.0)
include(GNUInstallDirs)

#######################
## Find Dependencies ##
#######################

# Add search directory for CMake on OS X
list(APPEND CMAKE_MODULE_PATH /usr/local/share/cmake/Modules)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

option(BUILD_GSTREAMER_PLUGIN "enable gstreamer plugin" "OFF")

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)
find_package(gazebo REQUIRED)
find_program(px4 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread timer)
if (BUILD_GSTREAMER_PLUGIN)
  find_package(GStreamer)
endif()
add_subdirectory( external/OpticalFlow OpticalFlow )
set( OpticalFlow_LIBS "OpticalFlow" )

# find ROS include directories for building against ROS mavlink version
find_package(roscpp)
if (roscpp_FOUND)
   include_directories(${roscpp_INCLUDE_DIRS})
endif()

# see if catkin was invoked to build this
if (CATKIN_DEVEL_PREFIX)
  message(STATUS "catkin ENABLED")
  find_package(catkin REQUIRED)
  if (catkin_FOUND)
    catkin_package()
  else()
    message(FATAL_ERROR "catkin not found")
  endif()
else()
  message(STATUS "catkin DISABLED")
endif()

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
#

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message("Gazebo version: ${GAZEBO_VERSION}")
endif()

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost 1.40 COMPONENTS system thread timer REQUIRED )

###########
## Build ##
###########

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -std=c99 -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")
# -Wno-deprecated-declarations: Shut up warnings about std::binder1st, std::binder2nd.

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
  include
  ${OpticalFlow_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Boost_INCLUDE_DIR}
  # Workaround for Eigen3
  ${Boost_INCLUDE_DIR}/eigen3
  ${EIGEN3_INCLUDE_DIRS}/eigen3
  # Workaround for OGRE include dirs on Mac OS
  /usr/local/include/OGRE
  /usr/local/include/OGRE/Paging
  ${GSTREAMER_INCLUDE_DIRS}
  ../../mavlink/include
  )

link_libraries(
  ${PROTOBUF_LIBRARY}
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${Boost_TIMER_LIBRARY_RELEASE}
  ${GSTREAMER_LIBRARIES}
  )

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

#--------------------------#
# Generation of SDF models #
#--------------------------#

set(enable_mavlink_interface "true")
set(enable_ground_truth "false")
set(enable_logging "false")
set(enable_camera "false")
set(enable_wind "false")
set(rotors_description_dir "${CMAKE_CURRENT_SOURCE_DIR}/models/rotors_description")
set(scripts_dir "${CMAKE_CURRENT_SOURCE_DIR}/scripts")

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
  COMMAND python ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/iris_base.urdf  ${rotors_description_dir}/urdf/iris_base.xacro enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_wind:=${enable_wind} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir}
  COMMAND gz sdf -p  ${rotors_description_dir}/urdf/iris_base.urdf >> ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
  COMMAND rm -f ${rotors_description_dir}/urdf/iris_base.urdf
  DEPENDS ${rotors_description_dir}/urdf/iris.xacro
  DEPENDS ${rotors_description_dir}/urdf/iris_base.xacro
  DEPENDS ${rotors_description_dir}/urdf/component_snippets.xacro
  )
add_custom_target(sdf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf)

#--------------------#
# Message Generation #
#--------------------#

set (msgs
  msgs/Float.proto
  msgs/SensorImu.proto
  msgs/opticalFlow.proto
  msgs/lidar.proto
  msgs/CommandMotorSpeed.proto
  msgs/MotorSpeed.proto
  #msgs/Wind.proto
  msgs/sonarSens.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(mav_msgs SHARED ${PROTO_SRCS})

#---------#
# Plugins #
#---------#


link_libraries(mav_msgs)

# add_library(hello_world SHARED src/hello_world.cc)

add_library(rotors_gazebo_gimbal_controller_plugin SHARED src/gazebo_gimbal_controller_plugin.cpp)
target_link_libraries(rotors_gazebo_gimbal_controller_plugin ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
# add_dependencies(rotors_gazebo_gimbal_controller_plugin)

add_library(rotors_gazebo_controller_interface SHARED src/gazebo_controller_interface.cpp)
add_library(rotors_gazebo_motor_model SHARED src/gazebo_motor_model.cpp)
add_library(rotors_gazebo_multirotor_base_plugin SHARED src/gazebo_multirotor_base_plugin.cpp)
add_library(rotors_gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)
add_library(gazebo_opticalFlow_plugin SHARED src/gazebo_opticalFlow_plugin.cpp)
target_link_libraries(gazebo_opticalFlow_plugin ${OpticalFlow_LIBS})
add_library(gazebo_lidar_plugin SHARED src/gazebo_lidar_plugin.cpp)
add_library(rotors_gazebo_mavlink_interface SHARED src/gazebo_mavlink_interface.cpp src/geo_mag_declination.cpp)
#add_library(rotors_gazebo_wind_plugin SHARED src/gazebo_wind_plugin.cpp)
add_library(gazebo_sonar_plugin SHARED src/gazebo_sonar_plugin.cpp)
add_library(gazebo_uuv_plugin SHARED src/gazebo_uuv_plugin.cpp)

set(plugins
  rotors_gazebo_controller_interface
  rotors_gazebo_motor_model
  rotors_gazebo_multirotor_base_plugin
  rotors_gazebo_imu_plugin
  gazebo_opticalFlow_plugin
  gazebo_lidar_plugin
  rotors_gazebo_mavlink_interface
  #rotors_gazebo_wind_plugin
  gazebo_sonar_plugin
  gazebo_uuv_plugin
  )

# ROS mavlink version not compatible with geotagged images plugin
if (NOT roscpp_FOUND)
  add_library(gazebo_geotagged_images_plugin SHARED src/gazebo_geotagged_images_plugin.cpp)
  list(APPEND plugins gazebo_geotagged_images_plugin)
endif()

if (GSTREAMER_FOUND)
  add_library(gazebo_gst_camera_plugin SHARED src/gazebo_gst_camera_plugin.cpp)
  set(plugins
    ${plugins}
    gazebo_gst_camera_plugin
  )
  message(STATUS "Found GStreamer: adding gst_camera_plugin")
endif()

# Linux is not consistent with plugin availability, even on Gazebo 7
#if("${GAZEBO_VERSION}" VERSION_LESS "7.0")
  add_library(LiftDragPlugin SHARED src/liftdrag_plugin/liftdrag_plugin.cpp)
  list(APPEND plugins LiftDragPlugin)
#endif()

foreach(plugin ${plugins})
  add_dependencies(${plugin} mav_msgs)
endforeach()

#############
## Install ##
#############

set(PLUGIN_PATH ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/plugins)
set(MODEL_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/models)
set(RESOURCE_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/models/.DS_Store)
file(GLOB models_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/models/*)

file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/worlds/.DS_Store)
file(GLOB worlds_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/worlds/*)

install(TARGETS ${plugins} mav_msgs DESTINATION ${PLUGIN_PATH})
install(DIRECTORY ${models_list} DESTINATION ${MODEL_PATH})
install(FILES ${worlds_list} DESTINATION ${RESOURCE_PATH}/worlds)

configure_file(src/setup.sh.in "${CMAKE_CURRENT_BINARY_DIR}/setup.sh" @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${RESOURCE_PATH})


#############
## Testing ##
#############

###############
## Packaging ##
###############

set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_CONTACT pxusers@googlegroups.com)
set(DEBIAN_PACKAGE_DEPENDS "")
set(RPM_PACKAGE_DEPENDS "")

set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEBIAN_PACKAGE_DEPENDS})
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "gazebo plugins for px4 sitl.")

set(CPACK_RPM_PACKAGE_REQUIRES "${DEBIAN_PACKAGE_DEPENDS}")
set(CPACK_RPM_PACKAGE_DESCRIPTION "Gazebo plugins for px4 sitl.")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")

include(CPack)
# vim: set et fenc= ff=unix sts=0 sw=2 ts=2 :
