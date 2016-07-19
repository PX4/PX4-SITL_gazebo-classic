cmake_minimum_required(VERSION 2.8.6 FATAL_ERROR)

if(NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install path prefix" FORCE)
endif()
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

project(mavlink_sitl_gazebo)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

#----------------------------------------------------------------------------
# Includes

include(GNUInstallDirs)

#----------------------------------------------------------------------------
# Parameters

set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 0)

#----------------------------------------------------------------------------
# Derived Variables

set(PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")


#----------------------------------------------------------------------------
# Find Dependencies:

# Add search directory for CMake on OS X
list(APPEND CMAKE_MODULE_PATH /usr/local/share/cmake/Modules)

find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
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
pkg_check_modules(PROTOBUF REQUIRED protobuf)
if (NOT "${PROTOBUF_VERSION}" VERSION_LESS "3.0.0" OR
  "${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version ${PROTOBUF_VERSOIN} not compatible, must be >= 2.5.0 and < 3.0.0")
endif()

find_package(Boost 1.40 COMPONENTS system thread REQUIRED )

#----------------------------------------------------------------------------
# Build:

add_definitions(
  -std=c++11
  # Shut up warnings about std::binder1st, std::binder2nd.
  -Wno-deprecated-declarations
)

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
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  ${Boost_INCLUDE_DIR}
  # Workaround for Eigen3
  ${Boost_INCLUDE_DIR}/eigen3
  ${EIGEN3_INCLUDE_DIRS}/eigen3
  # Workaround for OGRE include dirs on Mac OS
  /usr/local/include/OGRE
  /usr/local/include/OGRE/Paging
  )

set(enable_mavlink_interface "true")
set(enable_ground_truth "false")
set(enable_logging "false")
set(enable_camera "false")
set(rotors_description_dir "${CMAKE_SOURCE_DIR}/models/rotors_description")
set(scripts_dir "${CMAKE_SOURCE_DIR}/scripts")
add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/models/iris/iris.sdf
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMAND rm -f ${CMAKE_SOURCE_DIR}/models/iris/iris.sdf
  COMMAND python ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/iris_base.urdf  ${rotors_description_dir}/urdf/iris_base.xacro enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir} 
  COMMAND gz sdf -p  ${rotors_description_dir}/urdf/iris_base.urdf >> ${CMAKE_SOURCE_DIR}/models/iris/iris.sdf
  COMMAND rm -f ${rotors_description_dir}/urdf/iris_base.urdf
  )
add_custom_target(sdf ALL DEPENDS ${CMAKE_SOURCE_DIR}/models/iris/iris.sdf)


link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)

add_subdirectory(msgs)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_library(hello_world SHARED src/hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})


add_library(rotors_gazebo_controller_interface SHARED src/gazebo_controller_interface.cpp)
target_link_libraries(rotors_gazebo_controller_interface ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE} ${PROTOBUF_LIBRARY} mav_msgs)
add_dependencies(rotors_gazebo_controller_interface mav_msgs)
list(APPEND installed_libraries rotors_gazebo_controller_interface)

add_library(rotors_gazebo_motor_model SHARED src/gazebo_motor_model.cpp)
target_link_libraries(rotors_gazebo_motor_model mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(rotors_gazebo_motor_model mav_msgs)
list(APPEND installed_libraries rotors_gazebo_motor_model)

add_library(rotors_gazebo_multirotor_base_plugin SHARED src/gazebo_multirotor_base_plugin.cpp)
target_link_libraries(rotors_gazebo_multirotor_base_plugin mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(rotors_gazebo_multirotor_base_plugin mav_msgs)
list(APPEND installed_libraries rotors_gazebo_multirotor_base_plugin)

add_library(rotors_gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)
target_link_libraries(rotors_gazebo_imu_plugin mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(rotors_gazebo_imu_plugin mav_msgs)
list(APPEND installed_libraries rotors_gazebo_imu_plugin)

add_library(gazebo_opticalFlow_plugin SHARED src/gazebo_opticalFlow_plugin.cpp)
target_link_libraries(gazebo_opticalFlow_plugin mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE} ${OpenCV_LIBS})
add_dependencies(gazebo_opticalFlow_plugin mav_msgs)
list(APPEND installed_libraries gazebo_opticalFlow_plugin)

add_library(gazebo_lidar_plugin SHARED src/gazebo_lidar_plugin.cpp)
target_link_libraries(gazebo_lidar_plugin mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(gazebo_lidar_plugin mav_msgs)
list(APPEND installed_libraries gazebo_lidar_plugin)

add_library(rotors_gazebo_mavlink_interface SHARED src/gazebo_mavlink_interface.cpp src/geo_mag_declination.cpp)
target_link_libraries(rotors_gazebo_mavlink_interface mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(rotors_gazebo_mavlink_interface mav_msgs)
list(APPEND installed_libraries rotors_gazebo_mavlink_interface)

if("${GAZEBO_VERSION}" VERSION_LESS "7.0")
  add_library(LiftDragPlugin SHARED src/liftdrag_plugin/liftdrag_plugin.cpp)
  target_link_libraries(LiftDragPlugin mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
  add_dependencies(LiftDragPlugin mav_msgs)
  list(APPEND installed_libraries LiftDragPlugin)
endif()

#----------------------------------------------------------------------------
# Install

set(PLUGIN_DIR ${CMAKE_INSTALL_FULL_LIBDIR}/gazebo-${GAZEBO_MAJOR_VERSION}/plugins)
set(MODEL_DIR ${CMAKE_INSTALL_FULL_DATADIR}/gazebo-${GAZEBO_MAJOR_VERSION}/models)
set(WORLD_DIR ${CMAKE_INSTALL_FULL_DATADIR}/gazebo-${GAZEBO_MAJOR_VERSION}/worlds)

file(GLOB models ${PROJECT_SOURCE_DIR}/models/*)
file(GLOB worlds ${PROJECT_SOURCE_DIR}/worlds/*)

install(TARGETS ${installed_libraries} DESTINATION ${PLUGIN_DIR})
install(DIRECTORY ${models} DESTINATION ${MODEL_DIR})
install(FILES ${worlds} DESTINATION ${WORLD_DIR})

#----------------------------------------------------------------------------
# Package Creation:

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
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME_LOWER}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME_LOWER}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")

include(CPack)
# vim: set et fenc= ff=unix sts=0 sw=2 ts=2 :

