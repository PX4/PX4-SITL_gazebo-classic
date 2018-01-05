# - Try to find  MAVLink
# Once done, this will define
#
#  MAVLINK_FOUND        : library found
#  MAVLINK_INCLUDE_DIRS : include directories
#  MAVLINK_VERSION      : version

# macros
include(FindPackageHandleStandardArgs)

# Check for ROS_DISTRO
find_program(ROSVERSION rosversion)
execute_process(COMMAND ${ROSVERSION} -d
    OUTPUT_VARIABLE ROS_DISTRO
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(_MAVLINK_EXTRA_SEARCH_PATHS
    ${CMAKE_SOURCE_DIR}/mavlink/
    ../../mavlink/
    ../mavlink/
    ${CATKIN_DEVEL_PREFIX}/
    /usr/
    /usr/local/
    /opt/ros/${ROS_DISTRO}/
    )

# find the include directory
find_path(_MAVLINK_INCLUDE_DIR
    NAMES mavlink/v1.0/mavlink_types.h mavlink/v2.0/mavlink_types.h
    HINTS ${_MAVLINK_EXTRA_SEARCH_PATHS}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
    )

# read the version
if (EXISTS ${_MAVLINK_INCLUDE_DIR}/mavlink/config.h)
    file(READ ${_MAVLINK_INCLUDE_DIR}/mavlink/config.h MAVLINK_CONFIG_FILE)
    string(REGEX MATCH "#define MAVLINK_VERSION[ ]+\"(([0-9]+\\.)+[0-9]+)\""
        _MAVLINK_VERSION_MATCH "${MAVLINK_CONFIG_FILE}")
    set(MAVLINK_VERSION "${CMAKE_MATCH_1}")
else()
    set(MAVLINK_VERSION "2.0")
endif()

# handle arguments
set(MAVLINK_INCLUDE_DIRS ${_MAVLINK_INCLUDE_DIR})
find_package_handle_standard_args(
    MAVLink
    REQUIRED_VARS MAVLINK_INCLUDE_DIRS MAVLINK_VERSION
    VERSION_VAR MAVLINK_VERSION
    )
