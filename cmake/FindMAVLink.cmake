# - Try to find  MAVLink
# Once done, this will define
#
#  MAVLINK_FOUND        : library found
#  MAVLINK_INCLUDE_DIRS : include directories
#  MAVLINK_VERSION      : version

# macros
include(FindPackageHandleStandardArgs)

set(_MAVLINK_EXTRA_SEARCH_PATHS
    /usr/
    /usr/local/
    /opt/local/
    mavlink/
    ../mavlink/
    ../../mavlink/
    )

# find the include directory
find_path(_MAVLINK_INCLUDE_DIR
    NAMES mavlink/v1.0/mavlink_types.h mavlink/v2.0/mavlink_types.h
    PATHS ${_MAVLINK_EXTRA_SEARCH_PATHS}
    PATH_SUFFIXES include
    )

# read the version
if (EXISTS ${_MAVLINK_INCLUDE_DIR}/mavlink/config.h)
    file(READ ${_MAVLINK_INCLUDE_DIR}/mavlink/config.h MAVLINK_CONFIG_FILE)
    string(REGEX MATCH "#define MAVLINK_VERSION[ ]+\"(([0-9]+\\.)+[0-9]+)\""
        _MAVLINK_VERSION_MATCH "${MAVLINK_CONFIG_FILE}")
    set(MAVLINK_VERSION "${CMAKE_MATCH_1}")
else()
    set(MAVLINK_VERSION "")
endif()

# handle arguments
set(MAVLINK_INCLUDE_DIRS ${_MAVLINK_INCLUDE_DIR})
find_package_handle_standard_args(
    MAVLink
    REQUIRED_VARS MAVLINK_INCLUDE_DIRS MAVLINK_VERSION
    VERSION_VAR MAVLINK_VERSION
    )
