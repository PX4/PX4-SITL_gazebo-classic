
if(ENABLE_UNIT_TESTS)

    # Gimbal controller plugin
    add_executable(gazebo_gimbal_controller_plugin_test gazebo_gimbal_controller_plugin_test.cpp)

    target_link_libraries(gazebo_gimbal_controller_plugin_test
        PRIVATE gazebo_gimbal_controller_plugin
                ${GTEST_BOTH_LIBRARIES}
                ${CMAKE_THREAD_LIBS_INIT})

    add_test(gazebo_gimbal_controller_plugin_test gazebo_gimbal_controller_plugin_test)

    # GPS plugin
    add_executable(gazebo_gps_plugin_test gazebo_gps_plugin_test.cpp)

    target_link_libraries(gazebo_gps_plugin_test
        PRIVATE gazebo_gps_plugin
                ${GTEST_BOTH_LIBRARIES}
                ${CMAKE_THREAD_LIBS_INIT})

    add_test(gazebo_gps_plugin_test gazebo_gps_plugin_test)

endif(ENABLE_UNIT_TESTS)
