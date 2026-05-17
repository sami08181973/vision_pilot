add_executable(VisionPilot
        vision_pilot.cpp
        vision_pilot_config.cpp
)

target_include_directories(VisionPilot PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(VisionPilot
        PRIVATE
        engine
        models
        can_interface
        v4l2_interface
        camera_subscriber
        visualization
        rclcpp::rclcpp
)