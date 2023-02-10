# Copyright(c) 2019 livoxtech limited.
cmake_minimum_required(VERSION 3.0)

# we only need livox message definition here
project(livox_ros_driver)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        std_msgs
        message_generation
        )

add_message_files(FILES
        CustomPoint.msg
        CustomMsg.msg
        )

generate_messages(DEPENDENCIES
        std_msgs
        )

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif ()