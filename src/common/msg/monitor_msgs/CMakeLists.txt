cmake_minimum_required(VERSION 2.8.3)
project(monitor_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        message_generation
        std_msgs
        geometry_msgs
        )

add_message_files(
        FILES
        fault_info.msg
        fault_vec.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)
