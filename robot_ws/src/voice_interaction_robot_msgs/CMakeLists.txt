cmake_minimum_required(VERSION 2.8.3)
project(voice_interaction_robot_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        diagnostic_msgs
        message_generation
        )

add_message_files(
    DIRECTORY msg
    FILES
    FulfilledVoiceCommand.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        diagnostic_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
)
