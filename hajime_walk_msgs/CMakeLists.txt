cmake_minimum_required(VERSION 3.0.2)
project(hajime_walk_msgs)

find_package(
  catkin REQUIRED
  COMPONENTS std_msgs
             message_generation
             actionlib_msgs
)

add_message_files(DIRECTORY msg)
add_action_files(DIRECTORY action)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    std_msgs
    message_runtime
)
