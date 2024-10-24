cmake_minimum_required(VERSION 3.0.2)
project(hajime_walk_teleop)

find_package(
  catkin REQUIRED
  COMPONENTS hajime_walk_msgs
             joy
             rospy
             sensor_msgs
             std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    hajime_walk_msgs
    joy
    rospy
    sensor_msgs
    std_msgs
)
