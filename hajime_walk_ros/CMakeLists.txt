cmake_minimum_required(VERSION 3.0.2)
project(hajime_walk_ros)

add_compile_options(-std=c++17)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             sensor_msgs
             hajime_walk_msgs
             tf2
)

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    hajime_walk_msgs
    tf2
)

include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})

add_library(
  hr46
  src/hr46/acc.cpp
  src/hr46/calc_deg.cpp
  src/hr46/calc_mv.cpp
  src/hr46/cntr.cpp
  src/hr46/func.cpp
  src/hr46/gyro.cpp
  src/hr46/joy.cpp
  src/hr46/kine.cpp
  src/hr46/motion.cpp
  src/hr46/mvtbl.cpp
  src/hr46/serv.cpp
  src/hr46/sq_motion.cpp
  src/hr46/sq_ready.cpp
  src/hr46/sq_start.cpp
  src/hr46/sq_straight.cpp
  src/hr46/sq_walk.cpp
)
target_include_directories(hr46 PUBLIC include/${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  hr46
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/hr46
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS hr46
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
