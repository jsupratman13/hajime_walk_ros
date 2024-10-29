cmake_minimum_required(VERSION 3.0.2)
project(rqt_hajime_motion_creator)

find_package(
  catkin REQUIRED
  COMPONENTS rospy
             rqt_gui
             rqt_gui_py
             qt_gui
)

catkin_python_setup()

catkin_package()

install(
  FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS scripts/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
