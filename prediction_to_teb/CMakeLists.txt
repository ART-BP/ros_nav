cmake_minimum_required(VERSION 3.0.2)
project(prediction_to_teb)

find_package(catkin REQUIRED COMPONENTS
  costmap_converter
  geometry_msgs
  rospy
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/publish_dynamic_obstacle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
