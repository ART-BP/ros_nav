cmake_minimum_required(VERSION 3.0.2)
project(neupan_service)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  message_generation
  nav_msgs
  nav_core
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

add_service_files(
  FILES
  ComputeVelocity.srv
  SetPath.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES neupan_local_planner
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    message_runtime
    nav_msgs
    nav_core
    pluginlib
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(neupan_local_planner
  src/neupan_local_planner_ros.cpp
)
add_dependencies(neupan_local_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(neupan_local_planner
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/neupan_service_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/neupan_service_conda.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS neupan_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  neupan_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  config
  launch
  weights
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
