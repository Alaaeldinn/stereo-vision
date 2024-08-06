cmake_minimum_required(VERSION 3.0.2)
project(stereo_depth_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_install_python(PROGRAMS
  src/esp32_camera_node.py
  src/depth_map_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
