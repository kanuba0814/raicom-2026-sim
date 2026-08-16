cmake_minimum_required(VERSION 3.0.2)
project(raicom_round_table_navigation)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs geometry_msgs nav_msgs rosgraph rospy sensor_msgs std_msgs tf2_geometry_msgs tf2_ros)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS diagnostic_msgs geometry_msgs nav_msgs rosgraph rospy sensor_msgs std_msgs tf2_geometry_msgs tf2_ros)
catkin_install_python(PROGRAMS scripts/round_table_detector_node.py scripts/round_table_docking_adapter_node.py scripts/round_table_managed_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
