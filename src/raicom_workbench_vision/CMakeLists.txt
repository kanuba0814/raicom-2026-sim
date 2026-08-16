cmake_minimum_required(VERSION 3.0.2)
project(raicom_workbench_vision)
find_package(catkin REQUIRED COMPONENTS ar_track_alvar_msgs cv_bridge geometry_msgs rospy raicom_msgs sensor_msgs std_msgs tf)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS ar_track_alvar_msgs cv_bridge geometry_msgs rospy raicom_msgs sensor_msgs std_msgs tf)
catkin_install_python(PROGRAMS
  scripts/camera_workbench_slot_detector.py
  scripts/camera_payload_pick_detector.py
  scripts/hand_camera_workbench_overlay.py
  scripts/workbench_slot_localizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config src/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_payload_pick.py)
endif()
