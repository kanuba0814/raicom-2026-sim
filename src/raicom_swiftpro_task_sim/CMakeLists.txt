cmake_minimum_required(VERSION 3.0.2)
project(raicom_swiftpro_task_sim)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/swiftpro_sim_motion_adapter.py
  scripts/swiftpro_sim_middle_observation.py
  scripts/swiftpro_sim_relative_tcp.py
  scripts/swiftpro_sim_slot1_hover.py
  scripts/swiftpro_sim_slot_tour.py
  scripts/swiftpro_sim_workbench_height_oracle.py
  scripts/swiftpro_sim_cartesian_circles.py
  scripts/swiftpro_sim_two_block_round_trip.py
  scripts/swiftpro_sim_camera_payload_pick.py
  scripts/swiftpro_sim_camera_payload_fixture.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_motion_adapter.py)
  catkin_add_nosetests(test/test_arm_backend.py)
  catkin_add_nosetests(test/test_slot_pick_place.py)
  catkin_add_nosetests(test/test_camera_payload_pick.py)
endif()
