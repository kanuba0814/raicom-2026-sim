cmake_minimum_required(VERSION 3.0.2)
project(raicom_charging_sim)
find_package(catkin REQUIRED COMPONENTS ar_track_alvar_msgs gazebo_msgs geometry_msgs nav_msgs rospy std_msgs tf)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS ar_track_alvar_msgs gazebo_msgs geometry_msgs nav_msgs rospy std_msgs tf)
catkin_install_python(PROGRAMS
    scripts/spawn_charge_pile.py
    scripts/sim_charging_coarse_gate.py
    scripts/sim_charging_dock.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
