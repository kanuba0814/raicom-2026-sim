cmake_minimum_required(VERSION 3.0.2)
project(raicom_safety)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
    scripts/cmd_vel_gate.py
    scripts/localization_watchdog.py
    scripts/slam_health_watchdog.py
    scripts/verify_zero_twist.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
