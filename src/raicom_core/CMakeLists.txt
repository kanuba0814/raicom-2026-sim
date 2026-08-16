cmake_minimum_required(VERSION 3.0.2)
project(raicom_core)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
    scripts/mode_manager.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
