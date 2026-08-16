cmake_minimum_required(VERSION 3.0.2)
project(raicom_platform)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
    scripts/navigation_contract_probe.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
