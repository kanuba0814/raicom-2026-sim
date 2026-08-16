cmake_minimum_required(VERSION 3.0.2)
project(raicom_2026_sim)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
    scripts/arm_settled_probe.py
    scripts/red_blue_field_texture.py
    scripts/spawn_oryxbot.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
