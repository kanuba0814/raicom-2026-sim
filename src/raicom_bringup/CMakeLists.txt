cmake_minimum_required(VERSION 3.0.2)
project(raicom_bringup)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
    scripts/sim_mapping_sweep.py
    scripts/sim_frontier_survey.py
    scripts/frontier_analysis.py
    scripts/sim_navigation_yaw_probe.py
    scripts/sim_marker_baseline_align.py
    scripts/sim_relative_move.py
    scripts/sim_workbench_marker_align.py
    scripts/sim_navigation_observe.py
    scripts/sim_navigation_scenario.py
    scripts/sim_navigation_control.py
    scripts/sim_navigation_fault_probe.py
    scripts/sim_readonly_runtime.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
