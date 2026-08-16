#!/usr/bin/env python3
"""Read-only guard for one fixed SIM inward lateral recovery."""
import json
import math

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid

from raicom_round_table_navigation.boundary_guard import (
    Footprint, guard_allowed_half_plane, guard_footprint,
)
from raicom_round_table_navigation.docking_adapter import map_grid_from_message
from raicom_round_table_navigation.tangent_boundary import TangentBoundary


def main():
    rospy.init_node("round_table_inward_guard_probe", anonymous=True)
    boundary = TangentBoundary(3.325, 1.0, 0.0, 1.0, -1)
    message = rospy.wait_for_message("/raicom/team_boundary_mask", OccupancyGrid, timeout=5.0)
    grid = map_grid_from_message(message)
    if grid is None:
        raise RuntimeError("TEAM_BOUNDARY_MASK_INVALID")
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(3.0))
    listener = tf2_ros.TransformListener(buffer)
    rospy.sleep(0.3)
    pose = buffer.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(1.0))
    q = pose.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    # Fixed base vy=-0.08 m/s moves toward the blue allowed half-plane interior.
    vx_map = math.sin(yaw) * 0.08
    vy_map = -math.cos(yaw) * 0.08
    footprint = Footprint(0.24, 0.24, 0.20, 0.20)
    occupancy = guard_footprint(
        grid, footprint, pose.transform.translation.x, pose.transform.translation.y,
        yaw, vx_map, vy_map, reaction_time_s=0.20, deceleration_mps2=0.50)
    half_plane = guard_allowed_half_plane(
        boundary, footprint, pose.transform.translation.x, pose.transform.translation.y,
        yaw, vx_map, vy_map, reaction_time_s=0.20, deceleration_mps2=0.50,
        safety_margin_m=0.01)
    report = {
        "simulation_only": True,
        "motion_issued": False,
        "fixed_lateral_m": -0.08,
        "occupancy_safe": occupancy.safe,
        "occupancy_reason": occupancy.reason,
        "half_plane_safe": half_plane.safe,
        "half_plane_reason": half_plane.reason,
        "pose_map": {"x_m": pose.transform.translation.x,
                     "y_m": pose.transform.translation.y, "yaw_rad": yaw},
    }
    print(json.dumps(report, sort_keys=True))
    if not occupancy.safe or not half_plane.safe:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
