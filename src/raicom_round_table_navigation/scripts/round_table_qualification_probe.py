#!/usr/bin/env python3
"""Read-only SIM probe for the explicit round-table boundary contract."""
import argparse
import json
import math

import rospy
import tf2_geometry_msgs  # Registers PointStamped transforms.
import tf2_ros
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Bool

from raicom_round_table_navigation.tangent_boundary import (
    TangentBoundary, feasible_center_offset_interval,
    qualify_tangent_lead_in, signed_normal_distance,
)


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--boundary-point-x", type=float, required=True)
    parser.add_argument("--boundary-point-y", type=float, required=True)
    parser.add_argument("--boundary-tangent-x", type=float, required=True)
    parser.add_argument("--boundary-tangent-y", type=float, required=True)
    parser.add_argument("--allowed-normal-sign", type=int, choices=(-1, 1), required=True)
    parser.add_argument("--robot-center-offset", type=float, required=True)
    parser.add_argument("--cross-track-tolerance", type=float, default=0.05)
    parser.add_argument("--heading-tolerance", type=float, default=0.10)
    parser.add_argument("--circle-tangent-tolerance", type=float, default=0.08)
    parser.add_argument("--minimum-circle-ahead", type=float, default=0.12)
    parser.add_argument("--circle-uncertainty", type=float, default=0.005)
    parser.add_argument("--boundary-margin", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=8.0)
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("round_table_qualification_probe", anonymous=True)
    args = parse_arguments()
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
    listener = tf2_ros.TransformListener(buffer)
    rospy.sleep(0.3)
    circle = rospy.wait_for_message("/round_table/circle", Vector3Stamped, timeout=args.timeout)
    valid = rospy.wait_for_message("/round_table/valid", Bool, timeout=args.timeout)
    if not valid.data:
        raise RuntimeError("ROUND_TABLE_OBSERVATION_INVALID")
    point = PointStamped()
    point.header = circle.header
    point.point.x = circle.vector.x
    point.point.y = circle.vector.y
    point_map = buffer.transform(point, "map", rospy.Duration(1.0))
    pose = buffer.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(1.0))
    q = pose.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    boundary = TangentBoundary(
        args.boundary_point_x, args.boundary_point_y,
        args.boundary_tangent_x, args.boundary_tangent_y,
        args.allowed_normal_sign)
    qualification = qualify_tangent_lead_in(
        boundary, pose.transform.translation.x, pose.transform.translation.y, yaw,
        point_map.point.x, point_map.point.y, circle.vector.z,
        args.robot_center_offset, args.cross_track_tolerance,
        args.heading_tolerance, args.circle_tangent_tolerance,
        args.minimum_circle_ahead)
    circle_allowed_distance = args.allowed_normal_sign * signed_normal_distance(
        boundary, point_map.point.x, point_map.point.y)
    desired_feasible = feasible_center_offset_interval(
        circle_allowed_distance, circle.vector.z, 0.20, 0.03, 0.05,
        args.circle_uncertainty, args.boundary_margin)
    boundary_safe_feasible = feasible_center_offset_interval(
        circle_allowed_distance, circle.vector.z, 0.20, 0.055, 0.070,
        args.circle_uncertainty, args.boundary_margin)
    output = {
        "simulation_only": True,
        "motion_issued": False,
        "robot_map": {"x_m": pose.transform.translation.x,
                      "y_m": pose.transform.translation.y, "yaw_rad": yaw},
        "circle_map": {"x_m": point_map.point.x, "y_m": point_map.point.y,
                       "radius_m": circle.vector.z,
                       "allowed_signed_distance_m": circle_allowed_distance},
        "qualification": qualification.__dict__,
        "desired_3_5cm_offset_interval": desired_feasible.__dict__,
        "boundary_safe_5_5_7cm_offset_interval": boundary_safe_feasible.__dict__,
        "assumption_status": "SIM_WORKING_ASSUMPTION",
    }
    print(json.dumps(output, sort_keys=True))


if __name__ == "__main__":
    main()
