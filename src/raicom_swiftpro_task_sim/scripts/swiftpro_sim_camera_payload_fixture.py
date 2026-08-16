#!/usr/bin/env python3
"""Spawn one fixed textured QR04 fixture for the SIM camera-pick demonstration."""
import os

import rospy
import rospkg
import tf
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import PointStamped, Pose

MODEL_NAME = "qr_block_04"
PAYLOAD_TOP_BASE_M = (0.23151, -0.13720, 0.04704)
PHYSICAL_THICKNESS_M = 0.010


def main():
    if rospy.get_param("/raicom/backend", "") != "SIM":
        raise SystemExit("REAL_BACKEND_REJECTED")
    package = rospkg.RosPack().get_path("raicom_qr_task_sim")
    source = PointStamped()
    source.header.frame_id = "Base"
    source.header.stamp = rospy.Time(0)
    source.point.x, source.point.y = PAYLOAD_TOP_BASE_M[:2]
    source.point.z = PAYLOAD_TOP_BASE_M[2] - PHYSICAL_THICKNESS_M / 2.0
    listener = tf.TransformListener()
    # SIM fixture contract: Gazebo world and odom coincide at spawn. ROS does
    # not publish a world TF, so transform the camera-calibrated Base point to
    # odom and pass those coordinates to Gazebo's world reference frame.
    listener.waitForTransform("odom", "Base", rospy.Time(0), rospy.Duration(3.0))
    world = listener.transformPoint("odom", source)
    model_uri = "file://" + os.path.join(package, "models", MODEL_NAME)
    script_uri = model_uri + "/materials/scripts"
    texture_uri = model_uri + "/materials/textures"
    sdf = ("<sdf version='1.6'><model name='%s'><static>false</static><link name='link'>"
           "<inertial><mass>0.04</mass><inertia><ixx>0.00002</ixx><iyy>0.00002</iyy>"
           "<izz>0.00002</izz></inertia></inertial><collision name='collision'><geometry>"
           "<box><size>0.04 0.04 0.01</size></box></geometry></collision>"
           "<visual name='body'><geometry><box><size>0.04 0.04 0.01</size></box></geometry></visual>"
           "<visual name='qr'><pose>0 0 0.0055 0 0 0</pose><geometry><box>"
           "<size>0.04 0.04 0.001</size></box></geometry><material><script>"
           "<uri>%s</uri><uri>%s</uri><name>QrBlock04/texture</name>"
           "</script></material></visual></link></model></sdf>" %
           (MODEL_NAME, script_uri, texture_uri))
    pose = Pose()
    pose.position = world.point
    pose.orientation.w = 1.0
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=3.0)
    result = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)(
        MODEL_NAME, sdf, "", pose, "world")
    if not result.success:
        raise SystemExit("SPAWN_FAILED: " + result.status_message)
    print("fixture_spawned=qr_block_04 payload=SIM_QR_04 frame=Base", flush=True)


if __name__ == "__main__":
    rospy.init_node("swiftpro_sim_camera_payload_fixture", anonymous=True)
    main()
