#!/usr/bin/env python3
"""Publish the Gazebo field texture as a SIM_ONLY RViz mesh marker."""

import rospy
import tf.transformations
from visualization_msgs.msg import Marker


def required_vector(name, length):
    value = rospy.get_param(name)
    if not isinstance(value, list) or len(value) != length:
        raise rospy.ROSInitException("{} must contain {} values".format(name, length))
    return [float(component) for component in value]


def main():
    rospy.init_node("raicom_red_blue_field_texture")

    position = required_vector("~texture_marker/position", 3)
    roll, pitch, yaw = required_vector("~texture_marker/orientation_rpy", 3)
    scale = required_vector("~texture_marker/scale", 3)
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    marker = Marker()
    marker.header.frame_id = rospy.get_param("~texture_marker/frame_id", "map")
    marker.ns = "sim_only_red_blue_field_texture"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = float(rospy.get_param("~texture_marker/alpha", 0.82))
    marker.mesh_resource = rospy.get_param("~texture_marker/mesh_resource")
    marker.mesh_use_embedded_materials = True

    topic = rospy.get_param("~texture_marker/topic", "/raicom/red_blue_field_texture")
    publisher = rospy.Publisher(topic, Marker, queue_size=1, latch=True)
    marker.header.stamp = rospy.Time.now()
    publisher.publish(marker)

    rospy.logwarn(
        "SIM_ONLY Gazebo texture loaded with its existing red/blue markings; "
        "the visualization does not invent or enforce referee boundaries"
    )
    rospy.spin()


if __name__ == "__main__":
    main()
