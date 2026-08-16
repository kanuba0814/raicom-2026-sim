#!/usr/bin/env python3
"""Read-only ROS HMI presenter; subscribes to semantic mission snapshots only."""
from __future__ import print_function
import rospy
from std_msgs.msg import String
from raicom_hmi.presenter import render_for_log


def main():
    rospy.init_node("raicom_hmi_presenter")
    def display(message):
        # Log only: this node creates no publishers, services, or command clients.
        rospy.loginfo("HMI %s", render_for_log(message.data))
    rospy.Subscriber(rospy.get_param("~mission_snapshot_topic", "/raicom/mission/snapshot"), String, display, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()
