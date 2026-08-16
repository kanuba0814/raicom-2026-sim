#!/usr/bin/env python3
"""SIM-only passive slot localizer. It publishes poses and never commands motion."""
import rospy
import yaml
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
from raicom_msgs.msg import WorkbenchSlots
from raicom_workbench_vision.geometry import MarkerObservation, WorkbenchLocalizer


class WorkbenchSlotNode:
    def __init__(self) -> None:
        config_path = rospy.get_param("~config")
        with open(config_path, "r", encoding="utf-8") as config_file:
            self.localizer = WorkbenchLocalizer(yaml.safe_load(config_file))
        marker_topic = rospy.get_param("~marker_topic", "/ar_pose_marker")
        self.publisher = rospy.Publisher("/raicom/workbench/slots", WorkbenchSlots, queue_size=1)
        self.subscriber = rospy.Subscriber(marker_topic, AlvarMarkers, self._markers, queue_size=1)

    def _markers(self, message: AlvarMarkers) -> None:
        observations = []
        for marker in message.markers:
            pose = marker.pose.pose
            observations.append(MarkerObservation(marker.id, marker.header.frame_id,
                marker.header.stamp.to_sec(), (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)))
        estimate = self.localizer.process(observations, rospy.Time.now().to_sec())
        output = WorkbenchSlots()
        marker_ids = self.localizer.config["marker_ids"]
        required_ids = (marker_ids["top"], marker_ids["bottom"])
        source_stamps = [marker.header.stamp for marker in message.markers if marker.id in required_ids]
        output.header.stamp = max(source_stamps) if source_stamps else message.header.stamp
        output.header.frame_id = estimate.frame_id
        output.valid = estimate.valid
        output.stable_frames = estimate.stable_frames
        output.marker_separation_m = estimate.separation_m
        output.normal_disagreement_rad = estimate.normal_disagreement_rad
        output.detail = "config=SIM_WORKING_ASSUMPTION; " + estimate.detail
        if estimate.valid:
            for index, (position, orientation) in enumerate(estimate.slots):
                slot = Pose()
                slot.position.x, slot.position.y, slot.position.z = position
                slot.orientation.x, slot.orientation.y, slot.orientation.z, slot.orientation.w = orientation
                output.slots[index] = slot
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("workbench_slot_localizer")
    WorkbenchSlotNode()
    rospy.spin()
