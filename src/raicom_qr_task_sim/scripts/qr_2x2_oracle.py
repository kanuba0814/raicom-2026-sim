#!/usr/bin/env python3
"""Independent SIM-only placement oracle; publishes reports and accepts no control input."""
import rospy
import yaml
from gazebo_msgs.msg import ModelStates
from raicom_msgs.msg import QrOracleReport
from raicom_qr_task_sim.logic import complete_unique, load_sim_config, quaternion_rpy, slot_match

class Oracle:
    def __init__(self):
        with open(rospy.get_param("~config"), encoding="utf-8") as stream:
            self.config = load_sim_config(yaml.safe_load(stream))
        self.publisher = rospy.Publisher("/raicom/qr/oracle_report", QrOracleReport, queue_size=1)
        self.valid_since = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._states, queue_size=1)

    def _states(self, message):
        poses = dict(zip(message.name, message.pose))
        report = QrOracleReport()
        report.header.stamp = rospy.Time.now()
        report.slot_ids = list(self.config["slots"].keys())
        report.target_ids = [self.config["slots"][slot_id]["target_id"] for slot_id in report.slot_ids]
        matches = {}
        for slot_id in report.slot_ids:
            target_id = self.config["slots"][slot_id]["target_id"]
            pose = poses.get("qr_block_" + target_id[-2:])
            if pose is None:
                matches[slot_id] = False
                continue
            quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            try:
                rpy = quaternion_rpy(quaternion)
            except ValueError:
                matches[slot_id] = False
                continue
            matches[slot_id] = slot_match((pose.position.x, pose.position.y, pose.position.z), rpy, self.config["slots"][slot_id], self.config["oracle"] )
        report.slot_valid = [matches[slot] for slot in report.slot_ids]
        report.complete_2x2 = complete_unique(matches)
        now = rospy.Time.now()
        if report.complete_2x2:
            self.valid_since = self.valid_since or now
        else:
            self.valid_since = None
        report.stable = bool(self.valid_since and (now - self.valid_since).to_sec() >= self.config["oracle"]["stable_dwell_s"])
        report.detail = "SIM_WORKING_ASSUMPTION; read-only Gazebo oracle"
        self.publisher.publish(report)

if __name__ == "__main__":
    rospy.init_node("qr_2x2_oracle")
    Oracle()
    rospy.spin()
