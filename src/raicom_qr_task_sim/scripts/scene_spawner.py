#!/usr/bin/env python3
"""Spawn only the fixed QR-2x2 SIM model set; never delete or mutate models."""
import os
import rospy
import rospkg
import yaml
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from raicom_qr_task_sim.logic import load_sim_config, render_scene_sdf, rpy_quaternion

MODELS = ("qr_pick_tray", "qr_place_tray", "qr_block_01", "qr_block_02", "qr_block_03", "qr_block_04")

class SceneSpawner:
    def __init__(self):
        self.names = set()
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._states, queue_size=1)

    def _states(self, message):
        self.names = set(message.name)

    def run(self):
        config_path = rospy.get_param("~config")
        with open(config_path, encoding="utf-8") as stream:
            config = load_sim_config(yaml.safe_load(stream))
        rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5.0)
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        package = rospkg.RosPack().get_path("raicom_qr_task_sim")
        poses = {"qr_pick_tray": config["trays"]["pick"], "qr_place_tray": config["trays"]["place"]}
        poses.update({"qr_block_" + target["target_id"][-2:]: config["pick_poses"][target["target_id"]] for target in config["qr_targets"]})
        for model in MODELS:
            if model in self.names:
                rospy.logerr("fail closed: fixed model name already exists: %s", model)
                raise RuntimeError("model name conflict")
            pose_config = poses[model]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pose_config["xyz"]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rpy_quaternion(pose_config["rpy"])
            model_uri = "file://%s" % os.path.join(package, "models", model)
            response = spawn(model, render_scene_sdf(model, config, model_uri), "", pose, "world")
            if not response.success:
                raise RuntimeError("spawn failed for %s: %s" % (model, response.status_message))
            try:
                confirmation = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=3.0)
            except rospy.ROSException as exc:
                raise RuntimeError("spawn confirmation unavailable for %s: %s" % (model, exc))
            if model not in confirmation.name:
                raise RuntimeError("spawn confirmation missing fixed model %s" % model)
            self.names = set(confirmation.name)
        rospy.loginfo("spawned fixed SIM-only QR 2x2 scene")

if __name__ == "__main__":
    rospy.init_node("qr_2x2_scene_spawner")
    SceneSpawner().run()
