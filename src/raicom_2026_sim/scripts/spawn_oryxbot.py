#!/usr/bin/env python3
"""Spawn the official Oryx URDF and tolerate Gazebo's early insertion race."""

import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import quaternion_from_euler


class OryxbotSpawner:
    """Waits for an actual model-state confirmation after a Gazebo spawn call."""

    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "oryxbot")
        self.robot_namespace = rospy.get_param("~robot_namespace", "/")
        self.gazebo_namespace = rospy.get_param("~gazebo_namespace", "/gazebo")
        self.spawn_timeout_seconds = float(rospy.get_param("~spawn_timeout", 30.0))
        self.model_present = False
        self.model_states_received = False
        rospy.Subscriber(
            f"{self.gazebo_namespace}/model_states",
            ModelStates,
            self._on_model_states,
            queue_size=1,
        )

    def _on_model_states(self, message):
        self.model_states_received = True
        self.model_present = self.model_name in message.name

    def _wait_for_model(self, timeout_seconds):
        deadline = time.monotonic() + timeout_seconds
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.model_present:
                return True
            # This runs before and during /clock setup.  Wall sleep avoids a
            # startup deadlock when simulated time has not advanced yet.
            time.sleep(0.05)
        return self.model_present

    def _wait_for_model_states(self):
        deadline = time.monotonic() + self.spawn_timeout_seconds
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.model_states_received:
                return True
            time.sleep(0.05)
        return self.model_states_received

    def run(self):
        if not self._wait_for_model_states():
            rospy.logerr("Timed out waiting for Gazebo model_states")
            return 1
        if self.model_present:
            rospy.loginfo("Model [%s] is already present", self.model_name)
            return 0

        robot_description = rospy.get_param("robot_description", "")
        if not robot_description:
            rospy.logerr("robot_description is absent or empty")
            return 1

        x_position = float(rospy.get_param("~x", 0.0))
        y_position = float(rospy.get_param("~y", 0.0))
        z_position = float(rospy.get_param("~z", 0.05))
        yaw = float(rospy.get_param("~yaw", 1.57))
        orientation = quaternion_from_euler(0.0, 0.0, yaw)
        initial_pose = Pose(
            position=Point(x_position, y_position, z_position),
            orientation=Quaternion(*orientation),
        )

        service_name = f"{self.gazebo_namespace}/spawn_urdf_model"
        rospy.loginfo("Waiting for %s", service_name)
        try:
            rospy.wait_for_service(service_name, timeout=self.spawn_timeout_seconds)
            spawn_model = rospy.ServiceProxy(service_name, SpawnModel)
            response = spawn_model(
                self.model_name,
                robot_description,
                self.robot_namespace,
                initial_pose,
                "world",
            )
        except rospy.ROSException as error:
            rospy.logerr("Timed out waiting for Gazebo spawn service: %s", error)
            return 1
        except rospy.ServiceException as error:
            rospy.logerr("Gazebo spawn service failed: %s", error)
            return 1

        rospy.loginfo("Spawn request response: %s", response.status_message)
        if not response.success:
            # Gazebo classic can return this response before it has processed
            # its insertion queue.  Treat the model_states confirmation as the
            # source of truth instead of declaring a false negative.
            if self._wait_for_model(self.spawn_timeout_seconds):
                rospy.logwarn(
                    "Gazebo reported an early spawn failure, but model [%s] was confirmed",
                    self.model_name,
                )
                return 0
            rospy.logerr("Gazebo rejected the Oryx spawn request")
            return 1
        if not self._wait_for_model(self.spawn_timeout_seconds):
            rospy.logerr(
                "Gazebo accepted Oryx spawn but model [%s] did not appear within %.1f seconds",
                self.model_name,
                self.spawn_timeout_seconds,
            )
            return 1

        rospy.loginfo("Model [%s] confirmed in Gazebo model_states", self.model_name)
        return 0


def main():
    rospy.init_node("spawn_oryxbot")
    return OryxbotSpawner().run()


if __name__ == "__main__":
    sys.exit(main())
