# Camera-only workbench slot localizer

This passive ROS Noetic package detects exactly four painted number circles in the hand-camera image with OpenCV Hough circles and green-annulus rejection. It combines calibrated pixel rays with the observed planes of ALVAR IDs 100 and 101, both expressed in `hand_cam_link`, then publishes `raicom_msgs/WorkbenchSlots` on `/raicom/workbench/slots`.

Slots are assigned strictly as IDs `1/3` across the upper row and `2/4` across the lower row from marker-local left/down coordinates. The authority uses no configured slot offsets, Gazebo state, models, or textures. Missing, stale, skewed, malformed, invalid-plane, ambiguous, or non-exactly-four inputs publish `valid=false` and a camera-only failure reason. It never emits arm, gripper, or base commands.

Start the complete passive graph with:

```bash
roslaunch raicom_workbench_vision hand_camera_workbench_vision.launch
```

The legacy marker-offset localizer and `config/workbench_slots.yaml` remain only for historical tests and are not launched by the hand-camera graph. Consumers must require `valid=true`, a fresh `hand_cam_link` header stamp, and their own TF, arm reachability, collision, and TCP checks.
