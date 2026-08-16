# ALVAR detector provenance

`../ar_track_alvar` and `../ar_track_alvar_msgs` are copied unchanged from:

```text
vendor_upstream/oryxbot_ws/src/ar_track_alvar/ar_track_alvar
vendor_upstream/oryxbot_ws/src/ar_track_alvar/ar_track_alvar_msgs
```

They provide the already-validated ROS Noetic `individualMarkersNoKinect` detector and message type required by this passive vision package. The copy makes clean 2026 workspace builds reproducible; it does not import legacy task coordinates, marker semantics, motion control, or direct velocity publication.

The workbench IDs, marker size, view pose, and marker-to-slot offsets remain `SIM_WORKING_ASSUMPTION` configuration in `config/workbench_slots.yaml` and the launch file.
