from pathlib import Path
from types import SimpleNamespace

from raicom_round_table_navigation.docking_adapter import (AdmissionSnapshot, FaultLatch, ObservationDropoutGate,
    angular_stopping_horizon_s, base_velocity_to_map, canonical_occupancy_grid_sha256,
    map_grid_from_message, safe_suggestion, validate_admission)
from raicom_round_table_navigation.docking_geometry import DockingPhase, VelocitySuggestion
from raicom_round_table_navigation.boundary_guard import Footprint, OccupancyGridData, guard_footprint


def admitted(**changes):
    values = dict(enabled=True, backend="SIM", team="blue", expected_map_image="/maps/blue_table.pgm",
                  expected_grid_hash="sha256:test", map_identity_recorded=True, map_identity_matches=True,
                  observation_fresh=True, observation_stamp_fresh=True, valid_fresh=True, valid_paired=True,
                  scan_fresh=True, scan_stamp_fresh=True, map_fresh=True, tf_fresh=True, map_valid=True,
                  gate_is_unique_cmd_vel_publisher=True, relative_publishers_are_self_only=True,
                  move_base_absent=True, nav_has_no_publishers=True)
    values.update(changes)
    return AdmissionSnapshot(**values)


def test_admission_requires_explicit_sim_contract_and_gate():
    assert validate_admission(admitted()) == "ADMITTED"
    assert validate_admission(admitted(enabled=False)) == "DISABLED"
    assert validate_admission(admitted(team="")) == "TEAM_NOT_EXPLICIT"
    assert validate_admission(admitted(expected_map_image="/maps/red.pgm")) == "MAP_IMAGE_TEAM_MISMATCH"
    assert validate_admission(admitted(gate_is_unique_cmd_vel_publisher=False)) == "CMD_VEL_GATE_NOT_UNIQUE"


def test_map_conversion_rejects_bad_frame_cells_and_accepts_valid_grid():
    origin = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0), orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    valid = SimpleNamespace(header=SimpleNamespace(frame_id="map"), info=SimpleNamespace(resolution=.1, width=2, height=2, origin=origin), data=[0, 0, 0, 0])
    result = map_grid_from_message(valid)
    assert result.width == 2 and result.cells == [0, 0, 0, 0]
    valid.header.frame_id = "odom"
    assert map_grid_from_message(valid) is None
    valid.header.frame_id = "map"; valid.data = [0]
    assert map_grid_from_message(valid) is None


def test_canonical_grid_hash_is_stable_and_changes_for_one_cell():
    origin = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=0.0), orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    grid = SimpleNamespace(header=SimpleNamespace(frame_id="map"), info=SimpleNamespace(resolution=.1, width=2, height=2, origin=origin), data=[0, 0, 0, 0])
    stable = canonical_occupancy_grid_sha256(grid)
    assert stable == canonical_occupancy_grid_sha256(grid)
    grid.data[2] = 1
    assert canonical_occupancy_grid_sha256(grid) != stable


def test_base_velocity_map_rotation_and_angular_stop_model():
    vx, vy = base_velocity_to_map(.05, 0., 3.141592653589793 / 2.)
    assert abs(vx) < 1e-12 and abs(vy - .05) < 1e-12
    vx, vy = base_velocity_to_map(0., -.05, 3.141592653589793 / 2.)
    assert abs(vx - .05) < 1e-12 and abs(vy) < 1e-12
    assert angular_stopping_horizon_s(.2, .2, .4) == .45


def test_yaw_pi_over_two_map_velocity_drives_translation_guard_into_map_y():
    grid = OccupancyGridData(0., 0., .01, 1000, 1000, [0] * 1000000)
    # At yaw=pi/2 base-forward must be checked toward +map-y, not +map-x.
    grid.cells[514 * grid.width + 500] = 100
    footprint = Footprint(.1, .1, .1, .1)
    vx, vy = base_velocity_to_map(.1, 0., 3.141592653589793 / 2.)
    assert not guard_footprint(grid, footprint, 5., 5., 3.141592653589793 / 2., vx, vy, .2, .3).safe


def test_managed_run_finally_covers_admission_and_boundary_limit_is_not_success():
    source = (Path(__file__).parents[1] / "scripts" / "round_table_managed_run.py").read_text()
    assert source.index("try:") < source.index("REAL_BACKEND_REJECTED")
    assert source.index("try:") < source.index("SAFETY_REQUALIFICATION_FAILED")
    assert source.index("try:") < source.index("DETECTOR_NOT_STABLY_VALID")
    assert 'raise RuntimeError("SAFE_STOP_BOUNDARY_LIMITED:' in source
    finally_block = source[source.index("finally:"):]
    assert 'enabled", False' in finally_block
    assert "soft_estop.publish(Bool(True))" in finally_block


def test_launch_removes_tangent_and_boundary_half_plane_motion_args():
    launch = (Path(__file__).parents[1] / "launch" / "round_table_docking_adapter.launch").read_text()
    assert "target_side" not in launch
    assert "boundary_point" not in launch
    assert "boundary_tangent" not in launch
    assert "boundary_allowed" not in launch


def test_node_uses_one_locked_sensor_snapshot_per_tick():
    node_source = (Path(__file__).parents[1] / "scripts" / "round_table_docking_adapter_node.py").read_text()
    tick_method = node_source[node_source.index("def _tick"):]
    assert "snapshot = self._sensor_snapshot()" in tick_method
    assert "self._admission(snapshot," in tick_method
    assert "self._base_observation_and_pose(circle)" in tick_method
    assert "guard_footprint(" in tick_method and "guard_rotation_sweep(" in tick_method
    assert "threading.Lock()" in node_source


def test_observation_dropout_gate_stops_every_bad_frame_and_latches_repeated_dropout():
    gate = ObservationDropoutGate(latch_after_frames=2, recover_after_frames=3)
    assert [gate.update(True) for _ in range(3)] == ["HOLD_ZERO", "HOLD_ZERO", "ALLOW"]
    assert gate.update(False) == "HOLD_ZERO"
    assert gate.update(True) == "HOLD_ZERO"
    assert gate.update(False) == "HOLD_ZERO"
    assert gate.update(False) == "LATCH"
    gate.reset()
    assert gate.update(True) == "HOLD_ZERO"


def test_phase_specific_commands_forbid_turning_while_approaching():
    assert safe_suggestion(VelocitySuggestion(wz_rps=.01), DockingPhase.ALIGN_CENTER) == (True, "SAFE")
    assert safe_suggestion(VelocitySuggestion(vx_mps=.01), DockingPhase.ALIGN_CENTER) == (False, "ALIGN_REQUIRES_ROTATION_ONLY")
    assert safe_suggestion(VelocitySuggestion(vx_mps=.01), DockingPhase.APPROACH_CENTER) == (True, "SAFE")
    assert safe_suggestion(VelocitySuggestion(vx_mps=.01, wz_rps=.01), DockingPhase.APPROACH_CENTER) == (False, "APPROACH_REQUIRES_FORWARD_ONLY")
    assert safe_suggestion(VelocitySuggestion(vy_mps=.01), DockingPhase.APPROACH_CENTER) == (False, "APPROACH_REQUIRES_FORWARD_ONLY")
    latch = FaultLatch(); latch.fault("GUARD_REJECT")
    assert latch.latched and latch.reason == "GUARD_REJECT"
    latch.reset(); assert not latch.latched


def test_ros_master_admission_reads_publishers_not_subscribers():
    node_source = (Path(__file__).parents[1] / "scripts" / "round_table_docking_adapter_node.py").read_text()
    publishers_method = node_source[node_source.index("def _publishers"):node_source.index("def _admission")]
    assert "state[0]" in publishers_method
    assert "state[1] if name == topic" not in publishers_method
    assert "rosgraph.get_master_uri()" in node_source
    assert "rospy.get_master_uri()" not in node_source


def test_point_stamped_tf2_transform_registration_is_loaded():
    node_source = (Path(__file__).parents[1] / "scripts" / "round_table_docking_adapter_node.py").read_text()
    assert "import tf2_geometry_msgs" in node_source


def test_latest_amcl_map_pose_is_bounded_against_circle_stamp():
    node_source = (Path(__file__).parents[1] / "scripts" / "round_table_docking_adapter_node.py").read_text()
    transform_method = node_source[node_source.index("def _base_observation_and_pose"):node_source.index("def _tick")]
    assert 'lookup_transform("map", "base_footprint", rospy.Time(0)' in transform_method
    assert "self.map_tf_skew_s" in transform_method
    assert "pose.header.stamp - circle.header.stamp" in transform_method


def test_latched_static_map_uses_identity_and_managed_publisher_not_republish_age():
    node_source = (Path(__file__).parents[1] / "scripts" / "round_table_docking_adapter_node.py").read_text()
    assert 'map_publishers == {"/raicom_team_boundary_map_server"}' in node_source
    assert 'Subscriber("/raicom/team_boundary_mask"' in node_source
    assert 'Subscriber("/map"' not in node_source
    admission_call = node_source[node_source.index("return validate_admission"):node_source.index("def _base_observation_and_pose")]
    assert "self._fresh(self.map_received, self.timeout)" not in admission_call


def test_disabled_dwell_cannot_consume_future_phase_timeout_budget():
    node_source = (Path(__file__).parents[1] / "scripts" / "round_table_docking_adapter_node.py").read_text()
    assert "self.phase_started = rospy.Time.now()" in node_source[node_source.index("def _ack"):node_source.index("def _fresh")]
    enabled_edge = node_source[node_source.index("if not self.was_enabled:"):node_source.index("if self.requalification_pending and not self.enable_seen_after_ack:")]
    assert "self.phase_started = rospy.Time.now()" in enabled_edge


def test_guard_rejection_can_drive_zero_fault_latch_and_navigation_gate_rejected():
    blocked = OccupancyGridData(0., 0., .1, 10, 10, [100] * 100)
    assert not guard_footprint(blocked, Footprint(.1, .1, .1, .1), .5, .5, 0.).safe
    latch = FaultLatch(); latch.fault("GUARD_REJECT_UNKNOWN_OR_OCCUPIED")
    assert latch.latched  # ROS boundary must publish Twist() on this fault.
    assert validate_admission(admitted(map_identity_matches=False)) == "MAP_IDENTITY_MISMATCH"
    assert validate_admission(admitted(observation_stamp_fresh=False)) == "OBSERVATION_STAMP_STALE"
    assert validate_admission(admitted(valid_paired=False)) == "VALID_OBSERVATION_UNPAIRED"
    assert validate_admission(admitted(map_valid=False)) == "MAP_INVALID"
    assert validate_admission(admitted(move_base_absent=False)) == "MOVE_BASE_PRESENT"
    assert validate_admission(admitted(nav_has_no_publishers=False)) == "NAV_PUBLISHER_PRESENT"
    assert validate_admission(admitted(relative_publishers_are_self_only=False)) == "RELATIVE_TOPIC_CONFLICT"
