from raicom_state_server.snapshot import RobotContext

def test_missing_sources_are_explicit_and_hardware_is_unverified():
    context = RobotContext(now=lambda: 10.0)
    snapshot = context.snapshot()
    assert snapshot['localization']['state'] == 'MISSING'
    assert snapshot['arm']['state'] == 'HW_UNVERIFIED'
    assert context.health()['charging']['status'] == 'HW_UNVERIFIED'

def test_odom_is_not_mislabeled_as_map_localization():
    context = RobotContext(now=lambda: 0.0)
    odometry_pose = {'frame': 'odom', 'x_m': 1, 'y_m': 2, 'yaw_deg': 3}
    context.update('odom', odometry_pose)

    snapshot = context.snapshot()

    assert snapshot['robot']['pose'] == odometry_pose
    assert snapshot['robot']['odometry_pose'] == odometry_pose
    assert snapshot['localization']['state'] == 'MISSING'
    assert snapshot['localization']['pose']['frame'] == 'UNKNOWN'


def test_stale_localization_is_not_reported_ok():
    now = [0.0]
    context = RobotContext(freshness={'localization_ms': 300}, now=lambda: now[0])
    context.update('localization', {'frame': 'map', 'x_m': 1, 'y_m': 2, 'yaw_deg': 3})
    now[0] = 1.0
    assert context.snapshot()['localization']['state'] == 'STALE'


def test_map_localization_and_gazebo_ground_truth_are_separate():
    context = RobotContext(now=lambda: 5.0)
    map_pose = {'frame': 'map', 'x_m': 1.1, 'y_m': 2.2, 'yaw_deg': 30}
    truth_pose = {'frame': 'world', 'x_m': 1.0, 'y_m': 2.0, 'yaw_deg': 29}
    context.update('localization', map_pose)
    context.update('gazebo_ground_truth', truth_pose)

    snapshot = context.snapshot()

    assert snapshot['robot']['pose'] == map_pose
    assert snapshot['localization']['pose'] == map_pose
    assert snapshot['simulation']['ground_truth'] == truth_pose
    assert snapshot['verification'] == 'SIM_TESTED'

def test_timeline_is_bounded_and_ordered():
    context = RobotContext(now=lambda: 5.0)
    context.record_sample()
    assert len(context.history) == 1
    context.event('GRASP_UNKNOWN')
    assert context.timeline(seconds=8, focus='grasp', max_events=1)[0]['type'] == 'GRASP_UNKNOWN'


def test_semantic_mission_snapshot_is_normalized_and_preserved():
    context = RobotContext(now=lambda: 1.0)
    context.update('mission', {
        'mode': 'AUTO', 'state': 'REFEREE_WAIT', 'dispatch_queue': ['pick_a'],
        'readiness_reasons': ['camera UNKNOWN'],
        'referee': {'state': 'WAITING'},
        'safety': {'state': 'STOP_UNCONFIRMED', 'estop': False},
        'capabilities': {'base': {'source': 'sim', 'mode': 'AUTO', 'verification': 'SIM_TESTED', 'authorization': 'AUTHORIZED'}},
    })
    mission = context.snapshot()['mission']
    assert mission['dispatch_queue'] == ['pick_a']
    assert mission['referee']['state'] == 'WAITING'
    assert mission['safety']['state'] == 'STOP_UNCONFIRMED'
    assert mission['capabilities']['base']['verification'] == 'SIM_TESTED'


def test_invalid_semantic_mission_payload_has_explicit_safe_defaults():
    context = RobotContext(now=lambda: 1.0)
    context.update('mission', 'not-json')
    mission = context.snapshot()['mission']
    assert mission['mode'] == 'UNKNOWN'
    assert mission['dispatch_queue'] == []
    assert mission['referee']['state'] == 'WAITING'
