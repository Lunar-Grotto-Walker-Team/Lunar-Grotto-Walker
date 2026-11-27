from pointlio_cloud_relay.ground_filter import choose_output_stamp

NOW = object()
LIDAR = object()


def pick(mode, exact, age, tol=1.0):
    return choose_output_stamp(mode, exact, age, tol, NOW, LIDAR)


def test_never_always_restamps():
    assert pick('never', True, 0.05) is NOW
    assert pick('never', False, 0.05) is NOW


def test_auto_preserves_a_fresh_sweep():
    assert pick('auto', True, 0.08) is LIDAR
    assert pick('auto', True, -0.3) is LIDAR       # slightly ahead is fine


def test_auto_restamps_old_stamps():
    # bag replay: stamps are five days behind the local clock
    assert pick('auto', True, 5 * 86400.0) is NOW
    assert pick('auto', True, 2.0) is NOW
    assert pick('auto', True, -2.0) is NOW


def test_auto_restamps_when_the_pose_came_from_the_fallback():
    # pose and stamp would describe different instants
    assert pick('auto', False, 0.05) is NOW


def test_tolerance_is_respected():
    assert pick('auto', True, 0.9, tol=1.0) is LIDAR
    assert pick('auto', True, 1.1, tol=1.0) is NOW


def test_always_keeps_the_input_stamp_only_for_exact_lookups():
    assert pick('always', True, 5 * 86400.0) is LIDAR
    assert pick('always', False, 0.05) is NOW
