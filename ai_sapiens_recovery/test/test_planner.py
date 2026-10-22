import pytest
from ai_sapiens_recovery.planner import Action, RecoveryPlanner


def release(p, start=0):
    for i in range(8):
        t = start + i * 0.1
        p.rc(t, int(t * 1000) % 32768, True)
        p.safe(t)
    return t


def running():
    p = RecoveryPlanner(['a', 'b'], 'motion', backoff=0)
    t = release(p)
    hw, ctrl = dict(a='active', b='active'), dict(motion='active')
    assert p.step(t, hw, ctrl) is None
    assert p.phase == 'RUNNING'
    return p, t, hw, ctrl


def apply(p, now, hw, ctrl, action):
    if action.kind in ('stop_controller', 'stop_broadcaster'):
        ctrl[action.target] = 'inactive'
    elif action.kind in ('start_controller', 'start_broadcaster'):
        ctrl[action.target] = 'active'
    else:
        hw[action.target] = {'deactivate': 'inactive', 'cleanup': 'unconfigured',
                             'configure': 'inactive', 'activate': 'active'}[action.kind]
    p.complete(now, True)


def test_recovery_stops_and_prepares_without_activation():
    p, t, hw, ctrl = running()
    p.rc(t, 701, False)
    kinds = []
    for _ in range(12):
        action = p.step(t, hw, ctrl)
        if action is None:
            break
        kinds.append(action.kind)
        apply(p, t, hw, ctrl, action)
    assert kinds == ['stop_controller', 'deactivate', 'deactivate', 'cleanup', 'cleanup']
    assert p.phase == 'WAIT_RELEASE'
    t = release(p, 1.0)
    for _ in range(3):
        action = p.step(t, hw, ctrl)
        if action:
            kinds.append(action.kind)
            apply(p, t, hw, ctrl, action)
    assert p.phase == 'READY_TO_ARM'
    assert kinds[-2:] == ['configure', 'configure']
    assert ctrl['motion'] == 'inactive'
    assert hw == dict(a='inactive', b='inactive')


def test_startup_failure_does_not_start_recovery():
    p = RecoveryPlanner(['a'], 'motion')
    t = release(p)
    assert p.step(t, {'a': 'unconfigured'}, {}) is None
    assert p.phase == 'OBSERVING'


def test_repeated_release_message_with_frozen_tick_is_stale():
    p, t, hw, ctrl = running()
    for i in range(1, 12):
        p.rc(t + i * .1, 700, True)
    assert not p.safe(t + 1.1)
    assert p.step(t + 1.1, hw, ctrl) == Action('stop_controller', 'motion')


def test_tick_wrap_is_accepted():
    p = RecoveryPlanner(['a'], 'motion')
    p.rc(0, 32760, True)
    p.rc(.01, 2, True)
    assert p.tick_at == .01


def test_single_flight_timeout_never_resends():
    p, t, hw, ctrl = running()
    hw['a'] = 'unconfigured'
    action = p.step(t, hw, ctrl)
    assert p.step(t + .1, hw, ctrl) is None
    p.timeout()
    assert p.pending == action
    assert p.step(t + 999, hw, ctrl) is None
    assert p.phase == 'BLOCKED'


def test_retry_limit_blocks():
    p, t, hw, ctrl = running()
    hw['a'] = 'unconfigured'
    for _ in range(3):
        assert p.step(t, hw, ctrl) == Action('stop_controller', 'motion')
        p.complete(t, False)
    assert p.phase == 'BLOCKED'
    assert p.step(t, hw, ctrl) is None


@pytest.mark.parametrize('bad', ['finalized', 'unknown', None])
def test_unrecoverable_hardware_blocks(bad):
    p, t, hw, ctrl = running()
    hw['a'] = bad
    assert p.step(t, hw, ctrl) is None
    assert p.phase == 'BLOCKED'


def test_estop_during_configure_invalidates_success():
    p, t, hw, ctrl = running()
    p.phase = 'RESTORING'
    hw.update(a='unconfigured', b='unconfigured')
    ctrl['motion'] = 'inactive'
    action = p.step(t, hw, ctrl)
    assert action == Action('configure', 'a')
    p.rc(t, 701, False)
    apply(p, t, hw, ctrl, action)
    assert p.step(t, hw, ctrl) == Action('cleanup', 'a')
    assert not p.prepared


def test_backoff_prevents_busy_retries():
    p, t, hw, ctrl = running()
    p.backoff = 2
    hw['a'] = 'unconfigured'
    p.step(t, hw, ctrl)
    p.complete(t, False)
    assert p.step(t + .5, hw, ctrl) is None


def test_healthy_running_has_no_actions():
    p, t, hw, ctrl = running()
    for i in range(1, 101):
        now = t + i * .1
        p.rc(now, int(now * 1000), True)
        assert p.step(now, hw, ctrl) is None
    assert p.phase == 'RUNNING'


def test_success_response_without_state_change_is_still_bounded():
    p, t, hw, ctrl = running()
    hw['a'] = 'unconfigured'
    for _ in range(3):
        assert p.step(t, hw, ctrl) == Action('stop_controller', 'motion')
        p.complete(t, True)  # Response says OK, but controller stays active.
    assert p.step(t, hw, ctrl) is None
    assert p.phase == 'BLOCKED'


@pytest.mark.parametrize('mode', ['damping', 'startup'])
def test_auto_rearm_restores_broadcaster_and_controller_for_multiple_cycles(mode):
    p, t, hw, ctrl = running()
    p.auto_rearm = True
    p.rearm_mode = mode
    p.recovered_phase = 'RECOVERED_STARTUP' if mode == 'startup' else 'RECOVERED_DAMPING'
    p.broadcasters = ('joint_states',)
    ctrl['joint_states'] = 'active'
    for cycle in range(5):
        hw['a'] = 'unconfigured'
        kinds = []
        for _ in range(25):
            t += .05
            p.rc(t, int(t * 1000) % 32768, True)
            action = p.step(t, hw, ctrl)
            if action:
                kinds.append(action.kind)
                apply(p, t, hw, ctrl, action)
            if p.phase == p.recovered_phase:
                break
        assert p.phase == p.recovered_phase
        assert kinds[-4:] == ['activate', 'activate', 'start_broadcaster', 'start_controller']
        assert ctrl == dict(motion='active', joint_states='active')


def test_estop_between_motor_activations_deactivates_armed_motor():
    p, t, hw, ctrl = running()
    p.auto_rearm = True
    p.phase = 'ARMING'
    p.prepared = set(hw)
    hw.update(a='inactive', b='inactive')
    ctrl['motion'] = 'inactive'
    action = p.step(t, hw, ctrl)
    assert action == Action('activate', 'a')
    apply(p, t, hw, ctrl, action)
    p.rc(t, 701, False)
    assert p.step(t, hw, ctrl) == Action('deactivate', 'a')


def test_activation_failure_discards_prepared_state():
    p, t, hw, ctrl = running()
    p.auto_rearm = True
    p.phase = 'ARMING'
    p.prepared = set(hw)
    hw.update(a='inactive', b='inactive')
    ctrl['motion'] = 'inactive'
    assert p.step(t, hw, ctrl) == Action('activate', 'a')
    p.complete(t, False)
    assert p.step(t, hw, ctrl) == Action('cleanup', 'a')
    assert not p.prepared


def test_configure_freshness_gap_preserves_torque_off_preparation_and_waits():
    p, t, hw, ctrl = running()
    p.auto_rearm = True
    p.phase = 'RESTORING'
    p.prepared = {'a'}
    hw.update(a='inactive', b='unconfigured')
    ctrl['motion'] = 'inactive'
    action = p.step(t, hw, ctrl)
    assert action == Action('configure', 'b')
    # Reproduce the real CM global read pause during on_configure.
    p.rc(t + .11, 701, False, freshness_only=True)
    apply(p, t + .12, hw, ctrl, action)
    assert p.step(t + .12, hw, ctrl) is None
    assert p.phase == 'WAIT_RELEASE'
    assert p.prepared == {'a', 'b'}
    p.rc(t + .13, 710, True)
    assert p.step(t + .13, hw, ctrl) is None  # Still needs 0.5 s stability.
    assert p.phase == 'WAIT_RELEASE'
    t = release(p, t + .14)
    assert p.step(t, hw, ctrl) is None
    assert p.phase == 'ARMING'
    assert p.step(t, hw, ctrl) == Action('activate', 'a')
    assert Action('cleanup', 'a') not in p.attempts
    assert Action('cleanup', 'b') not in p.attempts


def test_missing_rc_messages_during_preparation_never_allow_activation():
    p, t, hw, ctrl = running()
    p.auto_rearm = True
    p.phase = 'RESTORING'
    p.prepared = set(hw)
    hw.update(a='inactive', b='inactive')
    ctrl['motion'] = 'inactive'
    assert p.step(t + .3, hw, ctrl) is None
    assert p.phase == 'WAIT_RELEASE'
    assert p.prepared == {'a', 'b'}
    assert p.step(t + 30, hw, ctrl) is None
    assert p.phase == 'WAIT_RELEASE'


def test_freshness_gap_after_activation_still_stops_armed_motor():
    p, t, hw, ctrl = running()
    p.auto_rearm = True
    p.phase = 'ARMING'
    p.prepared = set(hw)
    hw.update(a='inactive', b='inactive')
    ctrl['motion'] = 'inactive'
    apply(p, t, hw, ctrl, p.step(t, hw, ctrl))
    p.rc(t + .01, 701, False, freshness_only=True)
    assert p.step(t + .01, hw, ctrl) == Action('deactivate', 'a')
    assert not p.prepared
