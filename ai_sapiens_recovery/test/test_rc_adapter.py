"""Use the real RcStatus contract: rc_link_ok also depends on tick freshness."""
from types import SimpleNamespace

import pytest

from ai_sapiens_recovery.node import RecoveryNode
from ai_sapiens_recovery.planner import RecoveryPlanner


def stale_tick_message(**changes):
    values = dict(status_data_valid=True, realtime_tick_fresh=False, hardware_ok=True,
                  hardware_error_status=0, estop_active=False, estop_released=True,
                  is_control_input_safe=False, rc_link_ok=False, all_channels_valid=True,
                  crsf_failsafe=0, realtime_tick=100)
    values.update(changes)
    return SimpleNamespace(**values)


def adapter(phase='RESTORING'):
    planner = RecoveryPlanner(['motor'], 'motion', auto_rearm=True)
    planner.phase = phase
    planner.prepared = {'motor'}
    return SimpleNamespace(planner=planner, last_rc_validity=None,
                           get_logger=lambda: SimpleNamespace(info=lambda _: None))


def test_stale_tick_and_derived_link_false_pause_preparation():
    node = adapter()
    RecoveryNode.rc(node, stale_tick_message())
    assert not node.planner.invalidated
    assert not node.planner.rc_ok
    assert node.planner.step(node.planner.rc_at, {'motor': 'inactive'}, {'motion': 'inactive'}) is None
    assert node.planner.phase == 'WAIT_RELEASE'
    assert node.planner.prepared == {'motor'}


@pytest.mark.parametrize('changes', [
    dict(estop_active=True, estop_released=False),
    dict(hardware_ok=False, hardware_error_status=1),
    dict(crsf_failsafe=1),
    dict(status_data_valid=False),
    dict(realtime_tick_fresh=True),  # Actual RC/link problem with a fresh HAT.
])
def test_explicit_fault_or_fresh_unsafe_input_discards_preparation(changes):
    node = adapter()
    RecoveryNode.rc(node, stale_tick_message(**changes))
    assert node.planner.invalidated


@pytest.mark.parametrize('phase', ['ARMING', 'RECOVERED_DAMPING'])
def test_stale_tick_after_rearming_still_invalidates(phase):
    node = adapter(phase)
    RecoveryNode.rc(node, stale_tick_message())
    assert node.planner.invalidated
