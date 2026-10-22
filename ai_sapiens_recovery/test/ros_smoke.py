"""Isolated ROS adapter test. Fake services only; no device interface loaded."""
import os
import time
import json
import sys

assert os.environ.get('ROS_DOMAIN_ID') == '189', 'Use isolated ROS_DOMAIN_ID=189'
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState, SwitchController)
from controller_manager_msgs.msg import HardwareComponentState, ControllerState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ai_sapiens_interfaces.msg import RcStatus
from std_msgs.msg import UInt64
from ai_sapiens_recovery.node import RecoveryNode


class FakeCM(Node):
    def __init__(self, auto_rearm, freshness_gap=False, startup=False):
        super().__init__('recovery_fake_cm')
        self.hw = {f'k1_{part}': 'active' for part in
                   ('left_arm', 'right_arm', 'left_leg', 'right_leg')}
        self.controller = 'active'
        self.broadcaster = 'active'
        self.auto_rearm = auto_rearm
        self.startup = startup
        self.freshness_gap = freshness_gap
        self.gap_until = 0.0
        self.permitted = False
        self.damping = set()
        self.create_subscription(UInt64, '/recovery_test_permit', self.permit, 1)
        self.calls = []
        self.create_service(ListHardwareComponents, '/recovery_test_cm/list_hardware_components', self.list_hw)
        self.create_service(ListControllers, '/recovery_test_cm/list_controllers', self.list_ctrl)
        self.create_service(SetHardwareComponentState, '/recovery_test_cm/set_hardware_component_state', self.state)
        self.create_service(SwitchController, '/recovery_test_cm/switch_controller', self.switch)
        self.rc_pub = self.create_publisher(RcStatus, '/recovery_test_rc', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(.02, self.publish)

    def list_hw(self, request, response):
        for name, state in self.hw.items():
            component = HardwareComponentState()
            component.name, component.state.label = name, state
            component.state.id = {'active': 3, 'inactive': 2, 'unconfigured': 1}[state]
            response.component.append(component)
        return response

    def list_ctrl(self, request, response):
        response.controller = [ControllerState(name='joint_group_impedance_controller', state=self.controller),
                               ControllerState(name='joint_state_broadcaster', state=self.broadcaster)]
        return response

    def permit(self, msg):
        self.permitted = msg.data

    def state(self, request, response):
        assert request.name in self.hw
        assert request.target_state.id in ((1, 2, 3) if self.auto_rearm else (1, 2))
        if request.target_state.id == 3:
            assert self.permitted > time.monotonic_ns(), 'Activation needs a live permit'
            self.damping.add(request.name)
        self.calls.append(('state', request.name, request.target_state.id))
        self.hw[request.name] = {1: 'unconfigured', 2: 'inactive', 3: 'active'}[request.target_state.id]
        if self.freshness_gap and request.target_state.id == 2 and self.controller == 'inactive':
            self.gap_until = time.monotonic() + .12
            self.publish()
        response.ok = True
        response.state.id = request.target_state.id
        response.state.label = self.hw[request.name]
        return response

    def switch(self, request, response):
        assert self.auto_rearm or not request.activate_controllers
        self.calls.append(('switch', tuple(request.deactivate_controllers), tuple(request.activate_controllers)))
        for name in request.deactivate_controllers:
            if name == 'joint_state_broadcaster':
                self.broadcaster = 'inactive'
            else:
                self.controller = 'inactive'
        for name in request.activate_controllers:
            assert all(x == 'active' for x in self.hw.values())
            if name == 'joint_state_broadcaster':
                self.broadcaster = 'active'
            else:
                assert self.broadcaster == 'active'
                self.controller = 'active'
        response.ok = True
        return response

    def publish(self):
        rc = RcStatus()
        rc.realtime_tick = int(time.monotonic() * 1000) % 32768
        rc.status_data_valid = rc.hardware_ok = True
        rc.estop_released = rc.all_channels_valid = True
        rc.realtime_tick_fresh = time.monotonic() >= self.gap_until
        rc.rc_link_ok = rc.is_control_input_safe = rc.realtime_tick_fresh
        self.rc_pub.publish(rc)
        diag = DiagnosticArray()
        diag.status = [DiagnosticStatus(name=name + '/bus', values=[
            KeyValue(key='recovery_prepare_supported', value='true'),
            KeyValue(key='recovery_rearm_supported', value='true'),
            KeyValue(key='recovery_damping_active', value=str(not self.startup and name in self.damping).lower()),
            KeyValue(key='recovery_startup_active', value=str(self.startup and name in self.damping).lower())])
            for name in self.hw]
        self.diag_pub.publish(diag)


def main():
    auto_rearm = '--auto-rearm' in sys.argv
    startup = '--startup' in sys.argv
    rclpy.init(args=['--ros-args', '-p', 'execute:=true', '-p',
                    'controller_manager:=/recovery_test_cm', '-p', 'rc_status_topic:=/recovery_test_rc',
                    '-p', 'permit_topic:=/recovery_test_permit',
                    '-p', 'rearm_mode:=' + ('startup' if startup else 'damping'),
                    '-p', f'auto_rearm:={str(auto_rearm).lower()}'])
    cm, recovery = FakeCM(auto_rearm, '--freshness-gap' in sys.argv, startup), RecoveryNode()
    executor = SingleThreadedExecutor()
    executor.add_node(cm)
    executor.add_node(recovery)
    deadline, injected = time.monotonic() + 20, False
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if recovery.planner.phase == 'RUNNING' and not injected:
                cm.hw['k1_right_leg'] = 'unconfigured'
                injected = True
            if recovery.planner.phase == 'BLOCKED':
                raise AssertionError(recovery.planner.reason)
            if recovery.planner.phase == (recovery.planner.recovered_phase if auto_rearm else 'READY_TO_ARM'):
                break
        assert injected
        assert recovery.planner.phase == (recovery.planner.recovered_phase if auto_rearm else 'READY_TO_ARM')
        if startup and auto_rearm:
            # Allow the final diagnostic publication to reach the real adapter.
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline and not all(recovery.startup_active.values()):
                executor.spin_once(timeout_sec=.02)
            assert len(recovery.startup_active) == 4 and all(recovery.startup_active.values())
            assert not any(recovery.damping_active.values())
        assert all(state == ('active' if auto_rearm else 'inactive') for state in cm.hw.values())
        assert cm.controller == ('active' if auto_rearm else 'inactive')
        assert len(cm.calls) == (18 if auto_rearm else 12), cm.calls
        print(json.dumps({'result': 'passed', 'phase': recovery.planner.phase, 'calls': cm.calls}))
    finally:
        executor.shutdown()
        recovery.destroy_node()
        cm.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
