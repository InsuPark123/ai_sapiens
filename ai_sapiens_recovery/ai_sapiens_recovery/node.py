"""Non-real-time lifecycle coordinator; never opens a Dynamixel port."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ai_sapiens_interfaces.msg import RcStatus
from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState, SwitchController)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import UInt64
from .planner import RecoveryPlanner


class RecoveryNode(Node):
    def __init__(self):
        super().__init__('k1_recovery')
        defaults = dict(
            execute=False, controller_manager='/controller_manager',
            rc_status_topic='/ai_sapiens_rc/status',
            motor_components=['k1_left_arm', 'k1_right_arm', 'k1_left_leg', 'k1_right_leg'],
            command_controller='joint_group_impedance_controller', max_attempts=3,
            retry_backoff_sec=2.0, service_timeout_sec=30.0, auto_rearm=False,
            state_broadcasters=['joint_state_broadcaster'], rearm_mode='damping',
            permit_topic='/ai_sapiens/recovery/permit')
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.execute = param('execute')
        self.planner = RecoveryPlanner(param('motor_components'), param('command_controller'),
                                       param('max_attempts'), param('retry_backoff_sec'),
                                       param('auto_rearm'), param('state_broadcasters'), param('rearm_mode'))
        self.timeout_sec = param('service_timeout_sec')
        if not 0 < self.timeout_sec <= 120:
            raise ValueError('service_timeout_sec must be in (0, 120]')
        cm = param('controller_manager').rstrip('/')
        self.cm_clients = {
            'hardware': self.create_client(ListHardwareComponents, cm + '/list_hardware_components'),
            'controllers': self.create_client(ListControllers, cm + '/list_controllers'),
            'state': self.create_client(SetHardwareComponentState, cm + '/set_hardware_component_state'),
            'switch': self.create_client(SwitchController, cm + '/switch_controller')}
        self.create_subscription(RcStatus, param('rc_status_topic'), self.rc, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics, qos_profile_sensor_data)
        self.publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.permit_topic = param('permit_topic')
        self.permit_publisher = self.create_publisher(UInt64, self.permit_topic, 1)
        self.future = None
        self.request_kind = None
        self.started = 0.0
        self.hardware = {}
        self.capabilities = {}
        self.rearm_capabilities = {}
        self.damping_active = {}
        self.startup_active = {}
        self.next_poll = 'hardware'
        self.last_rc_validity = None
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish)
        self.create_timer(0.05, self.publish_permit)

    def rc(self, msg):
        valid = (msg.status_data_valid and msg.realtime_tick_fresh and msg.hardware_ok
                 and msg.hardware_error_status == 0 and not msg.estop_active
                 and msg.estop_released and msg.is_control_input_safe)
        validity = (msg.status_data_valid, msg.realtime_tick_fresh, msg.hardware_ok,
                    msg.hardware_error_status, msg.estop_active, msg.estop_released,
                    msg.is_control_input_safe)
        if validity != self.last_rc_validity:
            self.get_logger().info(
                f'RC gate changed: valid/tick_fresh/hardware_ok/error/estop/released/safe={validity}')
            self.last_rc_validity = validity
        # rc_link_ok already includes realtime_tick_fresh in the broadcaster.
        # Do not require it for a stale-tick pause. No permit is granted until
        # the full fresh/safe predicate has been stable again for 0.5 seconds.
        freshness_only = (msg.status_data_valid and msg.hardware_ok
                          and msg.hardware_error_status == 0 and not msg.estop_active
                          and msg.estop_released and msg.crsf_failsafe == 0
                          and msg.all_channels_valid
                          and not msg.realtime_tick_fresh)
        self.planner.rc(time.monotonic(), msg.realtime_tick, valid,
                        freshness_only=freshness_only)

    def diagnostics(self, msg):
        now = time.monotonic()
        for status in msg.status:
            for name in self.planner.motors:
                if status.name == f'{name}/bus':
                    values = {item.key: item.value for item in status.values}
                    self.capabilities[name] = (now, values.get('recovery_prepare_supported') == 'true')
                    self.rearm_capabilities[name] = values.get('recovery_rearm_supported') == 'true'
                    self.damping_active[name] = values.get('recovery_damping_active') == 'true'
                    self.startup_active[name] = values.get('recovery_startup_active') == 'true'

    def capable(self, now):
        return all(name in self.capabilities and self.capabilities[name][1]
                   and now - self.capabilities[name][0] < 2
                   and (not self.planner.auto_rearm or self.rearm_capabilities.get(name, False))
                   for name in self.planner.motors)

    def publish_permit(self):
        now = time.monotonic()
        allowed = (self.execute and self.planner.auto_rearm and self.planner.safe(now)
                   and not self.planner.invalidated and self.capable(now)
                   and self.planner.phase in ('ARMING', self.planner.recovered_phase)
                   and self.count_publishers(self.permit_topic) == 1)
        # CLOCK_MONOTONIC deadline shared by processes/containers on the robot.
        # Receiver time must not renew a delayed grant after its source expired.
        expires = min(now + .25, self.planner.rc_at + .25, self.planner.tick_at + .25) if allowed else 0
        self.permit_publisher.publish(UInt64(data=int(expires * 1_000_000_000)))

    def call(self, kind, request, client):
        if not self.cm_clients[client].service_is_ready():
            if kind == 'action':
                self.planner.complete(time.monotonic(), False)
            return
        self.future = self.cm_clients[client].call_async(request)
        self.request_kind, self.started = kind, time.monotonic()

    def tick(self):
        now = time.monotonic()
        # Evaluate the release watchdog even while a service is outstanding.
        if not self.planner.safe(now) and self.planner.phase in ('ARMING', self.planner.recovered_phase):
            if not self.planner.invalidated:
                self.get_logger().warn(
                    f'HAT gate lost in {self.planner.phase}: rc_ok={self.planner.rc_ok}, '
                    f'rc_at={self.planner.rc_at}, tick_at={self.planner.tick_at}, now={now}')
            self.planner.invalidated = True
        if self.future is not None:
            if now - self.started > self.timeout_sec:
                self.planner.timeout()
                return
            if not self.future.done():
                return
            future, self.future = self.future, None
            try:
                result = future.result()
            except Exception as error:
                self.planner.timeout()
                self.get_logger().error(str(error))
                return
            if self.request_kind == 'hardware':
                self.hardware = {x.name: x.state.label for x in result.component}
                self.next_poll = 'controllers'
            elif self.request_kind == 'controllers':
                controllers = {x.name: x.state for x in result.controller}
                action = self.planner.step(now, self.hardware, controllers)
                self.next_poll = 'hardware'
                if action is not None:
                    self.dispatch(action, now)
                    return
            else:
                action = self.planner.pending
                ok = result.ok
                if action.kind in ('configure', 'deactivate', 'cleanup', 'activate'):
                    expected = {'configure': 2, 'deactivate': 2, 'cleanup': 1, 'activate': 3}[action.kind]
                    ok = ok and result.state.id == expected
                self.get_logger().info(f'Recovery result: {action.kind} {action.target}: ok={ok}')
                self.planner.complete(now, ok)
                self.next_poll = 'hardware'
        if self.planner.phase == 'BLOCKED':
            return
        if self.next_poll == 'hardware':
            self.call('hardware', ListHardwareComponents.Request(), 'hardware')
        else:
            self.call('controllers', ListControllers.Request(), 'controllers')

    def dispatch(self, action, now):
        self.get_logger().info(f'Recovery request: {action.kind} {action.target}')
        capable = self.capable(now)
        if not self.execute or not capable:
            self.planner.phase = 'BLOCKED'
            self.planner.reason = (f'Preview: {action.kind} {action.target}' if not self.execute
                                   else 'DHI recovery capability missing/stale; no service sent')
            return
        if action.kind in ('stop_controller', 'stop_broadcaster', 'start_controller', 'start_broadcaster'):
            request = SwitchController.Request()
            if action.kind.startswith('start'):
                request.activate_controllers = [action.target]
            else:
                request.deactivate_controllers = [action.target]
            request.strictness = SwitchController.Request.BEST_EFFORT
            request.timeout.sec = 5
            self.call('action', request, 'switch')
        else:
            request = SetHardwareComponentState.Request()
            request.name = action.target
            request.target_state.id = {'configure': 2, 'deactivate': 2, 'cleanup': 1, 'activate': 3}[action.kind]
            self.call('action', request, 'state')

    def publish(self):
        status = DiagnosticStatus()
        status.name, status.hardware_id = 'ai_sapiens/recovery', 'k1'
        damping_verified = self.planner.phase == 'RECOVERED_DAMPING' and self.capable(time.monotonic()) and all(
            self.damping_active.get(x, False) for x in self.planner.motors)
        startup_verified = self.planner.phase == 'RECOVERED_STARTUP' and self.capable(time.monotonic()) and all(
            self.startup_active.get(x, False) for x in self.planner.motors)
        status.level = (DiagnosticStatus.OK if self.planner.phase == 'RUNNING' or damping_verified or startup_verified else
                        DiagnosticStatus.ERROR if self.planner.phase == 'BLOCKED' else DiagnosticStatus.WARN)
        status.message = f'{self.planner.phase}: {self.planner.reason}'
        status.values = [KeyValue(key='phase', value=self.planner.phase),
                         KeyValue(key='execute', value=str(self.execute).lower()),
                         KeyValue(key='auto_rearm', value=str(self.planner.auto_rearm).lower()),
                         KeyValue(key='rearm_mode', value=self.planner.rearm_mode),
                         KeyValue(key='startup_verified', value=str(startup_verified).lower()),
                         KeyValue(key='damping_verified', value=str(damping_verified).lower())]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
