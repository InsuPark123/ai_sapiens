# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Publish cached motor/IMU temperatures through standard /diagnostics at 5 Hz."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from control_msgs.msg import DynamicJointState
from .motor_temperature import MotorTemperatures, ImuTemperature


class TemperatureNode(Node):
    def __init__(self):
        super().__init__('motor_temperature_diagnostics', enable_rosout=False,
                         start_parameter_services=False)
        robot_id = self.declare_parameter('robot_id', 'unassigned').value
        motors_enabled = self.declare_parameter('motor_temperatures_enabled', True).value
        imu_enabled = self.declare_parameter('imu_temperature_enabled', False).value
        self.cache = MotorTemperatures(robot_id) if motors_enabled else None
        self.imu_cache = ImuTemperature(robot_id) if imu_enabled else None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 5)
        self.subscriptions_owned = [
            self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics, qos)]
        if self.cache:
            self.subscriptions_owned.append(self.create_subscription(DynamicJointState,
                '/motor_temperature_broadcaster/dynamic_joint_states', self.dynamic, qos))
        if self.imu_cache:
            self.subscriptions_owned.append(self.create_subscription(DynamicJointState,
                '/imu_temperature_broadcaster/dynamic_joint_states', self.imu_dynamic, qos))
        self.timer = self.create_timer(0.2, self.publish)

    def dynamic(self, msg):
        self.ingest_dynamic(self.cache, msg)

    def imu_dynamic(self, msg):
        self.ingest_dynamic(self.imu_cache, msg)

    def ingest_dynamic(self, cache, msg):
        if cache is None:
            return
        if len(msg.joint_names) != len(msg.interface_values):
            cache.samples = {}
            return
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if not 0 <= age <= cache.timeout:
            cache.samples = {}
            return
        cache.ingest_dynamic([(name, row.interface_names, row.values)
            for name, row in zip(msg.joint_names, msg.interface_values)], stamp)

    def diagnostics(self, msg):
        if len(msg.status) > 256:
            return
        for row in msg.status:
            if not row.name.startswith(('k1_', 'dhi/k1_')) or '/dxl/' not in row.name:
                continue
            if len(row.values) > 128:
                continue
            fields = {v.key: v.value for v in row.values}
            if len(fields) == len(row.values):
                for cache in (self.cache, self.imu_cache):
                    if cache:
                        cache.ingest_source(row.name, fields)

    def publish(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        for cache in (self.cache, self.imu_cache):
            if cache is None:
                continue
            for row in cache.snapshot():
                status = DiagnosticStatus(name=row['name'], hardware_id=row['hardware_id'],
                                          level=bytes([row['level']]), message=row['message'])
                status.values = [KeyValue(key=k, value=v) for k, v in row['fields'].items()]
                array.status.append(status)
        self.publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
