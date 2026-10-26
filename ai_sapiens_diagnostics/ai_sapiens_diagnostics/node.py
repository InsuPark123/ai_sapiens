# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Read-only ROS adapter for the health contract."""

import json
import math
import time
from collections import Counter

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .adapter import DiagnosticStore


class HealthNode(Node):
    def __init__(self):
        super().__init__('ai_sapiens_health')
        self.declare_parameter('robot_id', 'unassigned')
        self.declare_parameter('expected_components', [''])
        self.declare_parameter('producer_timeout_sec', 1.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('history_size', 64)
        self.declare_parameter('input_topic', '/diagnostics')
        self.declare_parameter('startup_run_id', '')
        expected = [name for name in self.get_parameter('expected_components').value if name]
        self.store = DiagnosticStore(
            expected, self.get_parameter('robot_id').value,
            self.get_parameter('producer_timeout_sec').value,
            self.get_parameter('history_size').value,
            self.get_parameter('startup_run_id').value)
        rate = self.get_parameter('publish_rate_hz').value
        if not math.isfinite(rate) or not 0.0 < rate <= 100.0:
            raise ValueError('publish_rate_hz must be in (0, 100]')
        snapshot_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.health_pub = self.create_publisher(String, '/ai_sapiens/health', snapshot_qos)
        self.event_pub = self.create_publisher(String, '/ai_sapiens/health/events', 50)
        self.subscription = self.create_subscription(
            DiagnosticArray, self.get_parameter('input_topic').value, self.receive, 10)
        self.timer = self.create_timer(
            1.0 / rate, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, message):
        received = time.monotonic()
        names = Counter(status.name for status in message.status)
        records = []
        has_failure = False
        for status in message.status:
            if names[status.name] != 1:
                self.store.rejected += 1
                continue
            fields = {item.key: item.value for item in status.values}
            if len(fields) != len(status.values):
                # Reject the complete ambiguous entry rather than choosing a key.
                self.store.rejected += 1
                continue
            records.append((status, fields))
            has_failure = has_failure or fields.get('phase') == 'FAILED'
        if has_failure:
            # Failure origins need validated bus/device pairing within this
            # exact array, including reversed publication order.
            self.store.ingest_batch([
                dict(name=status.name, hardware_id=status.hardware_id,
                     level=status.level, message=status.message, fields=fields)
                for status, fields in records], received)
        else:
            # Names and keys are already checked above. Normal frames need no
            # failure-origin pairing or another duplicate scan. Explicit empty
            # context also prevents borrowing a cached bus from another array.
            for status, fields in records:
                self.store.ingest(status.name, status.hardware_id, status.level,
                                  status.message, fields, received, failure_context={})

    def publish(self):
        snapshot, events = self.store.snapshot(time.monotonic())
        snapshot['generated_at_ros_ns'] = self.get_clock().now().nanoseconds
        for event in events:
            event['generated_at_ros_ns'] = snapshot['generated_at_ros_ns']
        self.health_pub.publish(String(data=json.dumps(snapshot, allow_nan=False)))
        for event in events:
            self.event_pub.publish(String(data=json.dumps(event, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = HealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
