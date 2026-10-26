"""Independent, opt-in H5 collector. Never loaded into DHI read/write."""
from concurrent.futures import ThreadPoolExecutor
from collections import deque
from datetime import datetime, timezone
import math
import time
import uuid

from .hat_collector import DEFAULT_BUSES, EvidenceFiles, FailureTrigger, diagnostic_records
from .hat_protocol import H5Client, collect


def collect_live(client, failure_mask=0):
    # Operation 1 latches a host fault on the MCU and indirectly adds Flash
    # records. The deployed firmware auto-queries sequence 0 whenever a record
    # commits, scanning its history on the bridge foreground loop. Operation 0
    # only freezes the RAM view; preserve host failure intent in the host file.
    evidence = collect(client, 0, journal_limit=0)
    evidence['requested_failure_mask'] = failure_mask
    evidence['mcu_failure_marking_enabled'] = False
    evidence['capture_mode'] = 'ram_view_only'
    return evidence


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    class HatCollector(Node):
        def __init__(self):
            super().__init__('hat_diagnostic_collector')
            for name, default in (
                ('host', '192.168.200.200'), ('service_id', 200),
                ('poll_period_sec', 10.0), ('fault_cooldown_sec', 5.0),
                ('output_directory', '~/.ros/hat-evidence'), ('file_capacity', 128),
                ('channel_buses', list(DEFAULT_BUSES)),
            ):
                self.declare_parameter(name, default)
            self.host = self.get_parameter('host').value
            self.device_id = self.get_parameter('service_id').value
            self.period = self.get_parameter('poll_period_sec').value
            self.cooldown = self.get_parameter('fault_cooldown_sec').value
            if (not math.isfinite(self.period) or not math.isfinite(self.cooldown) or
                    self.period < 5.0 or self.cooldown < 1.0):
                raise ValueError('poll >=5s and fault cooldown >=1s required')
            self.trigger = FailureTrigger(self.get_parameter('channel_buses').value)
            self.files = EvidenceFiles(self.get_parameter('output_directory').value,
                                       self.get_parameter('file_capacity').value)
            self.session = uuid.uuid4().hex
            self.sequence = self.successes = self.file_errors = 0
            self.last_query = self.last_fault_query = float('-inf')
            self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='hat-evidence')
            self.future = None
            self.files.update_latest(dict(complete=False, collector_state='starting',
                collector_session=self.session, errors=['first query pending'],
                host_started_utc=datetime.now(timezone.utc).isoformat(),
                host_started_monotonic_ns=time.monotonic_ns(), service_id=self.device_id,
                live=[], first=[]))
            self.previous = None
            self.latest = None
            self.path = ''
            self.last_saved_signature = None
            self.recent_captures = deque(maxlen=8)
            self.publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
            self.subscription = self.create_subscription(DiagnosticArray, '/diagnostics',
                                                          self.receive, 20)
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(0.2, self.tick, clock=self.steady_clock)
            self.heartbeat = self.create_timer(0.5, self.publish_evidence, clock=self.steady_clock)

        def receive(self, msg):
            for status in msg.status:
                self.trigger.ingest(status.name, {v.key: v.value for v in status.values})

        def tick(self):
            if self.future is not None:
                if not self.future.done():
                    return
                self.accept(self.future.result())
                self.future = None
            now = time.monotonic()
            fault_due = bool(self.trigger.pending) and now - self.last_fault_query >= self.cooldown
            if not fault_due and now - self.last_query < self.period:
                return
            mask, triggers = self.trigger.take() if fault_due else (0, [])
            self.last_query = now
            if fault_due:
                self.last_fault_query = now
            self.future = self.worker.submit(self.query, mask, triggers)

        def query(self, mask, triggers):
            # One worker owns network and disk state. ROS callbacks only enqueue
            # bounded triggers and publish; a slow query cannot block receipt.
            try:
                with H5Client(self.host, self.device_id) as client:
                    evidence = collect_live(client, mask)
            except (OSError, ValueError) as error:
                evidence = dict(complete=False, supported=False, errors=[str(error)],
                                service_id=self.device_id, live=[], first=[])
            evidence.setdefault('host_started_utc', datetime.now(timezone.utc).isoformat())
            evidence.setdefault('host_started_monotonic_ns', time.monotonic_ns())
            evidence.setdefault('requested_failure_mask', mask)
            evidence.setdefault('mcu_failure_marking_enabled', False)
            evidence.setdefault('capture_mode', 'ram_view_only')
            evidence['collector_session'] = self.session
            evidence['collector_state'] = 'running'
            evidence['poll_period_sec'] = self.period
            evidence['host_triggers'] = triggers
            signature = (evidence['complete'], tuple(evidence.get('errors', [])),
                         tuple((s['boot_id'], s['occurrences'], s['reason_mask'])
                               for s in evidence.get('live', [])))
            # Preserve the bounded RAM observations before an onset, without
            # extra MCU queries or nesting prior evidence recursively.
            evidence['preceding_captures'] = list(self.recent_captures)
            self.recent_captures.append({key: evidence[key] for key in (
                'host_started_utc', 'host_started_monotonic_ns', 'complete',
                'errors', 'live')})
            if triggers or signature != self.last_saved_signature:
                self.path = ''
                try:
                    self.path = self.files.save(evidence)
                    self.last_saved_signature = signature
                except (OSError, ValueError) as error:
                    self.file_errors += 1
                    self.get_logger().error('HAT evidence save failed: ' + str(error))
            evidence['file_errors'] = self.file_errors
            evidence['evidence_file'] = self.path
            evidence['retention_evicted'] = self.files.evicted
            try:
                self.files.update_latest(evidence)
            except (OSError, ValueError) as error:
                self.file_errors += 1
                evidence['file_errors'] = self.file_errors
                self.get_logger().error('HAT latest status save failed: ' + str(error))
            return evidence

        def accept(self, evidence):
            self.sequence += 1
            if evidence['complete']:
                self.successes += 1
            self.latest = diagnostic_records(evidence, self.previous, self.session,
                                             self.sequence, self.successes, evidence['evidence_file'])
            self.sample_time = evidence.get('host_started_monotonic_ns', time.monotonic_ns()) / 1e9
            if evidence['complete']:
                self.previous = evidence
            self.publish_evidence()

        def publish_evidence(self):
            if self.latest is None:
                return
            self.sequence += 1
            age_ms = (time.monotonic() - self.sample_time) * 1000
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            for record in self.latest:
                fields = dict(record['fields'], sequence=str(self.sequence))
                level, message_text = record['level'], record['message']
                if fields['quality'] == 'FRESH':
                    fields['sample_age_ms'] = str(age_ms)
                    fields['stale_after_ms'] = str((self.period + 5) * 1000)
                    if age_ms > (self.period + 5) * 1000:
                        fields.update(health='UNKNOWN', quality='STALE', reason='H5_EVIDENCE_EXPIRED')
                        level, message_text = 3, 'H5_EVIDENCE_EXPIRED'
                status = DiagnosticStatus()
                status.name, status.hardware_id = record['name'], record['hardware_id']
                status.level = bytes([level])
                status.message = message_text
                status.values = [KeyValue(key=k, value=v) for k, v in fields.items()]
                message.status.append(status)
            self.publisher.publish(message)

    rclpy.init(args=args)
    node = HatCollector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.worker.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        rclpy.try_shutdown()
