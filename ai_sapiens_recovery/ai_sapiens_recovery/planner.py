"""Single-flight recovery and optional rearming into damping. No ROS or device I/O in this module."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Action:
    kind: str
    target: str = ''


class RecoveryPlanner:
    def __init__(self, motors, controller, retries=3, backoff=2.0,
                 auto_rearm=False, broadcasters=(), rearm_mode='damping'):
        self.motors = tuple(motors)
        if not self.motors or len(set(self.motors)) != len(self.motors):
            raise ValueError('Unique motor component names are required')
        if retries < 1 or not math.isfinite(backoff) or backoff < 0:
            raise ValueError('Invalid retry policy')
        self.controller = controller
        self.auto_rearm = auto_rearm
        if rearm_mode not in ('startup', 'damping'):
            raise ValueError('rearm_mode must be startup or damping')
        self.rearm_mode = rearm_mode
        self.recovered_phase = 'RECOVERED_STARTUP' if rearm_mode == 'startup' else 'RECOVERED_DAMPING'
        self.broadcasters = tuple(broadcasters)
        self.armed = set()
        self.retries, self.backoff = retries, backoff
        self.phase, self.reason = 'OBSERVING', 'Waiting for a healthy running baseline'
        self.pending = None
        self.prepared, self.failures = set(), {}
        self.attempts = {}
        self.next_attempt = 0.0
        self.rc_at = self.tick_at = self.safe_since = None
        self.tick = None
        self.rc_ok = False
        self.invalidated = False

    def rc(self, now, tick, safe, *, freshness_only=False):
        if self.tick is None:
            self.tick = tick
        elif 0 < (tick - self.tick) % 32768 < 16384:
            self.tick_at = now
            self.tick = tick
        self.rc_at, self.rc_ok = now, safe
        if not safe:
            self.safe_since = None
            # CM lifecycle callbacks can briefly pause all hardware reads. A
            # freshness-only gap while torque is OFF pauses preparation; it
            # does not invalidate settings that activation will revalidate.
            preserve = freshness_only and self.phase in (
                'RESTORING', 'WAIT_RELEASE', 'READY_TO_ARM')
            if not preserve and self.phase not in ('OBSERVING', 'RUNNING'):
                self.invalidated = True

    def safe(self, now):
        fresh = (self.rc_ok and self.rc_at is not None and self.tick_at is not None
                 and 0 <= now - self.rc_at <= 0.25 and 0 <= now - self.tick_at <= 0.25)
        if not fresh:
            self.safe_since = None
            return False
        if self.safe_since is None:
            self.safe_since = now
        return now - self.safe_since >= 0.5

    def issue(self, action):
        if self.attempts.get(action, 0) >= self.retries:
            self.phase, self.reason = 'BLOCKED', f'Attempt limit: {action.kind} {action.target}'
            return None
        self.attempts[action] = self.attempts.get(action, 0) + 1
        self.pending = action
        return action

    def complete(self, now, ok):
        if self.pending is None:
            raise RuntimeError('No outstanding operation')
        action, self.pending = self.pending, None
        if not ok:
            self.failures[action] = self.failures.get(action, 0) + 1
            if self.failures[action] >= self.retries:
                self.phase, self.reason = 'BLOCKED', f'Retry limit: {action.kind} {action.target}'
            self.next_attempt = now + self.backoff
            if action.kind in ('activate', 'start_controller', 'start_broadcaster'):
                self.invalidated = True
        elif action.kind == 'configure':
            self.prepared.add(action.target)
        elif action.kind == 'activate':
            self.armed.add(action.target)

    def timeout(self):
        # A client timeout does not cancel a CM lifecycle callback. Keep pending
        # intact and never launch a second operation over the first one.
        self.phase, self.reason = 'BLOCKED', 'Service outcome unknown; inspect CM before restarting'

    def step(self, now, hardware, controllers):
        gate = self.safe(now)
        if self.phase == 'BLOCKED':
            return None
        if not gate and self.phase in ('ARMING', self.recovered_phase):
            self.invalidated = True
        if self.pending is not None:
            return None
        if self.invalidated:
            if self.phase == self.recovered_phase:
                self.failures.clear()
                self.attempts.clear()
            self.prepared.clear()
            self.armed.clear()
            self.phase, self.reason = 'STOPPING', 'Safety gate changed or activation failed; discard prepared state'
            self.invalidated = False
        if self.phase == 'OBSERVING':
            if (gate and all(hardware.get(x) == 'active' for x in self.motors)
                    and controllers.get(self.controller) == 'active'
                    and all(controllers.get(x) == 'active' for x in self.broadcasters)):
                self.phase, self.reason = 'RUNNING', 'Baseline observed'
            return None
        if self.phase in ('RUNNING', self.recovered_phase):
            if (gate and all(hardware.get(x) == 'active' for x in self.motors)
                    and controllers.get(self.controller) == 'active'
                    and all(controllers.get(x) == 'active' for x in self.broadcasters)):
                return None
            self.prepared.clear()
            self.armed.clear()
            self.failures.clear()
            self.attempts.clear()
            self.phase, self.reason = 'STOPPING', 'Control or hardware availability lost'
        if any(hardware.get(x) not in ('active', 'inactive', 'unconfigured') for x in self.motors):
            self.phase, self.reason = 'BLOCKED', 'Missing/finalized motor hardware; restart required'
            return None
        if controllers.get(self.controller) not in ('active', 'inactive'):
            self.phase, self.reason = 'BLOCKED', 'Command controller missing or unconfigured'
            return None
        if any(controllers.get(x) not in ('active', 'inactive') for x in self.broadcasters):
            self.phase, self.reason = 'BLOCKED', 'Required state broadcaster unavailable'
            return None
        if now < self.next_attempt:
            return None
        if self.phase == 'ARMING' and gate:
            for name in self.motors:
                if name in self.armed and hardware[name] != 'active':
                    self.invalidated = True
                    return None
                if name not in self.armed:
                    if hardware[name] != 'inactive':
                        self.invalidated = True
                        return None
                    return self.issue(Action('activate', name))
            for name in self.broadcasters:
                if controllers[name] != 'active':
                    return self.issue(Action('start_broadcaster', name))
            if controllers[self.controller] != 'active':
                return self.issue(Action('start_controller', self.controller))
            self.phase = self.recovered_phase
            self.reason = ('Hardware/controllers active; initial startup behavior restored'
                           if self.rearm_mode == 'startup' else 'Hardware/controllers active; DHI damping latched')
            return None
        if controllers[self.controller] == 'active':
            return self.issue(Action('stop_controller', self.controller))
        for name in self.broadcasters:
            if controllers[name] == 'active':
                return self.issue(Action('stop_broadcaster', name))
        for name in self.motors:
            if hardware[name] == 'active':
                return self.issue(Action('deactivate', name))
        for name in self.motors:
            if name not in self.prepared and hardware[name] == 'inactive':
                return self.issue(Action('cleanup', name))
        if not gate:
            self.phase, self.reason = 'WAIT_RELEASE', 'Waiting for fresh, stable HAT release signal'
            return None
        self.phase, self.reason = 'RESTORING', 'Restoring communication with torque disabled'
        for name in self.motors:
            if name in self.prepared and hardware[name] != 'inactive':
                self.prepared.remove(name)
            if name not in self.prepared:
                return self.issue(Action('configure', name))
        if self.auto_rearm:
            self.phase, self.reason = 'ARMING', f'Reactivating hardware in {self.rearm_mode} mode'
        else:
            self.phase, self.reason = 'READY_TO_ARM', 'Snapshot validated; torque OFF; auto rearm disabled'
        return None
