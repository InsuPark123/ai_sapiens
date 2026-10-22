import json
import struct
import zlib

import pytest

from ai_sapiens_diagnostics import hat_protocol as protocol
from ai_sapiens_diagnostics.hat_protocol import (
    collect, packet, status, crc16, decode_header, decode_slot, decode_journal_record,
)
from ai_sapiens_diagnostics.hat_collector import (
    FailureTrigger, EvidenceFiles, diagnostic_records, stage_observations,
)
from ai_sapiens_diagnostics.adapter import DiagnosticStore


class FakeH5:
    device_id = 200
    endpoint = ('192.168.200.200', 5008)

    def __init__(self, *, legacy=False, race=False, pre_race=False, reboot=False):
        self.legacy, self.race, self.pre_race, self.reboot = legacy, race, pre_race, reboot
        self.generation = self.cookie = 0
        self.commands = []
        self.reads = []
        self.slots_read = 0
        self.sequence = 0

    def read(self, address, length):
        self.reads.append((address, length))
        if address == 0:
            return struct.pack('<HI2B', 6050, 0, 28, 200)
        if address == 80:
            return b'BPRF' + struct.pack('<HHBBHHH', 6, 250, 5, 108, 96, 94, 0)
        if address == 640:
            return struct.pack('<BBHHBB7I', 1, 1, 5, 254, 0, 16, 1, 5, 168, 0, 0, 0, 0)
        if address == 1024:
            if self.legacy:
                raise ValueError('address not supported')
            gen = self.generation + int(self.race and self.slots_read > 0)
            cookie = self.cookie + int(self.pre_race and self.generation > 0)
            return struct.pack('<8I', 0x47444942, 1, 256, 5, cookie, gen, 168, 100)
        if 1056 <= address < 3616:
            i = (address-1056)//256
            w = [0]*64
            if i < 5:
                w[0:8] = [1, 169 if self.reboot else 168, 100, 0, i, 5001+i, 6000000, 1]
                for j in range(8):
                    w[32+j*3:35+j*3] = [10, 100, 90]
                w[63] = 28
            self.slots_read += 1
            return struct.pack('<64I', *w)
        if address == 680:
            raw = bytearray(64)
            struct.pack_into('<I4BIIIHH7III', raw, 0, 0x474f4c46, 1, 2, 8, 0,
                             self.sequence, 168, 90, 28, 112, *([0]*7), 1, 0)
            struct.pack_into('<II', raw, 56, zlib.crc32(raw[:56]), 0x54494d43)
            return bytes([1])+bytes(7)+raw
        return bytes(length)

    def freeze(self, failure_mask=0):
        self.commands.append(('freeze', failure_mask))
        self.generation += 1
        self.cookie = 123
        return self.cookie

    def select_journal(self, sequence):
        self.commands.append(('select', sequence))
        self.sequence = sequence


def fields(sequence=1, failures=0, result=0, session='a', **extra):
    return dict(diagnostics_schema='2', producer_session=session,
                publication_sequence=str(sequence), read_failures=str(failures),
                read_dhi_result=str(result), phase='RUNNING', **extra)


def test_crc_and_stuffing():
    # Protocol 2.0 documented unicast PING vector.
    assert packet(1, 1, b'').hex() == 'fffffd0001030001194e'
    payload = b'\xff\xff\xfd\xfd\xff\xff\xfd'
    frame = packet(200, 0x55, b'\x00'+payload)
    assert status(frame, 200, len(payload)) == payload
    with pytest.raises(ValueError): status(frame[:-1]+bytes([frame[-1]^1]), 200, len(payload))
    with pytest.raises(ValueError): status(frame, 1, len(payload))
    with pytest.raises(ValueError): status(packet(200, 0x55, b'\x04'), 200, 0)


def test_freeze_and_decode_and_bounded_journal():
    fake = FakeH5()
    result = collect(fake, 5, journal_limit=999)
    assert result['complete'] and not result['errors']
    assert fake.commands[0] == ('freeze', 5)
    assert len(result['journal']) == 5
    assert result['live'][2]['port'] == 5003
    assert result['header']['boot_id'] == 168
    assert result['host_completed_monotonic_ns'] >= result['host_started_monotonic_ns']
    assert not any(command[0] not in ('freeze', 'select') for command in fake.commands)


def test_live_failure_keeps_host_intent_without_mcu_fault_or_flash_selection():
    from ai_sapiens_diagnostics.hat_node import collect_live
    fake = FakeH5()
    result = collect_live(fake, 0x1f)
    assert result['complete']
    assert result['failure_mask'] == 0
    assert result['requested_failure_mask'] == 0x1f
    assert result['capture_mode'] == 'ram_view_only'
    assert result['mcu_failure_marking_enabled'] is False
    assert fake.commands == [('freeze', 0)]
    assert len(result['live']) == len(result['first']) == 5
    assert not result['journal']


@pytest.mark.parametrize('mode', ['race', 'pre_race', 'reboot'])
def test_reject_inconsistent_snapshot(mode):
    result = collect(FakeH5(**{mode: True}), 1)
    assert not result['complete'] and result['errors'] and not result['live']


def test_legacy_and_read_only_never_write():
    fake = FakeH5(legacy=True)
    result = collect(fake, 1)
    assert not result['supported'] and not result['complete']
    assert len(result['legacy_profile_hex']) == 5
    assert fake.commands == []
    fake = FakeH5()
    result = collect(fake, allow_freeze=False)
    assert not result['complete'] and fake.commands == []


def test_trigger_deduplicates_and_bounds_pending_and_retires_session():
    t = FailureTrigger()
    assert not t.ingest('other/bus', fields(result=1))
    assert not t.ingest('k1_right_arm/bus', fields())
    assert t.ingest('k1_right_arm/bus', fields(2, 1, 1))
    for n in range(3, 1000):
        assert not t.ingest('k1_right_arm/bus', fields(n, n, 1))
    assert t.ingest('k1_left_leg/bus', fields(2, 1, 1))
    mask, pending = t.take()
    assert mask == 5 and len(pending) == 2
    assert pending[0]['publication_sequence'] == 2
    assert t.take() == (0, [])
    assert not t.ingest('k1_right_arm/bus', fields(1, 0, 0, 'b'))
    assert not t.ingest('k1_right_arm/bus', fields(1001, 1001, 1, 'a'))
    assert t.ingest('k1_right_arm/bus', fields(2, 1, 1, 'b'))


def test_trigger_sparse_failure_and_startup():
    t = FailureTrigger()
    t.ingest('k1_right_arm/bus', fields())
    assert t.ingest('k1_right_arm/bus', fields(2, 1, 0))
    f = fields(); f.update(phase='FAILED', failure_origin='startup')
    assert t.ingest('k1_left_leg/bus', f)


def test_bounded_atomic_files_and_scope(tmp_path):
    files = EvidenceFiles(tmp_path, 2)
    protected = tmp_path/'other-evidence.json'
    protected.write_text('untouched')
    for i in range(5): files.save(dict(number=i))
    saved = sorted(tmp_path.glob('hat-evidence-*.json'))
    assert len(saved) == 2 and files.evicted == 3
    assert [json.loads(path.read_text())['number'] for path in saved] == [3, 4]
    assert protected.read_text() == 'untouched'
    assert not list(tmp_path.glob('*.tmp'))


def test_health_adapter_and_stale_failure_are_not_ok():
    evidence = collect(FakeH5())
    records = diagnostic_records(evidence, None, 'collector', 1, 1)
    store = DiagnosticStore([], producer_timeout=1.0)
    for record in records:
        assert store.ingest(**record, now=0)
    snapshot, _ = store.snapshot(0)
    components = snapshot['components']
    assert len(components) == 5
    assert all(item['component_type'] == 'hat_bridge_port' for item in components)
    assert all(item['health'] == 'UNKNOWN' for item in components)
    # Actual bidirectional counter progress can be reported within its scope.
    changed = collect(FakeH5())
    for slot in changed['live']:
        for stage in slot['stages'].values(): stage['count'] += 1; stage['bytes'] += 1
    records = diagnostic_records(changed, evidence, 'collector', 2, 2)
    for record in records: assert store.ingest(**record, now=0.5)
    assert all(c['health']=='OK' for c in store.snapshot(0.5)[0]['components'])
    failed = dict(complete=False, errors=['timeout'], service_id=200)
    for record in diagnostic_records(failed, changed, 'collector', 3, 2):
        assert store.ingest(**record, now=1)
    assert all(c['health']=='UNKNOWN' for c in store.snapshot(1)[0]['components'])
    assert all(c['reason']=='PRODUCER_EXPIRED' for c in store.snapshot(3)[0]['components'])


def test_socket_ports_held_until_capture_close_and_only_allowed_commands(monkeypatch):
    sockets = []
    class FakeSocket:
        closed = False
        def __init__(self, *args): sockets.append(self)
        def settimeout(self, value): assert 0 < value <= 0.1
        def connect(self, endpoint): assert endpoint == ('127.0.0.1', 5008)
        def send(self, raw): self.raw = raw
        def recv(self, limit):
            expected = int.from_bytes(self.raw[10:12], 'little') if self.raw[7] == 2 else 0
            return packet(200, 0x55, bytes(expected+1))
        def close(self): self.closed = True
    monkeypatch.setattr(protocol.socket, 'socket', FakeSocket)
    with protocol.H5Client('127.0.0.1', 200) as client:
        client.read(0, 8); client.read(80, 16); client.freeze(3)
        assert len(sockets) == 3 and not any(s.closed for s in sockets)
        assert sockets[-1].raw[7] == 3
        assert int.from_bytes(sockets[-1].raw[8:10], 'little') == 1040
        with pytest.raises(ValueError): client.freeze(32)
        with pytest.raises(ValueError): client.read(0, 257)
    assert all(s.closed for s in sockets)


def test_latest_is_atomic_and_does_not_evict_history(tmp_path):
    files = EvidenceFiles(tmp_path, 2)
    first = files.save(dict(complete=True, sequence=1))
    for i in range(10):
        files.update_latest(dict(complete=i != 9, query=i))
    assert json.loads((tmp_path/'latest.json').read_text()) == dict(complete=False, query=9)
    assert len(list(tmp_path.glob('hat-evidence-*.json'))) == 1
    assert files.evicted == 0
    assert not (tmp_path/'latest.tmp').exists()
    assert json.loads(__import__('pathlib').Path(first).read_text())['sequence'] == 1
