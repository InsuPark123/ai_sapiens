"""Bounded H5 control-service diagnostics, independent of the real-time SDK.

Only READ and two explicitly scoped RAM commands are exposed. No motor-bus
traffic, baud configuration, profile reset or journal clear is implemented.
"""
from contextlib import AbstractContextManager
from datetime import datetime, timezone
import secrets
import socket
import struct
import time
import zlib

BASE, COMMAND, HEADER_SIZE, SLOT_SIZE, CHANNELS = 1024, 1040, 32, 256, 5
STAGES = ('request', 'tx_start', 'dma_tc', 'uart_tc', 'rx', 'net_submit', 'net_done', 'net_fail')
REASONS = {1: 'tx_dma_error', 2: 'tx_completion_timeout', 3: 'rx_dma_error',
           4: 'uart_error', 5: 'rx_geometry_ambiguous', 6: 'rx_overflow',
           7: 'network_return_failed', 8: 'network_return_aborted',
           9: 'host_read_failure_observed_later', 10: 'network_receive_error'}


def crc16(data):
    crc = 0
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x8005 if crc & 0x8000 else 0)) & 0xffff
    return crc


def packet(device_id, instruction, parameters):
    body = bytes([instruction]) + parameters
    body = body.replace(b'\xff\xff\xfd', b'\xff\xff\xfd\xfd')
    frame = b'\xff\xff\xfd\x00' + bytes([device_id]) + struct.pack('<H', len(body) + 2) + body
    return frame + struct.pack('<H', crc16(frame))


def status(frame, device_id, expected):
    if (len(frame) < 11 or frame[:4] != b'\xff\xff\xfd\x00' or
            frame[4] != device_id or len(frame) != 7 + int.from_bytes(frame[5:7], 'little') or
            crc16(frame[:-2]) != int.from_bytes(frame[-2:], 'little')):
        raise ValueError('invalid H5 status identity, length or CRC')
    body = frame[7:-2].replace(b'\xff\xff\xfd\xfd', b'\xff\xff\xfd')
    if body[0] != 0x55 or body[1] or len(body) != expected + 2:
        raise ValueError('H5 status error or unexpected payload length')
    return body[2:]


class H5Client(AbstractContextManager):
    """One fresh source port per transaction; keep ports reserved until close.

    Protocol 2 has no request sequence in status packets. A delayed reply must
    not satisfy the next equal-length read from a different CT address.
    """
    def __init__(self, host, device_id, port=5008, timeout=0.10, budget=3.0):
        if not 0 <= device_id <= 252 or port != 5008:
            raise ValueError('explicit H5 service ID and UDP5008 required')
        self.endpoint = (socket.gethostbyname(host), port)
        self.device_id = device_id
        self.timeout = timeout
        self.deadline = time.monotonic() + budget
        self.sockets = []

    def __exit__(self, *args):
        for channel in self.sockets:
            channel.close()
        self.sockets.clear()

    def _exchange(self, instruction, parameters, expected):
        remaining = self.deadline - time.monotonic()
        if remaining <= 0 or len(self.sockets) >= 64:
            raise TimeoutError('H5 collection budget exhausted')
        channel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockets.append(channel)
        channel.settimeout(min(self.timeout, remaining))
        channel.connect(self.endpoint)
        channel.send(packet(self.device_id, instruction, parameters))
        return status(channel.recv(2048), self.device_id, expected)

    def read(self, address, length):
        if length < 1 or length > 256 or address < 0 or address + length > 3616:
            raise ValueError('read outside bounded diagnostic view')
        return self._exchange(2, struct.pack('<HH', address, length), length)

    def freeze(self, failure_mask=0):
        if not 0 <= failure_mask < 32:
            raise ValueError('invalid channel mask')
        cookie = secrets.randbits(32)
        self._exchange(3, struct.pack('<H', COMMAND) +
                       bytes([0xb1, int(bool(failure_mask)), failure_mask, 0]) +
                       struct.pack('<I', cookie), 0)
        return cookie

    def select_journal(self, sequence):
        if not 0 <= sequence <= 0xffffffff:
            raise ValueError('invalid journal sequence')
        return self._exchange(3, struct.pack('<HI', 676, sequence), 0)


def decode_header(raw):
    words = struct.unpack('<8I', raw)
    if words[:4] != (0x47444942, 1, SLOT_SIZE, CHANNELS):
        raise ValueError('unsupported H5 bridge diagnostic ABI')
    return dict(schema=words[1], cookie=words[4], generation=words[5], boot_id=words[6], uptime_ms=words[7])


def decode_slot(raw):
    w = struct.unpack('<64I', raw)
    return dict(event_sequence=w[0], boot_id=w[1], uptime_ms=w[2],
                trigger=w[3], trigger_name=REASONS.get(w[3], 'none'), channel=w[4],
                port=w[5], applied_baud=w[6], protocol=w[7] & 255, socket=w[7] >> 8,
                tx_state=w[8], tx_deadline_ms=w[9], tx_length=w[10], last_run_ms=w[11],
                uart_isr=w[12], uart_cr1=w[13], uart_cr3=w[14], tx_csr=w[15],
                tx_cbr1=w[16], tx_ccr=w[17], rx_wraps=w[18], rx_csr_before=w[19],
                rx_remaining_before=w[20], rx_csr_after=w[21], rx_remaining_after=w[22],
                rx_produced=w[23], rx_consumed=w[24], rx_epoch=w[25],
                socket_status=w[26], socket_irq=w[27] & 65535, last_send_irqs=w[27] >> 16, peer_ip=list(raw[112:116]),
                peer_port=w[29] & 65535, peer_valid=bool(w[29] >> 16),
                network_state=w[30], network_staged=w[31],
                stages={name: dict(count=w[32+i*3], bytes=w[33+i*3], last_ms=w[34+i*3])
                        for i, name in enumerate(STAGES)},
                occurrences=w[56], reason_mask=w[57], last_fault_ms=w[58],
                recv_failures=w[59], ambiguous_delivery=w[60], busy_polls=w[61],
                usb_debug=bool(w[62]), last_recv_failure_ms=w[63], raw_hex=raw.hex())


def decode_journal_status(raw):
    return dict(format=raw[0], state=raw[1], count=int.from_bytes(raw[2:4], 'little'),
                capacity=int.from_bytes(raw[4:6], 'little'), queue_depth=raw[6],
                queue_capacity=raw[7], **dict(zip(
                    ('oldest_sequence', 'newest_sequence', 'boot_id', 'dropped', 'corruption',
                     'flash_errors', 'deferred_erase'), struct.unpack('<7I', raw[8:36]))))


def decode_journal_record(raw):
    if (len(raw) != 64 or int.from_bytes(raw[:4], 'little') != 0x474f4c46 or
            raw[4] != 1 or int.from_bytes(raw[60:64], 'little') != 0x54494d43 or
            zlib.crc32(raw[:56]) != int.from_bytes(raw[56:60], 'little')):
        raise ValueError('invalid journal format, CRC or commit marker')
    return dict(format=raw[4], severity=raw[5], source=raw[6], flags=raw[7],
                firmware_version=int.from_bytes(raw[20:22], 'little'),
                sequence=int.from_bytes(raw[8:12], 'little'),
                boot_id=int.from_bytes(raw[12:16], 'little'),
                uptime_ms=int.from_bytes(raw[16:20], 'little'),
                event_code=int.from_bytes(raw[22:24], 'little'),
                details=list(struct.unpack('<7I', raw[24:52])),
                repeats=int.from_bytes(raw[52:56], 'little'), raw_hex=raw.hex())


def collect(client, failure_mask=0, journal_limit=0, allow_freeze=True):
    """Return partial evidence on failure; a missed query is never healthy.

    Legacy firmware is queried read-only and explicitly marked unsupported.
    RAM freeze is sent only after checking identity and the versioned header.
    """
    result = dict(host_started_utc=datetime.now(timezone.utc).isoformat(),
                  host_started_monotonic_ns=time.monotonic_ns(), service_id=client.device_id,
                  endpoint=client.endpoint, failure_mask=failure_mask, supported=False,
                  complete=False, errors=[], live=[], first=[], journal=[])
    try:
        identity = client.read(0, 8)
        result['identity_hex'] = identity.hex()
        result['identity'] = dict(model=int.from_bytes(identity[:2], 'little'),
                                  firmware_version=identity[6], device_id=identity[7])
        if result['identity']['model'] != 6050 or identity[7] != client.device_id:
            raise ValueError('configured endpoint is not the expected H5 board')
        profile = client.read(80, 16)
        result['profile_header_hex'] = profile.hex()
        if profile[:4] != b'BPRF' or int.from_bytes(profile[4:6], 'little') != 6 or profile[8:10] != bytes([5, 108]):
            raise ValueError('unsupported legacy bridge profile schema')
        result['journal_status'] = decode_journal_status(client.read(640, 36))
        try:
            decode_header(client.read(BASE, HEADER_SIZE))
        except (OSError, ValueError) as error:
            result['errors'].append('extended diagnostics unavailable: ' + str(error))
            result['legacy_profile_hex'] = [client.read(96+i*108, 108).hex() for i in range(5)]
            return result
        result['supported'] = True
        if not allow_freeze:
            result['errors'].append('RAM freeze disabled; no fresh snapshot collected')
            return result
        cookie = client.freeze(failure_mask)
        before = client.read(BASE, HEADER_SIZE)
        result['header'] = decode_header(before)
        if result['header']['cookie'] != cookie:
            raise ValueError('RAM capture replaced before first read')
        slots = [decode_slot(client.read(BASE+HEADER_SIZE+i*SLOT_SIZE, SLOT_SIZE))
                 for i in range(CHANNELS*2)]
        after = client.read(BASE, HEADER_SIZE)
        if before != after:
            raise ValueError('RAM view replaced by another diagnostic client')
        boot = result['header']['boot_id']
        for i, slot in enumerate(slots):
            if i < CHANNELS or slot['event_sequence']:
                if (slot['channel'] != i % CHANNELS or slot['boot_id'] != boot or
                        slot['port'] != 5001 + i % CHANNELS or slot['protocol'] != 1):
                    raise ValueError('inconsistent snapshot channel or MCU boot')
        if result['journal_status']['boot_id'] != boot:
            raise ValueError('MCU rebooted during collection')
        result['live'], result['first'] = slots[:CHANNELS], slots[CHANNELS:]
        result['complete'] = True
        # Bounded recent journal copy, optional on fault captures only. Selection
        # affects RAM, never clears the journal. Verify returned sequence against
        # concurrent collectors and tolerate deferred Flash writes as not present.
        newest = result['journal_status']['newest_sequence']
        for sequence in range(newest, max(0, newest-min(8, journal_limit)), -1):
            client.select_journal(sequence)
            record = client.read(680, 72)
            if record[0] != 1:
                result['errors'].append('journal selection not ready/found: ' + str(sequence))
                continue
            decoded = decode_journal_record(record[8:72])
            if decoded['sequence'] != sequence:
                raise ValueError('journal selector changed concurrently')
            result['journal'].append(decoded)
    except (OSError, ValueError) as error:
        result['errors'].append(str(error))
    finally:
        result['host_completed_utc'] = datetime.now(timezone.utc).isoformat()
        result['host_completed_monotonic_ns'] = time.monotonic_ns()
    return result
