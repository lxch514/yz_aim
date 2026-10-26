"""Offline regression tests. Never opens a real CAN socket or enables equipment."""
import errno
import json
import socket
import struct
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import rs00_test as m
import can_diagnostics as d


class Sock:
    def __init__(self):
        self.sent = []
    def bind(self, *a): pass
    def setblocking(self, *a): pass
    def setsockopt(self, *a): pass
    def getsockopt(self, *a): return 1
    def send(self, frame):
        self.sent.append(frame)
        return len(frame)
    def close(self): pass


class Frames:
    def write(self, *a): pass
    def note(self, *a): pass


def motor():
    sock = Sock()
    with patch.object(socket, 'AF_CAN', 29, create=True), patch.object(socket, 'socket', return_value=sock):
        return m.Motor('can0', 127, Frames(), lambda *a: None)


class Regression(unittest.TestCase):
    def test_motor_rejects_unrelated_and_short_frames(self):
        a = motor()
        for cid, payload in [(m.build_id(2, 126, 253), bytes(8)),
                             (m.build_id(2, 127, 252), bytes(8)),
                             (m.build_id(2, 127, 253), bytes(7))]:
            a._on_frame(cid, payload)
        self.assertIsNone(a.last_rx_motor)
        self.assertEqual(a.stats['feedback'], 0)
        a._on_frame(m.build_id(2, 127, 253), bytes(8))
        self.assertEqual(a.stats['feedback'], 1)

    def test_stale_read_cannot_ack_new_request(self):
        a = motor()
        a.pending = (m.M_READ, m.IDX_MECH_POS)
        a.pending_sent_ns = 100
        payload = struct.pack('<HHf', m.IDX_MECH_POS, 0, 1.25)
        a.arrival_ns = 99
        a._on_frame(m.build_id(m.M_READ, 127, 253), payload)
        self.assertIsNone(a.reply)
        a.arrival_ns = 101
        a._on_frame(m.build_id(m.M_READ, 127, 253), payload)
        self.assertEqual(a.reply, payload)

    def test_kernel_heartbeat_gap_separate_from_processing(self):
        a = motor()
        for ns in (10000000000, 10055000000, 10165000000):
            a.arrival_ns = ns
            a._on_frame(m.build_id(m.M_REPORT, 127, 253), bytes(8))
        self.assertAlmostEqual(a.gaps[0], 55)
        self.assertAlmostEqual(a.gaps[1], 110)
        self.assertLess(max(a.app_gaps), 10)

    def test_send_failures_are_not_response_timeouts(self):
        a = motor()
        with patch.object(a, '_send', return_value=False):
            self.assertFalse(a.write_param(m.IDX_LOC_REF, bytes(4), 100, 'test'))
            self.assertIsNone(a.read_param(m.IDX_MECH_POS, 50))
        self.assertEqual(a.stats['write_send_fail'], 1)
        self.assertEqual(a.stats['read_send_fail'], 1)
        self.assertEqual(a.stats['write_timeout'] + a.stats['read_timeout'], 0)

    def test_cleanup_does_not_extend_heartbeat_monitoring_window(self):
        a = motor()
        a.last_report = time.monotonic() - .055
        a.end_report_window()
        tail = a.report_tail_ms
        time.sleep(.01)
        a.end_report_window()
        self.assertEqual(a.report_tail_ms, tail)
        self.assertTrue(50 <= tail < 80)

    def test_bounded_queue_pressure_and_permanent_error(self):
        frame = struct.pack('=IB3x8s', 0x804E8464, 8, bytes(8))
        sock = Sock()
        with patch.object(sock, 'send', side_effect=[BlockingIOError(errno.EAGAIN, 'busy'), 16]):
            self.assertEqual(d.send_bounded(sock, frame), 1)
        with patch.object(sock, 'send', side_effect=OSError(errno.ENETDOWN, 'down')) as send:
            with self.assertRaises(OSError):
                d.send_bounded(sock, frame)
            self.assertEqual(send.call_count, 1)
        start = time.monotonic()
        with patch.object(sock, 'send', side_effect=BlockingIOError(errno.EAGAIN, 'busy')):
            with self.assertRaises(OSError):
                d.send_bounded(sock, frame)
        self.assertLess(time.monotonic() - start, .1)

    def test_rx_timestamp_and_socket_drops(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(d, 'snapshot', return_value={'state': 'ERROR-ACTIVE'}):
            diag = d.Diagnostics(tmp, 'can0')
            now = time.time_ns()
            sec, ns = divmod(now, 1000000000)
            stamp = diag.received([(socket.SOL_SOCKET, d.SO_TIMESTAMPNS, struct.pack('@ll', sec, ns)),
                                   (socket.SOL_SOCKET, d.SO_RXQ_OVFL, struct.pack('=I', 3))])
            self.assertEqual(stamp, now)
            self.assertEqual(diag.finish()['socket_drops'], 3)

    def test_check_only_never_enables_motor(self):
        a = motor()
        a.read_param = lambda index, *args: 0 if index == m.IDX_FAULT else 1.25
        a.drain = lambda *args: None
        with tempfile.TemporaryDirectory() as tmp, patch.object(m, 'ROOT', Path(tmp)), \
             patch.object(m.os, 'geteuid', return_value=0, create=True), \
             patch.object(m.Path, 'exists', return_value=True), \
             patch.object(m, 'Motor', return_value=a), \
             patch.object(m, 'read_bittiming', return_value=('ERROR-ACTIVE', .828)), \
             patch.object(m, 'read_sys_stats', return_value={}), \
             patch.object(a, 'enable', side_effect=AssertionError('enable called')), \
             patch.object(a, 'stop', side_effect=AssertionError('stop called')):
            # fcntl is Linux-only: this full lifecycle test is run on the board.
            if sys.platform != 'linux':
                self.skipTest('full lifecycle requires Linux fcntl')
            self.assertEqual(m.main(['--check-only', '--motor-id', '127']), 0)

if __name__ == '__main__':
    unittest.main(verbosity=2)
