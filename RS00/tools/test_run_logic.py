"""Full RS00 lifecycle with a simulated clock and CAN peer. No hardware sockets."""
import contextlib
import io
import json
import socket
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import rs00_test as m
import can_diagnostics as d
from selftest_rs00 import motor


class Peer:
    def __init__(self, a, clock, outage=False, report_period=.05, drop_report_count=0):
        self.a, self.clock = a, clock
        self.next_tx = 0
        self.enabled, self.report_on = False, False
        self.position = 1.25
        self.next_report = None
        self.outage = outage
        self.motion_at = None
        self.report_period = report_period
        self.drop_remaining = drop_report_count
        self.motion_report_index = 0

    def receive(self, mode, payload):
        cid = m.build_id(mode, 127, m.MASTER_ID)
        if mode in (m.M_FEEDBACK, m.M_REPORT):
            cid |= (2 if self.enabled else 0) << 22
        self.a.arrival_ns = None
        self.a._on_frame(cid, payload)

    def feedback(self, mode=m.M_FEEDBACK):
        vel = 1.0 if self.enabled else 0.0
        raw = struct.pack('>HHHH', m.f2u(self.position, m.P_MIN, m.P_MAX),
                          m.f2u(vel, m.V_MIN, m.V_MAX), 32767, 310)
        self.receive(mode, raw)

    def disconnected(self):
        return self.outage and self.motion_at is not None and self.clock[0] > self.motion_at + .15

    def pump(self, timeout):
        if self.next_tx < len(self.a.sock.sent):
            frame = self.a.sock.sent[self.next_tx]
            self.next_tx += 1
            self.clock[0] += .0005
            if self.disconnected():
                return
            cid = struct.unpack('=I', frame[:4])[0]
            mode = (cid >> 24) & 31
            body = frame[8:16]
            index = struct.unpack('<H', body[:2])[0]
            if mode == m.M_READ:
                val = {m.IDX_MECH_POS: self.position, m.IDX_MECH_VEL: 1.0 if self.enabled else 0,
                       m.IDX_VBUS: 23.1, m.IDX_FAULT: 0}.get(index, 0)
                raw = struct.pack('<I', int(val)) if index == m.IDX_FAULT else struct.pack('<f', val)
                self.receive(m.M_READ, struct.pack('<HH', index, 0) + raw)
            elif mode == m.M_WRITE:
                if index == m.IDX_LOC_REF:
                    self.position = struct.unpack('<f', body[4:8])[0]
                    if self.motion_at is None:
                        self.motion_at = self.clock[0]
                self.feedback()
            elif mode == m.M_ENABLE:
                self.enabled = True
                self.feedback()
            elif mode == m.M_STOP:
                self.enabled = False
                self.feedback()
            elif mode == m.M_REPORT:
                self.report_on = body[6] == 1
                self.next_report = self.clock[0] + self.report_period if self.report_on else None
                self.feedback()
            return
        if self.report_on and not self.disconnected() and self.next_report <= self.clock[0] + timeout:
            self.clock[0] = max(self.clock[0], self.next_report)
            self.next_report += self.report_period
            if self.motion_at is not None:
                self.motion_report_index += 1
                if self.drop_remaining and self.motion_report_index % 3 == 0:
                    self.drop_remaining -= 1
                    return
            self.feedback(m.M_REPORT)
        else:
            self.clock[0] += max(.0001, timeout)


class Lifecycle(unittest.TestCase):
    def run_simulation(self, argv, outage=False, state='ERROR-ACTIVE', xstats_error=False, report_period=.05, drop_report_count=0):
        if sys.platform != 'linux':
            self.skipTest('full lifecycle requires Linux fcntl')
        a = motor()
        clock = [100.0]
        peer = Peer(a, clock, outage, report_period, drop_report_count)
        initial = {'state': state, 'xstats': {k: 0 for k in d.XSTATS},
                   'bittiming': {'bitrate': 1000000, 'sample_point': .828, 'sjw': 8}}
        def snapshot(iface):
            row = json.loads(json.dumps(initial))
            if xstats_error and clock[0] > 100.5:
                row['xstats']['error_warning'] = 1
            return row
        original_exists = Path.exists
        def exists(path):
            return True if str(path) == '/sys/class/net/can0' else original_exists(path)
        def factory(iface, motor_id, frames, log):
            a.frames = frames
            a.log = log
            return a
        with tempfile.TemporaryDirectory() as tmp, patch.object(m, 'ROOT', Path(tmp)), \
             patch.object(m, 'Motor', side_effect=factory), patch.object(a, 'pump', side_effect=peer.pump), \
             patch.object(m.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(m.os, 'geteuid', return_value=0), patch.object(m.Path, 'exists', exists), \
             patch.object(m, 'read_bittiming', return_value=('ERROR-ACTIVE', .828)), \
             patch.object(m, 'read_sys_stats', return_value={'rx_errors': 0, 'tx_errors': 0}), \
             patch.object(d, 'snapshot', side_effect=snapshot), contextlib.redirect_stdout(io.StringIO()):
            code = m.main(argv)
            directory = next((Path(tmp) / 'logs').iterdir())
            summary = json.loads((directory / 'summary.json').read_text(encoding='utf-8'))
            report = next(directory.glob('RS00*.md')).read_text(encoding='utf-8')
            log = (directory / 'test.log').read_text(encoding='utf-8')
            return code, summary, report, log, a.sock.sent

    def test_successful_full_cycle_and_stop_confirmation(self):
        code, s, report, log, sent = self.run_simulation(
            ['--motor-id', '127', '--minutes', '.001', '--fwd-ms', '100', '--rev-ms', '100', '--hold-ms', '0'])
        self.assertEqual(code, 0, s)
        self.assertEqual(s['outcome'], 'completed')
        self.assertEqual(s['validation']['status'], 'pass')
        self.assertTrue(s['motor_stop_confirmed'])
        self.assertEqual(len(s['cycles']), 1)
        self.assertIn('心跳丢失逐帧', report)
        self.assertEqual(s['stats']['write_req'], s['stats']['write_ack'])
        self.assertTrue(sent)

    def test_continuous_outage_aborts_instead_of_finishing_timer(self):
        code, s, report, log, sent = self.run_simulation(
            ['--motor-id', '127', '--minutes', '.2', '--fwd-ms', '3000', '--rev-ms', '3000',
             '--hold-ms', '0', '--motor-lost-sec', '2'], outage=True)
        self.assertEqual(code, 1)
        self.assertEqual(s['outcome'], 'failed')
        self.assertEqual(s['validation']['status'], 'fail')
        self.assertIn('motor_offline', s['validation']['issues'])
        self.assertFalse(s['motor_stop_confirmed'])
        self.assertIn('中止测试', s['reason'])

    def test_controller_errors_fail_validation_even_with_all_replies(self):
        code, s, *_ = self.run_simulation(
            ['--motor-id', '127', '--minutes', '.001', '--fwd-ms', '100', '--rev-ms', '100', '--hold-ms', '0'],
            xstats_error=True)
        self.assertEqual(s['outcome'], 'completed')
        self.assertEqual(s['stats']['write_timeout'] + s['stats']['read_timeout'], 0)
        self.assertEqual(code, 1)
        self.assertIn('can_controller_errors', s['validation']['issues'])

    def test_unhealthy_start_does_not_send_enable(self):
        code, s, _, _, sent = self.run_simulation(['--motor-id', '127'], state='ERROR-PASSIVE')
        self.assertEqual(code, 1)
        self.assertEqual(s['outcome'], 'failed')
        self.assertEqual(sent, [])

    def test_regular_55ms_heartbeat_does_not_create_false_loss_checks(self):
        code, s, report, log, sent = self.run_simulation(
            ['--motor-id', '127', '--minutes', '.02', '--fwd-ms', '100', '--rev-ms', '100', '--hold-ms', '0'],
            report_period=.055)
        self.assertEqual(code, 0, s['validation'])
        self.assertTrue(s['heartbeat']['loss_metrics']['one_group_per_lost_frame'])
        probes = s['stats']['heartbeat_position_probes']
        self.assertEqual(len(probes), 0)
        self.assertEqual(s['heartbeat']['loss_metrics']['estimated_lost'], 0)
        self.assertNotIn('心跳丢失#', log)

    def test_main_one_lost_frame_yields_one_completed_data_group(self):
        code, s, report, log, _ = self.run_simulation(
            ['--motor-id', '127', '--minutes', '.02', '--fwd-ms', '100', '--rev-ms', '100', '--hold-ms', '0'],
            report_period=.055, drop_report_count=1)
        self.assertEqual(code, 1)  # A deliberately lost frame exceeds 0.5% in this very short test.
        metrics = s['heartbeat']['loss_metrics']
        self.assertEqual(metrics['estimated_lost'], 1)
        self.assertEqual(metrics['probe_groups'], 1)
        self.assertEqual(metrics['completed_probe_groups'], 1)
        self.assertIn('报告估算丢失1帧', report)
        self.assertIn('当前位置=', log)
        self.assertIn('当前速度=', log)

    def test_main_sixteen_lost_frames_yield_sixteen_completed_data_groups(self):
        _, s, report, log, _ = self.run_simulation(
            ['--motor-id', '127', '--minutes', '.06', '--fwd-ms', '100', '--rev-ms', '100', '--hold-ms', '0'],
            report_period=.055, drop_report_count=16)
        metrics = s['heartbeat']['loss_metrics']
        self.assertEqual(metrics['estimated_lost'], 16)
        self.assertEqual(metrics['probe_groups'], 16)
        self.assertEqual(metrics['completed_probe_groups'], 16)
        self.assertIn('逐帧诊断记录16组', report)
        self.assertEqual(sum('读当前位置：' in line for line in log.splitlines()), 16)
        self.assertEqual(sum('读当前速度：' in line for line in log.splitlines()), 16)



if __name__ == '__main__':
    unittest.main(verbosity=2)
