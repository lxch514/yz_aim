"""Confirmed missing-frame regression tests. Fake sockets only."""
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import rs00_test as m
from selftest_rs00 import motor


class ConfirmedHeartbeatLoss(unittest.TestCase):
    def setup_motor(self, reference=55.0):
        a = motor()
        logs = []
        a.log = logs.append
        a.report_ms = reference
        a.enable_heartbeat_probe(50)
        return a, logs

    def replay(self, gaps, reference):
        a, logs = self.setup_motor(reference)
        values = lambda index, budget: 1.25 if index == m.IDX_MECH_POS else -.75
        with patch.object(a, 'read_param', side_effect=values) as read:
            for gap in gaps:
                a.stats['gaps'].append(gap)
                a.stats['report'] += 1
                a._heartbeat_probe_on_report(gap)
                while a._heartbeat_probe_queue:
                    a.run_heartbeat_checks()
            return a, logs, read.call_count

    def test_normal_55ms_reports_never_generate_50ms_empty_window_reads(self):
        a, _, calls = self.replay([55.0] * 1000, 55.0)
        self.assertEqual(calls, 0)
        self.assertEqual(m.heartbeat_loss_metrics(a.stats)['estimated_lost'], 0)
        self.assertEqual(a.stats['heartbeat_position_probes'], [])

    def test_one_reported_missing_frame_means_exactly_one_pair(self):
        a, logs, calls = self.replay([50, 50, 100, 50], 50)
        self.assertEqual(calls, 2)
        metrics = m.heartbeat_loss_metrics(a.stats)
        self.assertEqual(metrics['estimated_lost'], 1)
        self.assertEqual(metrics['probe_groups'], 1)
        self.assertTrue(metrics['one_group_per_lost_frame'])
        self.assertIn('当前位置=1.250000rad', '\n'.join(logs))
        self.assertIn('当前速度=-0.750000rad/s', '\n'.join(logs))

    def test_sixteen_reported_missing_frames_mean_sixteen_pairs(self):
        gaps = [55] * 100
        for _ in range(16):
            gaps += [110] + [55] * 100
        a, _, calls = self.replay(gaps, 55)
        self.assertEqual(calls, 32)
        self.assertEqual(m.heartbeat_loss_metrics(a.stats)['estimated_lost'], 16)
        self.assertEqual(len(a.stats['heartbeat_position_probes']), 16)
        self.assertEqual([r['id'] for r in a.stats['heartbeat_position_probes']], list(range(1, 17)))

    def test_historical_one_loss_replay_produces_one_pair_instead_of_121(self):
        p = Path(__file__).parent / 'fixtures' / 'heartbeat_loss_1.json'
        d = json.loads(p.read_text(encoding='utf-8'))
        a, _, calls = self.replay(d['gaps_ms'], d['reference_ms'])
        self.assertEqual(m.heartbeat_loss_metrics(a.stats)['estimated_lost'], d['expected_lost'])
        self.assertEqual(calls, 2)
        self.assertEqual(len(a.stats['heartbeat_position_probes']), 1)

    def test_three_frames_missing_in_one_gap_get_three_distinct_pairs(self):
        a, _, calls = self.replay([55, 220, 55], 55)
        self.assertEqual(calls, 6)
        self.assertEqual(m.heartbeat_loss_metrics(a.stats)['estimated_lost'], 3)
        self.assertEqual([r['slot_in_gap'] for r in a.stats['heartbeat_position_probes']], [1, 2, 3])
        self.assertEqual(len(a.stats['heartbeat_loss_events']), 1)

    def test_silence_and_repeated_service_do_not_requery_same_frame(self):
        a, _ = self.setup_motor()
        with patch.object(a, 'read_param', return_value=0) as read:
            for _ in range(100):
                a.check_heartbeat_probe()
                a.run_heartbeat_checks()
            self.assertEqual(read.call_count, 0)
            a._heartbeat_probe_on_report(110)
            for _ in range(100):
                a.check_heartbeat_probe()
                a.run_heartbeat_checks()
            self.assertEqual(read.call_count, 2)

    def test_diagnostic_reads_wait_for_existing_request_without_overwriting_it(self):
        a, _ = self.setup_motor()
        a._heartbeat_probe_on_report(110)
        a.pending = (m.M_READ, m.IDX_VBUS)
        with patch.object(a, 'read_param', side_effect=AssertionError('nested read')):
            self.assertFalse(a.run_heartbeat_checks())
            self.assertEqual(a.pending, (m.M_READ, m.IDX_VBUS))
        a.pending = None
        with patch.object(a, 'read_param', return_value=0) as read:
            a.run_heartbeat_checks()
            self.assertEqual(read.call_count, 2)

    def test_position_timeout_still_attempts_velocity_and_keeps_one_group(self):
        a, _ = self.setup_motor()
        a._heartbeat_probe_on_report(110)
        with patch.object(a, 'read_param', side_effect=[None, .8]):
            a.run_heartbeat_checks()
        record = a.stats['heartbeat_position_probes'][0]
        self.assertEqual(record['status'], 'partial')
        self.assertEqual(record['readings']['position']['status'], 'timeout')
        self.assertEqual(record['readings']['velocity']['value'], .8)
        self.assertTrue(m.heartbeat_loss_metrics(a.stats)['one_group_per_lost_frame'])

    def test_normal_finish_flushes_each_confirmed_pair_once(self):
        a, _ = self.setup_motor()
        a._heartbeat_probe_on_report(220)
        with patch.object(a, 'read_param', return_value=0) as read:
            a.end_report_window(flush=True)
            self.assertEqual(read.call_count, 6)
        self.assertFalse(a.heartbeat_probe_enabled)
        self.assertEqual(m.heartbeat_loss_metrics(a.stats)['completed_probe_groups'], 3)

    def test_interrupt_marks_unexecuted_records_without_fake_data(self):
        a, _ = self.setup_motor()
        a._heartbeat_probe_on_report(165)
        with patch.object(a, 'read_param', side_effect=AssertionError('read during forced cleanup')):
            a.end_report_window()
        metrics = m.heartbeat_loss_metrics(a.stats)
        self.assertEqual(metrics['estimated_lost'], 2)
        self.assertEqual(metrics['probe_groups'], 2)
        self.assertEqual(metrics['completed_probe_groups'], 0)
        self.assertTrue(all(r['position_rad'] is None for r in a.stats['heartbeat_position_probes']))

    def test_startup_gaps_use_same_rule_and_are_not_processed_twice(self):
        a = motor()
        a.report_ms = 55
        a.stats['gaps'].extend([55, 110, 55])
        a.enable_heartbeat_probe(50)
        a.enable_heartbeat_probe(50)
        self.assertEqual(len(a.stats['heartbeat_position_probes']), 1)
        self.assertEqual(m.heartbeat_loss_metrics(a.stats)['estimated_lost'], 1)

    def test_report_loss_count_equals_number_of_data_rows(self):
        a, _, _ = self.replay([55, 110, 165, 55], 55)
        report = '\n'.join(m.heartbeat_position_probe_report(a.stats))
        self.assertIn('报告估算丢失3帧', report)
        self.assertIn('逐帧诊断记录3组', report)
        self.assertIn('数量一致：是', report)
        self.assertEqual(sum(line.startswith('| ') and line[2:3].isdigit() for line in report.splitlines()), 3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
