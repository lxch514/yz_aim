#!/usr/bin/env python3
"""RS00 关节电机位置模式 + 两台 X6 PLUS 推进风机 通信长测（同一路 CAN0）。

电机：正转 3s → 停止保持 1s → 反转 3s → 下一组；轮询 70ms，写等应答 100ms，读等应答 50ms。
风机：电调 0x01 用 CH1、0x02 用 CH2，初始化后以 50Hz 持续下发油门，默认油门 500。
"""
import argparse
import json
import os
import select
import signal
import socket
import struct
import subprocess
import sys
import time
from collections import deque
from can_diagnostics import Diagnostics, send_bounded, diagnostic_report, CAN_ERR_FLAG, CAN_RTR_FLAG
from datetime import datetime
from pathlib import Path

sys.dont_write_bytecode = True

# 协议量程
P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -33.0, 33.0
T_MIN, T_MAX = -14.0, 14.0
MASTER_ID = 0xFD

# SocketCAN 的 can_id 最高位为扩展帧标志，不置位会按 11 位标准帧发出
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

# 通信类型
M_FEEDBACK = 0x02
M_ENABLE = 0x03
M_STOP = 0x04
M_READ = 0x11
M_WRITE = 0x12
M_REPORT = 0x18

# 参数索引
IDX_RUN_MODE = 0x7005
IDX_LOC_REF = 0x7016
IDX_LIMIT_SPD = 0x7017
IDX_MECH_POS = 0x7019
IDX_MECH_VEL = 0x701B
IDX_VBUS = 0x701C
IDX_VEL_MAX = 0x7024
IDX_ACC_SET = 0x7025
IDX_EPS_TIME = 0x7026
IDX_FAULT = 0x3022

RUN_MODE_PP = 1
RUN_MODE_CSP = 5
EPS_REPORT_MS = 50.0
ROOT = Path(__file__).resolve().parent


def eps_time_value(report_ms):
    """把目标上报周期换算成 0x7026 的写入值：1 对应 10ms，之后每 +1 递增 5ms。"""
    steps = int(round((float(report_ms) - 10.0) / 5.0))
    return max(1, min(0xFFFF, 1 + steps))

# X6 PLUS 电调（HW-UAVCAN，29 位扩展帧）
HW_SRC_NODE = 0x64
HW_ESC_LEFT = 0x01
HW_ESC_RIGHT = 0x02
HW_PRIO_HIGH = 0
HW_PRIO_MED = 16
HW_MSG_READY = 20012
HW_MSG_GET_ID = 20013
HW_MSG_TELEM1 = 20050
HW_MSG_TELEM2 = 20051
HW_MSG_TELEM3 = 20052
HW_MSG_RAW = 20100
HW_SRV_THR_SELECT = 215
HW_TAIL = 0xC0
HW_MAX_THROTTLE = 8191
HW_TYPE_LO, HW_TYPE_HI = 0x4E00, 0x4EFF
HW_SAMPLE_POINT = 0.833
HW_MAX_SEND_FAIL = 20


def now_iso():
    return datetime.now().isoformat(timespec='milliseconds')


def build_id(mode, data, target):
    return ((mode & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (target & 0xFF)


def u2f(raw, low, high):
    return raw / 65535.0 * (high - low) + low


def f2u(value, low, high):
    value = min(max(value, low), high)
    return int((value - low) * 65535.0 / (high - low))


def hw_message_id(priority, type_id, source):
    return ((priority & 0x1F) << 24) | ((type_id & 0xFFFF) << 8) | (source & 0x7F)


def hw_service_id(priority, service, dest, source):
    return (((priority & 0x1F) << 24) | ((service & 0xFF) << 16) | (1 << 15) |
            ((dest & 0x7F) << 8) | (1 << 7) | (source & 0x7F))


def hw_tail(tid):
    return HW_TAIL | (tid & 0x1F)


def hw_sample_ok(sample_point):
    """采样点是否与参考风机程序一致（允许硬件量化的 ±0.01 偏差）。"""
    return sample_point is not None and abs(sample_point - HW_SAMPLE_POINT) <= 0.01


def hw_is_frame(can_id):
    """按数据区类型码区分风机帧与 RS00 帧。"""
    field = (can_id >> 8) & 0xFFFF
    if HW_TYPE_LO <= field <= HW_TYPE_HI:
        return True
    return ((can_id >> 16) & 0xFF) in (0xD2, 0xD7)


def hw_pack4(ch0, ch1, ch2, ch3):
    """四路 14 位油门打包成 7 字节，位序与原厂示例一致。"""
    out = bytearray(7)
    index = 0
    for value in (ch0, ch1, ch2, ch3):
        value &= 0x3FFF
        bits = [(value >> b) & 1 for b in range(7, -1, -1)]
        bits += [(value >> b) & 1 for b in range(13, 7, -1)]
        for bit in bits:
            if bit:
                out[index // 8] |= 0x80 >> (index % 8)
            index += 1
    return bytes(out)


def percentile(values, q):
    if not values:
        return 0.0
    data = sorted(values)
    k = min(len(data) - 1, max(0, int(round((len(data) - 1) * q))))
    return data[k]


def heartbeat_gap_loss_count(gap_ms, reference_ms):
    """The single loss rule shared by live diagnostics, summary and validation."""
    if reference_ms <= 0 or gap_ms is None or gap_ms <= 1.5 * reference_ms:
        return 0
    return max(0, int(round(gap_ms / reference_ms)) - 1)


def heartbeat_loss_metrics(stats):
    measured = percentile(stats.get('gaps', []), .5) if stats.get('gaps') else 0.0
    tracking = stats.get('heartbeat_loss_tracking_ready', False)
    reference = stats.get('heartbeat_loss_reference_ms', measured) or measured
    lost = (sum(e['missing_count'] for e in stats.get('heartbeat_loss_events', [])) if tracking else
            sum(heartbeat_gap_loss_count(g, reference) for g in stats.get('gaps', [])))
    records = stats.get('heartbeat_position_probes', [])
    completed = sum(r['status'] in ('success', 'partial', 'failed') for r in records)
    return {'observed_median_ms': measured, 'reference_ms': reference, 'estimated_lost': lost,
            'expected_reports': stats.get('report', 0) + lost, 'probe_groups': len(records),
            'completed_probe_groups': completed, 'uncompleted_probe_groups': len(records) - completed,
            'one_group_per_lost_frame': (len(records) == lost) if tracking else None,
            'tracking_enabled': tracking}


class Logger:
    def __init__(self, path):
        self.fp = open(path, 'a', buffering=1, encoding='utf-8')

    def __call__(self, text):
        line = '[{}] {}'.format(datetime.now().strftime('%H:%M:%S'), text)
        print(line, flush=True)
        self.fp.write(line + '\n')

    def close(self):
        self.fp.close()


class FrameLog:
    """逐帧记录收发，格式与 yz_aim 的 can-frames.log 保持一致。"""

    def __init__(self, path, ifname, t0):
        self.fp = open(path, 'w', buffering=1, encoding='utf-8')
        self.ifname = ifname
        self.t0 = t0
        self.count = 0
        self.fp.write('# CAN_START {} interface={}\n'.format(now_iso(), ifname))

    def write(self, direction, can_id, payload, note=''):
        self.count += 1
        elapsed = time.monotonic() - self.t0
        text = ' '.join('{:02X}'.format(b) for b in payload)
        line = '[CAN {} +{:.6f}s #{:d}] {:<8} ID={:08X} DLC={} {}'.format(
            self.ifname, elapsed, self.count, direction, can_id, len(payload), text)
        if note:
            line += ' | ' + note
        self.fp.write(line + '\n')

    def note(self, text):
        self.fp.write('[CAN监测 {}] {}\n'.format(datetime.now().strftime('%Y-%m-%d %H:%M:%S'), text))

    def close(self):
        self.fp.close()


class Fan:
    """两台 X6 PLUS 电调：初始化、50Hz 油门维持、遥测统计。"""

    def __init__(self, sock, frames, log, throttle, hz, nodes=(HW_ESC_LEFT, HW_ESC_RIGHT)):
        self.sock = sock
        self.frames = frames
        self.log = log
        self.target = max(0, min(HW_MAX_THROTTLE, int(throttle)))
        self.current = 0
        self.period = 1.0 / float(hz)
        self.tids = {}
        self.diag = None
        self.shutdown_status = '未执行'
        self.next_due = time.monotonic()
        self.last_tx = None
        self.logged_at = {}
        self.enabled = False
        self.active = False
        self.consecutive_fail = 0
        self.faulted = False
        self.stats = {'tx': 0, 'tx_fail': 0, 'late': 0, 'telem': 0, 'dup': 0, 'other': 0,
                      'period_ms': [], 'raw_tx': 0, 'send_retries': 0}
        self.nodes = {}
        for node in nodes:
            self.nodes[node] = {'node': node, 'telem': 0, 'dup': 0, 'last': None, 'gaps': [],
                                'last_tid': {}, 'last_raw': None, 'raw_gap_max': 0.0,
                                'streams': {}, 'status_changes': 0, 'ranges': {},
                                'rpm': None, 'throttle': None, 'status': None,
                                'volt': None, 'amp': None, 'temp': None, 'mos': None}

    # ---- 发送 ----
    def _next_tid(self, stream):
        # HW protocol section 4.1.3: independent counter per transfer stream.
        value = self.tids.get(stream, 0)
        self.tids[stream] = (value + 1) & 0x1F
        return value

    def _log_once(self, key, interval):
        now = time.monotonic()
        last = self.logged_at.get(key)
        if last is not None and now - last < interval:
            return False
        self.logged_at[key] = now
        return True

    def _send(self, can_id, payload, note):
        body = bytes(payload)[:8].ljust(8, b'\x00')
        frame = struct.pack('=IB3x8s', can_id | CAN_EFF_FLAG, len(payload), body)
        try:
            self.stats['send_retries'] += send_bounded(self.sock, frame, self.diag, 'fan')
        except OSError as exc:
            self.stats['tx_fail'] += 1
            self.consecutive_fail += 1
            if self.consecutive_fail >= HW_MAX_SEND_FAIL and not self.faulted:
                self.faulted = True
                self.frames.write('TX-ERR', can_id, body, '{} 连续发送失败 {} 次：{}'.format(
                    note, self.consecutive_fail, exc))
            else:
                self.frames.write('TX-ERR', can_id, body, '{} {}'.format(note, exc))
            return False
        self.stats['tx'] += 1
        self.consecutive_fail = 0
        if self._log_once(('tx', can_id), 1.0):
            self.frames.write('TX', can_id, bytes(payload), note)
        return True

    def send_select_can(self, node):
        """让指定电调改用 CAN 数字油门。"""
        can_id = hw_service_id(HW_PRIO_MED, HW_SRV_THR_SELECT, node, HW_SRC_NODE)
        return self._send(can_id, bytes([0x00, hw_tail(self._next_tid(('select', node)))]),
                          '风机 选CAN油门 ESC0x{:02X}'.format(node))

    def send_ready(self, on):
        """广播油门使能（0xAA）或失能（0x55）。"""
        can_id = hw_message_id(HW_PRIO_MED, HW_MSG_READY, HW_SRC_NODE)
        return self._send(can_id, bytes([0xAA if on else 0x55, hw_tail(self._next_tid('ready'))]),
                          '风机 油门使能' if on else '风机 油门失能')

    def _send_raw(self):
        # 四通道同值，与参考风机程序一致：节点 0x01 走 CH1、0x02 走 CH2 都拿到同一油门
        payload = hw_pack4(self.current, self.current, self.current, self.current) + \
            bytes([hw_tail(self._next_tid('raw'))])
        can_id = hw_message_id(HW_PRIO_HIGH, HW_MSG_RAW, HW_SRC_NODE)
        now = time.monotonic()
        if self.last_tx is not None:
            self.stats['period_ms'].append((now - self.last_tx) * 1000.0)
        self.last_tx = now
        ok = self._send(can_id, payload, '风机 油门 CH1~CH4={}'.format(self.current))
        if ok:
            self.stats['raw_tx'] += 1
        return ok

    def service(self):
        """到点补发一帧油门；返回到下一帧的剩余秒数。"""
        if not self.active:
            return 3600.0
        now = time.monotonic()
        if now < self.next_due:
            return self.next_due - now
        if now - self.next_due > 1.5 * self.period:
            self.stats['late'] += 1
            self.next_due = now + self.period
        else:
            while self.next_due <= now:
                self.next_due += self.period
        self._send_raw()
        return max(0.0, self.next_due - time.monotonic())

    # ---- 接收 ----
    def on_frame(self, can_id, payload, arrival_ns=None):
        """解析电调主动上报的遥测帧。"""
        field = (can_id >> 8) & 0xFFFF
        node = can_id & 0x7F
        info = self.nodes.get(node)
        if info is None:
            self.stats['other'] += 1
            return
        previous_status = info['status']
        if field in (HW_MSG_TELEM1, HW_MSG_TELEM2, HW_MSG_TELEM3) and (
                not payload or payload[-1] & 0xE0 != 0xC0):
            self.stats['other'] += 1
            if self.diag and self._log_once(('malformed', node, field), 1.0):
                self.diag.emit('esc_invalid_tail', node=node, field=field, data=payload.hex())
            return
        if field == HW_MSG_TELEM1 and len(payload) >= 7:
            info['rpm'] = payload[0] | (payload[1] << 8)
            info['throttle'] = payload[2] | (payload[3] << 8)
            info['status'] = payload[4] | (payload[5] << 8)
            kind, note = 'MSG1', '风机0x{:02X} 转速{}rpm 油门{} 状态0x{:04X}'.format(
                node, info['rpm'], info['throttle'], info['status'])
        elif field == HW_MSG_TELEM2 and len(payload) >= 6:
            info['volt'] = (payload[0] | (payload[1] << 8)) / 10.0
            info['amp'] = (payload[2] | (payload[3] << 8)) / 10.0
            info['temp'] = float(payload[4])
            kind, note = 'MSG2', '风机0x{:02X} 电压{:.1f}V 电流{:.1f}A 温度{:.0f}℃'.format(
                node, info['volt'], info['amp'], info['temp'])
        elif field == HW_MSG_TELEM3 and len(payload) >= 8:
            info['mos'] = float(payload[0])
            kind, note = 'MSG3', '风机0x{:02X} MOS温度{:.0f}℃'.format(node, info['mos'])
        else:
            self.stats['other'] += 1
            self.frames.write('RX', can_id, payload, '风机 未解析帧')
            return
        for key in ('rpm', 'volt', 'amp', 'temp', 'mos'):
            value = info[key]
            if value is not None:
                limits = info['ranges'].setdefault(key, {'min': value, 'max': value})
                limits['min'], limits['max'] = min(limits['min'], value), max(limits['max'], value)
        if info['status'] != previous_status:
            info['status_changes'] += 1
            if self.diag:
                self.diag.emit('esc_status_change', node=node, previous=previous_status,
                               status=info['status'], data=payload.hex(),
                               meaning='bit13 表示电调内部 CAN-MCU/TI-MCU 通信异常，非 CAN 总线错误')
        now = time.monotonic()
        if info['last_raw'] is not None:
            info['raw_gap_max'] = max(info['raw_gap_max'], (now - info['last_raw']) * 1000.0)
        info['last_raw'] = now
        info['telem'] += 1
        self.stats['telem'] += 1
        tail = payload[-1] & 0x1F
        stream = info['streams'].setdefault(kind, {
            'frames': 0, 'exact_duplicates': 0, 'same_tid_changed': 0,
            'tid_missing_estimate': 0, 'sequence_uncertain': 0,
            'gap_max_ms': 0.0, 'last_time': None, 'last_tid': None, 'last_payload': None})
        stream['frames'] += 1
        at = arrival_ns / 1e9 if arrival_ns is not None else now
        previous = stream['last_time']
        gap = at - previous if previous is not None else None
        if gap is not None:
            stream['gap_max_ms'] = max(stream['gap_max_ms'], gap * 1000.0)
        same_tid = stream['last_tid'] == tail
        exact = same_tid and stream['last_payload'] == payload.hex() and gap is not None and 0 <= gap < 2.0
        if exact:
            stream['exact_duplicates'] += 1
            info['dup'] += 1
            self.stats['dup'] += 1
            stream['last_time'] = at
            if self.diag and self._log_once(('dup', node, kind), 1.0):
                self.diag.emit('esc_duplicate_sample', node=node, kind=kind, tid=tail,
                               data=payload.hex(), arrival_ns=arrival_ns)
            return
        if same_tid and gap is not None and 0 <= gap < 2.0:
            stream['same_tid_changed'] += 1
        elif stream['last_tid'] is not None:
            step = (tail - stream['last_tid']) & 0x1F
            if gap is None or not 0 <= gap < 2.0 or step == 0 or step > 16:
                stream['sequence_uncertain'] += 1
            elif step > 1:
                stream['tid_missing_estimate'] += step - 1
        stream['last_time'], stream['last_tid'], stream['last_payload'] = at, tail, payload.hex()
        info['last_tid'][kind] = tail
        if info['last'] is not None:
            info['gaps'].append((now - info['last']) * 1000.0)
        info['last'] = now
        if self._log_once(('rx', node, kind), 5.0):
            self.frames.write('RX', can_id, payload, note)

    # ---- 流程 ----
    def init_sequence(self, motor, zero_seconds=2.0, keepalive=True):
        """选 CAN 油门 -> 油门使能 -> 连续零油门保持；keepalive 时电机同时保持收帧。"""
        self.log('风机：初始化（选 CAN 油门 → 油门使能 → 连续零油门 {:.1f}s）。'.format(zero_seconds))
        self.active = True
        self.next_due = time.monotonic()
        step = motor.hold if keepalive else motor.drain
        for node in sorted(self.nodes):
            self.send_select_can(node)
            step(0.1)
        self.send_ready(True)
        step(0.1)
        self.current = 0
        step(zero_seconds)
        self.enabled = True
        self.current = self.target
        self.log('风机：已使能，油门 {}（{:.1f}%）。'.format(
            self.target, self.target * 100.0 / HW_MAX_THROTTLE))

    def shutdown(self, motor, zero_frames=20):
        """零油门若干帧 -> 油门失能。"""
        self.log('风机：收尾（零油门 {} 帧 → 油门失能）。'.format(zero_frames))
        self.current = 0
        start_count = self.stats['raw_tx']
        end = time.monotonic() + max(1.0, zero_frames * self.period * 2)
        while self.stats['raw_tx'] - start_count < zero_frames and time.monotonic() < end:
            motor.drain(0.02)
        off_sent = self.send_ready(False)
        motor.drain(0.1)
        self.active = False
        zeros = self.stats['raw_tx'] - start_count
        self.shutdown_status = '零油门成功入队 {} 帧；失能帧入队 {}；电调失能未回读确认'.format(zeros, off_sent)
        if self.diag:
            self.diag.emit('fan_shutdown', zero_frames=zeros, disable_queued=off_sent)

    def node_state(self, node):
        info = self.nodes[node]
        if info['last_raw'] is None:
            return '未收到遥测'
        age = (time.monotonic() - info['last_raw']) * 1000.0
        return '在线（{:.0f}ms 前）'.format(age) if age < 1000.0 else '疑似掉线（{:.0f}ms 前）'.format(age)


class Motor:
    def __init__(self, ifname, motor_id, frames, log):
        self.ifname = ifname
        self.motor_id = motor_id
        self.frames = frames
        self.log = log
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, getattr(socket, 'CAN_RAW', 1))
        self.sock.bind((ifname,))
        self.sock.setblocking(False)
        self.reply = None
        self.reply_status = 0
        self.last_status = 0
        self.pending = None
        self.fan = None
        self.last_pos = 0.0
        self.last_vel = 0.0
        self.last_temp = 0.0
        self.last_torque = 0.0
        self.last_report = None
        self.first_report = None
        self.gaps = []
        self.report_ms = EPS_REPORT_MS
        self.write_ms = 100.0
        self.last_rx_motor = None
        self.diag = None
        self.arrival_ns = None
        self.pending_sent_ns = 0
        self.last_send_ok = True
        self.stop_confirmed = False
        self.last_motor_state = None
        self.report_tail_ms = 0.0
        self.report_end_at = None
        self.last_report_at = None
        self.last_report_clock = None
        self.app_gaps = []
        self.last_app_report = None
        self.heartbeat_probe_enabled = False
        self.heartbeat_probe_ms = 50.0
        self._heartbeat_probe_last_mono = None
        self._heartbeat_probe_queue = deque()
        self._heartbeat_probe_inflight = False
        self.stats = {
            'tx': 0, 'rx': 0, 'rx_fan': 0, 'tx_fail': 0, 'rx_overflow': 0,
            'write_req': 0, 'write_ack': 0, 'write_timeout': 0,
            'read_req': 0, 'read_ack': 0, 'read_timeout': 0,
            'write_send_fail': 0, 'read_send_fail': 0, 'send_retries': 0,
            'rx_other': 0, 'rx_malformed': 0, 'rx_recv_error': 0,
            'report': 0, 'feedback': 0,
            'init_retry': 0, 'motor_lost': 0, 'silent_max': 0.0,
            'gaps': self.gaps,
            'lat_write': [], 'lat_read': [],
            'idx': {},
            'rx_mode': {},
            'heartbeat_position_probes': [],
            'heartbeat_loss_reference_ms': 0.0, 'heartbeat_loss_tracking_ready': False,
            'heartbeat_loss_events': [],
        }

    # ---- 底层收发 ----
    def _send(self, mode, payload, note=''):
        can_id = build_id(mode, MASTER_ID, self.motor_id)
        body = bytes(payload).ljust(8, b'\x00')
        frame = struct.pack('=IB3x8s', can_id | CAN_EFF_FLAG, len(payload), body)
        try:
            self.stats['send_retries'] += send_bounded(self.sock, frame, self.diag, 'motor')
            self.stats['tx'] += 1
        except OSError as exc:
            self.stats['tx_fail'] += 1
            self.frames.write('TX-ERR', can_id, body, '{} {}'.format(note, exc))
            return False
        self.frames.write('TX', can_id, body[:len(payload)], note)
        return True

    def pump(self, timeout):
        if self.fan is not None:
            remain = self.fan.service()
            if remain < timeout:
                timeout = remain
        try:
            ready, _, _ = select.select([self.sock], [], [], max(0.0, timeout))
        except InterruptedError:
            return
        if not ready:
            return
        # Bounded batch drain: reduce backlog, but return to throttle scheduling within 2ms.
        batch_end = time.monotonic() + .002
        for _ in range(64):
            try:
                data, ancillary, flags, address = self.sock.recvmsg(16, 128)
            except BlockingIOError:
                return
            except OSError as exc:
                self.stats['rx_recv_error'] += 1
                if self.diag:
                    self.diag.emit('recv_error', errno=exc.errno, error=str(exc))
                return
            self.arrival_ns = self.diag.received(ancillary) if self.diag else None
            if self.diag:
                self.stats['rx_overflow'] = self.diag.socket_drops
            if len(data) != 16 or flags & (getattr(socket, 'MSG_TRUNC', 32) | getattr(socket, 'MSG_CTRUNC', 8)):
                self.stats['rx_malformed'] += 1
                continue
            can_id, dlc = struct.unpack('=IB', data[:5])
            if can_id & CAN_ERR_FLAG:
                if self.diag:
                    self.diag.error_frame(can_id, data[8:16])
                continue
            if not can_id & CAN_EFF_FLAG or can_id & CAN_RTR_FLAG or dlc > 8:
                self.stats['rx_other'] += 1
                continue
            self._on_frame(can_id & CAN_EFF_MASK, data[8:8 + dlc])
            if time.monotonic() >= batch_end:
                break

    def _on_frame(self, can_id, payload):
        if hw_is_frame(can_id):
            self.stats['rx'] += 1
            self.stats['rx_fan'] += 1
            if self.fan is not None:
                self.fan.on_frame(can_id, payload, self.arrival_ns)
            else:
                self.frames.write('RX', can_id, payload, '风机帧（未启用）')
            return
        mode = (can_id >> 24) & 0x1F
        if ((can_id >> 8) & 0xFF) != self.motor_id or (can_id & 0xFF) != MASTER_ID:
            self.stats['rx_other'] += 1
            return
        if mode not in (M_REPORT, M_FEEDBACK, M_READ, M_WRITE):
            self.stats['rx_other'] += 1
            return
        if len(payload) != 8:
            self.stats['rx_malformed'] += 1
            return
        self.stats['rx'] += 1
        self.last_rx_motor = time.monotonic()
        if self.arrival_ns is not None:
            self.last_rx_motor -= max(0.0, (time.time_ns() - self.arrival_ns) / 1e9)
        self.stats['rx_mode'][mode] = self.stats['rx_mode'].get(mode, 0) + 1
        note = ''
        if mode == M_REPORT:
            now = time.monotonic()
            gap = None
            at = self.arrival_ns / 1e9 if self.arrival_ns is not None else now
            self.stats['report'] += 1
            if self.first_report is None:
                self.first_report = now
            clock = 'kernel' if self.arrival_ns is not None else 'application'
            if self.last_report is not None and clock == self.last_report_clock:
                gap = (at - self.last_report_at) * 1000.0
                if gap >= 0:
                    self.gaps.append(gap)
                if gap > 3 * self.report_ms:
                    note = '上报间隔{:.1f}ms'.format(gap)
                    if self.diag:
                        self.diag.emit('motor_heartbeat_gap', gap_ms=gap, arrival_ns=self.arrival_ns)
            report_arrival_mono = now
            if self.arrival_ns is not None:
                report_arrival_mono -= max(0.0, (time.time_ns() - self.arrival_ns) / 1e9)
            if self.heartbeat_probe_enabled:
                self._heartbeat_probe_on_report(gap, report_arrival_mono)
            if self.last_app_report is not None:
                self.app_gaps.append((now - self.last_app_report) * 1000.0)
            self.last_app_report = now
            self.last_report = now
            self.last_report_at = at
            self.last_report_clock = clock
            self._heartbeat_probe_last_mono = now
            if self.arrival_ns is not None:
                self._heartbeat_probe_last_mono -= max(0.0, (time.time_ns() - self.arrival_ns) / 1e9)
        if mode in (M_FEEDBACK, M_REPORT):
            self.last_motor_state = (can_id >> 22) & 3
            if mode == M_FEEDBACK:
                self.stats['feedback'] += 1
            self.last_pos = u2f((payload[0] << 8) | payload[1], P_MIN, P_MAX)
            self.last_vel = u2f((payload[2] << 8) | payload[3], V_MIN, V_MAX)
            self.last_torque = u2f((payload[4] << 8) | payload[5], T_MIN, T_MAX)
            self.last_temp = ((payload[6] << 8) | payload[7]) / 10.0
            note = '角度{:.3f}rad 速度{:.3f}rad/s 力矩{:.2f}Nm 温度{:.1f}℃'.format(
                self.last_pos, self.last_vel, self.last_torque, self.last_temp)
            faults = (can_id >> 16) & 0x3F
            if faults and self.diag:
                self.diag.emit('motor_fault_bits', bits=faults, can_id=hex(can_id), data=payload.hex())
        self.frames.write('RX', can_id, payload, note)
        if self.pending is not None:
            want_mode, want_index = self.pending
            if mode == want_mode and (self.arrival_ns is None or self.arrival_ns >= self.pending_sent_ns):
                if mode == M_READ:
                    index = payload[0] | (payload[1] << 8)
                    if index != want_index:
                        return
                    self.reply_status = (can_id >> 16) & 0xFF
                self.reply = payload
                self.pending = None

    def drain(self, seconds):
        """Receive traffic and service only already-confirmed missing-frame checks."""
        deadline = time.monotonic() + seconds
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                self.service_heartbeat_probe(self.heartbeat_probe_ms)
                return
            self.pump(min(remain, .05))
            self.service_heartbeat_probe(self.heartbeat_probe_ms)

    def enable_heartbeat_probe(self, read_ms):
        if self.stats['heartbeat_loss_tracking_ready']:
            return
        self.heartbeat_probe_ms = float(read_ms)
        # Freeze the observed startup cadence. Reporting and live checks must use exactly this rule.
        reference = self.report_ms if self.report_ms > 0 else EPS_REPORT_MS
        self.stats['heartbeat_loss_reference_ms'] = reference
        self.stats['heartbeat_loss_tracking_ready'] = True
        self.heartbeat_probe_enabled = True
        for gap in self.gaps:
            self._record_heartbeat_gap_loss(gap, None, 'startup_calibration')
        self.log('逐帧心跳丢失检查已开启：设定上报周期沿用配置，统一判定参考周期{:.3f}ms；只在确认少帧时读取位置0x7019和速度0x701B，每丢失1帧检查1组，不按50ms空窗循环读取。'.format(reference))

    def _record_heartbeat_gap_loss(self, gap_ms, arrival_mono, phase):
        reference = self.stats['heartbeat_loss_reference_ms']
        missing = heartbeat_gap_loss_count(gap_ms, reference)
        if missing == 0:
            return 0
        now = time.monotonic()
        confirmed_at = now_iso()
        gap_id = len(self.stats['heartbeat_loss_events']) + 1
        event = {'gap_id': gap_id, 'gap_ms': float(gap_ms), 'reference_ms': reference,
                 'missing_count': missing, 'confirmed_at': confirmed_at, 'phase': phase}
        self.stats['heartbeat_loss_events'].append(event)
        for slot in range(1, missing + 1):
            expected_mono = (arrival_mono - gap_ms / 1000.0 + slot * reference / 1000.0
                             if arrival_mono is not None else None)
            record = {'id': len(self.stats['heartbeat_position_probes']) + 1,
                      'gap_id': gap_id, 'slot_in_gap': slot, 'gap_missing_count': missing,
                      'gap_ms': float(gap_ms), 'reference_period_ms': reference,
                      'estimated_missing_at': (datetime.fromtimestamp(time.time() - (now - expected_mono)).isoformat(timespec='milliseconds')
                                               if expected_mono is not None else None),
                      'confirmed_at': confirmed_at, 'detected_at': confirmed_at,
                      'confirmation_phase': phase, 'heartbeat_recovered_at': confirmed_at,
                      'status': 'queued', 'position_rad': None, 'velocity_rad_s': None,
                      'request_at': None, 'request_delay_ms': None, 'read_elapsed_ms': None,
                      'queued_mono': now, 'readings': {}}
            self.stats['heartbeat_position_probes'].append(record)
            self._heartbeat_probe_queue.append(record)
            self.log('心跳丢失#{}：相邻上报间隔{:.3f}ms，参考周期{:.3f}ms，确认少{}帧（第{}/{}帧），安排1组位置/速度读取。'.format(
                record['id'], gap_ms, reference, missing, slot, missing))
        return missing

    def check_heartbeat_probe(self, until=None):
        # Silence alone is not a confirmed missing frame. No requests are generated by this timer.
        return 0

    def _heartbeat_probe_on_report(self, gap_ms, arrival_mono=None):
        if gap_ms is not None:
            self._record_heartbeat_gap_loss(gap_ms, arrival_mono, 'adjacent_reports')

    def service_heartbeat_probe(self, budget_ms, finishing=False):
        if ((not self.heartbeat_probe_enabled and not finishing) or self._heartbeat_probe_inflight
                or self.pending is not None or not self._heartbeat_probe_queue):
            return False
        record = self._heartbeat_probe_queue.popleft()
        self._heartbeat_probe_inflight = True
        try:
            started = time.monotonic()
            record['request_at'] = now_iso()
            record['request_delay_ms'] = round((started - record['queued_mono']) * 1000.0, 3)
            for key, index, label, unit in (
                    ('position', IDX_MECH_POS, '当前位置', 'rad'),
                    ('velocity', IDX_MECH_VEL, '当前速度', 'rad/s')):
                reading_start = time.monotonic()
                reading = {'index': '0x{:04X}'.format(index), 'request_at': now_iso(),
                           'after_gap_confirmation': True, 'budget_ms': float(budget_ms), 'value': None}
                record['readings'][key] = reading
                value = self.read_param(index, budget_ms)
                reading['elapsed_ms'] = round((time.monotonic() - reading_start) * 1000.0, 3)
                reading['status_code'] = self.last_status
                if value is not None:
                    import math
                    reading['status'] = 'success' if math.isfinite(value) else 'invalid_value'
                    reading['value'] = value if math.isfinite(value) else None
                elif self.last_status:
                    reading['status'] = 'rejected'
                else:
                    reading['status'] = 'timeout' if self.last_send_ok else 'send_failed'
                result = ('成功，{}={:.6f}{}'.format(label, reading['value'], unit)
                          if reading['status'] == 'success' else reading['status'])
                self.log('心跳丢失#{}读{}：{}，{} {}，耗时{:.3f}ms，状态码{}；确认缺帧后读取的实时值。'.format(
                    record['id'], label, reading['request_at'], reading['index'], result,
                    reading['elapsed_ms'], reading['status_code']))
            record['position_rad'] = record['readings']['position']['value']
            record['velocity_rad_s'] = record['readings']['velocity']['value']
            successes = sum(r['status'] == 'success' for r in record['readings'].values())
            record['status'] = 'success' if successes == 2 else ('partial' if successes else 'failed')
            record['read_elapsed_ms'] = round((time.monotonic() - started) * 1000.0, 3)
            if self.diag:
                self.diag.emit('heartbeat_lost_frame_probe', **{k: v for k, v in record.items() if k != 'queued_mono'})
        except BaseException:
            record['status'] = 'interrupted'
            if record['readings']:
                reading = next(reversed(record['readings'].values()))
                reading.setdefault('status', 'interrupted')
            raise
        finally:
            self._heartbeat_probe_inflight = False
        return True

    def run_heartbeat_checks(self):
        """Dispatch a confirmed loss check only between finished protocol requests."""
        return self.service_heartbeat_probe(self.heartbeat_probe_ms)

    def motor_alive_ms(self):
        """距最近一次收到电机帧的毫秒数，None 表示本次还一帧都没收到。"""
        if self.last_rx_motor is None:
            return None
        return (time.monotonic() - self.last_rx_motor) * 1000.0

    def hold(self, seconds, interval=0.07):
        """按轮询节拍把目标位置写成当前反馈角度，电机保持不动且不掉线。"""
        end = time.monotonic() + seconds
        while True:
            remain = end - time.monotonic()
            if remain <= 0:
                return
            self.write_param(IDX_LOC_REF, struct.pack('<f', self.last_pos), self.write_ms,
                             '保持当前位置 {:.3f}'.format(self.last_pos))
            self.drain(min(interval, max(0.0, end - time.monotonic())))

    # ---- 协议动作 ----
    def _request(self, mode, payload, want_mode, want_index, budget_ms, note):
        self.reply = None
        self.reply_status = 0
        self.pending = (want_mode, want_index)
        sent = time.monotonic()
        self.pending_sent_ns = time.time_ns()
        ok = self._send(mode, payload, note)
        self.last_send_ok = ok
        if not ok:
            self.pending = None
            return None, 0.0
        deadline = sent + budget_ms / 1000.0
        while self.reply is None:
            remain = deadline - time.monotonic()
            if remain <= 0:
                break
            self.pump(remain)
        if self.reply is None:
            self.pending = None
            if self.diag:
                self.diag.emit('response_timeout', mode=mode, index=want_index,
                               budget_ms=budget_ms, waited_ms=(time.monotonic() - sent) * 1000.0)
            return None, (time.monotonic() - sent) * 1000.0
        return self.reply, (time.monotonic() - sent) * 1000.0

    def enable(self):
        return self._send(M_ENABLE, bytes(8), '使能运行')

    def stop(self):
        return self._send(M_STOP, bytes(8), '停止运行')

    def confirm_stop(self):
        self.stop_confirmed = False
        for _ in range(3):
            reply, _delay = self._request(M_STOP, bytes(8), M_FEEDBACK, None, 150,
                                          '停止运行并等待反馈')
            if reply is not None and self.last_motor_state == 0 and abs(self.last_vel) < .1:
                self.stop_confirmed = True
                break
            self.drain(.05)
        if self.diag:
            self.diag.emit('motor_stop_confirmation', confirmed=self.stop_confirmed,
                           motor_state=self.last_motor_state, last_velocity=self.last_vel)
        return self.stop_confirmed

    def end_report_window(self, flush=False):
        if self.report_end_at is None:
            self.report_end_at = time.monotonic()
            if self.last_report is not None:
                self.report_tail_ms = max(0.0, (self.report_end_at - self.last_report) * 1000.0)
        self.heartbeat_probe_enabled = False
        if flush:
            while self._heartbeat_probe_queue:
                if not self.service_heartbeat_probe(self.heartbeat_probe_ms, finishing=True):
                    break
        while self._heartbeat_probe_queue:
            record = self._heartbeat_probe_queue.popleft()
            record['status'] = 'not_run_before_shutdown'
            self.log('心跳丢失#{}：本轮中止收尾，未执行排队的位置/速度诊断，保留空值和状态。'.format(record['id']))

    def write_param(self, index, raw, budget_ms, label):
        payload = struct.pack('<H', index) + b'\x00\x00' + bytes(raw).ljust(4, b'\x00')
        self.stats['write_req'] += 1
        rec = self.stats['idx'].setdefault(index, {'r': 0, 'w': 0, 'ack': 0, 'to': 0, 'lat': []})
        rec['w'] += 1
        reply, delay = self._request(M_WRITE, payload, M_FEEDBACK, None, budget_ms,
                                     '写参数 0x{:04X} {}'.format(index, label))
        if reply is None:
            self.stats['write_timeout' if self.last_send_ok else 'write_send_fail'] += 1
            rec['to'] += 1
            return False
        self.stats['write_ack'] += 1
        self.stats['lat_write'].append(delay)
        rec['ack'] += 1
        rec['lat'].append(delay)
        return True

    def write_retry(self, index, raw, budget_ms, label, tries=4, gap=2.0):
        """带重试的写参数：电机重启或短暂掉线后能自己接上，不至于立刻中断测试。"""
        for attempt in range(1, max(1, tries) + 1):
            ok = self.write_param(index, raw, budget_ms, label) if attempt == 1 else \
                self.option_write(index, raw, budget_ms, label)
            if ok:
                self.stats['init_retry'] += attempt - 1
                return True
            if attempt >= tries:
                break
            self.log('写 {} 无应答（第 {} 次），{:.0f}s 后重试。'.format(label, attempt, gap))
            self.drain(gap)
        self.stats['init_retry'] += tries - 1
        return False

    def read_param(self, index, budget_ms, kind='float'):
        payload = struct.pack('<H', index) + b'\x00\x00' + bytes(4)
        self.last_status = 0
        self.stats['read_req'] += 1
        rec = self.stats['idx'].setdefault(index, {'r': 0, 'w': 0, 'ack': 0, 'to': 0, 'lat': []})
        rec['r'] += 1
        reply, delay = self._request(M_READ, payload, M_READ, index, budget_ms,
                                     '读参数 0x{:04X}'.format(index))
        if reply is None:
            self.stats['read_timeout' if self.last_send_ok else 'read_send_fail'] += 1
            rec['to'] += 1
            return None
        self.stats['read_ack'] += 1
        self.stats['lat_read'].append(delay)
        rec['ack'] += 1
        rec['lat'].append(delay)
        if self.reply_status:
            self.last_status = self.reply_status
            return None
        if kind == 'float':
            return struct.unpack('<f', reply[4:8])[0]
        if kind == 'uint8':
            return reply[4]
        return struct.unpack('<I', reply[4:8])[0]

    def option_write(self, index, raw, budget_ms, label):
        """设置项写入：无应答时回滚统计，不计入正式收发计数。"""
        if self.write_param(index, raw, budget_ms, label):
            return True
        self.stats['write_req'] -= 1
        self.stats['write_timeout' if self.last_send_ok else 'write_send_fail'] -= 1
        self._drop_idx(index, 'w')
        return False

    def option_read(self, index, budget_ms, kind='float'):
        """设置项读取：无应答时回滚统计，不计入正式收发计数。"""
        before = self.stats['read_timeout'] + self.stats['read_send_fail']
        value = self.read_param(index, budget_ms, kind)
        if value is None and self.stats['read_timeout'] + self.stats['read_send_fail'] > before:
            self.stats['read_req'] -= 1
            self.stats['read_timeout' if self.last_send_ok else 'read_send_fail'] -= 1
            self._drop_idx(index, 'r')
        return value

    def _drop_idx(self, index, side):
        rec = self.stats['idx'].get(index)
        if rec is None:
            return
        rec[side] -= 1
        rec['to'] -= 1
        if not (rec['r'] or rec['w'] or rec['ack'] or rec['to'] or rec['lat']):
            self.stats['idx'].pop(index, None)

    def probe(self, motor_id, budget_ms=30):
        """只读探测：读负载端机械角度，有应答即认为该 ID 上有电机。"""
        self.motor_id = motor_id
        return self.option_read(IDX_MECH_POS, budget_ms)


def read_sys_stats(ifname):
    base = Path('/sys/class/net') / ifname / 'statistics'
    result = {}
    if not base.is_dir():
        return result
    for item in sorted(base.iterdir()):
        try:
            result[item.name] = int(item.read_text().strip())
        except (OSError, ValueError):
            continue
    try:
        result['operstate'] = (Path('/sys/class/net') / ifname / 'operstate').read_text().strip()
    except OSError:
        result['operstate'] = 'unknown'
    return result


def read_bittiming(ifname):
    """读接口的控制器状态与位时序：(state, sample_point)，取不到时返回 (None, None)。"""
    try:
        proc = subprocess.run(['ip', '-details', 'link', 'show', ifname],
                              capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return None, None
    state = None
    sample_point = None
    for line in proc.stdout.splitlines():
        text = line.strip()
        if text.startswith('can ') and ' state ' in text:
            state = text.split(' state ', 1)[1].split('(berr', 1)[0].strip()
        elif text.startswith('bitrate') and 'sample-point' in text:
            parts = text.split('sample-point')
            if len(parts) == 2:
                try:
                    sample_point = float(parts[1].split()[0])
                except (IndexError, ValueError):
                    sample_point = None
    return state, sample_point


class Liveness:
    """电机在线监测：静默超过 warn_ms 记一次掉线，超过 abort_ms 返回 True 让主循环中止。"""

    def __init__(self, log, warn_ms=2000.0, abort_ms=10000.0):
        self.log = log
        self.warn_ms = warn_ms
        self.abort_ms = abort_ms
        self.missing = False
        self.peak = 0.0

    def check(self, motor):
        age = motor.motor_alive_ms()
        if age is None:
            return False
        if age > motor.stats['silent_max']:
            motor.stats['silent_max'] = age
        if self.missing and age > self.peak:
            self.peak = age
        if age >= self.warn_ms and not self.missing:
            self.missing = True
            self.peak = age
            motor.stats['motor_lost'] += 1
            self.log('警告：电机已 {:.0f}ms 收不到任何帧（bus-off、断电、接线松动都会这样）。'.format(age))
        elif age < self.warn_ms and self.missing:
            self.missing = False
            self.log('电机恢复通信（本次最长静默 {:.1f}s）。'.format(self.peak / 1000.0))
            self.peak = 0.0
        return self.missing and age >= self.abort_ms


class Phase:
    """单个动作阶段的采样统计。"""

    def __init__(self, name, direction, duration):
        self.name = name
        self.direction = direction
        self.duration = duration
        self.pos = []
        self.vel = []
        self.samples = 0
        self.timeout = 0

    def add(self, pos, vel, timeout):
        self.samples += 1
        if timeout:
            self.timeout += 1
        if pos is not None:
            self.pos.append(pos)
        if vel is not None:
            self.vel.append(vel)

    def summary(self, speed):
        item = {'name': self.name, 'direction': self.direction,
                'duration': self.duration, 'samples': self.samples, 'timeout': self.timeout}
        if self.pos:
            item['pos_start'] = round(self.pos[0], 4)
            item['pos_end'] = round(self.pos[-1], 4)
            item['pos_delta'] = round(self.pos[-1] - self.pos[0], 4)
        if self.vel:
            item['vel_min'] = round(min(self.vel), 4)
            item['vel_max'] = round(max(self.vel), 4)
            item['vel_avg'] = round(sum(self.vel) / len(self.vel), 4)
            if self.direction:
                want = self.direction * speed
                good = sum(1 for v in self.vel if v * self.direction > 0.2 * speed)
                item['dir_ok'] = good
                item['dir_ok_ratio'] = round(good / len(self.vel), 4)
                item['vel_target'] = want
            else:
                item['still'] = sum(1 for v in self.vel if abs(v) < max(0.1, 0.1 * speed))
                item['still_ratio'] = round(item['still'] / len(self.vel), 4)
        return item


def parse_throttle(text):
    """解析油门参数：纯数字为原始值 0~8191，带 % 为百分比。"""
    raw = str(text).strip()
    try:
        if raw.endswith('%'):
            value = int(round(float(raw[:-1]) * HW_MAX_THROTTLE / 100.0))
        else:
            value = int(raw, 0)
    except ValueError:
        raise argparse.ArgumentTypeError('油门需为 0~8191 的整数或百分比如 6%')
    if not 0 <= value <= HW_MAX_THROTTLE:
        raise argparse.ArgumentTypeError('油门需在 0~8191（或 0~100%）')
    return value


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='RS00 位置模式 + X6 PLUS 推进风机通信长测')
    parser.add_argument('--iface', default='can0', help='CAN 接口，默认 can0')
    parser.add_argument('--motor-id', type=int, default=0, help='电机 CAN ID，0 表示自动探测')
    parser.add_argument('--minutes', type=float, default=30.0, help='测试时长（分钟），默认 30')
    parser.add_argument('--speed', type=float, default=1.0, help='转动速度 rad/s，默认 1.0')
    parser.add_argument('--mode', choices=['pp', 'csp'], default='pp',
                        help='位置模式类型，默认 pp；csp 用 limit_spd 限速，跟随更紧')
    parser.add_argument('--fwd-ms', type=int, default=3000, help='正转时长，默认 3000ms')
    parser.add_argument('--hold-ms', type=int, default=1000, help='停止保持时长，默认 1000ms')
    parser.add_argument('--rev-ms', type=int, default=3000, help='反转时长，默认 3000ms')
    parser.add_argument('--acc', type=float, default=10.0, help='位置模式加速度 rad/s^2，默认 10')
    parser.add_argument('--poll-ms', type=int, default=70, help='轮询周期，默认 70ms')
    parser.add_argument('--read-ms', type=int, default=50, help='读应答等待上限，默认 50ms')
    parser.add_argument('--write-ms', type=int, default=100, help='写应答等待上限，默认 100ms')
    parser.add_argument('--report-ms', type=float, default=EPS_REPORT_MS,
                        help='期望的自动上报（心跳）周期，默认 {:.0f}ms；10 对应电机默认'.format(EPS_REPORT_MS))
    parser.add_argument('--retry', type=int, default=4,
                        help='初始化写参数无应答时的重试次数，默认 4（每次间隔 2s）')
    parser.add_argument('--motor-lost-sec', type=float, default=10.0,
                        help='电机连续无任何帧超过该秒数则中止测试，默认 10')
    parser.add_argument('--fan-throttle', type=parse_throttle, default=500,
                        help='风机油门 0~8191 或百分比（如 6%%），0 表示只使能不转，默认 500')
    parser.add_argument('--fan-hz', type=float, default=50.0, help='风机油门下发频率，默认 50Hz')
    parser.add_argument('--fan-zero-sec', type=float, default=2.0,
                        help='初始化后连续零油门时长，默认 2s')
    parser.add_argument('--no-fan', action='store_true', help='不控制或统计风机；不会使物理连接的电调停止上报')
    parser.add_argument('--fan-disabled', action='store_true', help='接收风机遥测，但不选择油门源、不使能、不发油门')
    parser.add_argument('--vbus-sec', type=float, default=1.0, help='电机电压采样间隔秒，默认1；0关闭额外采样')
    parser.add_argument('--check-only', action='store_true', help='只做连通性检查，不使能、不运动')
    args = parser.parse_args(argv)
    if not (0 <= args.motor_id <= 255):
        parser.error('电机 ID 需在 0~255')
    if not (args.vbus_sec == 0 or .5 <= args.vbus_sec <= 60):
        parser.error('电压采样间隔为0或0.5~60秒')
    if not (0 < args.minutes <= 120):
        parser.error('时长需在 0~120 分钟')
    if not (0.1 <= args.speed <= 10):
        parser.error('速度需在 0.1~10 rad/s')
    if not (100 <= args.fwd_ms <= 60000 and 0 <= args.hold_ms <= 60000 and 100 <= args.rev_ms <= 60000):
        parser.error('正转/反转 100~60000ms，停止保持 0~60000ms')
    if not (20 <= args.poll_ms <= 500 and 10 <= args.read_ms <= 200 and 10 <= args.write_ms <= 200):
        parser.error('轮询 20~500ms，读 10~200ms，写 10~200ms')
    if not (10 <= args.report_ms <= 500):
        parser.error('上报周期 10~500ms')
    if not (1 <= args.retry <= 20):
        parser.error('重试次数 1~20')
    if not (2 <= args.motor_lost_sec <= 120):
        parser.error('电机掉线中止门限 2~120s')
    if not (10 <= args.fan_hz <= 100):
        parser.error('风机频率需在 10~100Hz')
    if not (0 <= args.fan_zero_sec <= 10):
        parser.error('零油门保持时长需在 0~10s')
    return args


def make_report(directory, settings, stats, cycles, can_before, can_after, started_at, ended_at,
                outcome, reason, motor_id, travel_guard, final_pos=None, fan=None):
    view = dict(settings)
    view['mode'] = str(settings.get('mode', 'pp')).upper()
    lat_write = stats['lat_write']
    lat_read = stats['lat_read']
    gaps = stats['gaps']
    period = percentile(gaps, 0.5) if gaps else 0.0
    loss_metrics = heartbeat_loss_metrics(stats)
    lost_report = loss_metrics['estimated_lost']
    expected_report = loss_metrics['expected_reports']
    gap_max = max(gaps) if gaps else 0.0
    reference = loss_metrics['reference_ms']
    analyzed_gaps = ([e['gap_ms'] for e in stats.get('heartbeat_loss_events', [])]
                     if loss_metrics['tracking_enabled'] else gaps)
    gap_over = sum(1 for gap in analyzed_gaps if reference > 0 and gap > 3 * reference)
    single_gap = sum(1 for gap in analyzed_gaps if heartbeat_gap_loss_count(gap, reference) == 1)
    multi_gap = sum(1 for gap in analyzed_gaps if heartbeat_gap_loss_count(gap, reference) > 1)

    req_w = stats['write_req']
    req_r = stats['read_req']

    def rate(part, total):
        return 0.0 if not total else part / total * 100.0

    lines = []
    add = lines.append
    add('# RS00 电机 + X6 PLUS 推进风机 通信测试（{}）'.format(
        '{}分钟'.format(int(settings['minutes'])) if float(settings['minutes']).is_integer()
        else '{}分钟'.format(settings['minutes'])))
    add('')
    add('- 测试开始：{}；结束：{}。'.format(started_at, ended_at))
    add('- 总体状态：`{}`。{}'.format(outcome, (' 原因：' + reason) if reason else ''))
    add('- 接线：RS00 接在板子原生 `{}`，经典 CAN，1Mbps；电机 CAN ID {}，主机 ID 0x{:02X}。'.format(
        settings['iface'], motor_id, MASTER_ID))
    if settings.get('no_fan'):
        add('- 风机：本次未启用（`--no-fan`）。')
    else:
        add('- 风机：两台 X6 PLUS 电调接同一路 `{}`，节点 0x{:02X}（CH{}）/0x{:02X}（CH{}），'
            '源节点 0x{:02X}，' '油门 {}，{}Hz 持续下发。'.format(
                settings['iface'], HW_ESC_LEFT, HW_ESC_LEFT, HW_ESC_RIGHT, HW_ESC_RIGHT,
                HW_SRC_NODE, settings.get('fan_throttle', 0), settings.get('fan_hz', 50)))
    add('- 模式：位置模式（{mode}），速度 {speed} rad/s，加速度 {acc} rad/s²。'.format(**view))
    add('- 循环：正转 {fwd_ms}ms → 停止保持 {hold_ms}ms → 反转 {rev_ms}ms → 下一组；'
        '达到时长后完成当前组再停止失能。'.format(**view))
    add('- 时序：轮询周期 {poll_ms}ms，写位置指令等应答上限 {write_ms}ms，读角度/速度等应答上限 {read_ms}ms；'
        '心跳（自动上报）目标周期 {report_ms:g}ms。'.format(**view))
    if travel_guard:
        add('- 注意：初始位置接近量程上限，本次先反转后正转，避免位置指令越界。')
    point = settings.get('sample_point')
    add('- CAN 采样点：{}（参考风机程序要求 {:.3f}；采样点偏差是否为根因需要对照验证）。'.format(
            '读不到' if point is None else '{:.3f}'.format(point), HW_SAMPLE_POINT))
    add('')
    add('## 运行与动作反馈')
    add('')
    add('| 项目 | 本次 |')
    add('|---|---|')
    add('| 电机 CAN ID | {} |'.format(motor_id))
    add('| CAN 采样点（要求 {:.3f}） | {} |'.format(
        HW_SAMPLE_POINT, '--' if point is None else '{:.3f}'.format(point)))
    add('| 完成组数 | {} |'.format(len(cycles)))
    add('| 每组应有时长（ms） | {} |'.format(settings['fwd_ms'] + settings['hold_ms'] + settings['rev_ms']))
    add('| 命令运动累计（秒） | {:.1f} |'.format(
        len(cycles) * (settings['fwd_ms'] + settings['rev_ms']) / 1000.0))
    add('| 是否确认停止并失能 | {} |'.format(
        '是' if outcome == 'completed' else ('未使能（本次为只读检查）' if outcome == 'checked'
                                            else '否（{}）'.format(reason or '未确认'))))
    add('| 位置指令行程（rad） | {:.2f} |'.format(settings['speed'] * settings['fwd_ms'] / 1000.0))
    add('| 反馈角度首值 / 末值（rad） | {} |'.format(
        '-- / --' if not cycles else '{:.3f} / {:.3f}'.format(
            cycles[0]['fwd'].get('pos_start', 0.0), cycles[-1]['rev'].get('pos_end', 0.0))))
    add('')
    add('动作判读：角度与速度均来自电机反馈帧；只看到发送指令不能证明电机已转动。')
    add('')

    # 阶段汇总
    add('## 各阶段反馈汇总')
    add('')
    add('| 阶段 | 组数 | 平均采样 | 超时 | 速度最小 | 速度最大 | 速度均值 | 方向正确率 |')
    add('|---|---:|---:|---:|---:|---:|---:|---:|')
    for key, name in (('fwd', '正转'), ('hold', '停止保持'), ('rev', '反转')):
        rows = [c[key] for c in cycles if key in c]
        if not rows:
            add('| {} | 0 | 0 | 0 | -- | -- | -- | -- |'.format(name))
            continue
        avg = sum(r['samples'] for r in rows) / len(rows)
        to = sum(r['timeout'] for r in rows)
        vmin = min(r.get('vel_min', 0.0) for r in rows)
        vmax = max(r.get('vel_max', 0.0) for r in rows)
        vavg = sum(r.get('vel_avg', 0.0) for r in rows) / len(rows)
        if key == 'hold':
            ratio = sum(r.get('still', 0) for r in rows) / max(1, sum(r['samples'] for r in rows))
            add('| {} | {} | {:.0f} | {} | {:.3f} | {:.3f} | {:.3f} | 静止占比 {:.1f}% |'.format(
                name, len(rows), avg, to, vmin, vmax, vavg, ratio * 100))
        else:
            ratio = sum(r.get('dir_ok', 0) for r in rows) / max(1, sum(r['samples'] for r in rows))
            add('| {} | {} | {:.0f} | {} | {:.3f} | {:.3f} | {:.3f} | {:.1f}% |'.format(
                name, len(rows), avg, to, vmin, vmax, vavg, ratio * 100))
    add('')
    add('方向正确率＝反馈速度方向与指令一致（>20% 目标速度）的采样占比；'
        '停止保持静止占比＝反馈速度接近 0 的采样占比。')
    add('')

    add('## CAN 收发与应答')
    add('')
    add('| 项目 | 本次 |')
    add('|---|---|')
    add('| 发送帧 / 接收帧 | {} / {} |'.format(stats['tx'], stats['rx']))
    add('| 发送失败（次） | {} |'.format(stats['tx_fail']))
    add('| 接收缓冲溢出（次） | {} |'.format(stats['rx_overflow']))
    add('| 其中风机帧（接收） | {} |'.format(stats.get('rx_fan', 0)))
    add('| 写参数请求 / 应答 / 超时 | {} / {} / {} |'.format(req_w, stats['write_ack'], stats['write_timeout']))
    add('| 写应答超时率 | {:.3f}% |'.format(rate(stats['write_timeout'], req_w)))
    add('| 读参数请求 / 应答 / 超时 | {} / {} / {} |'.format(req_r, stats['read_ack'], stats['read_timeout']))
    add('| 读应答超时率 | {:.3f}% |'.format(rate(stats['read_timeout'], req_r)))
    add('| 写 / 读请求发送失败 | {} / {} |'.format(stats.get('write_send_fail', 0), stats.get('read_send_fail', 0)))
    add('| 电机反馈帧（类型2） | {} |'.format(stats['feedback']))
    add('| 初始化重试次数 / 电机掉线次数 | {} / {} |'.format(
        stats.get('init_retry', 0), stats.get('motor_lost', 0)))
    add('| 电机最长静默（ms） | {:.0f} |'.format(stats.get('silent_max', 0.0)))
    add('| 自动上报帧（心跳，类型0x18） | {} |'.format(stats['report']))
    add('| 心跳目标周期 / 实测中位间隔（ms） | {:g} / {:.2f} |'.format(
        float(settings.get('report_ms', EPS_REPORT_MS)), period))
    add('| 丢帧判定统一参考周期ms | {:.6f} |'.format(loss_metrics['reference_ms']))
    add('| 心跳期望帧数 / 估算丢失帧数 | {} / {} |'.format(expected_report, lost_report))
    add('| 心跳估算缺口率（告警阈值 0.5%） | {:.3f}% |'.format(rate(lost_report, expected_report)))
    add('| 心跳最大间隔（ms） | {:.1f} |'.format(gap_max))
    add('| 心跳间隔超过 3 倍中位间隔的次数 | {} |'.format(gap_over))
    add('| 写应答延迟 min/avg/p50/p95/p99/max（ms） | {} |'.format(fmt_lat(lat_write)))
    add('| 读应答延迟 min/avg/p50/p95/p99/max（ms） | {} |'.format(fmt_lat(lat_read)))
    add('')
    add('写应答为电机反馈帧（类型2），读应答为参数应答帧（类型0x11）；'
        '自动上报（类型0x18）本次按 {:g}ms 一帧设置（写寄存器 0x7026，1 代表 10ms、每 +1 增 5ms），'
        '用作心跳与丢帧参照。'.format(float(settings.get('report_ms', EPS_REPORT_MS))))
    add('')
    add('心跳丢帧数使用启动校准的实测参考周期推算，与逐帧诊断共用同一判据；'
        '设定值与实测中位间隔差异较大时，以实测值为准。')
    add('')
    add('心跳间隔优先按内核接收时间戳统计，不可用时退回程序读帧时间；'
        '缺口因此包含采集停顿，不等于总线丢帧。普通 rx_errors/tx_errors 为 0 不能排除 CAN 专用错误、驱动或采集异常。')
    add('')
    add('上表为本 socket 统计的电机发送和全部接收；风机发送另计，不能等同于总线全部流量。')
    add('')

    add('## 推进风机（X6 PLUS）')
    add('')
    if fan is None:
        add('本次不控制风机（`--no-fan`），不代表物理断开或总线上没有风机遥测。')
        add('')
    else:
        fs = fan.stats
        add('| 项目 | 本次 |')
        add('|---|---|')
        add('| 电调节点 / 油门通道 | {} |'.format(
            '、'.join('0x{:02X}→CH{}'.format(n, n) for n in sorted(fan.nodes))))
        add('| 油门指令（原始 / 百分比） | {} / {:.1f}% |'.format(
            fan.target, fan.target * 100.0 / HW_MAX_THROTTLE))
        add('| 初始化流程 | 逐台选 CAN 油门 → 广播油门使能 0xAA → 连续零油门 {:.1f}s（期间电机按 70ms 保持收帧） |'.format(
            float(settings.get('fan_zero_sec', 2.0))) if not (settings.get('fan_disabled') or settings.get('check_only')) else '| 初始化流程 | 本次只接收遥测，不发送风机控制 |')
        add('| 设定频率 / RawCommand成功入队帧数 / 风机总TX | {:g}Hz / {} / {} |'.format(
            float(settings.get('fan_hz', 50)), fs.get('raw_tx', 0), fs['tx']))
        add('| 发送失败（次）/ 连续失败上限 | {} / {} |'.format(fs['tx_fail'], HW_MAX_SEND_FAIL))
        add('| 发送周期 min/avg/p50/p95/p99/max（ms） | {} |'.format(fmt_lat(fs['period_ms'])))
        add('| 迟于 1.5 倍周期补发的次数 | {} |'.format(fs['late']))
        add('| 收尾（零油门 20 帧 + 失能 0x55） | {} |'.format(
            getattr(fan, 'shutdown_status', '未执行')))
        add('')
        add('| 节点 | 遥测帧 | 其中重复 | 状态 | 转速（rpm） | 电压（V） | 电流（A） | 温度（℃） | 非重复更新最大间隔（ms） |')
        add('|---|---:|---:|---|---:|---:|---:|---:|---:|')
        for node in sorted(fan.nodes):
            info = fan.nodes[node]
            add('| 0x{:02X}（CH{}） | {} | {} | {} | {} | {} | {} | {} | {} |'.format(
                node, node, info['telem'], info['dup'], fan.node_state(node),
                '--' if info['rpm'] is None else info['rpm'],
                '--' if info['volt'] is None else '{:.1f}'.format(info['volt']),
                '--' if info['amp'] is None else '{:.1f}'.format(info['amp']),
                '--' if info['temp'] is None else '{:.0f}'.format(info['temp']),
                '--' if not info['gaps'] else '{:.1f}'.format(max(info['gaps']))))
        add('')
        add('油门是 14 位原始值（0~8191），四通道同值下发（与参考风机程序一致）：'
            '0x01 取 CH1、0x02 取 CH2。遥测由电调主动上报：MSG1 转速/油门/状态、MSG2 电压/电流/温度、MSG3 MOS 温度。')
        add('')
        add('重复帧要求同一节点、同类消息、同 TID、同数据且间隔小于 2 秒；同 TID 内容变化单列。'
            '遥测是广播帧，无需主机逐帧应答，重复原因不能由这些日志直接确定。'
            '在线状态按原始收帧更新时间判断，新 TID 更新间隔与原始遥测间隔分别记录。')
        add('')
        add('风机油门帧 50Hz 持续下发是做保活，中断后电调会停转；帧记录里每类帧 1~5 秒记一条样本，'
            '统计仍按全部帧计算。')
        add('')

    add('## 按参数索引')
    add('')
    add('| 索引 | 读请求 | 写请求 | 应答 | 超时 | min / avg / p50 / p95 / p99 / max（ms） |')
    add('|---|---:|---:|---:|---:|---|')
    for index in sorted(stats['idx']):
        rec = stats['idx'][index]
        add('| 0x{:04X} | {} | {} | {} | {} | {} |'.format(
            index, rec['r'], rec['w'], rec['ack'], rec['to'], fmt_lat(rec['lat'])))
    add('')

    add('## CAN 控制器计数')
    add('')
    keys = ['rx_packets', 'tx_packets', 'rx_errors', 'tx_errors', 'rx_dropped', 'tx_dropped',
            'rx_over_errors', 'rx_crc_errors', 'rx_fifo_errors', 'tx_aborted_errors']
    add('| 计数 | 开始 | 结束 |')
    add('|---|---:|---:|')
    for key in keys:
        add('| {} | {} | {} |'.format(key, can_before.get(key, '--'), can_after.get(key, '--')))
    add('| operstate | {} | {} |'.format(can_before.get('operstate', '--'), can_after.get('operstate', '--')))
    add('')
    add('普通错误计数可能不覆盖 CAN 专用错误，请同时查看增强诊断中的 xstats；软件侧“等应答超时”只表示该窗口内没收到确认，'
        '不能直接等同于物理丢帧。')
    add('')

    add('## 结论与判读')
    add('')
    problems = []
    if point is not None and not hw_sample_ok(point):
        problems.append('接口采样点为 {:.3f}，不是参考风机程序要求的 {:.3f}；'
                        '此偏差需要核对硬件时序，但不能单凭采样点确定失联原因，'
                        '请用 ./run_test.sh 启动以重新配置。'.format(point, HW_SAMPLE_POINT))
    if stats['write_timeout'] or stats['read_timeout']:
        problems.append('存在写/读应答超时：写 {} 次、读 {} 次。'.format(
            stats['write_timeout'], stats['read_timeout']))
    if rate(lost_report, expected_report) > 0.5:
        problems.append('心跳估算缺口率 {:.2f}%，高于 0.5%；需结合时间戳、队列丢弃和控制器状态定位，尚不能确定原因。'.format(
            rate(lost_report, expected_report)))
    err_delta = (can_after.get('rx_errors', 0) - can_before.get('rx_errors', 0),
                 can_after.get('tx_errors', 0) - can_before.get('tx_errors', 0))
    if err_delta != (0, 0):
        problems.append('CAN 控制器错误计数增加：rx_errors +{}、tx_errors +{}。'.format(*err_delta))
    if stats.get('write_send_fail', 0) or stats.get('read_send_fail', 0):
        problems.append('部分请求在本地发送阶段失败，尚未进入等待应答阶段。')
    if stats['tx_fail']:
        problems.append('发送失败 {} 次。'.format(stats['tx_fail']))
    if stats.get('init_retry', 0):
        problems.append('初始化阶段有 {} 次无应答（重试后继续），仅表明初始化确认未及时完成。'.format(
            stats['init_retry']))
    if stats.get('motor_lost', 0):
        problems.append('电机会话中断 {} 次（最长静默 {:.0f}ms）：曾一度收不到电机帧，'
                        '是否恢复需查事件时间轴；原因尚未确定，需检查 CAN 接线、'
                        '终端电阻与共地，以及电机供电是否被拉掉。'.format(
                            stats['motor_lost'], stats.get('silent_max', 0.0)))
    if fan is not None:
        fs = fan.stats
        if fan.faulted:
            problems.append('风机帧连续 {} 次发送失败，连续发送失败；控制器实际状态见增强诊断。'.format(
                HW_MAX_SEND_FAIL))
        if fs['tx_fail']:
            problems.append('风机油门帧发送失败 {} 次。'.format(fs['tx_fail']))
        if fs['period_ms']:
            p95 = percentile(fs['period_ms'], 0.95)
            if p95 > 1.5 * 1000.0 / max(1.0, float(settings.get('fan_hz', 50))):
                problems.append('风机油门下发周期 p95 {:.1f}ms，明显大于设定周期。'.format(p95))
        for node in sorted(fan.nodes):
            info = fan.nodes[node]
            if info['telem'] - info['dup'] <= 0:
                problems.append('电调 0x{:02X} 未收到非重复遥测帧：检查原始遥测、供电、CAN接线与节点号。'.format(node))
            elif fan.target > 0 and not (info['rpm'] or 0):
                problems.append('电调 0x{:02X} 有油门指令但上报转速为 0：'
                                '可能未完成使能、桨叶卡滞或油门低于启动阈值。'.format(node))
    if outcome == 'interrupted':
        problems.append('用户主动中断，未完成计划时长；收尾执行情况见增强诊断。')
    elif outcome not in ('completed', 'checked'):
        problems.append('测试未正常完成，收尾执行情况见增强诊断。')
    if not problems:
        add('- 写 {} 次、读 {} 次请求全部有应答，超时 0 次；控制器错误计数无增加，本次通信链路正常。'.format(
            req_w, req_r))
        if lost_report:
            add('- 心跳估算丢失 {} 帧（丢帧率 {:.3f}%，未超过 0.5% 判定阈值）：单帧缺口 {} 个、'
                '多帧缺口 {} 个、最大缺口 {:.1f}ms。'.format(
                    lost_report, rate(lost_report, expected_report), single_gap, multi_gap, gap_max))
            add('- 心跳缺口只表示“该时刻没收到上报帧”，是通信节拍指标，不是电机故障码；'
                '本次普通错误计数为 0 且请求全部有应答，'
                '仍不能单凭这些指标确定缺口来自采集侧或总线侧，请结合增强诊断。')
        else:
            add('- 心跳全程无缺口。')
        if fan is not None and all(fan.nodes[n]['telem'] > 0 for n in fan.nodes):
            add('- 两台风机都有周期性遥测上报，油门帧按设定周期下发。')
    else:
        for item in problems:
            add('- ' + item)
    if final_pos is not None:
        add('- 结束时反馈角度 {:.3f} rad，位置指令 {:.3f} rad，偏差 {:.3f} rad。'.format(
            final_pos[0], final_pos[1], final_pos[1] - final_pos[0]))
    add('')
    add('## 文件说明')
    add('')
    add('- `can-frames.log`：逐帧收发记录，含每帧解析备注和每分钟通信摘要。')
    add('- `test.log`：运行进度、每组动作与每分钟通信摘要。')
    add('- `summary.json`：本次全部统计与逐组动作记录。')
    add('- `config.json`：本次参数与 CAN 控制器初始计数。')
    add('- 中文总结：本次测试的统计与判读，与上述文件同目录。')
    add('')
    lines.extend(heartbeat_position_probe_report(stats))
    return '\n'.join(lines)


def heartbeat_position_probe_report(stats):
    records = stats.get('heartbeat_position_probes', [])
    metrics = heartbeat_loss_metrics(stats)
    lines = ['## 心跳丢失逐帧读取位置和速度', '',
             '不再按固定50ms空窗反复读取。沿用报告的缺帧判据：间隔大于1.5倍参考周期时，'
             '估算少帧数=max(0, round(间隔/参考周期)-1)。参考周期在启动校准后固定，实时触发和最终报告使用同一值。',
             '心跳为设备主动上报，通常收到下一帧后才能确认前面少帧。每确认丢失1帧，只安排1组独立读取：'
             '位置0x7019（rad）及速度0x701B（rad/s）。数值是确认缺帧后读取的实时值，不是缺帧时刻的历史快照。', '']
    if not metrics['tracking_enabled']:
        return lines + ['本轮未开启逐帧诊断（例如只读连通检查）；历史统计不能补造当时的位置/速度。', '']
    completed = metrics['completed_probe_groups']
    lines += ['报告估算丢失{}帧；逐帧诊断记录{}组；已完成读取{}组，未完成{}组；数量一致：{}。'.format(
        metrics['estimated_lost'], len(records), completed, metrics['uncompleted_probe_groups'],
        '是' if metrics['one_group_per_lost_frame'] else '否'), '']
    if not records:
        return lines + ['本轮没有确认心跳缺帧，未发出额外位置/速度诊断读请求。', '']
    names = {'success': '成功', 'timeout': '超时', 'rejected': '拒绝', 'send_failed': '发送失败',
             'invalid_value': '数值无效', 'queued': '待执行', 'interrupted': '中断',
             'not_run_before_shutdown': '收尾前未执行'}
    lines += ['| 丢失帧编号 | 缺口编号 / 缺口内序号 | 预计缺帧时刻 | 确认时刻 / 读取开始 | 位置rad / 结果 | 速度rad/s / 结果 | 确认后延迟 / 总读取耗时ms |',
              '|---:|---|---|---|---|---|---|']
    fmt = lambda v: '--' if v is None else '{:.3f}'.format(v)
    for r in records:
        readings = r.get('readings', {})
        def result(key):
            entry = readings.get(key, {})
            value = entry.get('value')
            status = entry.get('status', r['status'])
            return '{} / {}(状态码{})'.format('--' if value is None else '{:.6f}'.format(value),
                                            names.get(status, status), entry.get('status_code', '--'))
        lines.append('| {} | {} / {} | {} | {} / {} | {} | {} | {} / {} |'.format(
            r['id'], r['gap_id'], r['slot_in_gap'], r['estimated_missing_at'] or '--',
            r['confirmed_at'], r['request_at'] or '--', result('position'), result('velocity'),
            fmt(r['request_delay_ms']), fmt(r['read_elapsed_ms'])))
    lines += ['', '每个编号只对应1组读取，不因持续静默反复读取同一帧。正常收尾会完成已确认的排队读取；'
              '用户中断或故障中止时，未执行项明确标注，不补造数值。读取失败也保留这一组记录，另一参数仍会尝试。', '']
    return lines


def read_fail_text(status):
    """读参数失败时的说明：状态码非 0 表示电机拒绝，0 表示窗口内没有应答。"""
    if status:
        return '读取被拒（状态码 {}，该固件可能不支持此索引）'.format(status)
    return '读取无应答'


def fmt_lat(values):
    if not values:
        return '-- / -- / -- / -- / -- / --'
    return '{:.3f} / {:.3f} / {:.3f} / {:.3f} / {:.3f} / {:.3f}'.format(
        min(values), sum(values) / len(values), percentile(values, 0.5),
        percentile(values, 0.95), percentile(values, 0.99), max(values))


def main(argv=None):
    args = parse_args(argv)
    if os.geteuid() != 0:
        print('需要 root 权限操作 CAN 接口。', flush=True)
        return 1
    if not Path('/sys/class/net/{}'.format(args.iface)).exists():
        print('CAN 接口不存在：{}'.format(args.iface), flush=True)
        return 1
    import fcntl
    # Serialize only this suite's controllers. Other CAN applications must still be stopped manually.
    lock_path = ROOT / ('.control-' + args.iface.replace('/', '_') + '.lock')
    inherited = os.environ.get('RS00_X6_LOCK_FD')
    if inherited:
        lock_fd = int(inherited)
        if os.fstat(lock_fd).st_ino != lock_path.stat().st_ino:
            raise RuntimeError('继承的控制锁无效')
    else:
        lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('本测试套件已有控制进程占用 {}，未启动。'.format(args.iface), flush=True)
        os.close(lock_fd)
        return 1

    stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    tag = '只读检查' if args.check_only else '{:g}分钟'.format(args.minutes)
    directory = ROOT / 'logs' / ('rs00-x6-{}-{}'.format(tag, stamp))
    directory.mkdir(parents=True, exist_ok=True)
    log = Logger(directory / 'test.log')
    t0 = time.monotonic()
    frames = FrameLog(directory / 'can-frames.log', args.iface, t0)
    started_at = now_iso()
    can_before = read_sys_stats(args.iface)

    log('CAN 接口 {}：{}，类型 {}'.format(
        args.iface, can_before.get('operstate'), 'CAN 2.0 1Mbps'))
    can_state, sample_point = read_bittiming(args.iface)
    args.sample_point = sample_point
    if sample_point is None:
        log('采样点：读不到接口位时序，请确认 ip 命令可用。')
    else:
        ok = hw_sample_ok(sample_point)
        log('控制器状态 {}，采样点 {:.3f}{}'.format(
            can_state or '未知', sample_point,
            '' if ok else '（参考协议采样点 {:.3f}，当前值偏差较大；'
                          '请用 ./run_test.sh 启动以重新配置）'.format(HW_SAMPLE_POINT)))
    log('计划：位置模式（PP）循环 正转{}ms → 保持{}ms → 反转{}ms，速度 {}rad/s；'
        '轮询{}ms、读{}ms、写{}ms；时长 {:g} 分钟。'.format(
            args.fwd_ms, args.hold_ms, args.rev_ms, args.speed,
            args.poll_ms, args.read_ms, args.write_ms, args.minutes))
    if args.no_fan:
        log('风机：不发送控制帧、不解析遥测（--no-fan）；物理接入的电调仍可能上报。')
    elif args.fan_disabled or args.check_only:
        log('风机：只接收遥测，本次不发送任何风机控制帧。')
    else:
        log('风机：节点 0x{:02X}(CH{}) / 0x{:02X}(CH{})，油门 {}（{:.1f}%），{}Hz 持续下发。'.format(
            HW_ESC_LEFT, HW_ESC_LEFT, HW_ESC_RIGHT, HW_ESC_RIGHT, args.fan_throttle,
            args.fan_throttle * 100.0 / HW_MAX_THROTTLE, args.fan_hz))

    motor = Motor(args.iface, args.motor_id or 0x7F, frames, log)
    diag = Diagnostics(directory, args.iface, motor)
    motor.diag = diag
    diag.enable_socket(motor.sock)
    motor.write_ms = float(args.write_ms)
    fan = None
    if not args.no_fan:
        fan = Fan(motor.sock, frames, log, args.fan_throttle, args.fan_hz)
        fan.diag = diag
        motor.fan = fan
        if fan.target > 2000:
            log('注意：风机油门 {}（{:.0f}%）偏高，请确认桨叶已装好或已可靠固定。'.format(
                fan.target, fan.target * 100.0 / HW_MAX_THROTTLE))
    outcome = 'failed'
    reason = ''
    cycles = []
    travel_guard = False
    target_base = 0.0
    motor_id = args.motor_id
    motor_enabled = False
    report_pos = 0
    eps_set = False
    diag.start()

    try:
        initial_state = diag.before.get('state')
        if not args.check_only and initial_state != 'ERROR-ACTIVE':
            raise RuntimeError('启动前控制器状态为 {}，应先恢复 ERROR-ACTIVE；未使能设备。'.format(initial_state))
        # 1. 找电机
        if not motor_id:
            candidates = [0x7F] + [i for i in range(1, 17) if i != 0x7F]
            for candidate in candidates:
                value = motor.probe(candidate)
                if value is not None:
                    motor_id = candidate
                    log('探测到电机：CAN ID {}，当前角度 {:.3f} rad。'.format(motor_id, value))
                    break
            if not motor_id:
                raise RuntimeError('未探测到电机（已扫描 ID 127 与 1~16，可用 --motor-id 指定）：'
                                   '请检查接线、120Ω 终端电阻和 1Mbps 波特率')
        else:
            motor.motor_id = motor_id
            value = motor.read_param(IDX_MECH_POS, args.read_ms)
            if value is None:
                raise RuntimeError('电机 CAN ID {} 读角度无应答'.format(motor_id))
            log('电机 CAN ID {} 应答正常，当前角度 {:.3f} rad。'.format(motor_id, value))

        vbus = motor.read_param(IDX_VBUS, args.read_ms)
        vbus_status = motor.last_status
        fault = motor.read_param(IDX_FAULT, args.read_ms, 'uint32')
        fault_status = motor.last_status
        log('母线电压 {}，故障码 {}。'.format(
            '{:.1f}V'.format(vbus) if vbus is not None else read_fail_text(vbus_status),
            '0x{:08X}'.format(fault) if fault is not None else read_fail_text(fault_status)))
        if fault:
            raise RuntimeError('电机存在故障码 0x{:08X}，请先清故障'.format(fault))

        # 2. Read parameters / observe telemetry only. Never enable either device.
        if args.check_only:
            motor.drain(1.0)
            log('只读检查完成，未使能、未运动。')
            outcome = 'checked'
            return 0

        # 3. 切位置模式
        run_mode = RUN_MODE_PP if args.mode == 'pp' else RUN_MODE_CSP
        if not motor.write_retry(IDX_RUN_MODE, bytes([run_mode]), args.write_ms,
                                 '位置模式' + args.mode.upper(), args.retry):
            raise RuntimeError('写入 run_mode 失败：电机连续 {} 次无应答（约 {:.0f}s）'.format(
                args.retry, (args.retry - 1) * 2.0))
        motor.drain(0.05)
        if args.mode == 'pp':
            if not motor.write_retry(IDX_VEL_MAX, struct.pack('<f', args.speed), args.write_ms,
                                     'vel_max {:.2f}rad/s'.format(args.speed), args.retry):
                raise RuntimeError('写入 vel_max 失败：电机连续 {} 次无应答'.format(args.retry))
            if not motor.write_retry(IDX_ACC_SET, struct.pack('<f', args.acc), args.write_ms,
                                     'acc_set {:.1f}rad/s²'.format(args.acc), args.retry):
                raise RuntimeError('写入 acc_set 失败：电机连续 {} 次无应答'.format(args.retry))
        else:
            if not motor.write_retry(IDX_LIMIT_SPD, struct.pack('<f', args.speed), args.write_ms,
                                     'limit_spd {:.2f}rad/s'.format(args.speed), args.retry):
                raise RuntimeError('写入 limit_spd 失败')

        # 4. 使能
        base = motor.last_pos
        enabled = False
        for attempt in range(1, args.retry + 1):
            if not motor.enable():
                raise RuntimeError('发送使能帧失败')
            motor_enabled = True
            before = motor.stats['feedback']
            motor.drain(0.3)
            if motor.stats['feedback'] > before:
                enabled = True
                break
            motor.stats['init_retry'] += 1
            if attempt < args.retry:
                log('使能后没收到反馈帧（第 {} 次），2s 后重试。'.format(attempt))
                motor.drain(2.0)
        if not enabled:
            raise RuntimeError('使能后连续 {} 次收不到反馈帧，电机可能已掉出总线'.format(args.retry))
        log('已发送使能帧，电机反馈正常。')
        log('电机状态：角度 {:.3f} rad，速度 {:.3f} rad/s，温度 {:.1f}℃。'.format(
            motor.last_pos, motor.last_vel, motor.last_temp))

        # 5. 设定上报周期并打开自动上报作为心跳
        report_on = False
        report_pos = 0
        eps_set = False
        motor.report_ms = args.report_ms
        eps_value = eps_time_value(args.report_ms)
        if motor.option_write(IDX_EPS_TIME, struct.pack('<H', eps_value), args.write_ms,
                              'EPScan_time={} 上报周期 {:.0f}ms'.format(eps_value, args.report_ms)):
            eps_set = True
            motor.drain(0.05)
            log('已下发上报周期 {:.0f}ms（寄存器 0x7026={}，1 代表 10ms，每 +1 增 5ms）。'.format(
                args.report_ms, eps_value))
        else:
            log('上报周期寄存器 0x7026 无应答，沿用电机当前周期（统计按实测间隔自适应）。')
        for pos in (6, 7):
            payload = bytearray([1, 2, 3, 4, 5, 6, 0, 0])
            payload[pos] = 1
            motor._send(M_REPORT, bytes(payload), '开启自动上报')
            before = motor.stats['report']
            mark = len(motor.gaps)
            motor.drain(0.4)
            if motor.stats['report'] > before:
                report_on = True
                report_pos = pos
                recent = motor.gaps[mark:]
                measured = percentile(recent, 0.5) if recent else 0.0
                if measured > 0:
                    motor.report_ms = measured
                log('自动上报已开启（F_CMD 位于 byte{}），目标周期 {:.0f}ms，实测间隔 {:.1f}ms。'.format(
                    pos, args.report_ms, measured))
                break
        if not report_on:
            log('自动上报未开启，心跳统计将无数据（不影响运动测试）。')
        else:
            motor.enable_heartbeat_probe(args.read_ms)

        # 6. 风机初始化：此时电机已使能并在保持位，期间电机照常每 70ms 收一帧位置指令
        if fan is not None and not args.fan_disabled:
            fan.init_sequence(motor, args.fan_zero_sec)
            if fan.faulted:
                raise RuntimeError('风机初始化期间连续 {} 次发送失败，总线已无法发送'
                                   '（仅凭发送失败无法确定控制器状态，见 bus-health.jsonl），'
                                   '请检查采样点是否为 {:.3f}'.format(HW_MAX_SEND_FAIL, HW_SAMPLE_POINT))

        # 7. 循环
        travel = args.speed * args.fwd_ms / 1000.0
        if base + travel > P_MAX - 0.5:
            travel_guard = True
            log('初始角度 {:.3f} rad 接近上限，本次先反转后正转。'.format(base))
        first_dir = -1 if travel_guard else 1
        log('开始循环：每 {:.1f}s 一组，共约 {:g} 分钟。'.format(
            (args.fwd_ms + args.hold_ms + args.rev_ms) / 1000.0, args.minutes))

        deadline = time.monotonic() + args.minutes * 60.0
        cycle_index = 0
        next_minute = time.monotonic() + 60.0
        liveness = Liveness(log, 2000.0, args.motor_lost_sec * 1000.0)
        next_vbus = time.monotonic() + args.vbus_sec
        phase_list = [('正转', first_dir, args.fwd_ms),
                      ('停止保持', 0, args.hold_ms),
                      ('反转', -first_dir, args.rev_ms)]
        target_base = base
        while time.monotonic() < deadline:
            cycle_index += 1
            record = {'cycle': cycle_index, 'started': now_iso()}
            for name, direction, duration_ms in phase_list:
                if duration_ms <= 0:
                    continue
                phase = Phase(name, direction, duration_ms)
                log('第{}组：{} {}，{}ms。'.format(
                    cycle_index, name,
                    '{:+.2f} rad/s'.format(direction * args.speed) if direction else '位置保持',
                    duration_ms))
                start_pos = target_base
                phase_t0 = time.monotonic()
                end_at = phase_t0 + duration_ms / 1000.0
                next_tick = phase_t0
                target = start_pos
                while True:
                    now = time.monotonic()
                    if now >= end_at:
                        break
                    motor.run_heartbeat_checks()
                    frac = min(now - phase_t0, duration_ms / 1000.0)
                    target = start_pos + direction * args.speed * frac
                    ok_w = motor.write_param(IDX_LOC_REF, struct.pack('<f', target),
                                             args.write_ms, '{:.3f}'.format(target))
                    motor.run_heartbeat_checks()
                    pos = motor.read_param(IDX_MECH_POS, args.read_ms)
                    motor.run_heartbeat_checks()
                    vel = motor.read_param(IDX_MECH_VEL, args.read_ms)
                    motor.run_heartbeat_checks()
                    phase.add(pos, vel, not (ok_w and pos is not None and vel is not None))
                    if args.vbus_sec and time.monotonic() >= next_vbus:
                        voltage = motor.read_param(IDX_VBUS, args.read_ms)
                        diag.emit('motor_bus_voltage', voltage=voltage, status=motor.last_status,
                                  last_pos=motor.last_pos, last_vel=motor.last_vel)
                        next_vbus = time.monotonic() + args.vbus_sec
                    if liveness.check(motor):
                        raise RuntimeError('电机连续 {:.0f}s 收不到任何帧（掉出总线 / 断电 / 接线松动）'.format(
                            args.motor_lost_sec))
                    if fan is not None and fan.faulted:
                        raise RuntimeError('风机帧连续 {} 次发送失败，总线已无法发送'
                                           '（仅凭发送失败无法确定控制器状态，见 bus-health.jsonl）'.format(HW_MAX_SEND_FAIL))
                    next_tick += args.poll_ms / 1000.0
                    remain = next_tick - time.monotonic()
                    if remain > 0:
                        motor.drain(remain)
                    else:
                        next_tick = time.monotonic()
                    if time.monotonic() >= next_minute:
                        next_minute += 60.0
                        report_progress(log, frames, motor, args, fan)
                target_base = start_pos + direction * args.speed * duration_ms / 1000.0
                key = {'正转': 'fwd', '停止保持': 'hold', '反转': 'rev'}[name]
                record[key] = phase.summary(args.speed)
                log('{}结束：角度 {:.3f} rad，速度 {:.3f}~{:.3f} rad/s。'.format(
                    name, motor.last_pos, min(phase.vel) if phase.vel else 0.0,
                    max(phase.vel) if phase.vel else 0.0))
            cycles.append(record)
            log('第{}组完成，累计 {:.1f} 秒。'.format(cycle_index, time.monotonic() - t0))

        # 7. 收尾
        motor.write_param(IDX_LOC_REF, struct.pack('<f', target_base), args.write_ms, '停在当前位置')
        log('测试时长到，已发送停止位置指令。')
        motor.end_report_window(flush=True)
        if report_pos:
            payload = bytearray([1, 2, 3, 4, 5, 6, 0, 0])
            motor._send(M_REPORT, bytes(payload), '关闭自动上报')
            motor.drain(0.2)
        if eps_set:
            # 恢复电机出厂默认周期（仅 RAM，不写 flash）
            motor.option_write(IDX_EPS_TIME, struct.pack('<H', 1), args.write_ms, '恢复上报周期 10ms')
        stopped = motor.confirm_stop()
        log('电机停止并失能确认：{}。'.format(stopped))
        if fan is not None and fan.active:
            fan.shutdown(motor)
        if not stopped:
            raise RuntimeError('停止指令未得到停机状态反馈，不能确认已停止并失能')
        outcome = 'completed'

    except KeyboardInterrupt:
        motor.end_report_window()
        outcome = 'interrupted'
        reason = '用户中断'
        log('收到中断，正在停止并失能。')
        try:
            if motor_enabled:
                motor.confirm_stop()
        except Exception:
            pass
        try:
            if fan is not None and fan.active:
                fan.shutdown(motor)
        except Exception:
            pass
    except Exception as exc:
        motor.end_report_window()
        outcome = 'failed'
        reason = str(exc)
        log('异常：{}'.format(exc))
        try:
            if motor_enabled:
                motor.confirm_stop()
        except Exception:
            pass
        try:
            if fan is not None and fan.active:
                fan.shutdown(motor)
        except Exception:
            pass
    finally:
        motor.end_report_window()
        if outcome != 'completed' and motor_enabled:
            # Snapshot the live reporting tail before cleanup changes reporting mode.
            if report_pos:
                payload = bytearray([1, 2, 3, 4, 5, 6, 0, 0])
                motor._send(M_REPORT, bytes(payload), '异常收尾关闭自动上报')
            if eps_set:
                motor.option_write(IDX_EPS_TIME, struct.pack('<H', 1), args.write_ms, '异常收尾恢复10ms')
        diagnostics = diag.finish()
        ended_at = now_iso()
        can_after = read_sys_stats(args.iface)
        stats = motor.stats
        if outcome == 'completed':
            report_progress(log, frames, motor, args, fan)
        frames.note('CAN STAT {} TX={} RX={} TX_ERR={}'.format(
            args.iface, stats['tx'], stats['rx'], stats['tx_fail']))
        fan_summary = None
        if fan is not None:
            fs = fan.stats
            fan_summary = {
                'target_throttle': fan.target, 'hz': args.fan_hz,
                'tx': fs['tx'], 'tx_fail': fs['tx_fail'], 'late': fs['late'],
                'raw_tx': fs['raw_tx'], 'send_retries': fs['send_retries'],
                'shutdown_status': fan.shutdown_status,
                'telem': fs['telem'], 'other': fs['other'],
                'period_ms': {q: round(percentile(fs['period_ms'], q), 4) for q in (0, .5, .95, .99, 1)},
                'nodes': {},
            }
            for node in sorted(fan.nodes):
                info = dict(fan.nodes[node])
                gaps = info.pop('gaps')
                info['gap_max'] = round(max(gaps), 4) if gaps else None
                fan_summary['nodes']['0x{:02X}'.format(node)] = info
        summary = {
            'outcome': outcome, 'reason': reason,
            'started_at': started_at, 'ended_at': ended_at,
            'settings': vars(args), 'motor_id': motor_id,
            'source_sha256': {name: __import__('hashlib').sha256((ROOT / name).read_bytes()).hexdigest()
                              for name in ('x6test.py', 'can_diagnostics.py') if (ROOT / name).is_file()},
            'stats': {k: v for k, v in stats.items() if k not in ('lat_write', 'lat_read', 'idx', 'gaps')},
            'latency': {
                'write': {q: round(percentile(stats['lat_write'], q), 4) for q in (0, .5, .95, .99, 1)},
                'read': {q: round(percentile(stats['lat_read'], q), 4) for q in (0, .5, .95, .99, 1)},
            },
            'params': {hex(k): v for k, v in stats['idx'].items()},
            'fan': fan_summary,
            'diagnostics': diagnostics,
            'motor_stop_confirmed': motor.stop_confirmed,
            'heartbeat': {'loss_metrics': heartbeat_loss_metrics(stats), 'kernel_gap_ms': {'p50': percentile(motor.gaps, .5),
                                           'max': max(motor.gaps, default=0)},
                          'application_gap_max_ms': max(motor.app_gaps, default=0),
                          'tail_silence_ms': motor.report_tail_ms},
            'cycles': cycles,
            'can_before': can_before, 'can_after': can_after,
        }
        issues = []
        if stats['write_timeout'] or stats['read_timeout']:
            issues.append('motor_response_timeout')
        if stats['tx_fail'] or (fan is not None and fan.stats['tx_fail']):
            issues.append('send_failed')
        if diagnostics['socket_drops']:
            issues.append('socket_drops')
        if any(diagnostics['xstats_peak_delta'].get(k, 0) > 0 for k in
               ('bus_error', 'error_warning', 'error_passive', 'bus_off', 'restarts')):
            issues.append('can_controller_errors')
        if diagnostics['controller_after'].get('state') != 'ERROR-ACTIVE':
            issues.append('controller_not_error_active')
        if stats['motor_lost']:
            issues.append('motor_offline')
        if not args.check_only and not stats['report']:
            issues.append('motor_heartbeat_unavailable')
        if not diagnostics['controller_before'].get('xstats') or not diagnostics['controller_after'].get('xstats'):
            issues.append('can_xstats_unavailable')
        if 'unavailable' in diagnostics['controller_before'] or 'unavailable' in diagnostics['controller_after']:
            issues.append('controller_diagnostics_unavailable')
        if diagnostics['error_frame_counts']:
            issues.append('can_error_frames_observed')
        if any(isinstance(diagnostics['socket_options'].get(k), dict) for k in
               ('rxq_ovfl', 'timestampns', 'error_filter')):
            issues.append('socket_diagnostics_incomplete')
        period = percentile(motor.gaps, .5)
        missing = heartbeat_loss_metrics(stats)['estimated_lost']
        if period and missing / max(1, stats['report'] + missing) > .005:
            issues.append('heartbeat_gap_above_threshold')
        if motor.report_tail_ms > 3 * max(period, args.report_ms):
            issues.append('heartbeat_tail_silence')
        if fan is not None:
            if fan.stats['late']:
                issues.append('throttle_deadline_miss')
            if any(info['last_raw'] is None or time.monotonic() - info['last_raw'] > 1
                   for info in fan.nodes.values()):
                issues.append('fan_telemetry_stale')
        if heartbeat_loss_metrics(stats)['one_group_per_lost_frame'] is False:
            issues.append('heartbeat_probe_count_mismatch')
        summary['validation'] = {'status': 'not_tested' if args.check_only else 'incomplete' if outcome == 'interrupted' else
                                 ('pass' if outcome == 'completed' and not issues else 'fail'),
                                 'issues': issues,
                                 'note': 'completed 仅表示流程完成；validation 单独判断通信。'}
        (directory / 'summary.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        (directory / 'config.json').write_text(json.dumps({
            'settings': vars(args), 'started_at': started_at, 'ended_at': ended_at,
            'can_before': can_before, 'can_after': can_after,
        }, ensure_ascii=False, indent=2), encoding='utf-8')
        try:
            final = (motor.last_pos, target_base) if outcome == 'completed' else None
            text = make_report(directory, vars(args), stats, cycles, can_before, can_after,
                               started_at, ended_at, outcome, reason, motor_id, travel_guard, final,
                               fan)
            text += diagnostic_report(diagnostics, motor, fan)
            text += '\n通信验收：`{}`；问题：{}。\n'.format(summary['validation']['status'], ', '.join(issues) or '无')
            name = 'RS00-X6PLUS_通信统计_{}_{}.md'.format(stamp, tag)
            (directory / name).write_text(text, encoding='utf-8')
            log('中文总结：{}'.format(directory / name))
        except Exception as exc:
            log('总结生成失败：{}'.format(exc))
        log('本次测试结束：{}'.format(outcome))
        frames.close()
        log.close()
        motor.sock.close()
        os.close(lock_fd)
        return 130 if outcome == 'interrupted' else (0 if outcome == 'checked' or summary['validation']['status'] == 'pass' else 1)


def report_progress(log, frames, motor, args, fan=None):
    stats = motor.stats
    text = ('通信小结：发送{}帧、接收{}帧；写请求/应答/超时={}/{}/{}；读请求/应答/超时={}/{}/{}；'
            '自动上报{}帧、最大间隔{:.1f}ms；发送失败{}次。').format(
        stats['tx'], stats['rx'],
        stats['write_req'], stats['write_ack'], stats['write_timeout'],
        stats['read_req'], stats['read_ack'], stats['read_timeout'],
        stats['report'], max(stats['gaps']) if stats['gaps'] else 0.0,
        stats['tx_fail'])
    if fan is not None:
        fs = fan.stats
        text += '风机总TX{}帧（失败{}，Raw周期中位{:.1f}ms）'.format(
            fs['tx'], fs['tx_fail'], percentile(fs['period_ms'], 0.5))
        for node in sorted(fan.nodes):
            info = fan.nodes[node]
            text += '；ESC0x{:02X} 遥测{}帧（重复{}）/{}rpm'.format(
                node, info['telem'], info['dup'],
                '--' if info['rpm'] is None else info['rpm'])
        text += '。'
    text += '确认心跳丢失帧/诊断组={}/{}。'.format(heartbeat_loss_metrics(stats)['estimated_lost'],len(stats.get('heartbeat_position_probes',[])))
    log(text)
    frames.note(text)


if __name__ == '__main__':
    def interrupted(_sig, _frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    raise SystemExit(main())
