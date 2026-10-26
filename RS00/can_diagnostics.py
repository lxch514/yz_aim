"""SocketCAN diagnostics. No interface configuration and no CAN transmissions here."""
import errno
import json
import socket
import struct
import subprocess
import threading
import time
from collections import Counter, deque
from pathlib import Path

CAN_ERR_FLAG = 0x20000000
CAN_RTR_FLAG = 0x40000000
CAN_EFF_FLAG = 0x80000000
SO_TIMESTAMPNS = 35  # Linux asm-generic/socket.h, old timespec (64-bit target)
SO_RXQ_OVFL = 40
XSTATS = ('restarts', 'bus_error', 'arbitration_lost', 'error_warning',
          'error_passive', 'bus_off')


def snapshot(iface):
    try:
        p = subprocess.run(['ip', '-j', '-details', '-statistics', 'link', 'show', iface],
                           capture_output=True, text=True, timeout=1)
        row = json.loads(p.stdout)[0]
        info = row.get('linkinfo', {})
        data = info.get('info_data', {})
        return {'state': data.get('state'), 'berr_counter': data.get('berr_counter'),
                'bittiming': data.get('bittiming'), 'xstats': info.get('info_xstats', {}),
                'net_stats': row.get('stats64', row.get('stats', {})),
                'restart_ms': data.get('restart_ms'), 'flags': row.get('flags'),
                'ctrlmode': data.get('ctrlmode'),
                'qdisc': row.get('qdisc'), 'txqlen': row.get('txqlen')}
    except (OSError, ValueError, IndexError, subprocess.SubprocessError) as exc:
        return {'unavailable': str(exc)}


def delta(before, after):
    return {k: after[k] - before[k] for k in before.keys() & after.keys()
            if isinstance(before[k], (int, float)) and isinstance(after[k], (int, float))}


class Diagnostics:
    def __init__(self, directory, iface, motor=None):
        self.iface, self.motor = iface, motor
        self.t0 = time.monotonic()
        self.events = open(Path(directory) / 'events.jsonl', 'w', buffering=1, encoding='utf-8')
        self.health = open(Path(directory) / 'bus-health.jsonl', 'w', buffering=1, encoding='utf-8')
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.before = snapshot(iface)
        self.after = self.before
        self.state_seen = Counter()
        self.max_xdelta = Counter()
        self.errnos = Counter()
        self.error_frames = Counter()
        self.rx_lags = deque(maxlen=100000)
        self.rx_lag_max = 0.0
        self.timestamp_frames = 0
        self.missing_timestamps = 0
        self.clock_anomalies = 0
        self.socket_drops = 0
        self.options = {}
        self.emit('initial_controller', snapshot=self.before)

    def emit(self, event_name, **fields):
        row = {'time': time.time(), 'elapsed': round(time.monotonic() - self.t0, 6),
               'event': event_name, **fields}
        with self.lock:
            self.events.write(json.dumps(row, ensure_ascii=False) + '\n')

    def enable_socket(self, sock):
        for level, option, value, name in (
                (socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024, 'rcvbuf'),
                (socket.SOL_SOCKET, SO_TIMESTAMPNS, 1, 'timestampns'),
                (socket.SOL_SOCKET, SO_RXQ_OVFL, 1, 'rxq_ovfl'),
                (getattr(socket, 'SOL_CAN_RAW', 101),
                 getattr(socket, 'CAN_RAW_ERR_FILTER', 2), 0x1FFFFFFF, 'error_filter')):
            try:
                sock.setsockopt(level, option, value)
                self.options[name] = sock.getsockopt(level, option)
            except OSError as exc:
                self.options[name] = {'unavailable': str(exc)}
        self.emit('socket_options', options=self.options)

    def received(self, ancillary):
        stamp_ns = None
        wall_ns = time.time_ns()
        for level, kind, body in ancillary:
            if level != socket.SOL_SOCKET:
                continue
            if kind == SO_TIMESTAMPNS and len(body) >= struct.calcsize('@ll'):
                sec, nsec = struct.unpack('@ll', body[:struct.calcsize('@ll')])
                stamp_ns = sec * 1000000000 + nsec
            elif kind == SO_RXQ_OVFL and len(body) >= 4:
                count = struct.unpack('=I', body[:4])[0]
                if count != self.socket_drops:
                    self.emit('socket_drop', previous=self.socket_drops, total=count)
                self.socket_drops = count
        if stamp_ns is None:
            self.missing_timestamps += 1
        else:
            lag = (wall_ns - stamp_ns) / 1000000.0
            if lag < 0:
                self.clock_anomalies += 1
            else:
                self.rx_lags.append(lag)
                self.rx_lag_max = max(self.rx_lag_max, lag)
            self.timestamp_frames += 1
        return stamp_ns

    def error_frame(self, can_id, payload):
        names = {0x01: 'tx_timeout', 0x02: 'arbitration_lost', 0x04: 'controller',
                 0x08: 'protocol', 0x10: 'transceiver', 0x20: 'ack', 0x40: 'bus_off',
                 0x80: 'bus_error', 0x100: 'restarted', 0x200: 'error_counters'}
        flags = [name for bit, name in names.items() if can_id & bit]
        self.error_frames.update(flags)
        # Keep representative raw evidence without flooding disk during a storm.
        total = sum(self.error_frames.values())
        if total < 30 or total % 1000 == 0:
            self.emit('can_error_frame', can_id=hex(can_id), data=payload.hex(), flags=flags)

    def send_error(self, exc, owner, can_id, attempt):
        key = '{}:{}'.format(owner, errno.errorcode.get(exc.errno, str(exc.errno)))
        self.errnos[key] += 1
        count = self.errnos[key]
        if count <= 10 or count % 100 == 0:
            self.emit('send_error', owner=owner, can_id=hex(can_id), errno=exc.errno,
                      error=str(exc), attempt=attempt, count=count, controller=self.after)

    def start(self):
        self.thread = threading.Thread(target=self._monitor, name='can-health', daemon=True)
        self.thread.start()

    def _monitor(self):
        previous = None
        prev_counts = {}
        prev_t = self.t0
        while not self.stop_event.is_set():
            now = time.monotonic()
            state = snapshot(self.iface)
            self.after = state
            self.state_seen[state.get('state', 'unavailable')] += 1
            xd = delta(self.before.get('xstats', {}), state.get('xstats', {}))
            for key, value in xd.items():
                self.max_xdelta[key] = max(self.max_xdelta[key], value)
            if state.get('state') != previous:
                self.emit('controller_state', previous=previous, snapshot=state)
                previous = state.get('state')
            counts = {}
            motor = self.motor
            if motor is not None:
                counts.update({k: motor.stats.get(k, 0) for k in
                               ('rx', 'tx', 'tx_fail', 'report', 'feedback',
                                'write_timeout', 'read_timeout')})
            row = {'elapsed': round(now - self.t0, 6), 'wall_time': time.time(),
                   'interval_sec': now - prev_t, 'snapshot': state, 'xstats_delta': xd,
                   'counts': counts, 'interval_counts': delta(prev_counts, counts),
                   'socket_drops': self.socket_drops,
                   'motor_last_rx_age_ms': motor.motor_alive_ms() if motor else None}
            self.health.write(json.dumps(row, ensure_ascii=False) + '\n')
            prev_counts, prev_t = counts, now
            self.stop_event.wait(1.0)

    def finish(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3)
        self.after = snapshot(self.iface)
        final_delta = delta(self.before.get('xstats', {}), self.after.get('xstats', {}))
        for k, v in final_delta.items():
            self.max_xdelta[k] = max(self.max_xdelta[k], v)
        self.emit('final_controller', snapshot=self.after)
        lags = sorted(self.rx_lags)
        def quantile(q):
            return lags[min(len(lags) - 1, round((len(lags) - 1) * q))] if lags else None
        result = {'controller_before': self.before, 'controller_after': self.after,
                  'xstats_delta': final_delta, 'xstats_peak_delta': dict(self.max_xdelta),
                  'states_observed': dict(self.state_seen), 'send_errno_counts': dict(self.errnos),
                  'error_frame_counts': dict(self.error_frames), 'socket_drops': self.socket_drops,
                  'socket_options': self.options, 'timestamp_frames': self.timestamp_frames,
                  'missing_timestamps': self.missing_timestamps, 'clock_anomalies': self.clock_anomalies,
                  'rx_lag_ms': {'p50': quantile(.5), 'p95': quantile(.95), 'p99': quantile(.99),
                                'max_all_frames': self.rx_lag_max, 'recent_sample_count': len(lags)}}
        self.health.close()
        self.events.close()
        return result


def send_bounded(sock, frame, diag=None, owner='motor', budget_ms=5.0):
    """Retry only temporary local queue pressure; never wait indefinitely."""
    deadline = time.monotonic() + budget_ms / 1000.0
    can_id = struct.unpack('=I', frame[:4])[0] & 0x1FFFFFFF
    attempts = 0
    while True:
        attempts += 1
        try:
            size = sock.send(frame)
            if size is not None and size != len(frame):
                raise OSError(errno.EIO, 'short CAN write')
            return attempts - 1
        except OSError as exc:
            if diag:
                diag.send_error(exc, owner, can_id, attempts)
            if exc.errno not in (errno.EAGAIN, errno.ENOBUFS, errno.EINTR) or time.monotonic() >= deadline:
                raise
            time.sleep(min(.001, max(0.0, deadline - time.monotonic())))


def diagnostic_report(data, motor):
    lines = ['\n## 增强诊断（v2）', '', '| 项目 | 记录 |', '|---|---|']
    lines.append('| 控制器开始 / 结束状态 | {} / {} |'.format(
        data['controller_before'].get('state'), data['controller_after'].get('state')))
    for k in XSTATS:
        lines.append('| {} 增量 / 观测最大增量 | {} / {} |'.format(
            k, data['xstats_delta'].get(k, '不可用'), data['xstats_peak_delta'].get(k, '不可用')))
    lines.append('| socket 接收队列丢弃（SO_RXQ_OVFL） | {} |'.format(data['socket_drops']))
    lines.append('| 内核时间戳帧 / 无时间戳帧 | {} / {} |'.format(
        data['timestamp_frames'], data['missing_timestamps']))
    lines.append('| 读取延迟 p50 / p95 / p99 / 全程最大（ms） | {} |'.format(data['rx_lag_ms']))
    lines.append('| 发送 errno 统计（含重试） | {} |'.format(data['send_errno_counts']))
    lines.append('| CAN 错误帧分类 | {} |'.format(data['error_frame_counts']))
    lines.append('| 电机写 / 读请求在发送阶段失败 | {} / {} |'.format(
        motor.stats.get('write_send_fail', 0), motor.stats.get('read_send_fail', 0)))
    lines.append('| 电机停止确认 | {} |'.format(getattr(motor, 'stop_confirmed', False)))
    lines.append('| 自动上报结束时未闭合静默（ms） | {:.1f} |'.format(
        getattr(motor, 'report_tail_ms', 0.0)))
    lines += ['', '心跳统计优先使用内核接收时间戳；它不是硬件总线分析仪时间戳。'
              '末尾静默单列，不会因再无下一帧而隐藏。',
              'SO_RXQ_OVFL 不可用时不能把 0 解读为已证明没有 socket 丢包。'
              '控制器普通错误计数为 0，不能排除 CAN 专用错误或驱动异常。',
              '电机读应答按来源、目标、索引和到达时间匹配；写反馈无事务编号，不能证明精确一一对应。']
    return '\n'.join(lines) + '\n'
