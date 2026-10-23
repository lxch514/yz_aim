#!/usr/bin/env python3
"""独立链路单电机往返测试：不更改硬件零点，失败即停止，不自动重试。"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import pty
import re
import select
import signal
import subprocess
import sys
import termios
import time

sys.dont_write_bytecode = True
from can_monitor import Statistics

PROMPT = re.compile(rb'yz\([^()\r\n]*\)> ')
BAD_RESPONSE = ('[失败]', '[!] 双轴同步异常', '故障需重启', '未能确认', '写入失败')


class TestFailure(RuntimeError):
    pass


STILL_RPM_LIMIT = 0.5


def read_speed(controller):
    """读一次 0x606C（固定 2 字节，避免宽度试探产生 SDO 拒绝），返回电机转速 RPM；读失败返回 None。"""
    text = controller.command('read-od 606c 2', timeout=10, allow_failure=True)
    found = re.search(r'= 0x([0-9A-Fa-f]+)（', text)
    if not found:
        return None
    raw = int(found.group(1), 16) & 0xFFFF
    return (raw - 0x10000 if raw >= 0x8000 else raw) / 10.0


def settle(controller, timeout=2.5):
    """等驱动停机后的残余转速收敛：连续两次 0x606C 都不超过静止阈值才放行，返回最后一次读数。"""
    last = None
    quiet = 0
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rpm = read_speed(controller)
        if rpm is not None:
            last = rpm
            quiet = quiet + 1 if abs(rpm) <= STILL_RPM_LIMIT else 0
            if quiet >= 2:
                return rpm
        time.sleep(0.05)
    return last


class Reporter:
    def __init__(self, path):
        self.file = path.open('a', encoding='utf-8', buffering=1)

    def __call__(self, message):
        line = f'[{datetime.now():%H:%M:%S}] {message}'
        print(line, flush=True)
        self.file.write(line + '\n')


class Communications:
    def __init__(self, path, report):
        self.path = path
        self.report = report
        self.reader = path.open('r', encoding='utf-8', errors='replace')
        self.append = path.open('a', encoding='utf-8', buffering=1)
        self.statistics = Statistics(self.emit)
        self.next_summary = time.monotonic() + 60

    def emit(self, message):
        self.report(message)
        self.append.write(f'[CAN监测 {datetime.now():%Y-%m-%d %H:%M:%S}] {message}\n')
        self.append.flush()

    def poll(self, live=True):
        # Keep following the producer during each 3-second action, not only between commands.
        for _ in range(20000):
            position = self.reader.tell()
            line = self.reader.readline()
            if not line:
                break
            if not line.endswith('\n'):
                self.reader.seek(position)
                break
            self.statistics.feed(line)
        else:
            return  # never diagnose heartbeat silence before catching up with the log
        s = self.statistics
        if live and s.epoch is not None and s.pid and Path('/proc', str(s.pid)).exists():
            s.expire(max(0, time.time() - s.epoch))
        if time.monotonic() >= self.next_summary:
            s.summary()
            self.next_summary = time.monotonic() + 60

    def result(self):
        self.poll(live=False)
        s = self.statistics
        return {'tx': max(s.count['TX'], s.driver['TX']),
                'rx': max(s.count['RX'], s.driver['RX']),
                'tx_failed': max(s.count['TX_ERR'], s.driver['TX_ERR']),
                'heartbeat_tx': s.count['heartbeat_tx'],
                'heartbeat_max_gap_ms': round(s.heartbeat_gap_max * 1000, 3),
                'heartbeat_timeouts': s.count['heartbeat_timeout'],
                'sdo_read_repeats_observed': s.count['read_retry'],
                'sdo_unconfirmed_timeouts': s.count['sdo_timeout'],
                'sdo_aborts': s.count['sdo_abort'],
                'kernel_rx_dropped': max(s.count['kernel_drop'], s.driver['KERNEL_RX_DROP']),
                'trace_records_dropped': s.driver['TRACE_DROP'],
                'trace_output_errors': s.driver['OUTPUT_DROP'],
                'can_error_frames': s.count['RX_ERROR'],
                'sdo_pending_at_end': len(s.pending),
                'sdo_requests_by_node': dict(s.nodes_tx),
                'sdo_responses_by_node': dict(s.nodes_rx)}

    def close(self):
        self.poll(live=False)
        self.statistics.summary()
        self.reader.close()
        self.append.close()


class Controller:
    def __init__(self, command, root, transcript):
        self.master, slave = pty.openpty()
        settings = termios.tcgetattr(slave)
        settings[3] &= ~(termios.ECHO | termios.ECHONL)
        termios.tcsetattr(slave, termios.TCSANOW, settings)
        self.process = subprocess.Popen(command, cwd=root, stdin=slave, stdout=slave,
                                        stderr=slave, start_new_session=True, close_fds=True)
        os.close(slave)
        self.transcript = transcript.open('ab', buffering=0)
        self.buffer = bytearray()
        self.on_idle = lambda: None
        self.ready = False

    def receive(self, timeout=25, expect_exit=False):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.on_idle()
            if not expect_exit and PROMPT.search(self.buffer):
                text = self.buffer.decode('utf-8', errors='replace').replace('\r', '')
                self.buffer.clear()
                self.ready = True
                return text
            readable, _, _ = select.select([self.master], [], [], 0.1)
            if readable:
                try:
                    chunk = os.read(self.master, 65536)
                except OSError:
                    chunk = b''  # Linux PTY returns EIO once the child closes the slave.
                if chunk:
                    self.transcript.write(chunk)
                    self.buffer.extend(chunk)
                    continue
            if self.process.poll() is not None:
                text = self.buffer.decode('utf-8', errors='replace').replace('\r', '')
                self.buffer.clear()
                if expect_exit:
                    return text
                raise TestFailure(f'控制程序提前退出，返回码{self.process.returncode}：\n{text[-1800:]}')
        raise TestFailure('等待控制程序完成命令超时；将执行停止与失能清理。')

    def command(self, command, timeout=25, allow_failure=False):
        if not self.ready:
            raise TestFailure('上一命令尚未确认结束，拒绝继续发送运动命令。')
        self.ready = False
        self.transcript.write(('\n[脚本命令] ' + command + '\n').encode())
        os.write(self.master, (command + '\n').encode())
        text = self.receive(timeout, expect_exit=command == 'quit')
        if not allow_failure and any(marker in text for marker in BAD_RESPONSE):
            raise TestFailure(text[-2500:])
        return text

    def cleanup(self, report):
        confirmed = False
        try:
            if self.process.poll() is None and not self.ready:
                self.process.send_signal(signal.SIGINT)
                try:
                    self.receive(timeout=18)
                except TestFailure as exc:
                    report('停止等待提示：' + str(exc)[-1200:])
            if self.process.poll() is None and self.ready:
                response = self.command('sync-disable', timeout=25, allow_failure=True)
                confirmed = ('[成功] A 轴停止并失能' in response and '[成功] B 轴停止并失能' in response
                             and not any(marker in response for marker in BAD_RESPONSE))
                if not confirmed:
                    report('失能清理提示：' + response[-1800:])
                self.command('quit', timeout=25, allow_failure=True)
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGTERM)
                self.process.wait(timeout=25)
        except (TestFailure, OSError, subprocess.TimeoutExpired) as exc:
            report('异常：未完整确认控制进程清理：' + str(exc))
        finally:
            self.transcript.close()
            os.close(self.master)
        return confirmed


def interface_counters(interface):
    base = Path('/sys/class/net') / interface / 'statistics'
    values = {}
    for name in ('rx_packets', 'tx_packets', 'rx_errors', 'tx_errors',
                 'rx_dropped', 'tx_dropped', 'rx_missed_errors'):
        try:
            values[name] = int((base / name).read_text().strip())
        except OSError:
            pass
    return values

