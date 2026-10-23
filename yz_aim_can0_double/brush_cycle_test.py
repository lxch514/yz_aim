#!/usr/bin/env python3
"""镜像毛刷往返实测：一份控制会话、固定软件基准，失败即停止，不自动重试。"""
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
                'can_error_frames': s.count['RX_ERROR']}

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
                confirmed = ('A 轴停止并失能' in response and 'B 轴停止并失能' in response
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


def parse_action(text, name, requested_ms):
    if f'[本次动作汇总] {name}，结束原因：requested duration elapsed' not in text:
        raise TestFailure('动作未按指定时间正常结束：\n' + text[-2200:])
    values = dict(re.findall(r'(requested_ms|stop_request_a_ms|stop_request_b_ms|max_valid_error_deg)=([\d.\-]+)', text))
    if any(key not in values for key in ('requested_ms', 'stop_request_a_ms', 'stop_request_b_ms')):
        raise TestFailure('缺少双轴停止时刻记录，停止继续测试。')
    stop_a, stop_b = float(values['stop_request_a_ms']), float(values['stop_request_b_ms'])
    if not (requested_ms <= stop_a <= requested_ms + 100 and requested_ms <= stop_b <= requested_ms + 100):
        raise TestFailure(f'停止请求时刻超出测试容差：A={stop_a}ms，B={stop_b}ms。')
    csv_path = re.search(r'CSV日志：([^\r\n]+)', text)
    return {'command': name, 'requested_ms': requested_ms, 'stop_a_ms': stop_a,
            'stop_b_ms': stop_b, 'max_error_deg': float(values.get('max_valid_error_deg', 0)),
            'motion_csv': csv_path[1].strip() if csv_path else ''}


def main():
    parser = argparse.ArgumentParser(description='双电机毛刷镜像往返测试；当前位置仅作为本次同步基准，不代表机械原点。')
    parser.add_argument('--rpm', type=int, default=300)
    parser.add_argument('--minutes', type=float, default=5)
    parser.add_argument('--move-ms', type=int, default=3000)
    parser.add_argument('--cycles', type=int, help='指定往返组数时替代minutes，便于短测')
    parser.add_argument('--if', dest='interface', default='can0')
    parser.add_argument('--node', type=int, default=1)
    parser.add_argument('--node2', type=int, default=2)
    parser.add_argument('--dry-run', action='store_true', help='仅打印计划，不启动控制程序')
    args = parser.parse_args()
    if not (1 <= args.rpm <= 300 and 1 <= args.move_ms <= 600000 and 0 < args.minutes <= 60):
        parser.error('速度1~300RPM，动作时间1~600000ms，测试总时长大于0且不超过60分钟')
    if args.node == args.node2 or not all(1 <= node <= 126 for node in (args.node, args.node2)):
        parser.error('两个节点必须不同且在1~126之间')
    if args.cycles is not None and args.cycles < 1:
        parser.error('cycles必须大于0')
    if args.dry_run:
        print(f'计划：安全使能 → 记录一次当前软件基准 → 循环extend {args.rpm} {args.move_ms} / '
              f'retract {args.rpm} {args.move_ms} → 停止并失能；{args.minutes:g}分钟或{args.cycles}组。')
        return 0
    root = Path(__file__).resolve().parent
    run_dir = root / 'logs' / f'brush-cycle-{datetime.now():%Y%m%d-%H%M%S}-{os.getpid()}'
    run_dir.mkdir(parents=True, exist_ok=False)
    report = Reporter(run_dir / 'test.log')
    controller = None
    communications = None
    actions = []
    cycles = 0
    started = None
    result = {'started_at': datetime.now().isoformat(timespec='seconds'), 'settings': vars(args),
              'run_directory': str(run_dir), 'outcome': 'not_started', 'reason': ''}
    before = interface_counters(args.interface)
    stop_confirmed = False
    try:
        report('测试开始：请勿同时使用其他控制程序；Ctrl+C会停止并失能两轴。')
        report('当前位置仅建立一次本次同步软件基准，不代表完全缩回的机械原点；循环中不重置基准。')
        command = [str(root / 'build/yz_aim_can0'), '--if', args.interface, '--node', str(args.node),
                   '--node2', str(args.node2), '--timeout', '3000']
        controller = Controller(command, root, run_dir / 'controller.log')
        startup = controller.receive()
        found = re.search(r'通讯详细日志："?([^"\r\n]+)', startup)
        if not found:
            raise TestFailure('未找到当前会话通信日志，不启动运动。\n' + startup[-1500:])
        trace_path = Path(found[1].strip())
        if not trace_path.resolve().is_relative_to(root / 'logs'):
            raise TestFailure('通信日志未位于本程序logs目录内。')
        communications = Communications(trace_path, report)
        result['communication_log'] = str(trace_path)
        controller.on_idle = communications.poll
        controller.command('sync-enable')
        zero = controller.command('sync-zero')
        if '固定软件零点：' not in zero:
            raise TestFailure('软件基准没有完整确认，不启动运动。')
        result['software_zero_message'] = zero.split('固定软件零点：', 1)[1].splitlines()[0]
        started = time.monotonic()
        while ((args.cycles is None and time.monotonic() - started < args.minutes * 60)
               or (args.cycles is not None and cycles < args.cycles)):
            for name in ('extend', 'retract'):
                report(f'第{cycles + 1}组：{"伸出" if name == "extend" else "缩进"} {args.rpm}RPM，{args.move_ms}毫秒。')
                response = controller.command(f'{name} {args.rpm} {args.move_ms}', timeout=args.move_ms / 1000 + 25)
                action = parse_action(response, name, args.move_ms)
                actions.append(action)
                (run_dir / 'progress.json').write_text(json.dumps({'completed_cycles': cycles,
                    'completed_actions': len(actions), 'elapsed_seconds': round(time.monotonic() - started, 3),
                    'latest_action': action}, ensure_ascii=False, indent=2), encoding='utf-8')
            cycles += 1
            report(f'第{cycles}组完成；已运行{time.monotonic() - started:.1f}秒，'
                   f'本组最大有效偏差{max(a["max_error_deg"] for a in actions[-2:]):.4f}°。')
        result['outcome'] = 'completed'
        report('测试时长已到，当前往返组已完整结束，正在停止并失能两轴。')
    except KeyboardInterrupt:
        result['outcome'] = 'interrupted'
        result['reason'] = '用户中断或会话关闭'
        report('收到中断，结束测试，不再下发新的运动命令。')
    except Exception as exc:
        result['outcome'] = 'failed'
        result['reason'] = str(exc)
        report('异常：测试停止，不自动重试或重新置零。\n' + str(exc))
    finally:
        if started is not None:
            result['wall_seconds'] = round(time.monotonic() - started, 3)
        if controller:
            stop_confirmed = controller.cleanup(report)
            result['controller_returncode'] = controller.process.poll()
        if communications:
            time.sleep(0.05)
            result['communication'] = communications.result()
            communications.close()
        result['disabled_confirmed'] = stop_confirmed
        result['completed_cycles'] = cycles
        result['completed_actions'] = len(actions)
        result['commanded_motion_seconds'] = len(actions) * args.move_ms / 1000
        result['max_error_deg'] = max((a['max_error_deg'] for a in actions), default=0)
        result['actions'] = actions
        after = interface_counters(args.interface)
        result['interface_before'] = before
        result['interface_after'] = after
        result['interface_delta'] = {key: after[key] - before[key] for key in before.keys() & after.keys()}
        if result['outcome'] == 'completed' and not stop_confirmed:
            result['outcome'] = 'failed'
            result['reason'] = '运动循环完成，但未完整确认两轴失能'
        result['ended_at'] = datetime.now().isoformat(timespec='seconds')
        (run_dir / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        report(f'测试结束：{result["outcome"]}；完整往返{cycles}组，动作{len(actions)}次，'
               f'两轴失能{"已确认" if stop_confirmed else "未完整确认"}。')
        report('结果文件：' + str(run_dir / 'summary.json'))
        report.file.close()
    return 0 if result['outcome'] == 'completed' else 130 if result['outcome'] == 'interrupted' else 1


if __name__ == '__main__':
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    raise SystemExit(main())
