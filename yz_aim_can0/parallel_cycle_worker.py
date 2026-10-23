#!/usr/bin/env python3
"""并行往返测试的单链路工作进程；由 parallel_cycle_test.py 调用。"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import signal
import sys
import time

sys.dont_write_bytecode = True
from single_cycle_test import (BAD_RESPONSE, Communications, Controller as BaseController, Reporter,
                               STILL_RPM_LIMIT, TestFailure, interface_counters,
                               read_speed)
from can_monitor import Statistics


def write_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    os.replace(temporary, path)


class Controller(BaseController):
    def __init__(self, command, root, transcript):
        self.transcript_path = transcript
        super().__init__(command, root, transcript)

    def cleanup(self, report):
        confirmed = super().cleanup(report)
        # The CLI may handle SIGINT by confirming output disable and exiting
        # directly instead of displaying another prompt. Require fresh final
        # cleanup evidence and a clean exit, never an earlier success marker.
        if not confirmed and self.process.poll() == 0:
            final = self.transcript_path.read_text(encoding='utf-8', errors='replace').rsplit('[脚本命令]', 1)[-1]
            success_at = final.rfind('[成功] 停止并失能输出')
            if success_at >= 0 and not any(marker in final[success_at:] for marker in BAD_RESPONSE):
                report('控制程序已自行确认停止并失能后退出。')
                confirmed = True
        return confirmed


class LinkStatistics(Statistics):
    def __init__(self, emit, node, backend):
        super().__init__(emit)
        self.node = node
        self.backend = backend
        self.enabled = {node: False}

    def feed(self, line):
        # A zero-valued kernel overflow counter is an observation, not an error.
        if re.search(r'\sRX_DROP\s+ID=0+\s', line):
            return None
        return super().feed(line)

    def summary(self):
        tx = max(self.count['TX'], self.driver['TX'])
        rx = max(self.count['RX'], self.driver['RX'])
        drops = max(self.count['kernel_drop'], self.driver['KERNEL_RX_DROP'])
        drop_text = (f'套接字接收队列明确丢弃{drops}帧' if self.backend == 'can0'
                     else 'USB适配器硬件丢帧计数不可用')
        self.emit(f'通信统计：发送{tx}帧、接收{rx}帧；节点{self.node} SDO请求/应答'
                  f'={self.nodes_tx[self.node]}/{self.nodes_rx[self.node]}；'
                  f'发送失败{max(self.count["TX_ERR"], self.driver["TX_ERR"])}次、'
                  f'总线错误帧{self.count["RX_ERROR"]}帧；主站心跳{self.count["heartbeat_tx"]}帧、'
                  f'最大有效间隔{self.heartbeat_gap_max * 1000:.1f}ms、超时{self.count["heartbeat_timeout"]}次，'
                  f'节点心跳{self.count["node_heartbeat"]}帧；SDO读重发{self.count["read_retry"]}次、'
                  f'监测未确认超时{self.count["sdo_timeout"]}次、拒绝{self.count["sdo_abort"]}次；'
                  f'{drop_text}，诊断记录丢弃{self.driver["TRACE_DROP"]}条、'
                  f'日志输出异常{self.driver["OUTPUT_DROP"]}次。SDO未确认不等于已证实的CAN物理丢帧。')


class LinkCommunications(Communications):
    def __init__(self, path, report, node, backend):
        super().__init__(path, report)
        self.statistics = LinkStatistics(self.emit, node, backend)

    def result(self):
        result = super().result()
        result['node_heartbeat_rx'] = self.statistics.count['node_heartbeat']
        result['sdo_repeated_writes_observed'] = self.statistics.count['repeat_object_write']
        return result


def wait_still(controller, timeout=3):
    end = time.monotonic() + timeout
    quiet = 0
    while time.monotonic() < end:
        rpm = read_speed(controller)
        if rpm is None:
            raise TestFailure('读取0x606C失败；不发送下一条运动命令。')
        quiet = quiet + 1 if abs(rpm) <= STILL_RPM_LIMIT else 0
        if quiet >= 2:
            return rpm
        time.sleep(.05)
    raise TestFailure('未连续两次确认电机静止；不发送下一条运动命令。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['can0', 'usb'], required=True)
    parser.add_argument('--node', type=int, required=True)
    parser.add_argument('--rpm', type=int, required=True)
    parser.add_argument('--move-ms', type=int, required=True)
    parser.add_argument('--stop-ms', type=int, required=True)
    parser.add_argument('--minutes', type=float, required=True)
    parser.add_argument('--log-dir', type=Path, required=True)
    parser.add_argument('--gate', type=Path, required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    parser.add_argument('--if', dest='interface', default='can0')
    parser.add_argument('--library', default='/root/yz_aim/lib/liblyscan.so')
    parser.add_argument('--lys', type=int, default=0)
    parser.add_argument('--channel', type=int, default=0)
    parser.add_argument('--sequence', choices=['forward-reverse', 'mirror'], default='forward-reverse',
                        help='forward-reverse：先正转后反转（并行测试默认）；mirror：先伸出后缩进')
    parser.add_argument('--hold-after', choices=['first', 'both'], default='first',
                        help='first：只在第一个动作后保持（并行测试默认）；both：两个方向后都保持')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    run_dir = args.log_dir.resolve()
    # 单轴测试把日志放在 /root/yz_aim/logs，与并行测试的产物并列，便于横向对比。
    shared = Path('/root/yz_aim/logs').resolve()
    inside = run_dir.is_relative_to(root / 'logs') or run_dir.is_relative_to(shared)
    if not inside or run_dir in (root / 'logs', shared):
        parser.error('日志目录必须位于本项目logs或/root/yz_aim/logs内')
    if not (1 <= args.node <= 126 and 1 <= args.rpm <= 300 and 1 <= args.move_ms <= 5000
            and 0 <= args.stop_ms <= 60000 and 0 < args.minutes <= 60):
        parser.error('测试参数超出允许范围')
    if (run_dir / 'summary.json').exists():
        parser.error('目录已有测试结果，不覆盖')
    report = Reporter(run_dir / 'test.log')
    controller = communications = started = None
    cycles = 0
    actions = []
    enable_attempted = disabled = False
    result = {'started_at': datetime.now().isoformat(timespec='milliseconds'),
              'settings': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
              'run_directory': str(run_dir), 'outcome': 'not_started', 'reason': ''}
    before = interface_counters(args.interface) if args.backend == 'can0' else {}
    if args.sequence == 'mirror':
        plan = [('extend', -1 if args.node == 1 else 1)]
        plan.append(('retract', -plan[0][1]))
    else:
        plan = [('forward', 1), ('reverse', -1)]
    labels = {'extend': '伸出', 'retract': '缩进', 'forward': '正转', 'reverse': '反转'}

    def parent_alive():
        return Path('/proc', str(args.parent_pid)).exists()

    def check_parent():
        if not parent_alive():
            raise TestFailure('主测试脚本已退出，停止本路并清理。')
        if controller and controller.process.poll() is not None:
            raise TestFailure('控制程序已经退出，停止本路。')
        if communications:
            communications.poll()

    try:
        if args.backend == 'can0':
            command = [str(root / 'build/yz_aim_can0'), '--if', args.interface,
                       '--node', str(args.node), '--bitrate', '1000000', '--timeout', '3000']
        else:
            command = [str(root / 'build-native/release/bin/yz_aim_cli'), '--node', str(args.node),
                       '--bitrate', '1000000', '--library', args.library, '--lys', str(args.lys),
                       '--channel', str(args.channel), '--timeout', '3000']
        report(f'{args.backend}节点{args.node}：只读检查，等待两路就绪后开始；不修改零点或节点号。')
        controller = Controller(command, root, run_dir / 'controller.log')
        startup = controller.receive()
        found = re.search(r'通讯详细日志："?([^"\r\n]+)', startup)
        if not found:
            raise TestFailure('未找到通信日志，不启动运动。' + startup[-1500:])
        source = Path(found[1].strip()).resolve()
        if not source.is_relative_to(root / 'logs'):
            raise TestFailure('通信日志不在本项目logs内')
        os.link(source, run_dir / 'can-frames.log')
        result.update(communication_log=str(run_dir / 'can-frames.log'),
                      source_communication_log=str(source))
        communications = LinkCommunications(source, report, args.node, args.backend)
        controller.on_idle = check_parent
        response = controller.command('read-od 1000 4')
        if not re.search(r'= 0x[0-9A-Fa-f]+（', response):
            raise TestFailure('设备类型SDO查询未确认。' + response[-1200:])
        wait_still(controller)
        write_json(run_dir / 'ready.json', {'node': args.node, 'backend': args.backend,
                                         'pid': os.getpid(), 'ready_at': datetime.now().isoformat()})
        deadline = time.monotonic() + 75
        while not args.gate.exists():
            check_parent()
            if time.monotonic() >= deadline:
                raise TestFailure('另一链路未就绪，取消测试。')
            time.sleep(.05)
        check_parent()
        enable_attempted = True
        enabled = controller.command('enable')
        if '[成功] 使能输出' not in enabled:
            raise TestFailure('未完整确认使能。' + enabled[-2000:])
        started = time.monotonic()
        result['motion_started_at'] = datetime.now().isoformat(timespec='milliseconds')
        while time.monotonic() - started < args.minutes * 60:
            for index, (name, sign) in enumerate(plan):
                check_parent()
                report(f'第{cycles + 1}组：' + labels[name] +
                       f' {sign * args.rpm:+d} RPM，{args.move_ms}ms。')
                began = time.monotonic()
                retries = 0
                while True:
                    settled = wait_still(controller)
                    response = controller.command(f'jog {sign * args.rpm} {args.move_ms}',
                                                  timeout=args.move_ms / 1000 + 25, allow_failure=True)
                    if '[成功] 点动' in response and not any(x in response for x in BAD_RESPONSE):
                        break
                    # Only retry a pre-motion stationary-check refusal; never replay an
                    # unconfirmed/started motion, SDO timeout, alarm, or generic failure.
                    if ('motor is moving' not in response or retries >= 3 or abs(settled) > STILL_RPM_LIMIT
                            or '[成功] 点动' in response):
                        raise TestFailure('点动未正常完成：' + response[-2300:])
                    retries += 1
                    report(f'静止门禁拒绝，重新确认静止后再试（{retries}/3）。')
                positions = [int(x) for x in re.findall(r'(-?\d+)\s+脉冲（', response)]
                speeds = [float(x) for x in re.findall(r'转速\s+(-?[\d.]+)\s+RPM', response)]
                action = {'command': name, 'signed_motor_rpm': sign * args.rpm,
                          'requested_ms': args.move_ms, 'command_wall_seconds': round(time.monotonic() - began, 4),
                          'still_retries': retries, 'first_progress_position': positions[0] if positions else None,
                          'last_progress_position': positions[-1] if positions else None,
                          'progress_position_delta': positions[-1] - positions[0] if len(positions) > 1 else None,
                          'measured_speed_min_rpm': min(speeds, default=None),
                          'measured_speed_max_rpm': max(speeds, default=None),
                          'confirmed_hold_seconds': 0}
                actions.append(action)
                # Literal requested cycle: first direction -> confirmed stopped 1s -> second direction.
                # jog itself confirms stopping after the second direction before the next cycle.
                if (args.hold_after == 'both' or index == 0) and args.stop_ms:
                    wait_still(controller)
                    held = time.monotonic()
                    report(f'{labels[name]}已停止，保持{args.stop_ms}ms。')
                    while time.monotonic() - held < args.stop_ms / 1000:
                        check_parent()
                        time.sleep(min(.05, max(0, args.stop_ms / 1000 - (time.monotonic() - held))))
                    action['confirmed_hold_seconds'] = round(time.monotonic() - held, 4)
                write_json(run_dir / 'progress.json', {'cycles': cycles, 'actions': len(actions),
                           'elapsed_seconds': round(time.monotonic() - started, 3), 'latest': action})
            cycles += 1
            report(f'第{cycles}组完成，累计{time.monotonic() - started:.1f}秒。')
        result['outcome'] = 'completed'
    except KeyboardInterrupt:
        result.update(outcome='interrupted', reason='用户中断、连接关闭或另一路失败；停止并清理')
        report(result['reason'])
    except Exception as exc:
        result.update(outcome='failed', reason=str(exc))
        report('异常：停止测试。' + str(exc))
    finally:
        # Further terminal signals must not interrupt the confirmed-stop cleanup.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        if started is not None:
            result['wall_seconds'] = round(time.monotonic() - started, 3)
        if controller:
            controller.on_idle = (lambda: communications.poll()) if communications else (lambda: None)
            disabled = controller.cleanup(report)
            result['controller_returncode'] = controller.process.poll()
        if communications:
            time.sleep(.05)
            result['communication'] = communications.result()
            communications.close()
        after = interface_counters(args.interface) if args.backend == 'can0' else {}
        result.update(disabled_confirmed=disabled, enable_attempted=enable_attempted,
                      completed_cycles=cycles, completed_actions=len(actions),
                      commanded_motion_seconds=len(actions) * args.move_ms / 1000, actions=actions,
                      interface_before=before, interface_after=after,
                      interface_delta={k: after[k] - before[k] for k in before.keys() & after.keys()},
                      ended_at=datetime.now().isoformat(timespec='milliseconds'))
        if result['outcome'] == 'completed' and not disabled:
            result.update(outcome='failed', reason='动作结束但未确认停止并失能')
        write_json(run_dir / 'summary.json', result)
        report(f'结束：{result["outcome"]}，完成{cycles}组、{len(actions)}次动作；'
               + ('已确认停止并失能。' if disabled else '未完整确认失能，请查看controller.log。'))
        report.file.close()
    return 0 if result['outcome'] == 'completed' else 1


if __name__ == '__main__':
    def interrupted(_sig, _frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    raise SystemExit(main())
