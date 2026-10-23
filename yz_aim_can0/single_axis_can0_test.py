#!/usr/bin/env python3
"""CAN0 单轴往返测试：两个电机都接在板载 CAN0 总线上，只驱动其中一个节点。

与 parallel_cycle_test.py 的区别：只有一路，不启动 USB 那一路，也不要求两路同时就绪；
日志目录、文件种类、summary/analysis 字段与并行测试保持一致，便于横向对比。
日志写入 /root/yz_aim/logs，与 USB 单轴测试的产物并列。
"""
import argparse
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent
SHARED_LOGS = Path('/root/yz_aim/logs')
BACKEND = 'can0'


def write_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    os.replace(temporary, path)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node', type=int, default=1, help='CAN0上要驱动的电机节点，默认1')
    parser.add_argument('--interface', default='can0', help='CAN 接口名，默认can0')
    parser.add_argument('--minutes', type=float, default=30, help='测试时长，默认30分钟；完成最后一组后结束')
    parser.add_argument('--rpm', type=int, default=100, help='电机RPM绝对值，默认100')
    parser.add_argument('--move-ms', type=int, default=3000, help='每次动作持续毫秒，默认3000')
    parser.add_argument('--stop-ms', type=int, default=1000, help='每次动作确认停止后的保持毫秒，默认1000')
    parser.add_argument('--sequence', choices=['mirror', 'forward-reverse'], default='mirror',
                        help='mirror：先伸出后缩进（节点1负/节点2正），默认；forward-reverse：先正转后反转')
    parser.add_argument('--hold-after', choices=['both', 'first'], default='both',
                        help='both：伸/缩两个方向后都保持，默认；first：只在第一个动作后保持')
    parser.add_argument('--dry-run', action='store_true', help='只显示计划，不开CAN、不创建日志、不运动')
    args = parser.parse_args(argv)
    if not (1 <= args.node <= 126):
        parser.error('节点号需为1~126；这只选择寻址，不修改电机本身节点号')
    if not (math.isfinite(args.minutes) and 0 < args.minutes <= 60 and 1 <= args.rpm <= 300
            and 1 <= args.move_ms <= 5000 and 0 <= args.stop_ms <= 60000):
        parser.error('时长0~60分钟、转速1~300RPM、单次1~5000ms、停止保持0~60000ms')
    if not args.interface or '/' in args.interface:
        parser.error('CAN 接口名无效')
    return args


def other_controllers():
    """列出正占用本机 CAN/USB 控制程序的其他进程，避免同一个电机被另一条链路驱动。"""
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            executable = os.readlink(entry / 'exe').removesuffix(' (deleted)')
        except OSError:
            continue
        if Path(executable).name.startswith('yz_aim'):
            found.append((int(entry.name), executable))
    return found


def main(argv=None):
    args = parse_args(argv)
    first = (-1 if args.node == 1 else 1) if args.sequence == 'mirror' else 1
    first_name = '伸出' if args.sequence == 'mirror' else '正转'
    back_name = '缩进' if args.sequence == 'mirror' else '反转'
    hold_text = '每次动作' if args.hold_after == 'both' else '第一个方向'
    print(f'计划：两个电机都接在板载{args.interface}总线上，只驱动节点{args.node}。\n'
          f'循环：{first_name} {first * args.rpm:+d} RPM {args.move_ms}ms → 确认停止并保持{args.stop_ms}ms → '
          f'{back_name} {-first * args.rpm:+d} RPM {args.move_ms}ms → 确认停止'
          + (f'并保持{args.stop_ms}ms' if args.hold_after == 'both' else '（第二方向后不另加保持）')
          + ' → 下一组。\n'
          f'持续约{args.minutes:g}分钟，最后一组完整结束；不修改零点/电机节点号；'
          f'{hold_text}后都做静止确认。', flush=True)
    if args.dry_run:
        print('仅显示计划；未打开CAN、未使能、未启动测试。')
        return 0

    for required in ['parallel_cycle_worker.py', 'single_cycle_test.py', 'can_monitor.py',
                     'single_axis_can0_report.py', 'parallel_trace_analysis.py',
                     'parallel_cycle_report.py']:
        if not (ROOT / required).is_file():
            print('缺少脚本：' + str(ROOT / required), flush=True)
            return 1
    binary = ROOT / 'build/yz_aim_can0'
    if not binary.is_file() or not os.access(binary, os.X_OK):
        print('控制程序不可执行：' + str(binary), flush=True)
        return 1
    if not Path('/sys/class/net', args.interface).exists():
        print('CAN 接口不存在：' + args.interface, flush=True)
        return 1
    try:
        SHARED_LOGS.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        print('无法创建日志根目录：' + str(exc), flush=True)
        return 1
    if shutil.disk_usage(SHARED_LOGS).free < 512 * 1024 * 1024:
        print('剩余空间不足512MB，请先整理历史日志：' + str(SHARED_LOGS), flush=True)
        return 1
    busy = other_controllers()
    if busy:
        print('已有控制程序在运行，会同时驱动同一对电机，请先正常退出：',
              ', '.join(f'PID={pid} {name}' for pid, name in busy), flush=True)
        return 1
    runtime = ROOT / '.runtime'
    runtime.mkdir(exist_ok=True)
    lock = (runtime / 'single-axis-cycle.lock').open('a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        print('已有单轴测试正在运行：' + str(runtime / 'single-axis-cycle.lock'), flush=True)
        return 1

    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    try:
        directory = SHARED_LOGS / ('single-30min-' + stamp)
        directory.mkdir(parents=True, exist_ok=False)
    except OSError as exc:
        lock.close()
        print('无法创建日志目录：' + str(exc), flush=True)
        return 1
    print(f'日志目录：{directory}', flush=True)

    gate = directory / 'start.gate'
    state = {'started_at': datetime.now().isoformat(timespec='milliseconds'),
             'stamp': stamp, 'coordinator_pid': os.getpid(), 'settings': vars(args),
             'phase': 'preparing', 'jobs': {}}
    state['jobs'][BACKEND] = {'node': args.node, 'directory': str(directory),
                              'launcher_log': str(directory / 'launcher.log')}
    write_json(directory / 'coordinator.json', state)
    process = None
    output = None
    try:
        command = [sys.executable, '-B', str(ROOT / 'parallel_cycle_worker.py'),
                   '--backend', BACKEND, '--node', str(args.node),
                   '--rpm', str(args.rpm), '--move-ms', str(args.move_ms), '--stop-ms', str(args.stop_ms),
                   '--minutes', str(args.minutes), '--log-dir', str(directory),
                   '--gate', str(gate), '--parent-pid', str(os.getpid()),
                   '--sequence', args.sequence, '--hold-after', args.hold_after,
                   '--if', args.interface]
        output = (directory / 'launcher.log').open('wb', buffering=0)
        process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                   stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        state['jobs'][BACKEND].update(pid=process.pid, command=command)
        state['phase'] = 'checking_link'
        write_json(directory / 'coordinator.json', state)
        deadline = time.monotonic() + 80
        while not (directory / 'ready.json').is_file():
            if process.poll() is not None:
                raise RuntimeError('只读检查失败，未开始运动；查看' + str(directory / 'test.log'))
            if time.monotonic() >= deadline:
                raise TimeoutError('只读检查未在80秒内完成，不开始运动')
            time.sleep(.1)
        state['phase'] = 'running'
        state['released_at'] = datetime.now().isoformat(timespec='milliseconds')
        write_json(directory / 'coordinator.json', state)
        gate.write_text(state['released_at'] + '\n', encoding='utf-8')
        print('只读查询通过，已放行；每分钟显示进度，详细CAN帧写入can-frames.log。Ctrl+C停止。', flush=True)
        started = time.monotonic()
        next_update = started
        while True:
            code = process.poll()
            state['jobs'][BACKEND]['returncode'] = code
            progress = directory / 'progress.json'
            if progress.is_file():
                try:
                    state['jobs'][BACKEND]['progress'] = json.loads(progress.read_text(encoding='utf-8'))
                except (OSError, ValueError):
                    pass
            state['elapsed_seconds'] = round(time.monotonic() - started, 3)
            if time.monotonic() >= next_update or code is not None:
                write_json(directory / 'coordinator.json', state)
                item = state['jobs'][BACKEND].get('progress', {})
                print(f'已运行{state["elapsed_seconds"]:.1f}秒；节点{args.node}：'
                      f'{item.get("cycles", 0)}组/{item.get("actions", 0)}次动作', flush=True)
                next_update = time.monotonic() + 60
            if code is not None:
                if code != 0:
                    raise RuntimeError('本次测试异常结束，详见' + str(directory / 'summary.json'))
                break
            allowance = max(180, (2 * args.move_ms + 2 * args.stop_ms) / 1000 + 150)
            if time.monotonic() - started > args.minutes * 60 + allowance:
                raise TimeoutError('超过计划时间和清理余量，停止测试')
            time.sleep(.2)
        state['phase'] = 'completed'
    except KeyboardInterrupt:
        state.update(phase='interrupted', reason='用户中断或终端连接关闭')
        print('收到中断，正在停止并失能电机，请等待总结生成。', flush=True)
        if process and process.poll() is None:
            process.send_signal(signal.SIGTERM)
    except Exception as exc:
        state.update(phase='failed', reason=str(exc))
        print('异常：' + str(exc), flush=True)
        if process and process.poll() is None:
            process.send_signal(signal.SIGTERM)
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        if gate.exists():
            gate.unlink()
        if process:
            try:
                process.wait(timeout=110)
            except subprocess.TimeoutExpired:
                state['jobs'][BACKEND]['cleanup_unconfirmed'] = True
                print('异常：工作进程未确认清理，请检查现场停止状态和controller.log。', flush=True)
            state['jobs'][BACKEND]['returncode'] = process.poll()
        if output:
            output.close()
        summary = directory / 'summary.json'
        if summary.is_file():
            try:
                result = json.loads(summary.read_text(encoding='utf-8'))
                state['jobs'][BACKEND].update(outcome=result.get('outcome'),
                                              disabled_confirmed=result.get('disabled_confirmed'))
            except (OSError, ValueError):
                pass
        state['ended_at'] = datetime.now().isoformat(timespec='milliseconds')
        write_json(directory / 'coordinator.json', state)
        try:
            from single_axis_can0_report import generate_reports
            for report in generate_reports(json.loads((directory / 'coordinator.json').read_text(encoding='utf-8'))):
                print('中文总结：' + str(report), flush=True)
        except Exception as exc:
            state['report_error'] = str(exc)
            if state['phase'] == 'completed':
                state['phase'] = 'report_failed'
            write_json(directory / 'coordinator.json', state)
            print('异常：总结生成失败，原始日志仍已保留：' + str(exc), flush=True)
        print('本次测试结束：' + state['phase'], flush=True)
        lock.close()
    return 0 if state['phase'] == 'completed' else 1


if __name__ == '__main__':
    def interrupted(_sig, _frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    raise SystemExit(main())
