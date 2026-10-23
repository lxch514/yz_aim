#!/usr/bin/env python3
"""CAN0、USB-CAN两路单轴并行往返测试；默认30分钟，不改节点号和零点。"""
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
ROOTS = {'can0': Path('/root/yz_aim_can0'), 'usb': Path('/root/yz_aim')}


def write_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    os.replace(temporary, path)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can0-node', type=int, default=2, help='CAN0上的电机节点，默认2')
    parser.add_argument('--usb-node', type=int, default=1, help='USB-CAN上的电机节点，默认1')
    parser.add_argument('--minutes', type=float, default=30, help='测试时长，默认30分钟；完成最后一组后结束')
    parser.add_argument('--rpm', type=int, default=300, help='电机RPM绝对值，默认300')
    parser.add_argument('--move-ms', type=int, default=3000, help='每次正/反转持续毫秒，默认3000')
    parser.add_argument('--stop-ms', type=int, default=1000, help='正转确认停止后的保持毫秒，默认1000')
    parser.add_argument('--if', dest='interface', default='can0')
    parser.add_argument('--library', default='/root/yz_aim/lib/liblyscan.so')
    parser.add_argument('--lys', type=int, default=0, help='USB设备序号，默认0')
    parser.add_argument('--channel', type=int, default=0, help='USB通道序号，默认0')
    parser.add_argument('--dry-run', action='store_true', help='只显示计划，不开CAN、不创建日志、不运动')
    args = parser.parse_args(argv)
    if not (1 <= args.can0_node <= 126 and 1 <= args.usb_node <= 126):
        parser.error('两个节点号均需为1~126；这只选择寻址，不修改电机本身节点号')
    if not (math.isfinite(args.minutes) and 0 < args.minutes <= 60 and 1 <= args.rpm <= 300
            and 1 <= args.move_ms <= 5000 and 0 <= args.stop_ms <= 60000):
        parser.error('时长0~60分钟、转速1~300RPM、单次1~5000ms、停止保持0~60000ms')
    if args.lys < 0 or args.channel < 0 or not args.interface or '/' in args.interface:
        parser.error('设备、通道或CAN接口参数无效')
    return args


def existing_controller(root):
    binaries = [(root / 'build/yz_aim_can0').resolve(),
                (root / 'build-native/release/bin/yz_aim_cli').resolve()]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            executable = Path(os.readlink(entry / 'exe').removesuffix(' (deleted)')).resolve()
            if executable in binaries:
                return int(entry.name)
        except OSError:
            pass
    return None


def main(argv=None):
    args = parse_args(argv)
    print(f'计划：CAN0节点{args.can0_node}，USB-CAN节点{args.usb_node}，两路并行。\n'
          f'每路：正转+{args.rpm}RPM {args.move_ms}ms → 确认停止并保持{args.stop_ms}ms → '
          f'反转-{args.rpm}RPM {args.move_ms}ms → 确认停止 → 下一组。\n'
          f'持续约{args.minutes:g}分钟，最后一组完整结束；不修改零点/电机节点号。', flush=True)
    if args.dry_run:
        print('仅显示计划；未打开CAN、未使能、未启动测试。')
        return 0
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    state = {'started_at': datetime.now().isoformat(timespec='milliseconds'),
             'stamp': stamp, 'coordinator_pid': os.getpid(), 'settings': vars(args),
             'phase': 'preparing', 'jobs': {}}
    jobs = {}
    locks = []
    gate = None

    def save_state():
        for job in jobs.values():
            write_json(job['directory'] / 'coordinator.json', state)

    def stop_workers():
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        for job in jobs.values():
            process = job.get('process')
            if process and process.poll() is None:
                process.send_signal(signal.SIGTERM)
        deadline = time.monotonic() + 110
        while any(j.get('process') and j['process'].poll() is None for j in jobs.values()):
            if time.monotonic() >= deadline:
                for name, job in jobs.items():
                    if job.get('process') and job['process'].poll() is None:
                        state['jobs'][name]['cleanup_unconfirmed'] = True
                print('异常：仍有进程未确认清理，请查看controller.log并检查现场停止状态。', flush=True)
                break
            time.sleep(.2)

    try:
        for name, root in ROOTS.items():
            if not root.is_dir():
                raise RuntimeError('项目目录不存在：' + str(root))
            for required in ['parallel_cycle_worker.py', 'single_cycle_test.py', 'can_monitor.py',
                             'parallel_cycle_report.py', 'parallel_trace_analysis.py']:
                if not (root / required).is_file():
                    raise RuntimeError('缺少脚本：' + str(root / required))
            binary = root / ('build/yz_aim_can0' if name == 'can0' else 'build-native/release/bin/yz_aim_cli')
            if not binary.is_file() or not os.access(binary, os.X_OK):
                raise RuntimeError('控制程序不可执行：' + str(binary))
            if shutil.disk_usage(root).free < 512 * 1024 * 1024:
                raise RuntimeError('剩余空间不足512MB，请先整理历史日志：' + str(root))
            runtime = root / '.runtime'
            runtime.mkdir(exist_ok=True)
            lock = (runtime / 'parallel-cycle.lock').open('a+')
            locks.append(lock)
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('已有并行测试正在使用：' + str(root))
            pid = existing_controller(root)
            if pid:
                raise RuntimeError(f'{root}已有控制进程PID={pid}，请正常退出旧会话再运行测试')
        if not Path(args.library).is_file():
            raise RuntimeError('USB库不存在：' + args.library)
        for name, root in ROOTS.items():
            directory = root / 'logs' / ('parallel-30min-' + stamp)
            directory.mkdir(parents=True, exist_ok=False)
            jobs[name] = {'root': root, 'directory': directory}
            node = args.can0_node if name == 'can0' else args.usb_node
            state['jobs'][name] = {'node': node, 'directory': str(directory),
                                  'launcher_log': str(directory / 'launcher.log')}
            print(f'{name}日志：{directory}', flush=True)
        gate = jobs['can0']['directory'] / 'start.gate'
        save_state()
        for name, job in jobs.items():
            command = [sys.executable, '-B', str(job['root'] / 'parallel_cycle_worker.py'),
                       '--backend', name, '--node', str(state['jobs'][name]['node']),
                       '--rpm', str(args.rpm), '--move-ms', str(args.move_ms), '--stop-ms', str(args.stop_ms),
                       '--minutes', str(args.minutes), '--log-dir', str(job['directory']),
                       '--gate', str(gate), '--parent-pid', str(os.getpid()), '--if', args.interface,
                       '--library', args.library, '--lys', str(args.lys), '--channel', str(args.channel)]
            job['output'] = (job['directory'] / 'launcher.log').open('wb', buffering=0)
            job['process'] = subprocess.Popen(command, cwd=job['root'], stdin=subprocess.DEVNULL,
                                             stdout=job['output'], stderr=subprocess.STDOUT,
                                             start_new_session=True)
            state['jobs'][name].update(pid=job['process'].pid, command=command)
        state['phase'] = 'checking_both_links'
        save_state()
        deadline = time.monotonic() + 80
        while not all((j['directory'] / 'ready.json').is_file() for j in jobs.values()):
            for name, job in jobs.items():
                if job['process'].poll() is not None:
                    raise RuntimeError(f'{name}只读检查失败；两路不开始运动。查看{job["directory"]}/test.log')
            if time.monotonic() >= deadline:
                raise TimeoutError('两路只读检查未在80秒内完成，不开始运动')
            time.sleep(.1)
        state['phase'] = 'running'
        state['released_at'] = datetime.now().isoformat(timespec='milliseconds')
        save_state()
        gate.write_text(state['released_at'] + '\n', encoding='utf-8')
        print('两路只读查询通过，已同时放行；每分钟显示进度，详细CAN帧写入各自can-frames.log。Ctrl+C停止两路。', flush=True)
        started = time.monotonic()
        next_update = started
        while True:
            active = False
            for name, job in jobs.items():
                code = job['process'].poll()
                state['jobs'][name]['returncode'] = code
                if code is None:
                    active = True
                elif code != 0:
                    raise RuntimeError(f'{name}异常结束，正在停止另一路。详见{job["directory"]}/summary.json')
                progress = job['directory'] / 'progress.json'
                if progress.is_file():
                    state['jobs'][name]['progress'] = json.loads(progress.read_text(encoding='utf-8'))
            state['elapsed_seconds'] = round(time.monotonic() - started, 3)
            if time.monotonic() >= next_update or not active:
                save_state()
                details = []
                for name, job in jobs.items():
                    progress = state['jobs'][name].get('progress', {})
                    details.append(f'{name}节点{state["jobs"][name]["node"]}：'
                                   f'{progress.get("cycles", 0)}组/{progress.get("actions", 0)}次动作')
                    # Communication summaries/errors are already emitted in Chinese by
                    # each worker and flushed into test.log, without terminal frame spam.
                print(f'已运行{state["elapsed_seconds"]:.1f}秒；' + '；'.join(details), flush=True)
                next_update = time.monotonic() + 60
            if not active:
                break
            # Finishing a pair and confirmed-stop cleanup may exceed the nominal duration.
            allowance = max(180, (2 * args.move_ms + args.stop_ms) / 1000 + 150)
            if time.monotonic() - started > args.minutes * 60 + allowance:
                raise TimeoutError('超过计划时间和清理余量，停止两路')
            time.sleep(.2)
        state['phase'] = 'completed'
    except KeyboardInterrupt:
        state.update(phase='interrupted', reason='用户中断或终端连接关闭')
        print('收到中断，正在停止并失能两路电机，请等待总结生成。', flush=True)
        stop_workers()
    except Exception as exc:
        state.update(phase='failed', reason=str(exc))
        print('异常：' + str(exc), flush=True)
        stop_workers()
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        if gate and gate.exists():
            gate.unlink()
        for name, job in jobs.items():
            process = job.get('process')
            if process:
                state['jobs'][name]['returncode'] = process.poll()
            if job.get('output'):
                job['output'].close()
            summary = job['directory'] / 'summary.json'
            if summary.is_file():
                result = json.loads(summary.read_text(encoding='utf-8'))
                state['jobs'][name].update(outcome=result['outcome'],
                                          disabled_confirmed=result['disabled_confirmed'])
        state['ended_at'] = datetime.now().isoformat(timespec='milliseconds')
        if jobs:
            save_state()
            try:
                from parallel_cycle_report import generate_reports
                reports = generate_reports(state)
                for report in reports:
                    print('中文总结：' + str(report), flush=True)
            except Exception as exc:
                state['report_error'] = str(exc)
                if state['phase'] == 'completed':
                    state['phase'] = 'report_failed'
                save_state()
                print('异常：总结生成失败，原始日志仍已保留：' + str(exc), flush=True)
        for lock in locks:
            lock.close()
        print('本次测试结束：' + state['phase'], flush=True)
    return 0 if state['phase'] == 'completed' else 1


if __name__ == '__main__':
    def interrupted(_sig, _frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    raise SystemExit(main())
