#!/usr/bin/env python3
"""USB-CAN双轴等幅速度往返（同一适配器两个节点）；默认只测1分钟，30分钟需显式指定。"""
import argparse
import fcntl
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
sys.dont_write_bytecode = True
from dual_test_support import Controller, Communications, Reporter, TestFailure

FALLBACK_LIBRARY = Path('/root/yz_aim/lib/liblyscan.so')


def atomic(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    os.replace(temp, path)


def settled(controller):
    end = time.monotonic() + 3
    quiet = 0
    while time.monotonic() < end:
        response = controller.command('sync-status', allow_failure=True)
        speeds = [float(x) for x in re.findall(r'转速\s+(-?[\d.]+)\s+电机RPM', response)]
        if '[失败]' in response or len(speeds) != 2:
            raise TestFailure('双轴静止状态读取失败：' + response[-1800:])
        quiet = quiet + 1 if all(abs(x) <= 0.5 for x in speeds) else 0
        if quiet >= 2:
            return
        time.sleep(0.05)
    raise TestFailure('两轴在3秒内未连续确认静止，未发送下一条运动命令')


def read_action(path, trim):
    text = path.read_text(encoding='utf-8')
    metadata = text.splitlines()[0]
    rows = list(csv_rows(text))
    stop = re.search(r'stop_request_a_ms=(-?\d+) stop_request_b_ms=(-?\d+)', text)
    end = re.findall(r'^# end_reason=(.*)$', text, re.M)
    target = re.search(r'nominal_rpm=(-?\d+)', metadata)
    actual_trim = re.search(r'max_trim_rpm=(\d+)', metadata)
    return {'csv': str(path), 'samples': len(rows),
            'nominal_rpm': int(target[1]) if target else None,
            'start_confirmation': 'readback_verified' if '# start_confirmation=readback_verified_after_missing_ack' in text else 'acknowledged',
            'stop_request_a_ms': int(stop[1]) if stop else None,
            'stop_request_b_ms': int(stop[2]) if stop else None,
            'end_reason': end[-1] if end else '',
            'max_trim_rpm': int(actual_trim[1]) if actual_trim else None,
            'trim_matches_request': bool(actual_trim) and int(actual_trim[1]) == trim,
            'maximum_recorded_position_error_deg': max((abs(float(x['error_output_deg'])) for x in rows), default=None)}


def csv_rows(text):
    import csv
    for row in csv.DictReader(line for line in text.splitlines() if not line.startswith('#')):
        yield row


def resolve_library(explicit):
    if explicit:
        path = Path(explicit)
        if not path.is_file():
            raise TestFailure('指定的USB库不存在：' + str(path))
        return path
    local = Path(__file__).resolve().parent / 'lib/liblyscan.so'
    if local.is_file():
        return local
    if FALLBACK_LIBRARY.is_file():
        return FALLBACK_LIBRARY
    raise TestFailure('未找到liblyscan.so；请放入本项目lib/或指定--library')


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rpm', type=int, default=100)
    parser.add_argument('--move-ms', type=int, default=3000)
    parser.add_argument('--pause-ms', type=int, default=1000)
    parser.add_argument('--minutes', type=float, default=1)
    parser.add_argument('--cycles', type=int)
    parser.add_argument('--mode', choices=['equal', 'position'], default='equal')
    parser.add_argument('--trim', type=int)
    parser.add_argument('--tol', type=float)
    parser.add_argument('--node', type=int, default=1)
    parser.add_argument('--node2', type=int, default=2)
    parser.add_argument('--library')
    parser.add_argument('--lys', type=int, default=0)
    parser.add_argument('--channel', type=int, default=0)
    parser.add_argument('--timeout', type=int, default=3000)
    parser.add_argument('--log-dir', type=Path)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if not (1 <= args.rpm <= 300 and 1 <= args.move_ms <= 5000 and 0 <= args.pause_ms <= 60000
            and 0 < args.minutes <= 30 and (args.cycles is None or 1 <= args.cycles <= 1000)):
        parser.error('RPM 1~300，单次1~5000ms，暂停0~60000ms，时长0~30分钟')
    trim = args.trim if args.trim is not None else (0 if args.mode == 'equal' else 5)
    tol = args.tol if args.tol is not None else (0.0 if args.mode == 'equal' else 0.5)
    if not (0 <= trim <= 10 and 0 <= tol <= 100):
        parser.error('修正0~10RPM，偏差阈值0~100度')
    equal_speed = trim == 0 and tol <= 0
    if args.dry_run:
        print(f'USB通道节点{args.node}/{args.node2}：extend {args.rpm} {args.move_ms}，'
              f'retract {args.rpm} {args.move_ms}，每次之间静止{args.pause_ms}ms；'
              f'修正{trim}RPM、阈值{tol}度；'
              + (f'{args.cycles}组' if args.cycles else f'{args.minutes}分钟') + '；位置差只记录。')
        return 0
    directory = args.log_dir or root / 'logs' / ('parallel-30min-' + datetime.now().strftime('%Y%m%d-%H%M%S'))
    if not directory.is_absolute():
        directory = root / 'logs' / directory
    if directory.resolve() == (root / 'logs').resolve() or not directory.resolve().is_relative_to((root / 'logs').resolve()):
        parser.error('日志目录必须在本项目logs下')
    directory.mkdir(parents=True, exist_ok=True)
    if any((directory / n).exists() for n in ['test.log', 'summary.json', 'controller.log', 'can-frames.log']):
        parser.error('目录已有测试结果，请使用新目录')
    runtime = root / '.runtime'
    runtime.mkdir(exist_ok=True)
    lock = (runtime / 'dual-cycle-test.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    library = resolve_library(args.library)
    frame_log = directory / 'can-frames.log'
    report = Reporter(directory / 'test.log')
    controller = comm = None
    started = None
    actions = []
    cycles = 0
    disabled = False
    result = {'started_at': datetime.now().isoformat(timespec='seconds'), 'run_directory': str(directory),
              'program': str(root / 'build/yz_aim_double'),
              'adapter': {'library': str(library), 'device': args.lys, 'channel': args.channel,
                          'bitrate': 1000000, 'label': f'usb{args.lys}/ch{args.channel}'},
              'settings': {'rpm': args.rpm, 'move_ms': args.move_ms, 'pause_ms': args.pause_ms,
                           'minutes': args.minutes, 'cycles': args.cycles,
                           'node_a': args.node, 'node_b': args.node2,
                           'max_trim_rpm': trim, 'sync_tolerance_deg': tol,
                           'mode': 'equal_speed' if equal_speed else 'position_sync_corrected'},
              'outcome': 'not_started', 'reason': ''}
    motion_dir = root / 'logs'
    try:
        report('USB-CAN双轴等速测试：同一适配器两个节点；节点%d负/节点%d正为伸出；位置差只记录。'
               % (args.node, args.node2))
        if equal_speed:
            report('模式：不修正、不因位置差停机（对应CAN0版sync-mode speed的等效设置）。')
        else:
            report(f'模式：位置同步，最大减速修正{trim}RPM，累计偏差阈值{tol}度。')
        command = [str(root / 'build/yz_aim_double'), '--library', str(library),
                   '--lys', str(args.lys), '--channel', str(args.channel), '--bitrate', '1000000',
                   '--node', str(args.node), '--node2', str(args.node2), '--timeout', str(args.timeout)]
        controller = Controller(command, root, directory / 'controller.log', frame_log)
        startup = controller.receive()
        if f'A节点 {args.node}，B节点 {args.node2}' not in startup or f'usb{args.lys}/ch{args.channel}' not in startup:
            raise TestFailure('控制程序未按双轴USB模式启动：' + startup[-1800:])
        comm = Communications(frame_log, report)
        controller.on_idle = comm.poll
        comm.attach_process(controller.process)
        response = controller.command(f'sync-dir {args.node2} {args.node}')
        if '[失败]' in response:
            raise TestFailure('镜像方向设置失败：' + response[-1800:])
        response = controller.command(f'sync-correct {trim}')
        if '[失败]' in response:
            raise TestFailure('修正幅度设置失败：' + response[-1800:])
        response = controller.command(f'sync-tol {tol:g}')
        if '[失败]' in response:
            raise TestFailure('偏差阈值设置失败：' + response[-1800:])
        settled(controller)
        comm.poll(live=False)
        if comm.frames() == 0:
            raise TestFailure('未从USB适配器捕获到任何CAN帧，拒绝运动')
        response = controller.command('sync-enable')
        if '两轴已使能' not in response:
            raise TestFailure('双轴使能未完整确认：' + response[-1800:])
        settled(controller)
        zero = controller.command('sync-zero')
        if '固定软件零点' not in zero:
            raise TestFailure('未确认日志基准：' + zero[-1800:])
        result['software_reference'] = zero
        started = time.monotonic()
        while (cycles < args.cycles if args.cycles is not None else time.monotonic() - started < args.minutes * 60):
            for name in ['extend', 'retract']:
                settled(controller)
                retries = 0
                began = time.monotonic()
                while True:
                    before_csv = set(motion_dir.glob('motion-*.csv'))
                    report(f'第{cycles + 1}组：{name} {args.rpm} {args.move_ms}，两轴一起动作。')
                    response = controller.command(f'{name} {args.rpm} {args.move_ms}',
                                                  timeout=args.move_ms / 1000 + 30, allow_failure=True)
                    if '到时正常停止。' in response:
                        break
                    # 只重试"运动前被判定未静止"这一种拒绝，已经动过的动作绝不重放。
                    if 'motor is moving' in response and '名义电机RPM' not in response and retries < 3:
                        retries += 1
                        report(f'运动前静止检查重试{retries}/3，先等待两轴静止。')
                        settled(controller)
                        time.sleep(.1)
                        continue
                    raise TestFailure('双轴指令未正常完成：' + response[-2800:])
                new_csv = set(motion_dir.glob('motion-*.csv')) - before_csv
                if len(new_csv) != 1:
                    raise TestFailure('未取得唯一完整动作CSV')
                source = new_csv.pop()
                target = directory / source.name
                shutil.move(str(source), str(target))
                item = read_action(target, trim)
                item.update(command=name, wall_seconds=round(time.monotonic() - began, 3), still_retries=retries)
                if not item['trim_matches_request']:
                    raise TestFailure('动作CSV记录的修正幅度与设置不符：' + str(item['max_trim_rpm']))
                if item['end_reason'] != 'requested duration elapsed' or '两轴已确认停止' not in response:
                    raise TestFailure('动作结束原因或双轴停稳未完整确认：' + response[-2500:])
                actions.append(item)
                atomic(directory / 'progress.json', {'cycles': cycles, 'actions': len(actions),
                                                     'elapsed_seconds': round(time.monotonic() - started, 3),
                                                     'latest': item})
                # 每两次动作之间保证至少 pause_ms 的静止时间。
                stop_at = time.monotonic()
                settled(controller)
                remain = args.pause_ms / 1000 - (time.monotonic() - stop_at)
                if remain > 0:
                    time.sleep(remain)
            cycles += 1
            report(f'第{cycles}组完成，累计{time.monotonic() - started:.1f}秒。')
        result['outcome'] = 'completed'
    except KeyboardInterrupt:
        result.update(outcome='interrupted', reason='收到中断，停止测试并清理')
        report(result['reason'])
    except Exception as exc:
        result.update(outcome='failed', reason=str(exc))
        report('异常：停止测试，不自动重放运动命令。' + str(exc))
    finally:
        if started is not None:
            result['wall_seconds'] = round(time.monotonic() - started, 3)
        if controller:
            disabled = controller.cleanup(report)
            result['controller_returncode'] = controller.process.poll()
        if comm:
            time.sleep(.1)
            result['communication'] = comm.result()
            comm.close()
        result.update(completed_cycles=cycles, completed_actions=len(actions), actions=actions,
                      disabled_confirmed=disabled)
        result['start_readback_verified_actions'] = sum(
            x.get('start_confirmation') == 'readback_verified' for x in actions)
        result['start_confirmation_failures'] = int('速度触发未完整确认' in result.get('reason', ''))
        if result['outcome'] == 'completed' and not disabled:
            result.update(outcome='failed', reason='循环完成但未确认双轴失能')
        result['ended_at'] = datetime.now().isoformat(timespec='seconds')
        atomic(directory / 'summary.json', result)
        report(f"测试结束：{result['outcome']}，{cycles}组/{len(actions)}次动作，双轴失能确认={disabled}。")
        if frame_log.exists() and 'communication' in result:
            p = subprocess.run([sys.executable, '-B', str(root / 'dual_test_report.py'), str(directory)],
                               capture_output=True, text=True, timeout=120)
            (directory / 'report-generation.log').write_text(p.stdout + p.stderr, encoding='utf-8')
            report('中文报告' + ('生成成功。' if p.returncode == 0 else '生成失败，请查看report-generation.log。'))
        report('结果：' + str(directory / 'summary.json'))
        report.file.close()
    return 0 if result['outcome'] == 'completed' else 1


if __name__ == '__main__':
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    raise SystemExit(main())
