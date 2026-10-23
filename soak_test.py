#!/usr/bin/env python3
"""长时老化测试：按轮次串行调用单轴测试脚本，每轮独立目录、独立报告、独立压缩。"""

import argparse
from datetime import datetime
import gzip
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent
LOGS = ROOT / 'logs'
MIN_FREE_BYTES = 1200 * 1024 * 1024
FRAME_RE = re.compile(r'\+([0-9.]+)s\s+#\d+\]\s+(TX|RX)\s+ID=([0-9A-F]+)\s+DLC=\d+\s+([0-9A-F ]+)')


def parse_args(argv=None):
    """解析老化测试参数。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node', type=int, default=2, help='要驱动的节点，默认2')
    parser.add_argument('--rounds', type=int, default=14, help='轮数，默认14（每轮60分钟约14小时）')
    parser.add_argument('--minutes', type=float, default=60, help='每轮时长分钟，1~60，默认60')
    parser.add_argument('--rpm', type=int, default=100, help='电机RPM绝对值，默认100')
    parser.add_argument('--move-ms', type=int, default=3000, help='每次动作毫秒，默认3000')
    parser.add_argument('--stop-ms', type=int, default=1000, help='每次动作后保持毫秒，默认1000')
    parser.add_argument('--sequence', choices=['mirror', 'forward-reverse'], default='mirror')
    parser.add_argument('--hold-after', choices=['both', 'forward'], default='both')
    parser.add_argument('--keep-frames', action='store_true', help='保留本轮帧日志原文，不压缩')
    parser.add_argument('--max-consecutive-failures', type=int, default=2,
                        help='连续失败多少轮后中止，默认2')
    args = parser.parse_args(argv)
    if not (1 <= args.node <= 126 and 1 <= args.rpm <= 300 and 1 <= args.move_ms <= 5000
            and 0 <= args.stop_ms <= 60000 and 0 < args.minutes <= 60):
        parser.error('节点1~126、转速1~300RPM、单次1~5000ms、保持0~60000ms、每轮0~60分钟')
    if args.rounds < 1:
        parser.error('轮数至少1')
    return args


def free_bytes():
    """返回日志分区剩余字节。"""
    return shutil.disk_usage(LOGS).free


def round_directories():
    """列出所有单轴轮次目录。"""
    return {p for p in LOGS.glob('single-*min-*') if p.is_dir()}


def temperature_stats(run_dir):
    """从某一轮的帧日志里统计 0x2612 温度应答。"""
    frames = run_dir / 'can-frames.log'
    packed = run_dir / 'can-frames.log.gz'
    if frames.is_file():
        handle = frames.open('r', errors='replace')
    elif packed.is_file():
        handle = gzip.open(packed, 'rt', errors='replace')
    else:
        return None
    pending = False
    values = []
    with handle:
        for line in handle:
            found = FRAME_RE.search(line)
            if not found:
                continue
            direction = found.group(2)
            payload = found.group(4).split()
            if len(payload) < 4 or payload[1:4] != ['12', '26', '00']:
                continue
            if direction == 'TX':
                pending = True
            elif pending and len(payload) >= 7:
                values.append(int(payload[4], 16) | (int(payload[5], 16) << 8))
                pending = False
    if not values:
        return None
    return {'first_c': values[0], 'max_c': max(values), 'last_c': values[-1],
            'samples': len(values)}


def compress_frames(run_dir):
    """压缩某一轮的帧日志并删掉硬链接源，节省磁盘。"""
    frames = run_dir / 'can-frames.log'
    if not frames.is_file():
        return
    source = None
    summary = run_dir / 'summary.json'
    if summary.is_file():
        try:
            source = json.loads(summary.read_text(encoding='utf-8')).get('source_communication_log')
        except (OSError, ValueError):
            source = None
    inode = frames.stat().st_ino
    target = run_dir / 'can-frames.log.gz'
    with frames.open('rb') as raw, gzip.open(target, 'wb', compresslevel=6) as packed:
        shutil.copyfileobj(raw, packed, 1 << 20)
    frames.unlink()
    if source:
        source_path = Path(source)
        try:
            if source_path.is_file() and source_path.stat().st_ino == inode:
                source_path.unlink()
        except OSError:
            pass


def write_index(soak_dir, args, records):
    """写出老化测试汇总表。"""
    lines = ['# YZ-AIM 长时老化测试汇总', '',
             f'- 起始：{records[0]["round_started_at"] if records else "-"}',
             f'- 节点：{args.node}　每轮：{args.minutes:g} 分钟　计划轮数：{args.rounds}'
             f'　动作：{args.rpm} RPM / {args.move_ms} ms',
             f'- 已完成轮数：{len(records)}', '',
             '| 轮 | 目录 | 结果 | 时长(s) | 组 | 动作 | SDO请求/应答 | 读重发 | 未确认超时'
             ' | 心跳超时 | 错误帧 | 温度首/最高 | 失能确认 |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for item in records:
        comm = item.get('communication') or {}
        warmth = item.get('temperature') or {}
        lines.append(
            '| {} | {} | {} | {} | {} | {} | {}/{} | {} | {} | {} | {} | {} | {} |'.format(
                item['round'], item['directory'], item.get('outcome', '-'),
                item.get('wall_seconds', '-'), item.get('completed_cycles', '-'),
                item.get('completed_actions', '-'),
                sum((comm.get('sdo_requests_by_node') or {}).values()),
                sum((comm.get('sdo_responses_by_node') or {}).values()),
                comm.get('sdo_read_repeats_observed', '-'),
                comm.get('sdo_unconfirmed_timeouts', '-'),
                comm.get('heartbeat_timeouts', '-'),
                comm.get('can_error_frames', '-'),
                '{}/{}'.format(warmth.get('first_c', '-'), warmth.get('max_c', '-')),
                item.get('disabled_confirmed', '-')))
    lines += ['', '## 逐轮备注', '']
    for item in records:
        if item.get('reason'):
            lines.append(f'- 第{item["round"]}轮：{item["reason"]}')
    (soak_dir / 'index.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(argv=None):
    """串行执行各轮单轴测试并汇总。"""
    args = parse_args(argv)
    entry = ROOT / 'single_axis_cycle_test.py'
    if not entry.is_file():
        print('缺少脚本：' + str(entry), flush=True)
        return 1
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    soak_dir = LOGS / ('soak-' + stamp)
    soak_dir.mkdir(parents=True, exist_ok=True)
    console = (soak_dir / 'soak.log').open('a', buffering=1, encoding='utf-8')
    records = []
    consecutive_failures = 0

    def say(text):
        line = f'[{datetime.now().strftime("%H:%M:%S")}] {text}'
        print(line, flush=True)
        console.write(line + '\n')

    say(f'老化测试开始：节点{args.node}，每轮{args.minutes:g}分钟，共{args.rounds}轮，'
        f'预计{args.minutes * args.rounds / 60:.1f}小时。')
    say(f'汇总目录：{soak_dir}')
    for number in range(1, args.rounds + 1):
        remaining = free_bytes()
        say(f'第{number}轮开始前剩余空间 {remaining / 1048576:.0f} MB。')
        if remaining < MIN_FREE_BYTES:
            say('剩余空间低于1200MB，停止后续轮次。')
            break
        before = round_directories()
        command = [sys.executable, '-B', str(entry), '--node', str(args.node),
                   '--minutes', str(args.minutes), '--rpm', str(args.rpm),
                   '--move-ms', str(args.move_ms), '--stop-ms', str(args.stop_ms),
                   '--sequence', args.sequence, '--hold-after', args.hold_after]
        started_at = datetime.now().isoformat(timespec='seconds')
        process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        output_lines = []
        try:
            for raw in process.stdout:
                text = raw.decode('utf-8', 'replace').rstrip('\n')
                output_lines.append(text)
                console.write(text + '\n')
            code = process.wait()
        except KeyboardInterrupt:
            say('收到中断，停止本轮并清理。')
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                try:
                    for raw in process.stdout:
                        console.write(raw.decode('utf-8', 'replace'))
                except Exception:
                    pass
            process.wait(timeout=180)
            break
        created = round_directories() - before
        record = {'round': number, 'round_started_at': started_at, 'returncode': code,
                  'directory': str(sorted(created)[-1]) if created else '-'}
        if created:
            run_dir = sorted(created)[-1]
            summary = run_dir / 'summary.json'
            if summary.is_file():
                try:
                    loaded = json.loads(summary.read_text(encoding='utf-8'))
                    record.update(loaded)
                except (OSError, ValueError):
                    pass
            warmth = temperature_stats(run_dir)
            if warmth:
                record['temperature'] = warmth
            if not args.keep_frames:
                compress_frames(run_dir)
        else:
            record['reason'] = '本轮未生成日志目录：' + ' | '.join(output_lines[-3:])
        records.append(record)
        write_index(soak_dir, args, records)
        outcome = record.get('outcome', 'failed')
        warmth = record.get('temperature') or {}
        say(f'第{number}轮结束：{outcome}，{record.get("completed_cycles", 0)}组/'
            f'{record.get("completed_actions", 0)}次动作，温度{warmth.get("first_c", "-")}→'
            f'{warmth.get("max_c", "-")}℃，失能确认{record.get("disabled_confirmed")}。')
        if outcome == 'completed':
            consecutive_failures = 0
        else:
            consecutive_failures += 1
            if record.get('reason') is None and (run_dir := Path(record['directory'])).is_dir():
                launcher = run_dir / 'launcher.log'
                if launcher.is_file():
                    record['reason'] = launcher.read_text(encoding='utf-8', errors='replace'
                                                          ).strip().splitlines()[-1]
                    write_index(soak_dir, args, records)
            if consecutive_failures >= args.max_consecutive_failures:
                say(f'连续{consecutive_failures}轮未完成，停止老化测试。')
                break

    write_index(soak_dir, args, records)
    say(f'老化测试结束：{len(records)}轮，汇总 {soak_dir / "index.md"}')
    console.close()
    return 0


if __name__ == '__main__':
    def interrupted(_sig, _frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    raise SystemExit(main())
