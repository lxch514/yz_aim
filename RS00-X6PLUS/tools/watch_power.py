#!/usr/bin/env python3
# 诊断：观察电机心跳与母线电压，并在指定时刻使能风机，判断风机使能是否打断电机
import argparse
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import x6test as m


def can_state(ifname):
    """取内核里 can0 的控制器状态。"""
    try:
        out = subprocess.run(['ip', '-details', 'link', 'show', ifname],
                             capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return '--'
    for line in out.splitlines():
        if 'can state' in line:
            return line.split('can state', 1)[1].split('(berr', 1)[0].strip()
    return '--'


def enable_report(motor, log):
    """打开电机主动上报，返回 F_CMD 所在字节（0 表示未打开）。"""
    for pos in (6, 7):
        payload = bytearray([1, 2, 3, 4, 5, 6, 0, 0])
        payload[pos] = 1
        before = motor.stats['report']
        motor._send(m.M_REPORT, bytes(payload), '开启自动上报')
        motor.drain(0.3)
        if motor.stats['report'] > before:
            log('自动上报已打开（F_CMD 在 byte{}）。'.format(pos))
            return pos
    log('自动上报未打开（电机可能不在线）。')
    return 0


def main():
    ap = argparse.ArgumentParser(description='观察电机心跳与母线电压')
    ap.add_argument('--iface', default='can0')
    ap.add_argument('--motor-id', type=int, default=127)
    ap.add_argument('--seconds', type=float, default=14.0, help='总观察时长')
    ap.add_argument('--arm-at', type=float, default=-1.0, help='在该时刻使能风机，负值不使能')
    ap.add_argument('--throttle', type=int, default=0, help='风机油门，默认 0')
    ap.add_argument('--step', type=float, default=0.5, help='打印行间隔')
    args = ap.parse_args()

    tmp = Path('/tmp/watch-power')
    tmp.mkdir(exist_ok=True)
    frames = m.FrameLog(tmp / 'frames.log', args.iface, time.monotonic())
    log = m.Logger(tmp / 'watch.log')
    motor = m.Motor(args.iface, args.motor_id, frames, log)
    fan = m.Fan(motor.sock, frames, log, args.throttle, 50)
    if args.arm_at >= 0:
        motor.fan = fan

    log('内核状态：{}'.format(can_state(args.iface)))
    report_pos = enable_report(motor, log)

    print('{:>5}  {:>7}  {:>7}  {:>7}  {:>13}  {:>13}  {:>6}'.format(
        't(s)', 'VBUS(V)', '上报帧', '间最大', 'ESC0x01 V/A', 'ESC0x02 V/A', '遥测'))
    t0 = time.monotonic()
    next_read = t0
    next_row = t0
    armed = False
    vbus = None
    vmin = None
    max_gap = 0.0
    row_report = motor.stats['report']
    row_telem = fan.stats['telem']
    esc_vmin = {}
    rows = []

    def esc_text(node):
        info = fan.nodes[node]
        if info['volt'] is None:
            return '--'
        return '{:.1f}V/{:.1f}A'.format(info['volt'], info['amp'])

    while True:
        now = time.monotonic()
        if now - t0 >= args.seconds:
            break
        if args.arm_at >= 0 and not armed and now - t0 >= args.arm_at:
            log('使能风机（油门 {}）：选 CAN 油门 → 0xAA → 零油门保持。'.format(args.throttle))
            fan.active = True
            fan.next_due = time.monotonic()
            for node in sorted(fan.nodes):
                fan.send_select_can(node)
                motor.drain(0.1)
            fan.send_ready(True)
            armed = True
        motor.drain(0.02)
        now = time.monotonic()
        if now >= next_read:
            value = motor.option_read(m.IDX_VBUS, 40)
            if value is None:
                vbus = None
            else:
                vbus = value
                vmin = value if vmin is None else min(vmin, value)
            next_read = now + 0.1
        gap = motor.gaps[-1] if motor.gaps else 0.0
        if gap > max_gap:
            max_gap = gap
        for node in sorted(fan.nodes):
            volt = fan.nodes[node]['volt']
            if volt is not None:
                esc_vmin[node] = volt if node not in esc_vmin else min(esc_vmin[node], volt)
        if now >= next_row:
            report = motor.stats['report']
            telem = fan.stats['telem']
            rows.append((now - t0, vbus, report - row_report, max_gap,
                         telem - row_telem, can_state(args.iface)))
            row_report = report
            row_telem = telem
            print('{:5.1f}  {:>7}  {:>7}  {:>7.1f}  {:>13}  {:>13}  {:>6}'.format(
                rows[-1][0],
                '--' if vbus is None else '{:.1f}'.format(vbus),
                rows[-1][2], max_gap,
                esc_text(m.HW_ESC_LEFT), esc_text(m.HW_ESC_RIGHT), rows[-1][4]))
            next_row = now + args.step

    print()
    print('合计：上报帧 {}、遥测帧 {}、上报最大间隔 {:.1f}ms'.format(
        motor.stats['report'], fan.stats['telem'], max_gap))
    print('VBUS 最低 {}、最后 {}；ESC 输入电压最低 {}'.format(
        '--' if vmin is None else '{:.1f}V'.format(vmin),
        '--' if vbus is None else '{:.1f}V'.format(vbus),
        '、'.join('0x{:02X} {}'.format(n, '{:.1f}V'.format(v)) for n, v in sorted(esc_vmin.items())) or '--'))
    silent = [r for r in rows if r[2] == 0 and r[0] > (args.arm_at if args.arm_at >= 0 else 0) + 1.0]
    print('使能后无上报的行数：{} / {}'.format(len(silent), len(rows)))
    if args.arm_at >= 0:
        fan.shutdown(motor, 20)
    print('结束状态：{}'.format(can_state(args.iface)))
    frames.close()
    log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
