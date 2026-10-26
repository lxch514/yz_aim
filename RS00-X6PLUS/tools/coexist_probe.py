#!/usr/bin/env python3
# 诊断：按时间轴观察电机心跳与风机使能的关系，定位"同时工作就掉线"的因果
import argparse
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import x6test as m

SYS_STATS = ['bus_off', 'error_warning', 'error_passive', 'tx_errors', 'rx_errors', 'restarts']


def can_state(ifname):
    """取内核里该接口的控制器状态。"""
    try:
        out = subprocess.run(['ip', '-details', 'link', 'show', ifname],
                             capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return '--'
    for line in out.splitlines():
        if 'can state' in line:
            return line.split('can state', 1)[1].split('(berr', 1)[0].strip()
    return '--'


def sys_counters(ifname):
    """读取 /sys 下 CAN 控制器的错误与总线关闭计数。"""
    base = Path('/sys/class/net') / ifname / 'statistics'
    out = {}
    for name in SYS_STATS:
        try:
            out[name] = int((base / name).read_text().strip())
        except (OSError, ValueError):
            out[name] = None
    return out


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
    ap = argparse.ArgumentParser(description='观察电机与风机使能的共存情况')
    ap.add_argument('--iface', default='can0')
    ap.add_argument('--motor-id', type=int, default=127)
    ap.add_argument('--seconds', type=float, default=30.0, help='总观察时长')
    ap.add_argument('--fan-on-at', type=float, default=6.0, help='在该时刻使能风机')
    ap.add_argument('--fan-off-at', type=float, default=20.0, help='在该时刻失能风机，负值不失能')
    ap.add_argument('--throttle', type=int, default=0, help='风机油门，默认 0')
    ap.add_argument('--poll', action='store_true', help='同时每 70ms 读一次角度（给总线配一个主机）')
    ap.add_argument('--skip-ready', action='store_true', help='不发 0xAA，只发 50Hz 油门帧')
    ap.add_argument('--skip-raw', action='store_true', help='不发 50Hz 油门帧，只发一次 0xAA')
    args = ap.parse_args()

    tmp = Path('/tmp/coexist')
    tmp.mkdir(exist_ok=True)
    frames = m.FrameLog(tmp / 'frames.log', args.iface, time.monotonic())
    log = m.Logger(tmp / 'coexist.log')
    motor = m.Motor(args.iface, args.motor_id, frames, log)
    fan = m.Fan(motor.sock, frames, log, args.throttle, 50)
    motor.fan = fan
    fan.active = False

    log('内核状态：{}'.format(can_state(args.iface)))
    enable_report(motor, log)
    before = sys_counters(args.iface)

    print('{:>5}  {:>7}  {:>7}  {:>6}  {:>7}  {:>8}  {:>9}  {}'.format(
        't(s)', '电机RX', '电机TX', 'TX失败', '遥测RX', 'bus_off', 'err_warn', '状态'))
    t0 = time.monotonic()
    next_row = t0
    next_poll = t0
    mark = dict(motor_rx=motor.stats['rx'], motor_tx=motor.stats['tx'],
                tx_fail=motor.stats['tx_fail'], telem=fan.stats['telem'])
    sys_mark = dict(before)
    armed = False
    disarmed = False
    last_state = can_state(args.iface)
    state_change = []

    while True:
        now = time.monotonic()
        if now - t0 >= args.seconds:
            break
        elapsed = now - t0
        if not armed and elapsed >= args.fan_on_at:
            log('使能风机（油门 {}，0xAA={}，50Hz={}）。'.format(
                args.throttle, '否' if args.skip_ready else '是',
                '否' if args.skip_raw else '是'))
            fan.active = not args.skip_raw
            fan.next_due = time.monotonic()
            for node in sorted(fan.nodes):
                fan.send_select_can(node)
                motor.drain(0.1)
            if not args.skip_ready:
                fan.send_ready(True)
            fan.enabled = True
            fan.current = args.throttle
            armed = True
        if args.fan_off_at >= 0 and armed and not disarmed and elapsed >= args.fan_off_at:
            log('失能风机。')
            fan.shutdown(motor, 20)
            disarmed = True
        if args.poll and now >= next_poll:
            motor.read_param(m.IDX_MECH_POS, 50)
            next_poll = now + 0.07
        motor.drain(0.02)

        now = time.monotonic()
        if now >= next_row:
            cur = sys_counters(args.iface)
            state = can_state(args.iface)
            if state != last_state:
                state_change.append((now - t0, state))
                log('控制器状态变为 {}（t={:.1f}s）'.format(state, now - t0))
                last_state = state
            print('{:5.1f}  {:>7}  {:>7}  {:>6}  {:>7}  {:>8}  {:>9}  {}'.format(
                now - t0,
                motor.stats['rx'] - mark['motor_rx'],
                motor.stats['tx'] - mark['motor_tx'],
                motor.stats['tx_fail'] - mark['tx_fail'],
                fan.stats['telem'] - mark['telem'],
                '--' if cur['bus_off'] is None else cur['bus_off'] - (sys_mark['bus_off'] or 0),
                '--' if cur['error_warning'] is None else cur['error_warning'] - (sys_mark['error_warning'] or 0),
                state))
            sys.stdout.flush()
            mark = dict(motor_rx=motor.stats['rx'], motor_tx=motor.stats['tx'],
                        tx_fail=motor.stats['tx_fail'], telem=fan.stats['telem'])
            next_row = now + 1.0

    if armed and not disarmed:
        fan.shutdown(motor, 20)
    print()
    print('状态变化：{}'.format('、'.join('{:.1f}s->{}'.format(t, s) for t, s in state_change) or '无'))
    print('合计：电机RX {}、电机TX {}（失败 {}）、遥测 {}'.format(
        motor.stats['rx'], motor.stats['tx'], motor.stats['tx_fail'], fan.stats['telem']))
    frames.close()
    log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
