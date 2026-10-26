#!/usr/bin/env python3
# 诊断：观察总线健康度，并可选地让电调失能，判断电调使能是否会把总线拉死
import argparse
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import x6test as m


def classify(can_id):
    """按协议把帧 ID 归类成电调遥测、电机心跳、电机应答三类。"""
    hi = (can_id >> 8) & 0xFFFF
    if 0x4E00 <= hi <= 0x4EFF:
        return 'esc'
    if (can_id >> 24) & 0x1F == m.M_READ or (can_id >> 24) & 0x1F == m.M_FEEDBACK:
        return 'motor'
    if (can_id >> 24) & 0x1F == m.M_REPORT:
        return 'motor'
    return 'other'


def main():
    ap = argparse.ArgumentParser(description='观察总线健康度')
    ap.add_argument('--iface', default='can0')
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--action', choices=['none', 'esc_off', 'esc_on'], default='none',
                    help='esc_off 发 0x55 让电调失能，esc_on 发 0xAA 使能')
    args = ap.parse_args()

    sock = struct_module_socket(args.iface)
    tid = 0
    sent = 0
    failed = 0

    def send(can_id, payload):
        nonlocal tid, sent, failed
        body = bytes(payload)[:8].ljust(8, b'\x00')
        try:
            sock.send(struct.pack('=IB3x8s', can_id | m.CAN_EFF_FLAG, len(payload), body))
            sent += 1
            return True
        except OSError:
            failed += 1
            return False

    if args.action != 'none':
        ready = 0x55 if args.action == 'esc_off' else 0xAA
        label = '失能 0x55' if args.action == 'esc_off' else '使能 0xAA'
        for _ in range(3):
            for node in (m.HW_ESC_LEFT, m.HW_ESC_RIGHT):
                send(m.hw_service_id(m.HW_PRIO_MED, m.HW_SRV_THR_SELECT, node, m.HW_SRC_NODE),
                     bytes([0x00, m.hw_tail(tid)]))
                tid = (tid + 1) & 0x1F
            send(m.hw_message_id(m.HW_PRIO_MED, m.HW_MSG_READY, m.HW_SRC_NODE),
                 bytes([ready, m.hw_tail(tid)]))
            tid = (tid + 1) & 0x1F
            for _ in range(20):
                send(m.hw_message_id(m.HW_PRIO_HIGH, m.HW_MSG_RAW, m.HW_SRC_NODE),
                     m.hw_pack4(0, 0, 0, 0) + bytes([m.hw_tail(tid)]))
                tid = (tid + 1) & 0x1F
                time.sleep(0.02)
        print('已发送{}：{} 帧（失败 {}）'.format(label, sent, failed))

    print('{:>5}  {:>9}  {:>9}  {:>8}'.format('t(s)', '电调遥测', '电机帧', '其它'))
    t0 = time.monotonic()
    next_row = t0 + 1.0
    counts = {'esc': 0, 'motor': 0, 'other': 0}
    rows = []
    while True:
        now = time.monotonic()
        if now - t0 >= args.seconds:
            break
        try:
            ready, _, _ = __import__('select').select([sock], [], [], 0.05)
        except InterruptedError:
            ready = []
        if ready:
            while True:
                try:
                    frame = sock.recv(16)
                except OSError:
                    break
                can_id = struct.unpack('=I', frame[:4])[0] & 0x1FFFFFFF
                counts[classify(can_id)] += 1
        if now >= next_row:
            rows.append((now - t0, counts['esc'], counts['motor'], counts['other']))
            print('{:5.1f}  {:>9}  {:>9}  {:>8}'.format(*rows[-1]))
            sys.stdout.flush()
            counts = {'esc': 0, 'motor': 0, 'other': 0}
            next_row = now + 1.0

    total = sum(r[1] for r in rows), sum(r[2] for r in rows)
    dead = [r[0] for r in rows if r[1] == 0]
    print()
    print('合计：电调遥测 {}、电机帧 {}'.format(total[0], total[1]))
    print('电调遥测为 0 的秒数：{} / {}'.format(len(dead), len(rows)))
    return 0


def struct_module_socket(ifname):
    """建立 RAW CAN socket 并绑定到指定接口。"""
    import socket
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, getattr(socket, 'CAN_RAW', 1))
    s.bind((ifname,))
    s.setblocking(False)
    return s


if __name__ == '__main__':
    sys.exit(main())
