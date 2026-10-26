#!/usr/bin/env python3
"""最后一种只读/低风险探测：向 1~127 发使能帧，看是否有类型 2 反馈帧回来。

使能帧不带位置/速度指令，运控模式下 kp/kd/力矩均为 0，电机不会出力。
"""
import select
import socket
import struct
import sys
import time

MASTER_ID = 0xFD


def build_id(mode, data, target):
    return ((mode & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (target & 0xFF)


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, getattr(socket, 'CAN_RAW', 1))
    sock.bind((iface,))
    sock.setblocking(False)
    hits = [0]

    def drain(seconds):
        end = time.monotonic() + seconds
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return
            ready, _, _ = select.select([sock], [], [], left)
            if not ready:
                return
            try:
                data = sock.recv(16)
            except OSError:
                return
            if len(data) < 16:
                continue
            can_id, dlc = struct.unpack('=IB', data[:5])
            hits[0] += 1
            print('  RX ID={:08X} type=0x{:02X} target={} {}'.format(
                can_id, (can_id >> 24) & 0x1F, can_id & 0xFF,
                ' '.join('{:02X}'.format(b) for b in data[8:8 + dlc])), flush=True)

    print('使能帧扫描 1~127 ...', flush=True)
    for target in range(1, 128):
        sock.send(struct.pack('=IB3x8s', build_id(0x03, MASTER_ID, target), 8, bytes(8)))
        drain(0.012)
    print('停止帧扫描 1~127 ...', flush=True)
    for target in range(1, 128):
        sock.send(struct.pack('=IB3x8s', build_id(0x04, MASTER_ID, target), 8, bytes(8)))
        drain(0.012)
    print('被动监听 3 秒 ...', flush=True)
    drain(3.0)
    print('结束，收到 {} 帧。'.format(hits[0]), flush=True)


if __name__ == '__main__':
    main()
