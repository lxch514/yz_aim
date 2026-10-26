#!/usr/bin/env python3
"""持续发请求并记录所有收发，用于抓电机的上电启动过程。

每 50ms 轮换发：取设备ID(0x00) / 读0x7019 / 读0x7005 / 使能(0x03)，目标 0x7F，主机 0xFD。
"""
import select
import socket
import struct
import sys
import time

MASTER_ID = 0xFD
TARGET = 0x7F


def bid(mode, data, target):
    return ((mode & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (target & 0xFF)


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 180.0
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, getattr(socket, 'CAN_RAW', 1))
    sock.bind((iface,))
    sock.setblocking(False)
    frames = [
        (bid(0x00, MASTER_ID, TARGET), bytes(8), '取ID'),
        (bid(0x11, MASTER_ID, TARGET), struct.pack('<H', 0x7019) + bytes(6), '读0x7019'),
        (bid(0x11, MASTER_ID, TARGET), struct.pack('<H', 0x7005) + bytes(6), '读0x7005'),
        (bid(0x03, MASTER_ID, TARGET), bytes(8), '使能'),
    ]
    t0 = time.monotonic()
    end = t0 + seconds
    nxt = t0
    index = 0
    print('窗口开始：{}s，每 50ms 轮换发 4 种请求，目标 ID {}(0x{:02X})'.format(
        int(seconds), TARGET, TARGET), flush=True)
    while True:
        now = time.monotonic()
        if now >= end:
            break
        ready, _, _ = select.select([sock], [], [], 0.005)
        if ready:
            try:
                data = sock.recv(16)
            except OSError:
                data = b''
            if len(data) >= 16:
                can_id, dlc = struct.unpack('=IB', data[:5])
                print('[+{:7.3f}s] RX ID={:08X} {}'.format(
                    time.monotonic() - t0, can_id,
                    ' '.join('{:02X}'.format(b) for b in data[8:8 + dlc])), flush=True)
        if now >= nxt:
            can_id, body, tag = frames[index % len(frames)]
            index += 1
            try:
                sock.send(struct.pack('=IB3x8s', can_id, 8, body))
            except OSError as exc:
                print('发送失败: {}'.format(exc), flush=True)
            nxt += 0.05
    print('窗口结束，共发出 {} 帧。'.format(index), flush=True)


if __name__ == '__main__':
    main()
