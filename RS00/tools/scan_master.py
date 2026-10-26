#!/usr/bin/env python3
"""只读扫描：枚举「主机 id × 电机 id」，找电机实际应答的组合。

有些固件会按配置里的 CAN_MASTER 过滤请求帧，主机 id 不对时只 ACK 不回应。
"""
import select
import socket
import struct
import sys
import time

IDX_MECH_POS = 0x7019


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
            print('  RX ID={:08X} type=0x{:02X} master=0x{:02X} target={} {}'.format(
                can_id, (can_id >> 24) & 0x1F, (can_id >> 8) & 0xFF, can_id & 0xFF,
                ' '.join('{:02X}'.format(b) for b in data[8:8 + dlc])), flush=True)

    body = struct.pack('<H', IDX_MECH_POS) + bytes(6)
    for master in (0x00, 0x7F, 0x01, 0xFD):
        print('主机 id = 0x{:02X}，扫描电机 id 1~127 ...'.format(master), flush=True)
        before = hits[0]
        for target in range(1, 128):
            try:
                sock.send(struct.pack('=IB3x8s', build_id(0x11, master, target), 8, body))
            except OSError as exc:
                print('  发送失败: {}'.format(exc), flush=True)
                break
            drain(0.010)
        print('  本组收到 {} 帧。'.format(hits[0] - before), flush=True)
    print('被动监听 2 秒 ...', flush=True)
    drain(2.0)
    print('结束，共收到 {} 帧。'.format(hits[0]), flush=True)


if __name__ == '__main__':
    main()
