#!/usr/bin/env python3
"""只读探测：向 1~127 发 CANopen SDO 读（索引 0x1000），看是否有 0x580+N 应答。"""
import select
import socket
import struct
import sys
import time


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, getattr(socket, 'CAN_RAW', 1))
    sock.bind((iface,))
    sock.setblocking(False)
    hits = 0

    def drain(seconds):
        nonlocal hits
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
            hits += 1
            print('  RX ID=0x{:03X} {}'.format(
                can_id, ' '.join('{:02X}'.format(b) for b in data[8:8 + dlc])), flush=True)

    print('CANopen SDO 读 0x1000 扫描 1~127 ...', flush=True)
    for node in range(1, 128):
        body = bytes([0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0])
        sock.send(struct.pack('=IB3x8s', 0x600 + node, 8, body))
        drain(0.012)
    print('NMT start 广播 ...', flush=True)
    sock.send(struct.pack('=IB3x8s', 0x000, 2, bytes([0x01, 0x00]) + bytes(6)))
    drain(0.3)
    print('被动监听 3 秒 ...', flush=True)
    drain(3.0)
    print('结束，收到 {} 帧。'.format(hits), flush=True)


if __name__ == '__main__':
    main()
