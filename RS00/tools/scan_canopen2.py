#!/usr/bin/env python3
"""CANopen 探测（先发 NMT Start 再扫 SDO，避免节点停在 stopped 状态不应答）。"""
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
            print('  RX ID=0x{:03X} {}'.format(
                can_id, ' '.join('{:02X}'.format(b) for b in data[8:8 + dlc])), flush=True)

    print('NMT Start 广播（让所有节点进 operational）...', flush=True)
    sock.send(struct.pack('=IB3x8s', 0x000, 2, bytes([0x01, 0x00]) + bytes(6)))
    drain(0.5)

    print('SDO 读 0x1000 扫描 1~127 ...', flush=True)
    for node in range(1, 128):
        body = bytes([0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0])
        sock.send(struct.pack('=IB3x8s', 0x600 + node, 8, body))
        drain(0.012)

    print('SDO 读 0x1008（设备名）扫描 ...', flush=True)
    for node in range(1, 128):
        body = bytes([0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0])
        sock.send(struct.pack('=IB3x8s', 0x600 + node, 8, body))
        drain(0.012)

    print('被动监听 4 秒（心跳 0x700~0x77F）...', flush=True)
    drain(4.0)
    print('结束，收到 {} 帧。'.format(hits[0]), flush=True)


if __name__ == '__main__':
    main()
