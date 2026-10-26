#!/usr/bin/env python3
"""只读扫描：向 1~127 逐个发私有协议读指令，打印所有收到的帧。

用于确认电机 CAN ID，或判断电机是否被切成 CANopen/MIT 协议。
"""
import select
import socket
import struct
import sys
import time

IDX_MECH_POS = 0x7019
MASTER_ID = 0xFD


def build_id(mode, data, target):
    return ((mode & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (target & 0xFF)


def hexs(data):
    return ' '.join('{:02X}'.format(b) for b in data)


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, getattr(socket, 'CAN_RAW', 1))
    sock.bind((iface,))
    sock.setblocking(False)

    replies = []

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
            payload = data[8:8 + dlc]
            replies.append((can_id, payload))
            print('  RX ID={:08X} type=0x{:02X} target={} data={}'.format(
                can_id, (can_id >> 24) & 0x1F, can_id & 0xFF, hexs(payload)), flush=True)

    print('扫描私有协议读 0x7019，目标 ID 1~127 ...', flush=True)
    for target in range(1, 128):
        payload = struct.pack('<H', IDX_MECH_POS) + bytes([0, 0, 0, 0, 0, 0])
        frame = struct.pack('=IB3x8s', build_id(0x11, MASTER_ID, target), 8, payload)
        try:
            sock.send(frame)
        except OSError as exc:
            print('  发送失败 id={}: {}'.format(target, exc), flush=True)
            continue
        drain(0.015)
        if replies:
            print('  ^ 目标 ID {} 有应答'.format(target), flush=True)

    print('广播取设备 ID（类型 0）...', flush=True)
    sock.send(struct.pack('=IB3x8s', build_id(0x00, 0x00FD, 0x7F), 8, bytes(8)))
    drain(0.3)
    print('被动监听 3 秒（看有无主动上报/启动帧）...', flush=True)
    drain(3.0)
    print('扫描结束，共收到 {} 帧。'.format(len(replies)), flush=True)


if __name__ == '__main__':
    main()
