#!/usr/bin/env python3
"""CANFD 探测：用 FD 帧（BRS）发私有协议读指令，看是否有 FD 应答。

用法：先把接口配成 FD，例如
  ip link set can0 down
  ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
  ip link set can0 up
参数：--arb 仲裁波特率仅用于打印。
"""
import select
import socket
import struct
import sys
import time

CANFD_BRS = 0x01
IDX_MECH_POS = 0x7019
MASTER_ID = 0xFD


def build_id(mode, data, target):
    return ((mode & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (target & 0xFF)


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, getattr(socket, 'CAN_RAW', 1))
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
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
                data = sock.recv(72)
            except OSError:
                return
            if len(data) == 72:
                can_id, length, flags = struct.unpack('=IBB', data[:6])
                hits[0] += 1
                print('  RX FD ID={:08X} len={} flags=0x{:02X} {}'.format(
                    can_id, length, flags,
                    ' '.join('{:02X}'.format(b) for b in data[8:8 + length])), flush=True)
            elif len(data) >= 16:
                can_id, dlc = struct.unpack('=IB', data[:5])
                hits[0] += 1
                print('  RX CLASSIC ID={:08X} {}'.format(
                    can_id, ' '.join('{:02X}'.format(b) for b in data[8:8 + dlc])), flush=True)

    def send_fd(target, mode=0x11, body=None):
        payload = body if body is not None else (struct.pack('<H', IDX_MECH_POS) + bytes(6))
        frame = struct.pack('=IBBBB64s', build_id(mode, MASTER_ID, target), len(payload),
                            CANFD_BRS, 0, 0, payload.ljust(64, b'\x00'))
        try:
            sock.send(frame)
            return True
        except OSError as exc:
            print('  发送失败 target={}: {}'.format(target, exc), flush=True)
            return False

    print('FD 帧扫描：类型 0x11 读 0x7019，目标 ID 1~127（BRS=1）...', flush=True)
    for target in range(1, 128):
        send_fd(target)
        drain(0.012)
    print('再试一次不带 BRS 的 FD 帧，目标 1~8 ...', flush=True)
    for target in range(1, 9):
        frame = struct.pack('=IBBBB64s', build_id(0x11, MASTER_ID, target),
                            8, 0, 0, 0, (struct.pack('<H', IDX_MECH_POS) + bytes(6)).ljust(64, b'\x00'))
        try:
            sock.send(frame)
        except OSError:
            pass
        drain(0.012)
    print('被动监听 2 秒 ...', flush=True)
    drain(2.0)
    print('结束，收到 {} 帧。'.format(hits[0]), flush=True)


if __name__ == '__main__':
    main()
