#!/usr/bin/env python3
"""回环自检用的假电机：在同一 CAN 接口上模拟 RS00 的应答，用于验证测试程序。

用法：先 ip link set can0 up，然后
    python3 -B tools/fake_motor.py --iface can0 --id 127
再另开终端运行 rs00_test.py --iface can0 --motor-id 127 --minutes 1
"""
import argparse
import select
import socket
import struct
import sys
import time

P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -33.0, 33.0
T_MIN, T_MAX = -14.0, 14.0

M_FEEDBACK = 0x02
M_ENABLE = 0x03
M_STOP = 0x04
M_READ = 0x11
M_WRITE = 0x12
M_REPORT = 0x18

IDX_RUN_MODE = 0x7005
IDX_LOC_REF = 0x7016
IDX_LIMIT_SPD = 0x7017
IDX_MECH_POS = 0x7019
IDX_MECH_VEL = 0x701B
IDX_VBUS = 0x701C
IDX_VEL_MAX = 0x7024
IDX_ACC_SET = 0x7025
IDX_FAULT = 0x3022

TEMP = 35.0


def f2u(value, low, high):
    value = min(max(value, low), high)
    return int((value - low) * 65535.0 / (high - low))


def build_id(mode, data, target):
    return ((mode & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (target & 0xFF)


class FakeMotor:
    def __init__(self, iface, motor_id):
        self.motor_id = motor_id
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, getattr(socket, 'CAN_RAW', 1))
        self.sock.bind((iface,))
        self.sock.setblocking(False)
        self.pos = 0.0
        self.vel = 0.0
        self.target = 0.0
        self.vel_max = 1.0
        self.run_mode = 0
        self.loc_ref_seen = False
        self.report = False
        self.report_ms = 10.0
        self.last_report = 0.0
        self.sent_report = 0
        self.received = 0

    def send(self, mode, payload):
        body = bytes(payload).ljust(8, b'\x00')
        frame = struct.pack('=IB3x8s', build_id(mode, self.motor_id, 0xFD), len(payload), body)
        try:
            self.sock.send(frame)
        except OSError:
            pass

    def feedback(self, mode=M_FEEDBACK):
        self.send(mode, struct.pack('>HHHH',
                                    f2u(self.pos, P_MIN, P_MAX),
                                    f2u(self.vel, V_MIN, V_MAX),
                                    f2u(0.0, T_MIN, T_MAX),
                                    int(TEMP * 10)))

    def step(self, dt):
        diff = self.target - self.pos
        want = max(-self.vel_max, min(self.vel_max, diff * 12.0))
        self.vel = want
        self.pos += self.vel * dt

    def handle(self, can_id, payload):
        mode = (can_id >> 24) & 0x1F
        target = can_id & 0xFF
        if target != self.motor_id:
            return
        self.received += 1
        if mode == M_ENABLE:
            self.run_mode = 2 if self.run_mode == 0 else self.run_mode
            self.feedback()
        elif mode == M_STOP:
            self.vel = 0.0
            self.run_mode = 0
            self.feedback()
        elif mode == M_WRITE:
            index = payload[0] | (payload[1] << 8)
            raw = payload[4:8]
            if index == IDX_RUN_MODE:
                self.run_mode = raw[0]
            elif index == IDX_LOC_REF:
                self.target = struct.unpack('<f', raw)[0]
                self.loc_ref_seen = True
            elif index == IDX_VEL_MAX:
                self.vel_max = struct.unpack('<f', raw)[0]
            self.feedback()
        elif mode == M_READ:
            index = payload[0] | (payload[1] << 8)
            if index == IDX_MECH_POS:
                value = struct.pack('<f', self.pos)
            elif index == IDX_MECH_VEL:
                value = struct.pack('<f', self.vel)
            elif index == IDX_VBUS:
                value = struct.pack('<f', 48.0)
            elif index == IDX_FAULT:
                value = struct.pack('<I', 0)
            else:
                value = struct.pack('<f', 0.0)
            self.send(M_READ, struct.pack('<H', index) + bytes([0, 0]) + value)
        elif mode == M_REPORT:
            self.report = payload[6] == 1 or payload[7] == 1
            self.feedback()

    def run(self):
        print('假电机启动：ID {}，等待测试程序。'.format(self.motor_id), flush=True)
        last = time.monotonic()
        while True:
            now = time.monotonic()
            timeout = 0.002
            ready, _, _ = select.select([self.sock], [], [], timeout)
            if ready:
                try:
                    data = self.sock.recv(16)
                except OSError:
                    data = b''
                if len(data) >= 16:
                    can_id, dlc = struct.unpack('=IB', data[:5])
                    self.handle(can_id, data[8:8 + dlc])
            now = time.monotonic()
            dt = now - last
            last = now
            self.step(min(dt, 0.05))
            if self.report and (now - self.last_report) * 1000.0 >= self.report_ms:
                self.last_report = now
                self.feedback(M_REPORT)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--iface', default='can0')
    parser.add_argument('--id', type=int, default=0x7F)
    args = parser.parse_args()
    FakeMotor(args.iface, args.id).run()


if __name__ == '__main__':
    main()
