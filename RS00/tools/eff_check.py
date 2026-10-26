#!/usr/bin/env python3
# 检查发出去的帧到底是不是 29 位扩展帧（对比是否置 CAN_EFF_FLAG）
import socket, struct, subprocess, time

s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind(('can0',))

cases = [
    ('不置标志', 0x00007F7F),
    ('置标志  ', 0x00007F7F | 0x80000000),
    ('不置标志', 0x11007FFD),
    ('置标志  ', 0x11007FFD | 0x80000000),
]
print('发送 4 帧，期望 candump 输出里 11 位 ID 会显示成 3 位十六进制（如 77F），')
print('29 位扩展帧显示成 8 位十六进制（如 00007F7F）。')
p = subprocess.Popen(['candump', '-ta', '-n', '4', 'can0'],
                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(0.7)
for name, cid in cases:
    s.send(struct.pack('=IB3x8s', cid, 8, bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88])))
    print('  发出 %s 变量值=%08X' % (name, cid))
    time.sleep(0.2)
out, _ = p.communicate(timeout=10)
print()
print('== candump 实际看到的 4 帧 ==')
for line in out.strip().splitlines():
    print('  ' + line)
