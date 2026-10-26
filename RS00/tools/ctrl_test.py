#!/usr/bin/env python3
"""对照实验：确认电机是「应答 type 0」还是「随机重连广播」。"""
import select
import socket
import struct
import time


def bid(mode, data, target):
    return ((mode & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (target & 0xFF)


sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, getattr(socket, 'CAN_RAW', 1))
sock.bind(('can0',))
sock.setblocking(False)


def collect(seconds):
    end = time.monotonic() + seconds
    got = []
    while True:
        left = end - time.monotonic()
        if left <= 0:
            break
        ready, _, _ = select.select([sock], [], [], left)
        if not ready:
            break
        data = sock.recv(16)
        if len(data) >= 16:
            can_id, dlc = struct.unpack('=IB', data[:5])
            got.append('{:08X} {}'.format(can_id, ' '.join('{:02X}'.format(b) for b in data[8:8 + dlc])))
    return got


def send(mode, host, target, payload=bytes(8), wait=0.5, tag=''):
    sock.send(struct.pack('=IB3x8s', bid(mode, host, target), 8, payload))
    got = collect(wait)
    print('{:<34} -> {}'.format(tag, got if got else '无'))


print('== 被动监听 10s ==')
print('  ', collect(10.0) or '无')

print('== type 0 (取ID) 到 0x7F，host=0，连发 5 次 ==')
for i in range(5):
    send(0x00, 0x00, 0x7F, tag='  #%d type0 host=00 target=7F' % (i + 1))

print('== type 5 (不存在的类型) 连发 3 次 ==')
for i in range(3):
    send(0x05, 0x00, 0x7F, tag='  #%d type5 host=00 target=7F' % (i + 1))

print('== type 0 换不同 host / target ==')
send(0x00, 0x00, 0x7F, tag='  type0 host=00 target=7F')
send(0x00, 0x01, 0x7F, tag='  type0 host=01 target=7F')
send(0x00, 0xFD, 0x7F, tag='  type0 host=FD target=7F')

print('== 被动监听 10s ==')
print('  ', collect(10.0) or '无')
