#!/usr/bin/env python3
# 逐个读取电机参数并打印原始应答，用于确认索引与数值格式
import select, socket, struct, time

IFACE, MOTOR, HOST = 'can0', 0x7F, 0xFD
EFF = 0x80000000

IDX = [('run_mode', 0x7005, 'b'), ('loc_ref', 0x7016, 'f'), ('limit_spd', 0x7017, 'f'),
       ('mechPos', 0x7019, 'f'), ('mechVel', 0x701B, 'f'), ('VBUS', 0x701C, 'f'),
       ('vel_max', 0x7024, 'f'), ('acc_set', 0x7025, 'f'),
       ('faultSta', 0x3022, 'I'), ('warnSta', 0x3023, 'I'),
       ('CAN_ID', 0x200a, 'b'), ('CAN_MASTER', 0x200b, 'b')]

s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, 1)
s.bind((IFACE,))
s.setblocking(False)


def send(mode, payload):
    body = bytes(payload).ljust(8, b'\x00')
    cid = ((mode & 0x1F) << 24) | ((HOST & 0xFF) << 8) | MOTOR
    s.send(struct.pack('=IB3x8s', cid | EFF, 8, body))


def ask(index, budget=0.3):
    send(0x11, struct.pack('<H', index) + b'\x00\x00' + bytes(4))
    end = time.time() + budget
    while time.time() < end:
        r, _, _ = select.select([s], [], [], 0.02)
        if not r:
            continue
        f = s.recv(16)
        cid, dlc = struct.unpack('=IB3x', f[:8])
        cid &= 0x1FFFFFFF
        if ((cid >> 24) & 0x1F) != 0x11:
            continue
        data = f[8:8 + dlc]
        if (data[0] | (data[1] << 8)) != index:
            continue
        return (cid >> 16) & 0xFF, data
    return None, None


print('%-11s %-6s %-4s %-22s %s' % ('参数', 'index', '状态', '原始数据', '解析值'))
for name, index, kind in IDX:
    status, data = ask(index)
    if data is None:
        print('%-11s 0x%04X  --   无应答' % (name, index))
        continue
    raw = data.hex(' ').upper()
    if status != 0:
        print('%-11s 0x%04X  失败  %-22s -' % (name, index, raw))
        continue
    value = struct.unpack('<' + kind, data[4:4 + struct.calcsize(kind)])[0]
    print('%-11s 0x%04X  成功  %-22s %s' % (name, index, raw, value))
