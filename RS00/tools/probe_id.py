#!/usr/bin/env python3
# 判定电机是否 ACK 我们的帧，并按手册默认值（电机ID 127 / 主机ID 127）请求应答
import socket, struct, sys, time

IFACE = 'can0'
MOTOR = 0x7F
OPT_OWN = 5

IDX_RUN_MODE = 0x7005
IDX_MECH_POS = 0x7019
IDX_CAN_ID = 0x200a
IDX_CAN_MASTER = 0x200b
IDX_FAULT = 0x3022


def ext(t, mid, host):
    return ((t & 0x1F) << 24) | ((mid & 0xFF) << 8) | (host & 0xFF)


def pkt(cid, data):
    data = bytes(data)
    return struct.pack('=IB3x8s', cid, len(data), data[:8].ljust(8, b'\x00'))


def decode(cid, data):
    return 'type=%02X 区2=%02X 区1=%02X | %s' % (
        (cid >> 24) & 0x1F, (cid >> 8) & 0xFF, cid & 0xFF,
        data[:8].hex(' ').upper())


def open_sock(own):
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.setsockopt(socket.SOL_CAN_RAW, OPT_OWN, struct.pack('i', 1 if own else 0))
    s.bind((IFACE,))
    s.settimeout(0)
    return s


def drain(s, dur):
    end = time.time() + dur
    out = []
    while time.time() < end:
        try:
            f = s.recv(16)
        except BlockingIOError:
            time.sleep(0.002)
            continue
        cid, dlc = struct.unpack('=IB3x', f[:8])
        out.append((cid, f[8:8 + dlc]))
    return out


def main():
    s = open_sock(True)

    # 第 1 步：发一帧特征数据，看是否被回灌（回灌 = 已被总线上某个节点 ACK）
    print('== 第 1 步：自环回灌 / ACK 判定 ==')
    drain(s, 0.2)
    mark = bytes([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22])
    got = []
    for i in range(3):
        s.send(pkt(ext(0x00, 0x7F, 0x7F), mark))
        got += drain(s, 0.15)
    echo = [x for x in got if x[1][:8] == mark]
    if echo:
        print('  回灌到 %d 帧 -> 帧发送成功，总线上有节点 ACK' % len(echo))
    else:
        print('  没有回灌 -> 没有任何节点 ACK，物理层/波特率有问题')
    for cid, d in got[:6]:
        print('  RX ' + decode(cid, d))

    # 第 2 步：按手册默认主机 ID 请求取设备 ID 与读参数
    print()
    print('== 第 2 步：取设备 ID / 读参数（目标电机 0x%02X） ==' % MOTOR)
    cases = []
    for host in (0x7F, 0xFD, 0xFF, 0x00):
        cases.append(('取ID host=%02X' % host, ext(0x00, MOTOR, host), bytes(8)))
    for host in (0x7F, 0xFD):
        for name, idx in (('读0x7019位置', IDX_MECH_POS), ('读0x7005模式', IDX_RUN_MODE),
                          ('读0x200a电机ID', IDX_CAN_ID), ('读0x200b主机ID', IDX_CAN_MASTER),
                          ('读0x3022故障', IDX_FAULT)):
            cases.append(('%s host=%02X' % (name, host), ext(0x11, MOTOR, host),
                          struct.pack('<H', idx) + bytes(6)))

    hit = 0
    for name, cid, data in cases:
        res = []
        for i in range(3):
            s.send(pkt(cid, data))
            res += drain(s, 0.12)
        res = [x for x in res if x[1][:8] != data and x[1][:8] != mark]
        print('  %-22s %08X -> %s' % (name, cid, '无' if not res else ''))
        for c2, d2 in res:
            hit += 1
            print('        RX ' + decode(c2, d2))

    # 第 3 步：纯监听，看是否有主动上报
    print()
    print('== 第 3 步：被动监听 20s ==')
    res = drain(s, 20)
    res = [x for x in res if x[1][:8] != mark]
    if not res:
        print('  无')
    for cid, d in res[:20]:
        print('  RX ' + decode(cid, d))
    print()
    print('小结：应答帧合计 %d' % hit)
    return 0 if hit else 1


if __name__ == '__main__':
    sys.exit(main())
