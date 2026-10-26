#!/usr/bin/env python3
# 用两路套接字判定帧是否被总线节点 ACK，并按手册默认值请求应答
import os, socket, struct, subprocess, sys, time

IFACE = 'can0'
MOTOR = 0x7F
OPT_OWN = 4
OPT_LOOPBACK = 3

IDX = [('0x7019位置', 0x7019), ('0x7005模式', 0x7005), ('0x200a电机ID', 0x200a),
       ('0x200b主机ID', 0x200b), ('0x3022故障', 0x3022)]


def ext(t, mid, host):
    return ((t & 0x1F) << 24) | ((mid & 0xFF) << 8) | (host & 0xFF)


def pkt(cid, data):
    data = bytes(data)[:8].ljust(8, b'\x00')
    return struct.pack('=IB3x8s', cid, len(data), data)


def show(cid, data):
    return 'type=%02X 区2=%02X 区1=%02X | %s' % (
        (cid >> 24) & 0x1F, (cid >> 8) & 0xFF, cid & 0xFF, data[:8].hex(' ').upper())


def sock(own=0):
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.setsockopt(socket.SOL_CAN_RAW, OPT_LOOPBACK, struct.pack('i', 1))
    s.setsockopt(socket.SOL_CAN_RAW, OPT_OWN, struct.pack('i', own))
    s.bind((IFACE,))
    s.settimeout(0)
    return s


def drain(s, dur):
    end, out = time.time() + dur, []
    while time.time() < end:
        try:
            f = s.recv(16)
        except BlockingIOError:
            time.sleep(0.002)
            continue
        cid, dlc = struct.unpack('=IB3x', f[:8])
        out.append((cid, f[8:8 + dlc]))
    return out


def state():
    o = subprocess.run(['ip', '-details', '-statistics', 'link', 'show', IFACE],
                       capture_output=True, text=True).stdout.splitlines()
    keep = [x.strip() for x in o if 'state' in x or 'error-pass' in x or x.strip().startswith(('0 ', '2 '))]
    return ' || '.join(keep[:4])


def main():
    mark = bytes([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22])
    tx = sock(0)
    rx = sock(0)

    # 第 1 步：本机 socket 回环是否开启（发送后自己能否收到）
    print('== 第 1 步：本机发送回环 ==')
    tx.send(pkt(ext(0x00, 0x7F, 0x7F), mark))
    got = drain(rx, 0.3)
    echo = [x for x in got if x[1][:8] == mark]
    print('  另一路 socket 收到回环 %d 帧（回环开着才看得到，不代表有外部节点）' % len(echo))

    # 第 2 步：关闭 restart-ms 后发帧，没人 ACK 就会进入 ERROR-PASSIVE/BUS-OFF
    print()
    print('== 第 2 步：ACK 判定（restart-ms=0） ==')
    os.system('ip link set %s down; ip link set %s type can bitrate 1000000 restart-ms 0 loopback off; '
              'ip link set %s up; sleep 0.3' % (IFACE, IFACE, IFACE))
    tx2 = sock(0)
    for i in range(3):
        try:
            tx2.send(pkt(ext(0x00, 0x7F, 0x7F), mark))
        except OSError as e:
            print('  发送报错：%s' % e)
        time.sleep(0.05)
    time.sleep(0.8)
    print('  ' + state())

    # 第 3 步：恢复接口，按手册默认值请求应答
    os.system('ip link set %s down; ip link set %s type can bitrate 1000000 restart-ms 100 loopback off; '
              'ip link set %s txqueuelen 1000; ip link set %s up; sleep 0.3' % (IFACE, IFACE, IFACE, IFACE))
    s = sock(0)
    print()
    print('== 第 3 步：请求应答（目标电机 0x%02X） ==' % MOTOR)
    cases = [('取ID host=%02X' % h, ext(0x00, MOTOR, h), bytes(8)) for h in (0x7F, 0xFD)]
    for h in (0x7F, 0xFD):
        for name, idx in IDX:
            cases.append(('%s host=%02X' % (name, h), ext(0x11, MOTOR, h),
                          struct.pack('<H', idx) + bytes(6)))
    hit = 0
    for name, cid, data in cases:
        res = []
        for i in range(4):
            s.send(pkt(cid, data))
            res += drain(s, 0.1)
        res = [x for x in res if x[1][:8] != data]
        print('  %-24s %08X -> %s' % (name, cid, '无' if not res else ''))
        for c2, d2 in res:
            hit += 1
            print('        RX ' + show(c2, d2))

    print()
    print('== 第 4 步：被动监听 15s ==')
    res = [x for x in drain(s, 15)]
    print('  无' if not res else '')
    for cid, d in res[:20]:
        print('  RX ' + show(cid, d))
    print()
    print('应答帧合计 %d；%s' % (hit, state()))
    return 0 if hit else 1


if __name__ == '__main__':
    sys.exit(main())
