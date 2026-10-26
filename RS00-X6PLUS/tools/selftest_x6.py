#!/usr/bin/env python3
# 离线自检：校验油门打包、帧 ID、风机流程与中文总结生成
import sys
import time
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import x6test as m


class FakeSock:
    def __init__(self):
        self.sent = []

    def send(self, frame):
        self.sent.append(frame)


class FakeMotor:
    def __init__(self, fan):
        self.fan = fan
        self.stats = {}
        self.last_pos = 0.0
        self.write_ms = 100

    def drain(self, seconds):
        end = time.monotonic() + seconds
        while True:
            remain = end - time.monotonic()
            if remain <= 0:
                return
            self.fan.service()
            time.sleep(min(remain, 0.002))

    def write_param(self, *args):
        return True

    def hold(self, seconds, interval=0.07):
        self.drain(seconds)


def unpack4(payload):
    bits = []
    for byte in payload:
        bits += [(byte >> b) & 1 for b in range(7, -1, -1)]
    out = []
    for channel in range(4):
        low = 0
        for i in range(8):
            low = (low << 1) | bits[channel * 14 + i]
        high = 0
        for i in range(6):
            high = (high << 1) | bits[channel * 14 + 8 + i]
        out.append(low | (high << 8))
    return out


def check_pack():
    got = m.hw_pack4(1000, 1000, 1000, 1000).hex(' ').upper()
    want = 'E8 0F A0 3E 80 FA 03'
    assert got == want, '油门打包不符：{} != {}'.format(got, want)
    print('油门打包 1000 -> {}  与原厂期望一致'.format(got))
    assert m.hw_pack4(0, 0, 0, 0) == bytes(7)
    assert m.hw_pack4(8191, 8191, 8191, 8191)[:2] == bytes([0xFF, 0x7F])
    for case in ((0, 0, 0, 0), (500, 500, 0, 0), (8191, 8191, 8191, 8191), (1, 2, 3, 4)):
        packed = m.hw_pack4(*case)
        assert len(packed) == 7
        assert unpack4(packed) == list(case), '{} 打包回读不符：{}'.format(case, unpack4(packed))
    print('油门打包 0 / 8191 边界与 4 组值往返一致')


def check_ids():
    raw = m.hw_message_id(m.HW_PRIO_HIGH, m.HW_MSG_RAW, m.HW_SRC_NODE)
    ready = m.hw_message_id(m.HW_PRIO_MED, m.HW_MSG_READY, m.HW_SRC_NODE)
    select = m.hw_service_id(m.HW_PRIO_MED, m.HW_SRV_THR_SELECT, 0x01, m.HW_SRC_NODE)
    assert raw == 0x004E8464, hex(raw)
    assert ready == 0x104E2C64, hex(ready)
    assert select == 0x10D781E4, hex(select)
    assert m.hw_sample_ok(0.828) and not m.hw_sample_ok(0.747)
    print('采样点判定：0.828 合格、0.747 不合格（要求 {:.3f}）'.format(m.HW_SAMPLE_POINT))
    print('RawCommand   0x{:08X}'.format(raw))
    print('ReadyControl 0x{:08X}'.format(ready))
    print('ThrottleSel  0x{:08X}'.format(select))
    assert (raw >> 8) & 0xFFFF == m.HW_MSG_RAW
    assert (raw & 0x7F) == m.HW_SRC_NODE
    assert (ready >> 24) & 0x1F == m.HW_PRIO_MED
    assert (select >> 16) & 0xFF == m.HW_SRV_THR_SELECT
    assert (select >> 8) & 0x7F == 0x01
    for can_id in (raw, ready, select,
                   m.hw_message_id(m.HW_PRIO_HIGH, m.HW_MSG_TELEM1, 0x01),
                   m.hw_message_id(m.HW_PRIO_HIGH, m.HW_MSG_TELEM3, 0x02)):
        assert m.hw_is_frame(can_id), '风机帧被漏判 0x{:08X}'.format(can_id)
    rs00 = [m.build_id(0x02, 0x007F, 0xFD), m.build_id(0x11, 0xFD7F, 0x00),
            m.build_id(0x18, 0x007F, 0xFD), m.build_id(0x12, 0xFD7F, 0x00)]
    for can_id in rs00:
        assert not m.hw_is_frame(can_id), 'RS00 帧被误判为风机帧 0x{:08X}'.format(can_id)
    print('风机帧 / RS00 帧判别正确')


def check_flow(tmp):
    sock = FakeSock()
    frames = m.FrameLog(Path(tmp) / 'can-frames.log', 'can0', time.monotonic())
    log = m.Logger(Path(tmp) / 'test.log')
    fan = m.Fan(sock, frames, log, 500, 50)
    motor = FakeMotor(fan)

    motor.drain(0.1)
    assert not sock.sent, '初始化前不应发送风机帧'

    fan.init_sequence(motor, 0.2)
    assert fan.current == 500, '初始化后应切到目标油门'
    motor.drain(0.25)

    raws = [f for f in sock.sent
            if ((m.struct.unpack('=I', f[:4])[0]) >> 8) & 0xFFFF == m.HW_MSG_RAW]
    assert raws, '没有发出油门帧'
    assert raws[-1][8:15] == m.hw_pack4(500, 500, 500, 500), '四通道未同值下发'
    assert not fan.faulted, '发送正常时不应置 faulted'
    print('油门帧四通道同值：{}'.format(raws[-1][8:15].hex(' ').upper()))

    fan.on_frame(m.hw_message_id(m.HW_PRIO_HIGH, m.HW_MSG_TELEM1, 0x01),
                 bytes([0x88, 0x13, 0xF4, 0x01, 0x00, 0x00, 0xC0]))
    fan.on_frame(m.hw_message_id(m.HW_PRIO_HIGH, m.HW_MSG_TELEM2, 0x01),
                 bytes([0x50, 0x01, 0x2C, 0x01, 0x28, 0xC0]))
    fan.on_frame(m.hw_message_id(m.HW_PRIO_HIGH, m.HW_MSG_TELEM3, 0x01),
                 bytes([35, 30, 40, 0, 0, 0, 0, 0xC0]))
    info = fan.nodes[m.HW_ESC_LEFT]
    assert info['rpm'] == 5000 and abs(info['volt'] - 33.6) < 0.01, info
    assert abs(info['amp'] - 30.0) < 0.01 and info['temp'] == 40.0, info
    print('遥测解析：{}rpm {:.1f}V {:.1f}A {:.0f}℃ 状态 0x{:04X}'.format(
        info['rpm'], info['volt'], info['amp'], info['temp'], info['status']))

    before = info['telem']
    for _ in range(2):
        fan.on_frame(m.hw_message_id(m.HW_PRIO_HIGH, m.HW_MSG_TELEM1, 0x01),
                     bytes([0x88, 0x13, 0xF4, 0x01, 0x00, 0x00, 0xC0]))
    assert info['telem'] == before + 2 and info['dup'] == 2, (info['telem'], info['dup'])
    print('重复帧判定：同一 TID 再收 2 次 -> 重复计数 {}'.format(info['dup']))

    fan.shutdown(motor, 5)
    last_ready = [f for f in sock.sent if ((m.struct.unpack('=I', f[:4])[0]) >> 8) & 0xFFFF == m.HW_MSG_READY]
    assert last_ready, '没有发出使能/失能帧'
    assert last_ready[-1][8] == 0x55, '收尾未发送失能 0x55'
    print('收尾最后一条 Ready 帧数据 = 0x{:02X}（0x55 为失能）'.format(last_ready[-1][8]))

    ids = [m.struct.unpack('=I', f[:4])[0] for f in sock.sent]
    assert all(i & 0x80000000 for i in ids), '存在未置 CAN_EFF_FLAG 的帧'
    print('共发送 {} 帧，全部带扩展帧标志'.format(len(ids)))
    periods = fan.stats['period_ms']
    print('油门帧 {} 条，周期中位 {:.1f}ms，迟发 {} 次'.format(
        fan.stats['tx'], m.percentile(periods, 0.5), fan.stats['late']))
    frames.close()
    log.close()
    return fan


def check_eps_time():
    table = [(10, 1), (15, 2), (50, 9), (100, 19), (500, 99), (5, 1)]
    for ms, want in table:
        got = m.eps_time_value(ms)
        assert got == want, '{}ms 应换算为 {}，实际 {}'.format(ms, want, got)
    print('上报周期换算：10->1  50->9  100->19  500->99（0x7026）')


def check_report(tmp, fan):
    settings = {'iface': 'can0', 'minutes': 1.0, 'mode': 'pp', 'speed': 1.0, 'acc': 10.0,
                'fwd_ms': 3000, 'hold_ms': 1000, 'rev_ms': 3000, 'poll_ms': 70,
                'read_ms': 50, 'write_ms': 100, 'no_fan': False, 'fan_throttle': 500,
                'fan_hz': 50.0, 'fan_zero_sec': 2.0, 'report_ms': 50.0}
    gaps = [50.0] * 400 + [100.0]
    stats = {'tx': 2738, 'rx': 9136, 'rx_fan': 6395, 'tx_fail': 0, 'rx_overflow': 0,
             'write_req': 913, 'write_ack': 913, 'write_timeout': 0,
             'read_req': 1821, 'read_ack': 1821, 'read_timeout': 0,
             'feedback': 918, 'report': len(gaps) + 1, 'gaps': gaps,
             'lat_write': [0.5, 0.6, 0.7], 'lat_read': [0.4, 0.5, 0.6],
             'idx': {0x7016: {'r': 0, 'w': 910, 'ack': 910, 'to': 0, 'lat': [0.6]},
                     0x7019: {'r': 910, 'w': 0, 'ack': 910, 'to': 0, 'lat': [0.6]}}}
    cycles = [{'cycle': 1, 'fwd': {'samples': 43, 'timeout': 0, 'pos_start': 1.87,
                                   'pos_end': 4.56, 'pos_delta': 2.69, 'vel_min': 0.1,
                                   'vel_max': 1.1, 'vel_avg': 0.85, 'dir_ok': 39,
                                   'dir_ok_ratio': 0.9},
               'hold': {'samples': 15, 'timeout': 0, 'vel_min': -0.1, 'vel_max': 0.1,
                        'vel_avg': 0.02, 'still': 12, 'still_ratio': 0.8},
               'rev': {'samples': 43, 'timeout': 0, 'pos_start': 4.87, 'pos_end': 2.09,
                       'pos_delta': -2.78, 'vel_min': -1.1, 'vel_max': 0.0,
                       'vel_avg': -0.9, 'dir_ok': 41, 'dir_ok_ratio': 0.95}}]
    text = m.make_report(Path(tmp), settings, stats, cycles, {'rx_errors': 0, 'tx_errors': 0},
                         {'rx_errors': 0, 'tx_errors': 0}, 'a', 'b', 'completed', '', 127,
                         False, (2.09, 1.87), fan)
    out = Path(tmp) / 'report.md'
    out.write_text(text, encoding='utf-8')
    assert '## 推进风机（X6 PLUS）' in text
    assert '0x01（CH1）' in text and '0x02（CH2）' in text
    assert '| 心跳目标周期 / 实测中位间隔（ms） | 50 / 50.00 |' in text
    assert '自动上报（类型0x18）本次按 50ms 一帧设置' in text
    assert '| 心跳期望帧数 / 估算丢失帧数 | {} / 1 |'.format(len(gaps) + 2) in text
    print('中文总结生成正常，{} 行，含风机章节与 50ms 心跳统计'.format(text.count('\n')))


def main():
    tmp = tempfile.mkdtemp(prefix='x6selftest-')
    check_pack()
    check_ids()
    check_eps_time()
    fan = check_flow(tmp)
    check_report(tmp, fan)
    print('\n全部自检通过，临时文件在', tmp)
    return 0


if __name__ == '__main__':
    sys.exit(main())
