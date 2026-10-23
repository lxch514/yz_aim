#!/usr/bin/env python3
"""只读取本目录通信日志；不发送CAN帧、不使能电机。"""
import argparse
from collections import Counter
from datetime import datetime
from pathlib import Path
import re
import time

FRAME = re.compile(r'^\[CAN \S+ \+([\d.]+)s #\d+\]\s+(\w+)\s+ID=([0-9A-Fa-f]+) DLC=(\d+)(.*)$')
START = re.compile(r'^# CAN_START epoch_ms=(\d+) pid=(\d+)')


class Statistics:
    def __init__(self, emit, ack_timeout=0.5):
        self.emit = emit
        self.ack_timeout = ack_timeout
        self.count = Counter()
        self.nodes_tx = Counter()
        self.nodes_rx = Counter()
        self.pending = {}
        self.enabled = {1: False, 2: False}
        self.alarm = {}
        self.heartbeat_at = None
        self.heartbeat_gap_max = 0.0
        self.heartbeat_warned = False
        self.driver = Counter()
        self.epoch = None
        self.pid = None
        self.process_dead_warned = False

    def error(self, message):
        self.emit('异常：' + message)

    def expire(self, elapsed):
        for key, request in list(self.pending.items()):
            if elapsed - request['at'] <= self.ack_timeout:
                continue
            self.pending.pop(key)
            self.count['sdo_timeout'] += 1
            node, index, sub = key
            action = '读' if request['read'] else '写'
            self.error(f'节点{node}的SDO{action}请求0x{index:04X}:{sub:02X}超过'
                       f'{self.ack_timeout * 1000:.0f}毫秒未见确认；疑似应答丢失或延迟，不能据此认定物理丢帧。')
        if (any(self.enabled.values()) and self.heartbeat_at is not None and
                elapsed - self.heartbeat_at > 0.7 and not self.heartbeat_warned):
            self.count['heartbeat_timeout'] += 1
            self.heartbeat_warned = True
            self.error('仍有节点处于使能状态，但超过700毫秒未记录到0x77F主站心跳，请立即检查控制程序与通信日志。')

    def feed(self, line):
        start = START.match(line)
        if start:
            self.epoch = int(start[1]) / 1000.0
            self.pid = int(start[2])
            return None
        if line.startswith('[CAN STAT '):
            values = {key: int(value) for key, value in re.findall(r'(\w+)=(\d+)', line)}
            for key in ('TX_ERR', 'KERNEL_RX_DROP', 'TRACE_DROP', 'OUTPUT_DROP'):
                if values.get(key, 0) > self.driver.get(key, 0):
                    descriptions = {'TX_ERR': '传输层发送失败', 'KERNEL_RX_DROP': '控制程序套接字接收队列丢弃',
                                    'TRACE_DROP': '诊断记录队列丢弃（不等于CAN丢帧）',
                                    'OUTPUT_DROP': '通信日志输出失败或不完整（不等于CAN丢帧）'}
                    self.error(f'{descriptions[key]}累计{values[key]}次。')
            self.driver = Counter(values)
            return None
        match = FRAME.match(line)
        if not match:
            return None
        elapsed, direction, identifier, dlc, payload = match.groups()
        elapsed, identifier, dlc = float(elapsed), int(identifier, 16), int(dlc)
        self.expire(elapsed)
        data = bytes.fromhex(payload.strip()) if payload.strip() else b''
        self.count[direction] += 1
        if direction == 'TX_ERR':
            self.error(f'发送CAN帧0x{identifier:03X}失败，请检查接口状态和发送队列。')
            return match
        if direction == 'RX_DROP':
            self.count['kernel_drop'] += identifier
            self.error(f'控制程序接收队列溢出，本次明确丢弃{identifier}帧。')
            return match
        if direction == 'RX_ERROR':
            self.error(f'收到CAN总线错误帧0x{identifier:08X}，原始数据{data.hex(" ")}。')
            return match
        if direction == 'TX' and identifier == 0x77F:
            self.count['heartbeat_tx'] += 1
            if self.heartbeat_at is not None and any(self.enabled.values()):
                self.heartbeat_gap_max = max(self.heartbeat_gap_max, elapsed - self.heartbeat_at)
            self.heartbeat_at = elapsed
            self.heartbeat_warned = False
        if direction == 'RX' and 0x701 <= identifier <= 0x77E:
            self.count['node_heartbeat'] += 1
        if direction == 'TX' and 0x601 <= identifier <= 0x67E:
            node = identifier - 0x600
            self.nodes_tx[node] += 1
            if dlc != 8 or len(data) != 8 or data[0] not in (0x40, 0x23, 0x2B, 0x2F):
                return match
            index, sub = int.from_bytes(data[1:3], 'little'), data[3]
            key = (node, index, sub)
            if key in self.pending:
                self.count['read_retry' if data[0] == 0x40 else 'repeat_object_write'] += 1
            self.pending[key] = {'at': elapsed, 'read': data[0] == 0x40,
                                 'value': int.from_bytes(data[4:], 'little')}
        if direction == 'RX' and 0x581 <= identifier <= 0x5FE:
            node = identifier - 0x580
            self.nodes_rx[node] += 1
            if len(data) != 8:
                self.error(f'节点{node}的SDO应答长度异常：{len(data)}字节。')
                return match
            index, sub = int.from_bytes(data[1:3], 'little'), data[3]
            request = self.pending.pop((node, index, sub), None)
            if data[0] == 0x80:
                self.count['sdo_abort'] += 1
                self.error(f'节点{node}拒绝对象0x{index:04X}:{sub:02X}，SDO终止码0x{int.from_bytes(data[4:], "little"):08X}。')
            elif data[0] in (0x43, 0x47, 0x4B, 0x4F):
                width = 4 - ((data[0] >> 2) & 3)
                value = int.from_bytes(data[4:4 + width], 'little')
                if index == 0x2601:
                    self.enabled[node] = bool(value & 1)
                if index == 0x260E:
                    previous = self.alarm.get(node)
                    self.alarm[node] = value
                    if value and value != previous:
                        if value == 0x20 and not self.enabled.get(node, False):
                            self.emit(f'提示：节点{node}报警0x0020且输出禁止；停用主站心跳后可能出现，需由安全使能流程恢复。')
                        else:
                            self.error(f'节点{node}报告驱动报警0x{value:04X}。')
            elif data[0] == 0x60 and request:
                if index == 0x2601:
                    self.enabled[node] = bool(request['value'] & 1)
                elif index == 0x6040:
                    if request['value'] == 0x100:
                        self.enabled[node] = False
                    elif request['value'] == 0xF:
                        self.enabled[node] = True
        return match

    def summary(self):
        tx = max(self.count['TX'], self.driver['TX'])
        rx = max(self.count['RX'], self.driver['RX'])
        dropped = max(self.count['kernel_drop'], self.driver['KERNEL_RX_DROP'])
        self.emit(f'通信统计：发送{tx}帧，接收{rx}帧，发送失败{max(self.count["TX_ERR"], self.driver["TX_ERR"])}次；'
                  f'节点1 SDO发/收={self.nodes_tx[1]}/{self.nodes_rx[1]}，节点2={self.nodes_tx[2]}/{self.nodes_rx[2]}；'
                  f'主站心跳{self.count["heartbeat_tx"]}帧、最大有效间隔{self.heartbeat_gap_max * 1000:.1f}毫秒、'
                  f'超时{self.count["heartbeat_timeout"]}次，节点心跳{self.count["node_heartbeat"]}帧；'
                  f'SDO读重发{self.count["read_retry"]}次、未确认超时{self.count["sdo_timeout"]}次、'
                  f'拒绝{self.count["sdo_abort"]}次；接收队列明确丢弃{dropped}帧，'
                  f'诊断记录丢弃{self.driver["TRACE_DROP"]}条，日志输出异常{self.driver["OUTPUT_DROP"]}次。'
                  '心跳为单向发送，收发数量不必相等；未确认请求不等于已证实的物理丢帧。')


def main():
    parser = argparse.ArgumentParser(description='独立通信监测：每分钟中文汇总，异常即时提示；不控制电机。')
    parser.add_argument('--interval', type=float, default=60, help='中文汇总间隔秒，默认60')
    parser.add_argument('--frames', action='store_true', help='在本监测终端逐帧刷屏，不影响控制终端')
    parser.add_argument('--log', type=Path, help='指定本目录logs下的一份can-frames日志，默认自动跟随最新会话')
    args = parser.parse_args()
    if args.interval < 1:
        parser.error('汇总间隔至少1秒')
    root = Path(__file__).resolve().parent
    logs = root / 'logs'
    current = None
    reader = None
    append = None
    statistics = None
    next_summary = time.monotonic() + args.interval
    print('通信监测已启动：只读统计，不发送CAN命令；Ctrl+C仅退出监测。', flush=True)

    def emit(message):
        line = f'[CAN监测 {datetime.now():%Y-%m-%d %H:%M:%S}] {message}'
        print(line, flush=True)
        if append:
            append.write(line + '\n')
            append.flush()

    try:
        while True:
            candidates = [args.log] if args.log else sorted(logs.glob('can-frames-*.log'))
            candidate = candidates[-1] if candidates else None
            if candidate and candidate.is_file() and candidate != current:
                if not candidate.resolve().is_relative_to(logs.resolve()):
                    parser.error('仅允许监测本程序logs目录内的文件')
                if statistics:
                    statistics.summary()
                if reader:
                    reader.close()
                    append.close()
                current = candidate
                reader = current.open('r', encoding='utf-8', errors='replace')
                append = current.open('a', encoding='utf-8', buffering=1)
                statistics = Statistics(emit)
                emit('正在监测：' + str(current))
                next_summary = time.monotonic() + args.interval
            if reader:
                for _ in range(2000):
                    position = reader.tell()
                    line = reader.readline()
                    if not line:
                        break
                    if not line.endswith('\n'):
                        reader.seek(position)
                        break
                    frame = statistics.feed(line)
                    if args.frames and frame:
                        direction = {'TX': '发送', 'RX': '接收', 'TX_ERR': '发送失败',
                                     'RX_ERROR': '总线错误', 'RX_DROP': '接收丢弃'}.get(frame[2], frame[2])
                        print(f'[{direction}] 运行{frame[1]}秒 ID=0x{frame[3]} DLC={frame[4]}{frame[5]}', flush=True)
                else:
                    continue  # finish historical backlog before judging live heartbeat silence
                if statistics.epoch is not None and statistics.pid and Path('/proc', str(statistics.pid)).exists():
                    statistics.expire(max(0, time.time() - statistics.epoch))
                elif statistics.pid and any(statistics.enabled.values()) and not statistics.process_dead_warned:
                    statistics.process_dead_warned = True
                    statistics.error('控制程序已退出，但最后记录尚未确认两轴输出禁止，请检查现场停止状态。')
                if time.monotonic() >= next_summary:
                    statistics.summary()
                    next_summary = time.monotonic() + args.interval
            time.sleep(0.1)
    except KeyboardInterrupt:
        if statistics:
            statistics.summary()
        print('监测已退出，控制程序不受影响。', flush=True)
    finally:
        if reader:
            reader.close()
            append.close()


if __name__ == '__main__':
    main()
