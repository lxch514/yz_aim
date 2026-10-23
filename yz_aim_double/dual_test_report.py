#!/usr/bin/env python3
"""读取已结束的USB-CAN双轴测试目录，生成中文Markdown和可复核JSON，不发送CAN帧。"""
import argparse
import csv
import hashlib
import json
import re
import sys
from pathlib import Path
sys.dont_write_bytecode = True
from dual_trace_analysis import parse_trace, distribution


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    d = args.directory.resolve()
    if not d.is_relative_to(root / 'logs'):
        parser.error('只能读取本项目logs目录')
    summary = json.loads((d / 'summary.json').read_text(encoding='utf-8'))
    raw = (d / 'can-frames.log').read_bytes()
    parsed = {str(n): parse_trace(raw, n) for n in [1, 2]}
    communication = summary.get('communication', {})
    assert parsed['1']['directions']['TX'] == communication['tx'] and \
        parsed['1']['directions']['RX'] == communication['rx'], '收发统计与逐帧日志不一致'
    settings = summary['settings']
    rpm = settings['rpm']
    allrows = []
    stable = []
    timing = []
    trim_errors = []
    for action in summary['actions']:
        file = Path(action['csv'])
        assert file.resolve().is_relative_to(d)
        text = file.read_text(encoding='utf-8')
        rows = list(csv.DictReader(line for line in text.splitlines() if not line.startswith('#')))
        allrows.extend(rows)
        stable.extend(row for row in rows if row['event'] == 'sample' and row['quality'] == 'ok')
        match = re.search(r'trigger_a_ms=(-?\d+) trigger_b_ms=(-?\d+) stop_request_a_ms=(-?\d+) stop_request_b_ms=(-?\d+)', text)
        if match:
            ta, tb, sa, sb = map(int, match.groups())
            timing.append({'csv': str(file), 'trigger_gap_ms': abs(ta - tb), 'stop_gap_ms': abs(sa - sb),
                           'requested_ms': settings['move_ms'], 'stop_request_a_ms': sa, 'stop_request_b_ms': sb})
        if f"max_trim_rpm={settings['max_trim_rpm']}" not in text.splitlines()[0]:
            trim_errors.append(str(file))
    diff = [abs(float(r['actual_a_rpm']) + float(r['actual_b_rpm'])) for r in stable]
    bad_target = [r for r in allrows if float(r['target_a_rpm']) + float(r['target_b_rpm']) != 0
                  or abs(float(r['target_a_rpm'])) != rpm]
    motion = {'csv_files': len(summary['actions']), 'all_rows': len(allrows), 'steady_rows': len(stable),
              'target_mismatch_rows': len(bad_target), 'wrong_trim_csv': trim_errors,
              'steady_speed_difference_rpm': distribution(diff),
              'steady_abs_rpm_a': distribution([abs(float(r['actual_a_rpm'])) for r in stable]),
              'steady_abs_rpm_b': distribution([abs(float(r['actual_b_rpm'])) for r in stable]),
              'all_abs_rpm_a': distribution([abs(float(r['actual_a_rpm'])) for r in allrows]),
              'all_abs_rpm_b': distribution([abs(float(r['actual_b_rpm'])) for r in allrows]),
              'maximum_recorded_position_error_deg': max((abs(float(r['error_output_deg'])) for r in allrows), default=None),
              'trigger_gap_ms': distribution([x['trigger_gap_ms'] for x in timing]),
              'stop_gap_ms': distribution([x['stop_gap_ms'] for x in timing]),
              'stop_request_a_ms': distribution([x['stop_request_a_ms'] for x in timing]),
              'stop_request_b_ms': distribution([x['stop_request_b_ms'] for x in timing]),
              'still_check_retries': sum(x.get('still_retries', 0) for x in summary['actions']),
              'action_timing': timing}
    analysis = {'nodes': parsed, 'motion': motion, 'trace_sha256': hashlib.sha256(raw).hexdigest()}
    (d / 'analysis.json').write_text(json.dumps(analysis, ensure_ascii=False, indent=2), encoding='utf-8')
    stamp = summary['started_at'].replace('-', '').replace(':', '').replace('T', '-')[:13]
    minutes = settings['minutes']
    duration_label = f'{minutes:g}分钟' if not settings.get('cycles') else '短时验证'
    filename = f'YZ-AIM_两轴通信统计_{stamp}_USBCAN双轴_{duration_label}.md'
    lines = []

    def add(s=''):
        lines.append(s)

    def table(headers, rows):
        add()
        add('| ' + ' | '.join(headers) + ' |')
        add('|' + '|'.join(['---'] * len(headers)) + '|')
        for row in rows:
            add('| ' + ' | '.join(str(x) for x in row) + ' |')

    adapter = summary.get('adapter', {})
    add('# YZ-AIM USB-CAN两轴等速测试与通信统计')
    add()
    add(f"测试：{summary['started_at']} → {summary['ended_at']}（北京时间）；"
        f"结果`{summary['outcome']}`，实际{summary.get('wall_seconds', '—')}秒，"
        f"{summary['completed_cycles']}组/{summary['completed_actions']}次动作，"
        f"双轴失能确认={summary['disabled_confirmed']}。")
    if summary.get('reason'):
        add()
        add('结束原因：' + summary['reason'])
    add()
    add('## 条件与判断范围')
    add()
    add(f"- 程序`/root/yz_aim_double`；两个节点{settings['node_a']}和{settings['node_b']}接到同一个"
        f"LYS USBCANFD适配器（设备{adapter.get('device')}、通道{adapter.get('channel')}、"
        f"经典CAN 1 Mbit/s），由厂商SDK一收一发共享同一条物理总线，每轴仍各有自己的SDO接收队列。"
        f"节点{settings['node_a']}为A、节点{settings['node_b']}为B。伸出A负/B正，缩进A正/B负；"
        f"目标幅值{rpm}电机RPM，每方向{settings['move_ms']}ms，两次动作之间静止至少{settings.get('pause_ms', 0)}ms。")
    if settings.get('mode') == 'equal_speed':
        add(f"- 采用等速记录方式：最大减速修正{settings['max_trim_rpm']}RPM、累计偏差阈值"
            f"{settings['sync_tolerance_deg']}度，即不按位置差调速度、也不因位置差停机。"
            "这是本程序上对应CAN0版`sync-mode speed`的等效设置（该版本没有`sync-mode`命令）；位置反馈照常记录。")
    else:
        add(f"- 采用位置同步方式：最大减速修正{settings['max_trim_rpm']}RPM，累计偏差阈值"
            f"{settings['sync_tolerance_deg']}度，超过阈值按设计停机。")
    add('- 保留驱动报警、限位、方向/超速、通信及反馈失效保护。不存在无条件不停机保证。'
        '目标速度相同不等于实际速度每个采样点严格相等，实测差值如下。')
    add('- 控制和反馈全部使用SDO；两轴共用一个适配器，主站心跳由同一生产者按200ms节流发送。'
        '目标速度与控制字分别成对连续发送，仍存在总线串行时间，不是PDO+SYNC硬同步。')
    add('- 两轴写确认由旧版配对写逻辑处理（本轮程序为1.2.0-trace，没有1.2.3的启动读回降级）：'
        '配对写失败会同时判两轴失败并锁止会话，因此一旦出现SDO应答缺失，本轮会立即停止而不是重发运动命令。')
    add('- 为避免同一节点应答后立即密集请求，SDO事务之间保留原有间隔；保持心跳服务，直接停止帧不受此间隔延迟。'
        '读请求有限重试，写请求不自动重放。')
    add('- 动作时间从两轴目标下发前计起；到时先向两轴发停止帧，再确认停稳。运动结束的物理减速、状态检查、'
        '静止等待包含在墙钟时间内。')
    add('- 运行前仅记录一次软件位置基准，没有写入硬件零点。每约60秒输出中文通信统计，异常即时记录，'
        '不向控制输入终端刷原始帧。')
    add()
    add('## 速度、位置与动作时序')
    table(['项目', '实测'],
          [['完成动作', summary['completed_actions']],
           ['等幅反向目标不符的CSV行', motion['target_mismatch_rows']],
           ['修正幅度记录不符的CSV', len(trim_errors)],
           ['稳态反馈样本', len(stable)],
           ['A实测速度幅值RPM', motion['steady_abs_rpm_a']],
           ['B实测速度幅值RPM', motion['steady_abs_rpm_b']],
           ['两轴实际速度差RPM', motion['steady_speed_difference_rpm']],
           ['两轴触发间隔ms', motion['trigger_gap_ms']],
           ['两轴停止帧间隔ms', motion['stop_gap_ms']],
           ['A停止请求时刻ms', motion['stop_request_a_ms']],
           ['B停止请求时刻ms', motion['stop_request_b_ms']],
           ['最大记录位置差（输出度，只记录）', motion['maximum_recorded_position_error_deg']],
           ['运动前静止检查重试', motion['still_check_retries']]])
    add()
    add('稳态样本仅统计CSV中`sample`且反馈质量为`ok`的记录，排除启动和停止阶段；'
        '短时低速验证可能没有稳态样本。位置是电机反馈，不能代替机械绝对零点/行程测量。'
        'CSV时序为应用观测时间，非硬件同步时间戳。')
    add()
    add('## CAN收发与心跳')
    table(['指标', '本轮值'], [[k, communication.get(k, '未取得')] for k in
                            ['tx', 'rx', 'driver_tx', 'driver_rx', 'tx_failed', 'can_error_frames',
                             'kernel_rx_dropped', 'trace_records_dropped', 'trace_output_errors',
                             'heartbeat_tx', 'heartbeat_max_gap_ms', 'heartbeat_timeouts',
                             'sdo_read_repeats_observed', 'sdo_unconfirmed_timeouts', 'sdo_aborts',
                             'sdo_pending_at_end']])
    table(['适配器与通道', '本轮值'], [['程序', summary.get('program', '—')],
                                 ['USB库', adapter.get('library', '—')],
                                 ['设备号/通道', f"{adapter.get('device')} / {adapter.get('channel')}"],
                                 ['位标识', adapter.get('label', '—')],
                                 ['波特率', adapter.get('bitrate', '—')]])
    add()
    add('TX含SDO请求和主站心跳，RX含SDO应答、TPDO及节点心跳，TX和RX无需相等。'
        'USB通道由厂商SDK收发，内核不提供该通道的接口包计数、接收队列丢弃与硬件CRC/重传计数，'
        '表中`kernel_rx_dropped`标为不适用，不能据此写成"0丢弃证明物理层无错"。'
        'TX只表示SDK已接受，是否到达驱动器要看同ID的RX应答。'
        'SDO配对、Abort、未确认超时与诊断队列丢弃分别报告；SDO全成功也不能证明物理CAN绝对无丢帧。')
    add()
    add("- 表中`tx`/`rx`是逐帧日志实际记录的行数，`driver_tx`/`driver_rx`是控制程序内部计数；"
        "两者相差的就是`trace_records_dropped`（诊断队列写满时按设计丢弃记录，不影响控制）。"
        "USB适配器一次读到多帧时，诊断队列更容易在这几帧上丢记录，因此`requests_without_unique_reply`"
        "里可能有少量缺口是被丢弃的记录造成的，而不是驱动器真的没应答；判断时要对着`unmatched_responses`一起看。")
    for node, p in parsed.items():
        add()
        add(f'## 节点{node} SDO与反馈')
        table(['指标', '值'],
              [['SDO请求', sum(x['reads'] + x['writes'] for x in p['objects'].values())],
               ['SDO应答', sum(x['replies'] for x in p['objects'].values())],
               ['重复对象请求', len(p['duplicate_requests'])],
               ['未配对请求', len(p['pending_requests'])],
               ['无唯一应答请求', p['requests_without_unique_reply']],
               ['无在途请求的应答', len(p['unmatched_responses'])],
               ['日志观测SDO延迟ms', p['latency_ms']],
               ['反馈转速范围RPM', f"{p['speed_min_rpm']} 至 {p['speed_max_rpm']}"],
               ['初末位置脉冲', f"{p['position_first']} → {p['position_last']}"],
               ['报警码读取次数', p.get('alarm_counts', {})]])
        table(['对象', '读', '写', '应答', 'Abort', '延迟中位ms', 'P95ms', '最大ms'],
              [[obj, v['reads'], v['writes'], v['replies'], v['abort'],
                (v['latency_ms'] or {}).get('p50', '—'), (v['latency_ms'] or {}).get('p95', '—'),
                (v['latency_ms'] or {}).get('max', '—')]
               for obj, v in sorted(p['objects'].items(), key=lambda x: -(x[1]['reads'] + x[1]['writes']))])
        add()
        add('非零报警读数时间和值：`' + json.dumps(p.get('nonzero_alarms', []), ensure_ascii=False)
            + '`。退出停发主站心跳后可能出现0x0020，应与运行期间报警分开看。')
    previous = []
    for file in sorted((root / 'logs').glob('parallel-30min-*/summary.json')):
        if file.parent == d:
            continue
        try:
            old = json.loads(file.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            continue
        if old.get('settings', {}).get('mode') != 'equal_speed' or \
                old.get('started_at', '') >= summary['started_at'] or \
                old.get('started_at', '')[:10] != summary['started_at'][:10]:
            continue
        c = old.get('communication', {})
        previous.append([str(file.parent), old.get('outcome'), old.get('completed_actions'),
                         c.get('sdo_read_repeats_observed', '—'), c.get('sdo_pending_at_end', '—'),
                         old.get('disabled_confirmed')])
    if previous:
        add()
        add('## 此前准备验证记录（不混入本轮统计）')
        table(['目录', '结果', '完成动作', 'SDO读重发', '结束待确认SDO', '该会话失能确认'], previous)
    add()
    add('## 日志、结论与复现')
    add()
    add(f'- 全部记录：`{d}`。`can-frames.log`是原始TX/RX（控制程序的stderr整份重定向到此文件）；'
        '`controller.log`是完整命令过程；`test.log`是中文过程/分钟统计；'
        '每个`motion-*.csv`是两轴速度、位置、时序记录。')
    add('- `summary.json`保存最终结果，`analysis.json`保存逐帧配对和CSV统计。'
        '本次仅对实际完成时长下结论，1分钟通过不写成30分钟通过。')
    faults = sum(communication.get(k, 0) for k in
                 ['tx_failed', 'can_error_frames', 'trace_records_dropped', 'trace_output_errors',
                  'sdo_unconfirmed_timeouts', 'sdo_aborts', 'heartbeat_timeouts'])
    add(f"- 本轮通信显式异常项目合计{faults}；两节点无唯一应答请求合计"
        f"{sum(p['requests_without_unique_reply'] for p in parsed.values())}；"
        f"目标等幅检查不符{len(bad_target)}行。")
    add()
    add('```sh\ncd /root/yz_aim_double\n'
        'python3 -B dual_cycle_test.py --rpm 100 --move-ms 3000 --minutes 1\n'
        '# 收到30分钟测试指令后才运行：\n'
        '# python3 -B dual_cycle_test.py --rpm 100 --move-ms 3000 --minutes 30\n```')
    report = d / filename
    report.write_text('\n'.join(lines) + '\n', encoding='utf-8', newline='\n')
    print(json.dumps({'report': str(report), 'report_sha256': hashlib.sha256(report.read_bytes()).hexdigest(),
                      'analysis': str(d / 'analysis.json'), 'motion': motion}, ensure_ascii=False))


if __name__ == '__main__':
    main()
