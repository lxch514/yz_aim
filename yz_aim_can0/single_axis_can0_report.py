#!/usr/bin/env python3
"""CAN0 单轴往返测试的中文报告；从本次 summary 与 CAN 原始帧生成，不控制设备。

与 parallel_cycle_report 的区别只有一处：只渲染一个节点的一列。
复用 parallel_cycle_report.load_job，保证 analysis.json 与并行测试完全同构。
"""
import argparse
from datetime import datetime
import json
from pathlib import Path

from parallel_cycle_report import latency_text, load_job, safe, shown
from parallel_trace_analysis import distribution

FIELDS = [('发送失败（次）', 'tx_failed'), ('CAN错误帧（帧）', 'can_error_frames'),
          ('SDO读请求重发迹象（次）', 'sdo_read_repeats_observed'),
          ('SDO重复写请求迹象（次）', 'sdo_repeated_writes_observed'),
          ('SDO监测未确认超时（500ms阈值，次）', 'sdo_unconfirmed_timeouts'),
          ('SDO Abort拒绝（次）', 'sdo_aborts'), ('主站心跳发送（帧）', 'heartbeat_tx'),
          ('主站心跳最大有效间隔（ms）', 'heartbeat_max_gap_ms'),
          ('主站心跳监测超时（次）', 'heartbeat_timeouts'), ('节点心跳接收（帧）', 'node_heartbeat_rx'),
          ('诊断队列记录丢弃（条）', 'trace_records_dropped'), ('日志输出异常（次）', 'trace_output_errors')]

INTERFACE = [('rx_packets', '接收包'), ('tx_packets', '发送包'), ('rx_errors', '接收错误'),
             ('tx_errors', '发送错误'), ('rx_dropped', '接收丢弃'), ('tx_dropped', '发送丢弃'),
             ('rx_missed_errors', '接收漏失')]


def generate_reports(state):
    jobs = state['jobs']
    name = next(iter(jobs))
    job = jobs[name]
    result, analysis = load_job(job)
    analysis['motion'] = analysis.get('motion', {})
    settings = state['settings']
    minutes = settings['minutes']
    stamp = datetime.strptime(state['stamp'], '%Y%m%d-%H%M%S')
    filename = f'YZ-AIM_单轴通信统计_{stamp:%Y%m%d-%H%M%S}_CAN0_{minutes:g}分钟.md'
    plan = ('伸出' if settings.get('sequence') == 'mirror' else '正转')
    back = ('缩进' if settings.get('sequence') == 'mirror' else '反转')
    both = settings.get('hold_after', 'both') == 'both'
    rows = [f'# YZ-AIM CAN0 单轴往返通信测试（{minutes:g}分钟）', '',
            f'- 测试开始：{state["started_at"]}；结束：{state.get("ended_at", "尚未结束")}。',
            f'- 总体状态：`{state["phase"]}`。' + (f' 原因：{safe(state["reason"])}' if state.get('reason') else ''),
            f'- 接线：两个电机都接在板载 `{settings.get("interface", "can0")}` 总线上，本次**只寻址节点 '
            f'{job["node"]}**，另一节点不被访问、不使能、不运动。',
            f'- 设定：节点 {job["node"]} 每次 {plan} **{settings["rpm"]} RPM {settings["move_ms"]} ms** → '
            f'确认停止并保持 **{settings["stop_ms"]} ms** → {back} **{settings["rpm"]} RPM '
            f'{settings["move_ms"]} ms** → 确认停止'
            + (f'并保持 {settings["stop_ms"]} ms' if both else '') + ' → 下一组。',
            '- 使用 `/root/yz_aim_can0` 已验证的 CAN0 单轴控制程序，发送 `jog 带符号RPM 毫秒`；不修改零点或节点号。',
            '- 时长为循环经过时间，包含启停、通信、静止检查与停止保持；达到时长后完成最后一组并停止失能。', '',
            '## 运行与动作反馈', '', '| 项目 | 本次 |', '|---|---|']

    def row(label, value):
        rows.append('| ' + label + ' | ' + safe(value) + ' |')

    row('寻址节点', job['node'])
    row('程序结果', result['outcome'])
    row('循环经过时间（秒）', shown(result.get('wall_seconds')))
    row('完成往返组数 / 动作次数', f'{result.get("completed_cycles", 0)} / {result.get("completed_actions", 0)}')
    row('第一方向 / 反方向次数',
        f'{analysis["motion"].get("forward_count", 0)} / {analysis["motion"].get("reverse_count", 0)}')
    row('累计命令运动时间（秒）', result.get('commanded_motion_seconds', 0))
    row('已确认停止并失能', '是' if result.get('disabled_confirmed') else '未完整确认')
    row('反馈中观察到正确转速方向的动作数', analysis['motion'].get('speed_direction_observed_actions'))
    row('反馈中观察到正确位置变化方向的动作数',
        analysis['motion'].get('position_direction_observed_actions'))
    row('实际转速最小 / 最大（RPM）', f'{shown(analysis.get("speed_min_rpm"))} / {shown(analysis.get("speed_max_rpm"))}')
    row('位置首值 / 末值（电机脉冲）',
        f'{shown(analysis.get("position_first"))} / {shown(analysis.get("position_last"))}')
    row('每次动作后停止保持最小值（秒）',
        shown((distribution([x.get('confirmed_hold_seconds', 0) for x in result.get('actions', [])
                             if x.get('confirmed_hold_seconds')]) or {}).get('min')))
    row('静止门禁拒绝后的重新确认次数', analysis['motion'].get('still_gate_retries'))
    row('失败 / 中断原因', result.get('reason') or '无')
    rows += ['', '转速与位置来自驱动反馈；只看到发送运动命令不能证明电机已转动。'
             '若某项反馈计数不足，只表示未获得足够采样证据，不能补写成动作成功。', '',
             '## CAN收发、SDO与心跳', '', '| 项目 | 本次 |', '|---|---|']
    analysis_comm = analysis
    row('应用层记录TX / RX（帧）',
        f'{analysis_comm.get("directions", {}).get("TX", 0)} / {analysis_comm.get("directions", {}).get("RX", 0)}')
    row('SDO请求 / 应答（帧）',
        f'{analysis_comm.get("frame_types", {}).get("SDO请求", {}).get("TX", 0)} / '
        f'{analysis_comm.get("frame_types", {}).get("SDO应答", {}).get("RX", 0)}')
    comm = result.get('communication', {})
    for label, key in FIELDS:
        row(label, shown(comm.get(key)))
    row('SocketCAN 接收队列明确丢弃（帧）', shown(comm.get('kernel_rx_dropped')))
    row('日志末尾仍待确认的SDO请求（条）', len(analysis.get('pending_requests', [])))
    row('未找到前置请求的SDO应答（条）', len(analysis.get('unmatched_responses', [])))
    row('SDO延迟 min / avg / p50 / p95 / p99 / max（ms）', latency_text(analysis.get('latency_ms')))
    rows += ['', 'TX与RX不必相等：发送包含主站心跳与SDO请求，接收还可能包含TPDO和节点心跳。'
             '应比较同节点、同对象的SDO请求与确认，而不是直接把总收发数量相减当作丢帧。', '',
             'SDO读重发表示上一读请求在控制程序的等待窗口内未获得确认，可能是请求/应答未到、延迟或接收处理问题；'
             '仅凭应用日志不能确认哪一段发生物理CAN丢帧。统计监测的500ms阈值独立于控制程序自身超时，'
             '因此“读重发”和“监测超时”计数可以不同。', '',
             '本次只有一个节点被访问，另一个节点既不使能也不运动，它的帧（如果驱动器自发TPDO/心跳）'
             '仍会出现在can-frames.log里，但不计入本节点的SDO配对统计。', '',
             '## 按CAN帧类型', '', '| 类型 | TX | RX |', '|---|---:|---:|']
    for kind, counts in sorted(analysis.get('frame_types', {}).items()):
        rows.append(f'| {kind} | {counts.get("TX", 0)} | {counts.get("RX", 0)} |')
    rows += ['', '## 按SDO对象', '', '| 对象 | 读请求 | 写请求 | 应答 | Abort | 平均 / p95 / 最大延迟（ms） |',
             '|---|---:|---:|---:|---:|---|']
    for obj, item in sorted(analysis.get('objects', {}).items()):
        latency = item['latency_ms']
        detail = f'{latency["avg"]} / {latency["p95"]} / {latency["max"]}' if latency else '未获得'
        rows.append(f'| {obj} | {item["reads"]} | {item["writes"]} | {item["replies"]} | '
                    f'{item["abort"]} | {detail} |')
    rows += ['', '## 异常与判读', '']
    flags = []
    for label, key in FIELDS:
        if key not in ['heartbeat_tx', 'heartbeat_max_gap_ms', 'node_heartbeat_rx'] and comm.get(key, 0):
            flags.append(f'{label}：{comm[key]}。')
    if comm.get('kernel_rx_dropped', 0):
        flags.append(f'SocketCAN 接收队列明确丢弃：{comm["kernel_rx_dropped"]}帧。')
    if analysis.get('pending_requests'):
        flags.append(f'日志末尾有{len(analysis["pending_requests"])}条SDO未匹配确认，详见analysis.json。')
    if analysis.get('duplicate_requests'):
        flags.append(f'同对象旧请求未匹配确认即出现后续请求{len(analysis["duplicate_requests"])}次，'
                     '详见analysis.json。')
    if result['outcome'] != 'completed':
        flags.append('测试未正常完成：' + (result.get('reason') or result['outcome']))
    if not result.get('disabled_confirmed'):
        flags.append('未完整确认停止并失能，不能当作安全结束。')
    rows.extend('- ' + safe(x) for x in flags)
    if not flags:
        rows.append('- 本次已记录范围内未发现上述发送失败、SDO重发/超时/拒绝、心跳超时或可观测的丢弃异常。')
    rows += [f'- 报警对象0x260E读回分布：`{json.dumps(analysis.get("alarm_counts", {}), ensure_ascii=False)}`。',
             '- 使能前或退出后出现0x0020可能与主站尚未发送或已停止心跳有关；运行中的报警以controller.log和test.log的时序为准。',
             '- 节点心跳为0可能是驱动未配置心跳生产者，不能单独据此认定丢帧。', '',
             '## can0 接口计数增量', '', '| 项目 | 增量 |', '|---|---:|']
    delta = result.get('interface_delta') or {}
    if delta:
        for key, label in INTERFACE:
            if key in delta:
                rows.append(f'| {label}（{key}） | {delta[key]} |')
    else:
        rows.append('| 未取得 | — |')
    rows += ['', '接口包计数与节点过滤器的覆盖范围不同，不能把接口计数与SDO配对数的差值判成丢帧。', '',
             '## 文件说明', '',
             '- `can-frames.log`：逐帧发送、接收、错误、驱动统计及中文通信摘要，实时写入。',
             '- `controller.log`：原控制程序完整输出和实际发送的命令。',
             '- `test.log` / `launcher.log`：本次动作进度、约每分钟通信摘要和即时中文异常。',
             '- `progress.json`：最近一次完成动作；`summary.json`：本次最终结果与全部动作记录。',
             '- `coordinator.json`：本次配置、进程与总体状态；`analysis.json`：SDO配对、延迟及异常帧明细。',
             '- 本中文总结在本次测试目录和 `/root/yz_aim/logs` 根目录各保存一份。', '']
    content = '\n'.join(rows)
    targets = []
    directory = Path(job['directory'])
    for destination in [directory / filename, directory.parent / filename]:
        destination.write_text(content, encoding='utf-8')
        targets.append(destination)
    return targets


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('coordinator', type=Path, help='本次目录下的coordinator.json，可离线重新生成报告')
    args = parser.parse_args()
    for path in generate_reports(json.loads(args.coordinator.read_text(encoding='utf-8'))):
        print(path)
