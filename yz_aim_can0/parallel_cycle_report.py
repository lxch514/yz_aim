#!/usr/bin/env python3
"""从本次两路summary和CAN原始帧生成中文报告；不控制设备。"""
import argparse
from datetime import datetime
import json
from pathlib import Path

from parallel_trace_analysis import distribution, parse_trace


def shown(value):
    return '未获得' if value is None else str(value)


def safe(value):
    return str(value).replace('|', '\\|').replace('\r', '').replace('\n', '<br>')


def latency_text(value):
    if not value:
        return '未获得'
    return (f'{value["min"]} / {value["avg"]} / {value["p50"]} / '
            f'{value["p95"]} / {value["p99"]} / {value["max"]}')


def load_job(job):
    directory = Path(job['directory'])
    summary = directory / 'summary.json'
    result = json.loads(summary.read_text(encoding='utf-8')) if summary.is_file() else {
        'outcome': '结果未完整生成', 'reason': '未找到summary.json；请查看launcher.log/controller.log',
        'disabled_confirmed': False, 'actions': []}
    trace = directory / 'can-frames.log'
    analysis = parse_trace(trace.read_bytes(), job['node']) if trace.is_file() else {}
    actions = result.get('actions', [])
    analysis['motion'] = {
        'completed_actions': len(actions),
        'forward_count': sum(x['signed_motor_rpm'] > 0 for x in actions),
        'reverse_count': sum(x['signed_motor_rpm'] < 0 for x in actions),
        'position_direction_observed_actions': sum((x.get('progress_position_delta') or 0)
                                                  * x['signed_motor_rpm'] > 0 for x in actions),
        'speed_direction_observed_actions': sum(
            (x.get('measured_speed_max_rpm') or 0) > x['signed_motor_rpm'] * .5
            if x['signed_motor_rpm'] > 0 else
            (x.get('measured_speed_min_rpm') or 0) < x['signed_motor_rpm'] * .5 for x in actions),
        'still_gate_retries': sum(x.get('still_retries', 0) for x in actions),
        'action_wall_seconds': distribution([x['command_wall_seconds'] for x in actions]),
        'confirmed_forward_hold_seconds': distribution(
            [x['confirmed_hold_seconds'] for x in actions
             if x['signed_motor_rpm'] > 0 and x.get('confirmed_hold_seconds', 0) > 0])}
    (directory / 'analysis.json').write_text(json.dumps(analysis, ensure_ascii=False, indent=2), encoding='utf-8')
    return result, analysis


def generate_reports(state):
    jobs = state['jobs']
    loaded = {name: load_job(job) for name, job in jobs.items()}
    settings = state['settings']
    minutes = settings['minutes']
    stamp = datetime.strptime(state['stamp'], '%Y%m%d-%H%M%S')
    # Seconds avoid overwriting a previous report if two runs start in one minute.
    filename = f'YZ-AIM_两轴通信统计_{stamp:%Y%m%d-%H%M%S}_{minutes:g}分钟.md'
    rows = [f'# YZ-AIM 两路单轴并行通信测试（{minutes:g}分钟）', '',
            f'- 测试开始：{state["started_at"]}；结束：{state.get("ended_at", "尚未结束")}。',
            f'- 总体状态：`{state["phase"]}`。' + (f' 原因：{safe(state["reason"])}' if state.get('reason') else ''),
            f'- 设定：两路各自正转 **+{settings["rpm"]} RPM {settings["move_ms"]} ms** → '
            f'确认停止并保持 **{settings["stop_ms"]} ms** → 反转 **−{settings["rpm"]} RPM {settings["move_ms"]} ms** '
            '→ 确认停止 → 下一组。反转后的下一组不另加1秒保持。',
            '- 使用各自已经验证的单轴控制程序，发送`jog 正/负RPM 毫秒`，两路并行、各自按反馈推进；'
            '不作两轴位置或相位同步，不修改零点或电机节点号。',
            '- 30分钟为循环经过时间，包含启停、通信、静止检查和停止保持；达到时长后完成最后一组并停止失能，'
            '实际时间可能稍长。收发统计包含只读检查、使能和清理阶段。', '',
            '## 运行与动作反馈', '',
            '| 项目 | CAN0 | USB-CAN |', '|---|---|---|']

    def table_row(label, getter):
        values = []
        for name in ['can0', 'usb']:
            if name in loaded:
                result, analysis = loaded[name]
                values.append(safe(getter(name, result, analysis)))
            else:
                values.append('未启动')
        rows.append('| ' + label + ' | ' + ' | '.join(values) + ' |')

    table_row('寻址节点', lambda n, r, a: jobs[n]['node'])
    table_row('程序结果', lambda n, r, a: r['outcome'])
    table_row('循环经过时间（秒）', lambda n, r, a: shown(r.get('wall_seconds')))
    table_row('完成往返组数 / 动作次数', lambda n, r, a: f'{r.get("completed_cycles", 0)} / {r.get("completed_actions", 0)}')
    table_row('正转 / 反转次数', lambda n, r, a: f'{a["motion"]["forward_count"]} / {a["motion"]["reverse_count"]}')
    table_row('累计命令运动时间（秒）', lambda n, r, a: r.get('commanded_motion_seconds', 0))
    table_row('已确认停止并失能', lambda n, r, a: '是' if r.get('disabled_confirmed') else '未完整确认')
    table_row('反馈中观察到正确转速方向的动作数', lambda n, r, a: a['motion']['speed_direction_observed_actions'])
    table_row('反馈中观察到正确位置变化方向的动作数', lambda n, r, a: a['motion']['position_direction_observed_actions'])
    table_row('实际转速最小 / 最大（RPM）', lambda n, r, a: f'{shown(a.get("speed_min_rpm"))} / {shown(a.get("speed_max_rpm"))}')
    table_row('位置首值 / 末值（电机脉冲）', lambda n, r, a: f'{shown(a.get("position_first"))} / {shown(a.get("position_last"))}')
    table_row('正转后停止保持最小值（秒）', lambda n, r, a: shown((a['motion']['confirmed_forward_hold_seconds'] or {}).get('min')))
    table_row('静止门禁拒绝后的重新确认次数', lambda n, r, a: a['motion']['still_gate_retries'])
    table_row('失败 / 中断原因', lambda n, r, a: r.get('reason') or '无')
    rows += ['', '转速与位置来自驱动反馈；只看到发送运动命令不能证明电机已转动。'
             '若某项反馈计数不足，只表示未获得足够采样证据，不能补写成动作成功。', '',
             '## CAN收发、SDO与心跳', '', '| 项目 | CAN0 | USB-CAN |', '|---|---|---|']
    table_row('应用层记录TX / RX（帧）', lambda n, r, a: f'{a.get("directions", {}).get("TX", 0)} / {a.get("directions", {}).get("RX", 0)}')
    table_row('SDO请求 / 应答（帧）', lambda n, r, a: f'{a.get("frame_types", {}).get("SDO请求", {}).get("TX", 0)} / {a.get("frame_types", {}).get("SDO应答", {}).get("RX", 0)}')
    fields = [('发送失败（次）', 'tx_failed'), ('CAN错误帧（帧）', 'can_error_frames'),
              ('SDO读请求重发迹象（次）', 'sdo_read_repeats_observed'),
              ('SDO重复写请求迹象（次）', 'sdo_repeated_writes_observed'),
              ('SDO监测未确认超时（500ms阈值，次）', 'sdo_unconfirmed_timeouts'),
              ('SDO Abort拒绝（次）', 'sdo_aborts'), ('主站心跳发送（帧）', 'heartbeat_tx'),
              ('主站心跳最大有效间隔（ms）', 'heartbeat_max_gap_ms'),
              ('主站心跳监测超时（次）', 'heartbeat_timeouts'), ('节点心跳接收（帧）', 'node_heartbeat_rx'),
              ('诊断队列记录丢弃（条）', 'trace_records_dropped'), ('日志输出异常（次）', 'trace_output_errors')]
    for label, key in fields:
        table_row(label, lambda n, r, a, key=key: shown(r.get('communication', {}).get(key)))
    table_row('套接字接收队列明确丢弃（帧）', lambda n, r, a: shown(r.get('communication', {}).get('kernel_rx_dropped')) if n == 'can0' else '厂商SDK不提供该计数')
    table_row('日志末尾仍待确认的SDO请求（条）', lambda n, r, a: len(a.get('pending_requests', [])))
    table_row('未找到前置请求的SDO应答（条）', lambda n, r, a: len(a.get('unmatched_responses', [])))
    table_row('SDO延迟 min / avg / p50 / p95 / p99 / max（ms）', lambda n, r, a: latency_text(a.get('latency_ms')))
    rows += ['', 'TX与RX不必相等：发送包含主站心跳与SDO请求，接收还可能包含TPDO和节点心跳。'
             '应比较同节点、同对象的SDO请求与确认，而不是直接把总收发数量相减当作丢帧。', '',
             'SDO读重发表示上一读请求在控制程序的等待窗口内未获得确认，可能是请求/应答未到、延迟或接收处理问题；'
             '仅凭应用日志不能确认哪一段发生物理CAN丢帧。统计监测的500ms阈值独立于控制程序自身超时，'
             '因此“读重发”和“监测超时”计数可以不同。', '',
             'USB SDK发送成功表示调用被接受；该型号接口未提供可用的硬件丢帧/CRC/自动重发计数。'
             'USB一栏不能用零值证明底层无丢帧；CAN0可额外核对下面的内核计数。', '']
    if 'can0' in loaded:
        result, _ = loaded['can0']
        rows += ['### CAN0内核接口计数', '', '| 计数器 | 测试前 | 测试后 | 增量 |', '|---|---:|---:|---:|']
        for key in ['rx_packets', 'tx_packets', 'rx_errors', 'tx_errors', 'rx_dropped', 'tx_dropped', 'rx_missed_errors']:
            rows.append(f'| {key} | {shown(result.get("interface_before", {}).get(key))} | '
                        f'{shown(result.get("interface_after", {}).get(key))} | '
                        f'{shown(result.get("interface_delta", {}).get(key))} |')
        rows += ['', '接口计数覆盖整个接口，若同时有其他程序使用该接口也会计入；负增量表示计数器可能被重置。', '']
    for name, job in jobs.items():
        result, analysis = loaded[name]
        rows += [f'## {"CAN0" if name == "can0" else "USB-CAN"}：节点{job["node"]}明细', '',
                 f'日志目录：`{job["directory"]}`', '',
                 '### 按CAN帧类型', '', '| 类型 | TX | RX |', '|---|---:|---:|']
        for kind, counts in sorted(analysis.get('frame_types', {}).items()):
            rows.append(f'| {kind} | {counts.get("TX", 0)} | {counts.get("RX", 0)} |')
        rows += ['', '### 按SDO对象', '', '| 对象 | 读请求 | 写请求 | 应答 | Abort | 平均 / p95 / 最大延迟（ms） |',
                 '|---|---:|---:|---:|---:|---|']
        for obj, item in sorted(analysis.get('objects', {}).items()):
            latency = item['latency_ms']
            detail = f'{latency["avg"]} / {latency["p95"]} / {latency["max"]}' if latency else '未获得'
            rows.append(f'| {obj} | {item["reads"]} | {item["writes"]} | {item["replies"]} | {item["abort"]} | {detail} |')
        rows += ['', '### 异常与判读', '']
        flags = []
        comm = result.get('communication', {})
        for label, key in fields:
            if key not in ['heartbeat_tx', 'heartbeat_max_gap_ms', 'node_heartbeat_rx'] and comm.get(key, 0):
                flags.append(f'{label}：{comm[key]}。')
        if name == 'can0':
            if comm.get('kernel_rx_dropped', 0):
                flags.append(f'套接字接收队列明确丢弃{comm["kernel_rx_dropped"]}帧。')
            for key in ['rx_errors', 'tx_errors', 'rx_dropped', 'tx_dropped', 'rx_missed_errors']:
                if result.get('interface_delta', {}).get(key, 0):
                    flags.append(f'CAN0内核{key}增量={result["interface_delta"][key]}。')
        if analysis.get('pending_requests'):
            flags.append(f'日志末尾有{len(analysis["pending_requests"])}条SDO未匹配确认，详见analysis.json。')
        if analysis.get('duplicate_requests'):
            flags.append(f'同对象旧请求未匹配确认即出现后续请求{len(analysis["duplicate_requests"])}次，详见analysis.json。')
        if result['outcome'] != 'completed':
            flags.append('测试未正常完成：' + (result.get('reason') or result['outcome']))
        if not result.get('disabled_confirmed'):
            flags.append('未完整确认停止并失能，不能当作安全结束。')
        rows.extend('- ' + safe(x) for x in flags)
        if not flags:
            rows.append('- 本次已记录范围内未发现上述发送失败、SDO重发/超时/拒绝、心跳超时或可观测的丢弃异常。')
        rows += [f'- 报警对象0x260E读回分布：`{json.dumps(analysis.get("alarm_counts", {}), ensure_ascii=False)}`。',
                 '- 使能前或退出后出现0x0020可能与主站尚未发送/已停止心跳有关；运行中的报警以controller.log和test.log的时序为准。',
                 '- 节点心跳为0可能是驱动未配置心跳生产者，不能单独据此认定丢帧。', '']
    rows += ['## 文件说明', '',
             '- `can-frames.log`：逐帧发送、接收、错误、驱动统计及中文通信摘要，实时写入。',
             '- `controller.log`：原控制程序完整输出和实际发送的命令。',
             '- `test.log` / `launcher.log`：本路动作进度、约每分钟通信摘要和即时中文异常。',
             '- `progress.json`：最近一次完成动作；`summary.json`：本路最终结果与全部动作记录。',
             '- `coordinator.json`：两路配置、进程与总体状态；`analysis.json`：SDO配对、延迟及异常帧明细。',
             '- 本中文总结在两路本次测试目录和两个项目的logs根目录各保存一份。', '']
    content = '\n'.join(rows)
    targets = []
    for job in jobs.values():
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
