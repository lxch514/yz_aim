#!/usr/bin/env python3
"""读取已结束的双轴测试目录，生成中文Markdown和可复核JSON，不发送CAN帧。"""
import argparse,csv,hashlib,json,re,sys
from pathlib import Path
sys.dont_write_bytecode=True
from dual_trace_analysis import parse_trace,distribution

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path);args=parser.parse_args()
 root=Path(__file__).resolve().parent;d=args.directory.resolve()
 if not d.is_relative_to(root/'logs'):parser.error('只能读取本项目logs目录')
 summary=json.loads((d/'summary.json').read_text(encoding='utf-8'));raw=(d/'can-frames.log').read_bytes()
 parsed={str(n):parse_trace(raw,n) for n in [1,2]};communication=summary.get('communication',{})
 assert parsed['1']['directions']['TX']==communication['tx'] and parsed['1']['directions']['RX']==communication['rx'],'收发统计与逐帧日志不一致'
 allrows=[];stable=[];timing=[];mode_errors=[]
 for action in summary['actions']:
  file=Path(action['csv']);assert file.resolve().is_relative_to(d)
  text=file.read_text(encoding='utf-8');rows=list(csv.DictReader(line for line in text.splitlines() if not line.startswith('#')))
  allrows.extend(rows);stable.extend(row for row in rows if row['event']=='sample' and row['quality']=='ok')
  match=re.search(r'trigger_a_ms=(-?\d+) trigger_b_ms=(-?\d+) stop_request_a_ms=(-?\d+) stop_request_b_ms=(-?\d+)',text)
  if match:
   ta,tb,sa,sb=map(int,match.groups());timing.append({'csv':str(file),'trigger_gap_ms':abs(ta-tb),'stop_gap_ms':abs(sa-sb),'requested_ms':summary['settings']['move_ms'],'stop_request_a_ms':sa,'stop_request_b_ms':sb})
  if 'mode=equal_speed' not in text.splitlines()[0]:mode_errors.append(str(file))
 rpm=summary['settings']['rpm'];diff=[abs(float(r['actual_a_rpm'])+float(r['actual_b_rpm'])) for r in stable]
 bad_target=[r for r in allrows if float(r['target_a_rpm'])+float(r['target_b_rpm'])!=0 or abs(float(r['target_a_rpm']))!=rpm]
 motion={'csv_files':len(summary['actions']),'all_rows':len(allrows),'steady_rows':len(stable),'target_mismatch_rows':len(bad_target),'wrong_mode_csv':mode_errors,
  'steady_speed_difference_rpm':distribution(diff),'steady_abs_rpm_a':distribution([abs(float(r['actual_a_rpm'])) for r in stable]),
  'steady_abs_rpm_b':distribution([abs(float(r['actual_b_rpm'])) for r in stable]),'all_abs_rpm_a':distribution([abs(float(r['actual_a_rpm'])) for r in allrows]),'all_abs_rpm_b':distribution([abs(float(r['actual_b_rpm'])) for r in allrows]),
  'maximum_recorded_position_error_deg':max((abs(float(r['error_output_deg'])) for r in allrows),default=None),
  'trigger_gap_ms':distribution([x['trigger_gap_ms'] for x in timing]),'stop_gap_ms':distribution([x['stop_gap_ms'] for x in timing]),
  'stop_request_a_ms':distribution([x['stop_request_a_ms'] for x in timing]),'stop_request_b_ms':distribution([x['stop_request_b_ms'] for x in timing]),
  'still_check_retries':sum(x.get('still_retries',0) for x in summary['actions']),'action_timing':timing}
 analysis={'nodes':parsed,'motion':motion,'trace_sha256':hashlib.sha256(raw).hexdigest()}
 (d/'analysis.json').write_text(json.dumps(analysis,ensure_ascii=False,indent=2),encoding='utf-8')
 stamp=summary['started_at'].replace('-','').replace(':','').replace('T','-')[:13]
 minutes=summary['settings']['minutes'];duration_label=f'{minutes:g}分钟' if not summary['settings'].get('cycles') else '短时验证'
 filename=f'YZ-AIM_两轴通信统计_{stamp}_CAN0双轴_{duration_label}.md';lines=[]
 def add(s=''):lines.append(s)
 def table(headers,rows):
  add();add('| '+' | '.join(headers)+' |');add('|'+'|'.join(['---']*len(headers))+'|')
  for row in rows:add('| '+' | '.join(str(x) for x in row)+' |')
 add('# YZ-AIM CAN0两轴等速测试与通信统计');add()
 add(f"测试：{summary['started_at']} → {summary['ended_at']}（北京时间）；结果`{summary['outcome']}`，实际{summary.get('wall_seconds','—')}秒，{summary['completed_cycles']}组/{summary['completed_actions']}次动作，双轴失能确认={summary['disabled_confirmed']}。")
 if summary.get('reason'):add();add('结束原因：'+summary['reason'])
 add();add('## 条件与判断范围');add()
 add(f'- 程序`/root/yz_aim_can0_double`；两个节点共同连接CAN0，经典CAN 1 Mbit/s；节点1为A、节点2为B。伸出A负/B正，缩进A正/B负；目标幅值{rpm}电机RPM，每方向{summary["settings"]["move_ms"]}ms。')
 add('- 使用`sync-mode speed`：下发等幅反向目标速度，不用位置差修正速度，不因累计位置差停止；位置反馈仍记录。原位置同步模式及其保护设置保留。')
 add('- 保留驱动报警、限位、方向/超速、通信及反馈失效保护。不存在无条件不停机保证。目标速度相同不等于实际速度每个采样点严格相等，实测差值如下。')
 add('- 控制和反馈使用SDO；两节点分别接收自己的SDO/TPDO/心跳，共享一个主站心跳生产者。启动目标和控制字分别连续发送，仍存在总线串行时间，非PDO+SYNC硬同步。')
 add('- 两节点写确认同时轮询；仅速度模式启动0x6040=0x000F确认缺失时，允许在原运动截止时间内，通过两轴模式、目标、状态、报警、输出及实际转速全部读回验证来确认已启动，不重发启动命令。任何读回失败、不符或SDO Abort仍停止。')
 add(f"- 启动ACK缺失但读回验证通过的动作{summary.get('start_readback_verified_actions',sum(x.get('start_confirmation')=='readback_verified' for x in summary['actions']))}次；启动确认失败{summary.get('start_confirmation_failures',int('速度触发未完整确认' in summary.get('reason','')))}次。启动等待窗口约300ms，独立监测SDO未确认阈值500ms，两者不能混为一项；提前退出时也要看结束待确认数与控制器失败原因。")
 add('- 为避免同一节点应答后立即密集请求，SDO事务之间增加至少10ms间隔；保持心跳服务，直接停止帧不受此间隔延迟。读请求有限重试，写请求不自动重放。')
 add('- 动作时间从两轴目标下发前计起；到时先向两轴发停止帧，再确认停稳。运动结束的物理减速、状态检查、静止等待包含在墙钟时间内。')
 add('- 运行前仅记录一次软件位置基准，没有写入硬件零点。每约60秒输出中文通信统计，异常即时记录，不向控制输入终端刷原始帧。')
 add();add('## 速度、位置与动作时序')
 table(['项目','实测'],[['完成动作',summary['completed_actions']],['等幅反向目标不符的CSV行',motion['target_mismatch_rows']],['模式不符的CSV',len(mode_errors)],['稳态反馈样本',len(stable)],['A实测速度幅值RPM',motion['steady_abs_rpm_a']],['B实测速度幅值RPM',motion['steady_abs_rpm_b']],['两轴实际速度差RPM',motion['steady_speed_difference_rpm']],['两轴触发间隔ms',motion['trigger_gap_ms']],['两轴停止帧间隔ms',motion['stop_gap_ms']],['A停止请求时刻ms',motion['stop_request_a_ms']],['B停止请求时刻ms',motion['stop_request_b_ms']],['最大记录位置差（输出度，只记录）',motion['maximum_recorded_position_error_deg']],['运动前静止检查重试',motion['still_check_retries']]])
 add();add('稳态样本仅统计CSV中`sample`且反馈质量为`ok`的记录，排除启动和停止阶段；短时低速验证可能没有稳态样本。位置是电机反馈，不能代替机械绝对零点/行程测量。CSV时序为应用观测时间，非硬件同步时间戳。')
 add();add('## CAN收发与心跳')
 table(['指标','本轮值'],[[k,communication.get(k,'未取得')] for k in ['tx','rx','tx_failed','can_error_frames','kernel_rx_dropped','trace_records_dropped','trace_output_errors','heartbeat_tx','heartbeat_max_gap_ms','heartbeat_timeouts','sdo_read_repeats_observed','sdo_unconfirmed_timeouts','sdo_aborts','sdo_pending_at_end']])
 table(['接口统计增量','本轮值'],sorted(summary.get('interface_delta',{}).items()))
 add();add('TX含SDO请求和主站心跳，RX含SDO应答、TPDO及节点心跳，TX和RX无需相等。接口包计数和两套节点过滤器的覆盖范围不同，不能把两者差值判成丢帧。SocketCAN明确丢弃、CAN错误和SDO配对分别报告；不能据SDO全成功证明物理CAN绝对无丢帧。硬件CRC错误和自动重发没有逐帧独立计数。')
 for node,p in parsed.items():
  add();add(f'## 节点{node} SDO与反馈')
  table(['指标','值'],[['SDO请求',sum(x['reads']+x['writes'] for x in p['objects'].values())],['SDO应答',sum(x['replies'] for x in p['objects'].values())],['重复对象请求',len(p['duplicate_requests'])],['未配对请求',len(p['pending_requests'])],['无唯一应答请求',p['requests_without_unique_reply']],['无在途请求的应答',len(p['unmatched_responses'])],['日志观测SDO延迟ms',p['latency_ms']],['反馈转速范围RPM',f"{p['speed_min_rpm']} 至 {p['speed_max_rpm']}"],['初末位置脉冲',f"{p['position_first']} → {p['position_last']}"],['报警码读取次数',p.get('alarm_counts',{})]])
  table(['对象','读','写','应答','Abort','延迟中位ms','P95ms','最大ms'],[[obj,v['reads'],v['writes'],v['replies'],v['abort'],(v['latency_ms'] or {}).get('p50','—'),(v['latency_ms'] or {}).get('p95','—'),(v['latency_ms'] or {}).get('max','—')] for obj,v in sorted(p['objects'].items(),key=lambda x:-(x[1]['reads']+x[1]['writes']))])
  add();add('非零报警读数时间和值：`'+json.dumps(p.get('nonzero_alarms',[]),ensure_ascii=False)+'`。退出停发主站心跳后可能出现0x0020，应与运行期间报警分开看。')
 previous=[]
 for file in sorted((root/'logs').glob('parallel-30min-*/summary.json')):
  if file.parent==d:continue
  try:old=json.loads(file.read_text(encoding='utf-8'))
  except (OSError,ValueError):continue
  if old.get('settings',{}).get('mode')!='equal_speed' or old.get('started_at','')>=summary['started_at'] or old.get('started_at','')[:10]!=summary['started_at'][:10]:continue
  c=old.get('communication',{})
  previous.append([str(file.parent),old.get('outcome'),old.get('completed_actions'),c.get('sdo_read_repeats_observed','—'),c.get('sdo_pending_at_end','—'),old.get('disabled_confirmed')])
 if previous:
  add();add('## 此前准备验证记录（不混入本轮统计）')
  table(['目录','结果','完成动作','SDO读重发','结束待确认SDO','该会话失能确认'],previous)
  add();add('最初短时验证在运动前出现节点2位置读取无应答、读重试并触发共享通信保护，未开始运动。该会话未能完整确认失能，随后另开控制会话已确认两轴停止、失能；之后增加SDO事务间隔并重新测试。保留失败记录，不能将后续通过结果写成全过程从未异常。')
 add();add('## 日志、结论与复现');add()
 add(f'- 全部记录：`{d}`。`can-frames.log`是原始TX/RX；`controller.log`是完整命令过程；`test.log`是中文过程/分钟统计；每个`motion-*.csv`是两轴速度、位置、时序记录。')
 add('- `summary.json`保存最终结果，`analysis.json`保存逐帧配对和CSV统计。本次仅对实际完成时长下结论，1分钟通过不写成30分钟通过。')
 faults=sum(communication.get(k,0) for k in ['tx_failed','can_error_frames','kernel_rx_dropped','trace_records_dropped','trace_output_errors','sdo_unconfirmed_timeouts','sdo_aborts','heartbeat_timeouts'])
 add(f"- 本轮通信显式异常项目合计{faults}；两节点无唯一应答请求合计{sum(p['requests_without_unique_reply'] for p in parsed.values())}；目标等幅检查不符{len(bad_target)}行。")
 add();add('```text\n'+summary.get('can0_link_after','').rstrip()+'\n```')
 add();add('```sh\ncd /root/yz_aim_can0_double\npython3 -B dual_cycle_test.py --rpm 300 --move-ms 3000 --minutes 1\n# 收到30分钟测试指令后才运行：\n# python3 -B dual_cycle_test.py --rpm 300 --move-ms 3000 --minutes 30\n```')
 report=d/filename;report.write_text('\n'.join(lines)+'\n',encoding='utf-8',newline='\n')
 print(json.dumps({'report':str(report),'report_sha256':hashlib.sha256(report.read_bytes()).hexdigest(),'analysis':str(d/'analysis.json'),'motion':motion},ensure_ascii=False))

if __name__=='__main__':main()
