#!/usr/bin/env python3
"""CAN0双轴等幅速度往返；默认只测试1分钟，30分钟需显式指定。"""
import argparse,csv,fcntl,json,os,re,signal,subprocess,sys,time
from datetime import datetime
from pathlib import Path
sys.dont_write_bytecode=True
from dual_test_support import Controller,Communications,Reporter,TestFailure,interface_counters

def atomic(path,value):
 temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8');os.replace(temp,path)

def settled(controller):
 end=time.monotonic()+3;quiet=0
 while time.monotonic()<end:
  response=controller.command('sync-status',allow_failure=True)
  speeds=[float(x) for x in re.findall(r'转速\s+(-?[\d.]+)\s+电机RPM',response)]
  if '[失败]' in response or len(speeds)!=2:raise TestFailure('双轴静止状态读取失败：'+response[-1800:])
  quiet=quiet+1 if all(abs(x)<=0.5 for x in speeds) else 0
  if quiet>=2:return
  time.sleep(.05)
 raise TestFailure('两轴在3秒内未连续确认静止，未发送下一条运动命令')

def read_action(path):
 text=path.read_text(encoding='utf-8');metadata=text.splitlines()[0]
 rows=list(csv.DictReader(line for line in text.splitlines() if not line.startswith('#')))
 stop=re.search(r'stop_request_a_ms=(-?\d+) stop_request_b_ms=(-?\d+)',text)
 end=re.findall(r'^# end_reason=(.*)$',text,re.M)
 target=re.search(r'nominal_rpm=(-?\d+)',metadata)
 return {'csv':str(path),'samples':len(rows),'nominal_rpm':int(target[1]) if target else None,
  'start_confirmation':'readback_verified' if '# start_confirmation=readback_verified_after_missing_ack' in text else 'acknowledged',
  'stop_request_a_ms':int(stop[1]) if stop else None,'stop_request_b_ms':int(stop[2]) if stop else None,
  'end_reason':end[-1] if end else '', 'maximum_recorded_position_error_deg':max((abs(float(x['error_output_deg'])) for x in rows),default=None)}

def main():
 root=Path(__file__).resolve().parent;parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--rpm',type=int,default=300);parser.add_argument('--move-ms',type=int,default=3000)
 parser.add_argument('--minutes',type=float,default=1);parser.add_argument('--cycles',type=int)
 parser.add_argument('--log-dir',type=Path);parser.add_argument('--dry-run',action='store_true');args=parser.parse_args()
 if not(1<=args.rpm<=300 and 1<=args.move_ms<=5000 and 0<args.minutes<=30 and (args.cycles is None or 1<=args.cycles<=1000)):parser.error('RPM 1~300，单次1~5000ms，时长0~30分钟')
 if args.dry_run:print(f'CAN0节点1/2：extend {args.rpm} {args.move_ms}，retract {args.rpm} {args.move_ms}；'+(f'{args.cycles}组' if args.cycles else f'{args.minutes}分钟')+'；位置差只记录。');return 0
 directory=args.log_dir or root/'logs'/('parallel-30min-'+datetime.now().strftime('%Y%m%d-%H%M%S'))
 if not directory.is_absolute():directory=root/'logs'/directory
 if directory.resolve()==(root/'logs').resolve() or not directory.resolve().is_relative_to((root/'logs').resolve()):parser.error('日志目录必须在本项目logs下')
 directory.mkdir(parents=True,exist_ok=True)
 if any((directory/n).exists() for n in ['test.log','summary.json','controller.log','can-frames.log']):parser.error('目录已有测试结果，请使用新目录')
 runtime=root/'.runtime';runtime.mkdir(exist_ok=True);lock=(runtime/'dual-cycle-test.lock').open('a')
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 report=Reporter(directory/'test.log');controller=comm=None;started=None;actions=[];cycles=0;disabled=False
 result={'started_at':datetime.now().isoformat(timespec='seconds'),'run_directory':str(directory),
  'settings':{'rpm':args.rpm,'move_ms':args.move_ms,'minutes':args.minutes,'cycles':args.cycles,'node_a':1,'node_b':2,'mode':'equal_speed','position_guard':False},'outcome':'not_started','reason':''}
 before=interface_counters('can0');link_before=subprocess.run(['ip','-details','-statistics','link','show','can0'],capture_output=True,text=True).stdout
 try:
  report('双轴等速测试：CAN0节点1负/节点2正为伸出；位置差只记录，不因位置差调速或停机。')
  controller=Controller([str(root/'build/yz_aim_can0'),'--if','can0','--node','1','--node2','2','--timeout','3000','--log-dir',str(directory)],root,directory/'controller.log')
  startup=controller.receive();found=re.search(r'通讯详细日志："?([^"\r\n]+)',startup)
  if not found or Path(found[1].strip()).resolve()!=(directory/'can-frames.log').resolve():raise TestFailure('未取得本测试目录内的CAN日志，拒绝运动')
  comm=Communications(directory/'can-frames.log',report);controller.on_idle=comm.poll
  response=controller.command('sync-mode speed')
  if '已选等速测试模式' not in response:raise TestFailure('未确认进入等速测试模式')
  controller.command('sync-dir 2 1');settled(controller)
  response=controller.command('sync-enable')
  if '两轴已使能' not in response:raise TestFailure('双轴使能未完整确认：'+response[-1800:])
  settled(controller);zero=controller.command('sync-zero')
  if '固定软件零点' not in zero:raise TestFailure('未确认日志基准：'+zero[-1800:])
  result['software_reference']=zero;started=time.monotonic()
  while (cycles<args.cycles if args.cycles is not None else time.monotonic()-started<args.minutes*60):
   for name in ['extend','retract']:
    settled(controller);retries=0;began=time.monotonic();before_csv=set(directory.glob('motion-*.csv'))
    while True:
     before_csv=set(directory.glob('motion-*.csv'))
     report(f'第{cycles+1}组：{name} {args.rpm} {args.move_ms}，两轴一起动作。')
     response=controller.command(f'{name} {args.rpm} {args.move_ms}',timeout=args.move_ms/1000+30,allow_failure=True)
     if '到时正常停止。' in response:break
     # Only retry a pre-motion stationary refusal; never replay a started/failed move.
     if ('motor is moving' in response and '名义电机RPM' not in response and retries<3):
      retries+=1;report(f'运动前静止检查重试{retries}/3，先等待两轴静止。');settled(controller);time.sleep(.1);continue
     raise TestFailure('双轴指令未正常完成：'+response[-2800:])
    new_csv=set(directory.glob('motion-*.csv'))-before_csv
    if len(new_csv)!=1:raise TestFailure('未取得唯一完整动作CSV')
    item=read_action(new_csv.pop());item.update(command=name,wall_seconds=round(time.monotonic()-began,3),still_retries=retries)
    if item['start_confirmation']=='readback_verified':report('通信提示：本次启动确认帧缺失，已通过两轴状态与实际转速读回验证；未重复发送启动命令。')
    if item['end_reason']!='requested duration elapsed' or '两轴已确认停止' not in response:raise TestFailure('动作结束原因或双轴停稳未完整确认：'+response[-2500:])
    actions.append(item);atomic(directory/'progress.json',{'cycles':cycles,'actions':len(actions),'elapsed_seconds':round(time.monotonic()-started,3),'latest':item})
   cycles+=1;report(f'第{cycles}组完成，累计{time.monotonic()-started:.1f}秒。')
  result['outcome']='completed'
 except KeyboardInterrupt:result.update(outcome='interrupted',reason='收到中断，停止测试并清理');report(result['reason'])
 except Exception as exc:result.update(outcome='failed',reason=str(exc));report('异常：停止测试，不自动重放运动命令。'+str(exc))
 finally:
  if started is not None:result['wall_seconds']=round(time.monotonic()-started,3)
  if controller:disabled=controller.cleanup(report);result['controller_returncode']=controller.process.poll()
  if comm:
   time.sleep(.1);result['communication']=comm.result();comm.close()
  after=interface_counters('can0');result.update(completed_cycles=cycles,completed_actions=len(actions),actions=actions,disabled_confirmed=disabled,interface_before=before,interface_after=after,interface_delta={k:after[k]-before[k] for k in before.keys()&after.keys()},can0_link_before=link_before)
  result['start_readback_verified_actions']=sum(x.get('start_confirmation')=='readback_verified' for x in actions)
  result['start_confirmation_failures']=int('速度触发未完整确认' in result.get('reason',''))
  result['can0_link_after']=subprocess.run(['ip','-details','-statistics','link','show','can0'],capture_output=True,text=True).stdout
  if result['outcome']=='completed' and not disabled:result.update(outcome='failed',reason='循环完成但未确认双轴失能')
  result['ended_at']=datetime.now().isoformat(timespec='seconds');atomic(directory/'summary.json',result)
  report(f"测试结束：{result['outcome']}，{cycles}组/{len(actions)}次动作，双轴失能确认={disabled}。")
  if (directory/'can-frames.log').exists() and 'communication' in result:
   p=subprocess.run([sys.executable,'-B',str(root/'dual_test_report.py'),str(directory)],capture_output=True,text=True,timeout=60)
   (directory/'report-generation.log').write_text(p.stdout+p.stderr,encoding='utf-8')
   report('中文报告'+('生成成功。' if p.returncode==0 else '生成失败，请查看report-generation.log。'))
  report('结果：'+str(directory/'summary.json'));report.file.close()
 return 0 if result['outcome']=='completed' else 1

if __name__=='__main__':
 def interrupted(_signum,_frame):raise KeyboardInterrupt
 signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGHUP,interrupted)
 raise SystemExit(main())
