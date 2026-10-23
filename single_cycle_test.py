#!/usr/bin/env python3
"""独立链路单电机往返测试：不更改硬件零点，失败即停止，不自动重试。"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import pty
import re
import select
import signal
import subprocess
import sys
import termios
import time

sys.dont_write_bytecode = True
from can_monitor import Statistics

PROMPT = re.compile(rb'yz\([^()\r\n]*\)> ')
BAD_RESPONSE = ('[失败]', '[!] 双轴同步异常', '故障需重启', '未能确认', '写入失败')


class TestFailure(RuntimeError):
    pass


STILL_RPM_LIMIT = 0.5


def read_speed(controller):
    """读一次 0x606C（固定 2 字节，避免宽度试探产生 SDO 拒绝），返回电机转速 RPM；读失败返回 None。"""
    text = controller.command('read-od 606c 2', timeout=10, allow_failure=True)
    found = re.search(r'= 0x([0-9A-Fa-f]+)（', text)
    if not found:
        return None
    raw = int(found.group(1), 16) & 0xFFFF
    return (raw - 0x10000 if raw >= 0x8000 else raw) / 10.0


def settle(controller, timeout=2.5):
    """等驱动停机后的残余转速收敛：连续两次 0x606C 都不超过静止阈值才放行，返回最后一次读数。"""
    last = None
    quiet = 0
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rpm = read_speed(controller)
        if rpm is not None:
            last = rpm
            quiet = quiet + 1 if abs(rpm) <= STILL_RPM_LIMIT else 0
            if quiet >= 2:
                return rpm
        time.sleep(0.05)
    return last


class Reporter:
    def __init__(self, path):
        self.file = path.open('a', encoding='utf-8', buffering=1)

    def __call__(self, message):
        line = f'[{datetime.now():%H:%M:%S}] {message}'
        print(line, flush=True)
        self.file.write(line + '\n')


class Communications:
    def __init__(self, path, report):
        self.path = path
        self.report = report
        self.reader = path.open('r', encoding='utf-8', errors='replace')
        self.append = path.open('a', encoding='utf-8', buffering=1)
        self.statistics = Statistics(self.emit)
        self.next_summary = time.monotonic() + 60

    def emit(self, message):
        self.report(message)
        self.append.write(f'[CAN监测 {datetime.now():%Y-%m-%d %H:%M:%S}] {message}\n')
        self.append.flush()

    def poll(self, live=True):
        # Keep following the producer during each 3-second action, not only between commands.
        for _ in range(20000):
            position = self.reader.tell()
            line = self.reader.readline()
            if not line:
                break
            if not line.endswith('\n'):
                self.reader.seek(position)
                break
            self.statistics.feed(line)
        else:
            return  # never diagnose heartbeat silence before catching up with the log
        s = self.statistics
        if live and s.epoch is not None and s.pid and Path('/proc', str(s.pid)).exists():
            s.expire(max(0, time.time() - s.epoch))
        if time.monotonic() >= self.next_summary:
            s.summary()
            self.next_summary = time.monotonic() + 60

    def result(self):
        self.poll(live=False)
        s = self.statistics
        return {'tx': max(s.count['TX'], s.driver['TX']),
                'rx': max(s.count['RX'], s.driver['RX']),
                'tx_failed': max(s.count['TX_ERR'], s.driver['TX_ERR']),
                'heartbeat_tx': s.count['heartbeat_tx'],
                'heartbeat_max_gap_ms': round(s.heartbeat_gap_max * 1000, 3),
                'heartbeat_timeouts': s.count['heartbeat_timeout'],
                'sdo_read_repeats_observed': s.count['read_retry'],
                'sdo_unconfirmed_timeouts': s.count['sdo_timeout'],
                'sdo_aborts': s.count['sdo_abort'],
                'kernel_rx_dropped': max(s.count['kernel_drop'], s.driver['KERNEL_RX_DROP']),
                'trace_records_dropped': s.driver['TRACE_DROP'],
                'trace_output_errors': s.driver['OUTPUT_DROP'],
                'can_error_frames': s.count['RX_ERROR'],
                'sdo_pending_at_end': len(s.pending),
                'sdo_requests_by_node': dict(s.nodes_tx),
                'sdo_responses_by_node': dict(s.nodes_rx)}

    def close(self):
        self.poll(live=False)
        self.statistics.summary()
        self.reader.close()
        self.append.close()


class Controller:
    def __init__(self, command, root, transcript):
        self.master, slave = pty.openpty()
        settings = termios.tcgetattr(slave)
        settings[3] &= ~(termios.ECHO | termios.ECHONL)
        termios.tcsetattr(slave, termios.TCSANOW, settings)
        self.process = subprocess.Popen(command, cwd=root, stdin=slave, stdout=slave,
                                        stderr=slave, start_new_session=True, close_fds=True)
        os.close(slave)
        self.transcript = transcript.open('ab', buffering=0)
        self.buffer = bytearray()
        self.on_idle = lambda: None
        self.ready = False

    def receive(self, timeout=25, expect_exit=False):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.on_idle()
            if not expect_exit and PROMPT.search(self.buffer):
                text = self.buffer.decode('utf-8', errors='replace').replace('\r', '')
                self.buffer.clear()
                self.ready = True
                return text
            readable, _, _ = select.select([self.master], [], [], 0.1)
            if readable:
                try:
                    chunk = os.read(self.master, 65536)
                except OSError:
                    chunk = b''  # Linux PTY returns EIO once the child closes the slave.
                if chunk:
                    self.transcript.write(chunk)
                    self.buffer.extend(chunk)
                    continue
            if self.process.poll() is not None:
                text = self.buffer.decode('utf-8', errors='replace').replace('\r', '')
                self.buffer.clear()
                if expect_exit:
                    return text
                raise TestFailure(f'控制程序提前退出，返回码{self.process.returncode}：\n{text[-1800:]}')
        raise TestFailure('等待控制程序完成命令超时；将执行停止与失能清理。')

    def command(self, command, timeout=25, allow_failure=False):
        if not self.ready:
            raise TestFailure('上一命令尚未确认结束，拒绝继续发送运动命令。')
        self.ready = False
        self.transcript.write(('\n[脚本命令] ' + command + '\n').encode())
        os.write(self.master, (command + '\n').encode())
        text = self.receive(timeout, expect_exit=command == 'quit')
        if not allow_failure and any(marker in text for marker in BAD_RESPONSE):
            raise TestFailure(text[-2500:])
        return text

    def cleanup(self, report):
        confirmed = False
        try:
            if self.process.poll() is None and not self.ready:
                self.process.send_signal(signal.SIGINT)
                try:
                    self.receive(timeout=18)
                except TestFailure as exc:
                    report('停止等待提示：' + str(exc)[-1200:])
            if self.process.poll() is None and self.ready:
                response = self.command('disable', timeout=25, allow_failure=True)
                confirmed = ('[成功] 停止并失能输出' in response
                             and not any(marker in response for marker in BAD_RESPONSE))
                if not confirmed:
                    report('失能清理提示：' + response[-1800:])
                self.command('quit', timeout=25, allow_failure=True)
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGTERM)
                self.process.wait(timeout=25)
        except (TestFailure, OSError, subprocess.TimeoutExpired) as exc:
            report('异常：未完整确认控制进程清理：' + str(exc))
        finally:
            self.transcript.close()
            os.close(self.master)
        return confirmed


def interface_counters(interface):
    base = Path('/sys/class/net') / interface / 'statistics'
    values = {}
    for name in ('rx_packets', 'tx_packets', 'rx_errors', 'tx_errors',
                 'rx_dropped', 'tx_dropped', 'rx_missed_errors'):
        try:
            values[name] = int((base / name).read_text().strip())
        except OSError:
            pass
    return values


def main():
    root=Path(__file__).resolve().parent
    backend_default='can0' if root.name=='yz_aim_can0' else 'usb'
    parser=argparse.ArgumentParser(description='独立单电机往返：每次3秒，默认300RPM、约5分钟；不会写硬件零点。')
    parser.add_argument('--backend',choices=['can0','usb'],default=backend_default)
    parser.add_argument('--node',type=int)
    parser.add_argument('--rpm',type=int,default=300)
    parser.add_argument('--move-ms',type=int,default=3000)
    parser.add_argument('--minutes',type=float,default=5)
    parser.add_argument('--cycles',type=int)
    parser.add_argument('--log-dir',help='本次parallel测试目录，必须位于本程序logs内')
    parser.add_argument('--sequence',choices=['mirror','forward-reverse'],default='mirror',help='forward-reverse: 正转后反转；mirror: 沿用节点镜像伸缩方向')
    parser.add_argument('--if',dest='interface',default='can0')
    parser.add_argument('--library',default='liblyscan.so')
    parser.add_argument('--lys',type=int,default=0)
    parser.add_argument('--channel',type=int,default=0)
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    if args.node is None:args.node=1 if args.backend=='can0' else 2
    if not(1<=args.node<=126 and 1<=args.rpm<=300 and 1<=args.move_ms<=5000 and 0<args.minutes<=60):
        parser.error('节点1~126、速度1~300RPM、动作时间1~5000ms、总时长0~60分钟')
    if args.cycles is not None and args.cycles<1:parser.error('cycles需大于0')
    extend_sign=-1 if args.node==1 else 1
    sequence=[('extend',extend_sign),('retract',-extend_sign)] if args.sequence=='mirror' else [('forward',1),('reverse',-1)]
    labels={'extend':'伸出','retract':'缩进','forward':'正转','reverse':'反转'}
    if args.dry_run:
        print(f'计划：{args.backend}节点{args.node}，依次'+', '.join(f'{labels[name]} jog {sign*args.rpm} {args.move_ms}' for name,sign in sequence)+f'；'+(f'{args.cycles}组' if args.cycles else f'约{args.minutes:g}分钟')+'。')
        return 0
    run_dir=Path(args.log_dir) if args.log_dir else root/'logs'/f'parallel-30min-{datetime.now():%Y%m%d-%H%M%S}'
    if not run_dir.is_absolute():run_dir=root/'logs'/run_dir
    if not run_dir.resolve().is_relative_to((root/'logs').resolve()) or run_dir.resolve()==(root/'logs').resolve():
        parser.error('测试目录必须位于本程序logs下')
    run_dir.mkdir(parents=True,exist_ok=True)
    if any((run_dir/name).exists() for name in ['test.log','controller.log','summary.json','progress.json']):
        parser.error('测试目录已有结果，请使用新的目录')
    report=Reporter(run_dir/'test.log')
    controller=None;communications=None;started=None;cycles=0;actions=[];enable_attempted=False;disabled=False
    result={'started_at':datetime.now().isoformat(timespec='seconds'),'settings':vars(args),'run_directory':str(run_dir),'outcome':'not_started','reason':''}
    before=interface_counters(args.interface) if args.backend=='can0' else {}
    try:
        if args.backend=='can0':
            command=[str(root/'build/yz_aim_can0'),'--if',args.interface,'--node',str(args.node),'--bitrate','1000000','--timeout','3000']
        else:
            command=[str(root/'build-native/release/bin/yz_aim_cli'),'--node',str(args.node),'--bitrate','1000000','--library',args.library,'--lys',str(args.lys),'--channel',str(args.channel),'--timeout','3000']
        report(f'启动{args.backend}节点{args.node}独立往返；Ctrl+C停止本次测试，不更改硬件零点。')
        controller=Controller(command,root,run_dir/'controller.log')
        startup=controller.receive()
        found=re.search(r'通讯详细日志："?([^"\r\n]+)',startup)
        if not found:raise TestFailure('未找到通讯日志，不启动运动。'+startup[-1200:])
        trace_path=Path(found[1].strip())
        if not trace_path.resolve().is_relative_to(root/'logs'):raise TestFailure('通信日志目录无效')
        communications=Communications(trace_path,report)
        grouped_trace=run_dir/'can-frames.log'
        os.link(trace_path,grouped_trace)
        result['communication_log']=str(grouped_trace)
        result['source_communication_log']=str(trace_path)
        controller.on_idle=communications.poll
        enable_attempted=True
        enabled=controller.command('enable')
        if '[成功] 使能输出' not in enabled:raise TestFailure('使能未完整确认。'+enabled)
        started=time.monotonic()
        while ((args.cycles is None and time.monotonic()-started<args.minutes*60) or
               (args.cycles is not None and cycles<args.cycles)):
            for name,sign in sequence:
                report(f'第{cycles+1}组：{labels[name]}，节点{args.node} {sign*args.rpm}RPM，{args.move_ms}ms。')
                began=time.monotonic()
                still_retries=0
                while True:
                    settled=settle(controller)
                    response=controller.command(f'jog {sign*args.rpm} {args.move_ms}',timeout=args.move_ms/1000+25,allow_failure=True)
                    if '[成功] 点动' in response:break
                    # 驱动停机后 0x606C 仍有 0.2~1.8 RPM 抖动，只有这一种失败在确认转速已收敛后重发。
                    if ('motor is moving' not in response or still_retries>=3 or
                        settled is None or abs(settled)>STILL_RPM_LIMIT):
                        raise TestFailure('点动没有正常完成：'+response[-2200:])
                    still_retries+=1
                    report(f'静止判定重试 {still_retries}/3：{name} 前 0x606C={settled:.1f}RPM，等转速收敛后重发。')
                positions=[int(x) for x in re.findall(r'(-?\d+)\s+脉冲（',response)]
                speeds=[float(x) for x in re.findall(r'转速\s+(-?[\d.]+)\s+RPM',response)]
                actions.append({'command':name,'signed_motor_rpm':sign*args.rpm,'requested_ms':args.move_ms,
                                'command_wall_seconds':round(time.monotonic()-began,3),
                                'still_retries':still_retries,
                                'first_progress_position':positions[0] if positions else None,
                                'last_progress_position':positions[-1] if positions else None,
                                'max_abs_measured_rpm':max(map(abs,speeds),default=None),
                                'measured_speed_min_rpm':min(speeds,default=None),
                                'measured_speed_max_rpm':max(speeds,default=None),
                                'progress_position_delta':positions[-1]-positions[0] if len(positions)>1 else None})
                (run_dir/'progress.json').write_text(json.dumps({'cycles':cycles,'actions':len(actions),
                  'elapsed_seconds':round(time.monotonic()-started,3),'latest':actions[-1]},ensure_ascii=False,indent=2),encoding='utf-8')
            cycles+=1
            report(f'第{cycles}组完成，累计{time.monotonic()-started:.1f}秒。')
        result['outcome']='completed'
    except KeyboardInterrupt:
        result['outcome']='interrupted';result['reason']='用户中断或连接关闭';report('收到中断，停止测试并清理。')
    except Exception as exc:
        result['outcome']='blocked_missing_usb_sdk' if args.backend=='usb' and 'liblyscan' in str(exc) else 'failed'
        result['reason']=str(exc);report('异常：停止测试，不自动重试。\n'+str(exc))
    finally:
        if started is not None:result['wall_seconds']=round(time.monotonic()-started,3)
        if controller:
            disabled=controller.cleanup(report);result['controller_returncode']=controller.process.poll()
        if communications:
            time.sleep(.05);result['communication']=communications.result();communications.close()
        result.update(disabled_confirmed=disabled,enable_attempted=enable_attempted,completed_cycles=cycles,
                      completed_actions=len(actions),commanded_motion_seconds=len(actions)*args.move_ms/1000,actions=actions)
        after=interface_counters(args.interface) if args.backend=='can0' else {}
        result['interface_before']=before;result['interface_after']=after
        result['interface_delta']={k:after[k]-before[k] for k in before.keys()&after.keys()}
        if result['outcome']=='completed' and not disabled:
            result['outcome']='failed';result['reason']='动作循环完成但未确认失能'
        result['ended_at']=datetime.now().isoformat(timespec='seconds')
        (run_dir/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        report(f'测试结束：{result["outcome"]}，往返{cycles}组，动作{len(actions)}次；'+
               ('失能已确认。' if disabled else '未尝试使能或未完整确认清理，请查看结果。'))
        report('结果文件：'+str(run_dir/'summary.json'));report.file.close()
    return 0 if result['outcome']=='completed' else 130 if result['outcome']=='interrupted' else 1

if __name__=='__main__':
    def interrupted(_signal,_frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGHUP,interrupted)
    raise SystemExit(main())
