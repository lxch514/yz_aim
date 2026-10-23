#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import sys
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('can_monitor',Path(__file__).resolve().parents[1]/'can_monitor.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
messages=[]
s=module.Statistics(messages.append)
def f(t,kind,ident,data):
 raw=bytes(data);return f'[CAN can0 +{t:.6f}s #1] {kind} ID={ident:03X} DLC={len(raw)} '+raw.hex(' ')+'\n'
s.feed('# CAN_START epoch_ms=1000000 pid=123 interface=can0\n')
assert s.epoch==1000 and s.pid==123
s.feed(f(.0,'TX',0x601,[0x40,0x41,0x60,0,0,0,0,0]))
s.feed(f(.01,'RX',0x581,[0x4b,0x41,0x60,0,0x37,0x04,0xaa,0xaa]))
assert not s.pending and s.nodes_tx[1]==s.nodes_rx[1]==1
# Correct width: reserved bytes do not turn zero alarm into a false fault.
s.feed(f(.02,'RX',0x581,[0x4b,0x0e,0x26,0,0,0,0xaa,0xaa]))
assert not messages
s.feed(f(.03,'RX',0x581,[0x4b,1,0x26,0,1,0,0,0]))
s.feed(f(.04,'TX',0x77f,[5]));s.feed(f(.24,'TX',0x77f,[5]))
assert s.count['heartbeat_tx']==2 and abs(s.heartbeat_gap_max-.2)<1e-8
s.expire(.95);assert s.count['heartbeat_timeout']==1
s.expire(1.0);assert s.count['heartbeat_timeout']==1
s.feed(f(1.01,'TX',0x77f,[5]))
s.feed(f(1.02,'TX',0x602,[0x23,0xff,0x60,0,0,0,0,0]))
s.expire(1.6);assert s.count['sdo_timeout']==1 and any('节点2的SDO写' in m for m in messages)
s.feed(f(1.61,'TX_ERR',0x602,[0x40,0x41,0x60,0,0,0,0,0]))
s.feed(f(1.62,'RX_DROP',7,[]))
s.feed(f(1.63,'RX_ERROR',0x20000040,[0]*8))
s.feed(f(1.64,'RX',0x582,[0x80,0xff,0x60,0,0,0,2,6]))
assert s.count['TX_ERR']==1 and s.count['kernel_drop']==7 and s.count['sdo_abort']==1
s.feed('[CAN STAT can0] TX=20 RX=19 TX_ERR=1 KERNEL_RX_DROP=7 TRACE_DROP=2 OUTPUT_DROP=1\n')
s.feed('[CAN STAT can0] TX=21 RX=20 TX_ERR=1 KERNEL_RX_DROP=7 TRACE_DROP=2 OUTPUT_DROP=1\n')
assert s.driver['TX']==21 and s.driver['KERNEL_RX_DROP']==7
s.summary();assert '通信统计' in messages[-1] and '接收队列明确丢弃7帧' in messages[-1]
# A completed disable must not trigger a false missing-heartbeat alert.
s.feed(f(1.7,'RX',0x581,[0x4b,1,0x26,0,0,0,0,0]))
count=s.count['heartbeat_timeout'];s.expire(100)
assert s.count['heartbeat_timeout']==count
# Repeated reads are observed as retries, without inventing a physical packet loss.
s.feed(f(101,'TX',0x601,[0x40,0x64,0x60,0,0,0,0,0]))
s.feed(f(101.1,'TX',0x601,[0x40,0x64,0x60,0,0,0,0,0]))
s.feed(f(101.2,'RX',0x581,[0x43,0x64,0x60,0,0,0,0,0]))
assert s.count['read_retry']==1 and not s.pending
print('can_monitor_test: PASS (Chinese errors, heartbeat, ACK timeout, explicit drops, reserved bytes, retries)')
