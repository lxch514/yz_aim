#!/usr/bin/env python3
"""只读分析原始CAN日志；不连接设备。"""
from pathlib import Path
from collections import Counter,defaultdict
import hashlib,json,re,statistics,tarfile
BASE=Path(__file__).resolve().parent
FRAME=re.compile(r'^\[CAN \S+ \+([\d.]+)s #\d+\]\s+(\w+)\s+ID=([0-9A-Fa-f]+) DLC=(\d+)(.*)$')
def distribution(values):
 if not values:return None
 values=sorted(values)
 def pct(q):
  at=(len(values)-1)*q;low=int(at);high=min(low+1,len(values)-1)
  return round(values[low]+(values[high]-values[low])*(at-low),4)
 return {'samples':len(values),'min':round(values[0],4),'avg':round(statistics.mean(values),4),
         'p50':pct(.5),'p95':pct(.95),'p99':pct(.99),'max':round(values[-1],4)}
def parse_trace(data,node):
 directions=Counter();categories=defaultdict(Counter);objects=defaultdict(lambda:{'reads':0,'writes':0,'replies':0,'abort':0,'latencies_ms':[]})
 pending={};latencies=[];heartbeats=[];speeds=[];positions=[];driver=[];events=[];unmatched=[];duplicate=[];alarms=[]
 kernel_drops=0;first=None;last=None;lines=data.decode('utf-8',errors='replace').splitlines()
 for number,line in enumerate(lines,1):
  if line.startswith('[CAN STAT '):
   driver.append({'line':number,**{k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',line)}})
  match=FRAME.match(line)
  if not match:continue
  at,direction,identifier,dlc,payload=match.groups();at=float(at);identifier=int(identifier,16);dlc=int(dlc);payload=bytes.fromhex(payload.strip())
  first=at if first is None else first;last=at;directions[direction]+=1
  if direction not in ['TX','RX']:
   if direction=='RX_DROP':kernel_drops+=identifier
   if direction!='RX_DROP' or identifier:events.append({'line':number,'text':line})
   continue
  if identifier==0x600+node:category='SDO请求'
  elif identifier==0x580+node:category='SDO应答'
  elif identifier==0x77f:category='主站心跳'
  elif identifier==0x700+node:category='节点心跳'
  elif identifier in [0x180+node,0x280+node,0x380+node,0x480+node]:category='TPDO'+str((identifier-node)//0x100)
  else:category=f'其它0x{identifier:X}'
  categories[category][direction]+=1
  if direction=='TX' and identifier==0x77f:heartbeats.append(at)
  if dlc!=8 or len(payload)!=8:continue
  index=int.from_bytes(payload[1:3],'little');sub=payload[3];key=(index,sub);text_key=f'0x{index:04X}:{sub:02X}'
  if direction=='TX' and identifier==0x600+node:
   objects[text_key]['reads' if payload[0]==0x40 else 'writes']+=1
   if key in pending:duplicate.append({'key':text_key,'previous':pending[key],'new_at':at,'new_line':number})
   pending[key]={'at':at,'line':number,'command':payload[0]}
  if direction=='RX' and identifier==0x580+node:
   objects[text_key]['replies']+=1
   request=pending.pop(key,None)
   if request:
    latency=(at-request['at'])*1000;latencies.append(latency);objects[text_key]['latencies_ms'].append(latency)
   else:unmatched.append({'line':number,'text':line})
   if payload[0]==0x80:objects[text_key]['abort']+=1
   elif payload[0] in [0x43,0x47,0x4b,0x4f]:
    width=4-((payload[0]>>2)&3)
    if index==0x606c and width==2:speeds.append(int.from_bytes(payload[4:6],'little',signed=True)/10)
    if index==0x6064 and width==4:positions.append(int.from_bytes(payload[4:8],'little',signed=True))
    if index==0x260e:alarms.append({'seconds':at,'value':int.from_bytes(payload[4:4+width],'little')})
 object_out={key:{**{k:v for k,v in item.items() if k!='latencies_ms'},'latency_ms':distribution(item['latencies_ms'])} for key,item in objects.items()}
 return {'directions':dict(directions),'frame_types':{k:dict(v) for k,v in categories.items()},'trace_seconds':round((last or 0)-(first or 0),6),
         'objects':object_out,'latency_ms':distribution(latencies),'heartbeat_interval_ms':distribution([(b-a)*1000 for a,b in zip(heartbeats,heartbeats[1:])]),
         'speed_min_rpm':min(speeds,default=None),'speed_max_rpm':max(speeds,default=None),
         'position_min':min(positions,default=None),'position_max':max(positions,default=None),'position_first':positions[0] if positions else None,
         'position_last':positions[-1] if positions else None,'net_position_change':positions[-1]-positions[0] if positions else None,
         'kernel_drop_increment':kernel_drops,'driver_samples':driver,'events':events,'unmatched_responses':unmatched,
         'pending_requests':[{'key':f'0x{k[0]:04X}:{k[1]:02X}',**v} for k,v in pending.items()],
         'duplicate_requests':duplicate,'requests_without_unique_reply':len(pending)+len(duplicate),
         'alarm_counts':dict(Counter(f"0x{x['value']:04X}" for x in alarms)),
         'nonzero_alarms':[x for x in alarms if x['value']], 'last_alarm':alarms[-1] if alarms else None}