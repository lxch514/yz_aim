#!/usr/bin/env python3
"""Passive diagnostics: never reconfigures CAN, enables devices, or transmits frames."""
import argparse
import hashlib
import json
import sys
import time
from datetime import datetime
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import x6test as m
from can_diagnostics import Diagnostics


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--iface', default='can0')
    ap.add_argument('--seconds', type=float, default=30)
    ap.add_argument('--motor-id', type=int, default=127)
    args = ap.parse_args()
    if not 1 <= args.seconds <= 7200:
        ap.error('seconds must be 1..7200')
    directory = m.ROOT / 'logs' / ('passive-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    directory.mkdir(parents=True)
    log = m.Logger(directory / 'test.log')
    frames = m.FrameLog(directory / 'can-frames.log', args.iface, time.monotonic())
    motor = m.Motor(args.iface, args.motor_id, frames, log)
    fan = m.Fan(motor.sock, frames, log, 0, 50)
    motor.fan = fan
    diag = Diagnostics(directory, args.iface, motor)
    motor.diag = fan.diag = diag
    diag.enable_socket(motor.sock)
    diag.start()
    log('被动观察 {} 秒：不重配接口、不发送任何 CAN 帧。'.format(args.seconds))
    outcome, reason = 'completed', ''
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            motor.pump(min(.02, max(0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        outcome, reason = 'interrupted', '用户中断'
        log('用户中断观察。')
    except Exception as exc:
        outcome, reason = 'failed', str(exc)
        raise
    finally:
        diagnostics = diag.finish()
        nodes = {}
        for node, data in fan.nodes.items():
            nodes[hex(node)] = {k: v for k, v in data.items() if k != 'gaps'}
        summary = {'mode': 'passive_no_transmissions', 'outcome': outcome, 'reason': reason, 'settings': vars(args),
                   'stats': motor.stats, 'fan_nodes': nodes, 'diagnostics': diagnostics,
                   'tx_motor': motor.stats['tx'], 'tx_fan': fan.stats['tx'],
                   'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                     for p in (m.ROOT / 'x6test.py', m.ROOT / 'can_diagnostics.py')}}
        (directory / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        (directory / '诊断.md').write_text('# CAN0 被动观察\n\n本进程 CAN 发送帧：0。\n' +
                                         m.diagnostic_report(diagnostics, motor, fan), encoding='utf-8')
        assert motor.stats['tx'] == fan.stats['tx'] == 0
        log('结束状态 {}；CAN TX=0；结果 {}'.format(outcome, directory))
        frames.close()
        motor.sock.close()
        log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
