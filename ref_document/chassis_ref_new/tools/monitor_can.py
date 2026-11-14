#!/usr/bin/env python3
"""Read live RL firmware RAM over Horco CMSIS-DAP. Never halt/reset/flash/enable.

Two increasing per-ID counters establish live reception. Old/stale firmware is
rejected by its vector fingerprint. Motor output authorization stays in source.
"""
import argparse
import datetime
import json
from pathlib import Path
import struct
import subprocess
ROOT=Path(__file__).resolve().parents[1]
DEFAULT_OPENOCD=Path.home()/'.eide/tools/openocd_7a1adfbec_mingw32/bin/openocd.exe'
SYMBOLS=('g_can_monitor','g_chassis','g_remote','g_imu','g_measurement',
         'g_motor_snapshot','g_board_debug','g_can_tx_fail','g_can_rx_overflow','g_rl')
NODE_LABELS=('right_hip_DM4','right_knee_DM3','right_wheel_M3508_3',
             'left_hip_DM1','left_knee_DM2','left_wheel_M3508_2')

def elf_symbols(elf,names=SYMBOLS):
    nm=subprocess.run(['arm-none-eabi-nm','-S',str(elf)],capture_output=True,text=True,check=True).stdout
    symbols={}
    for line in nm.splitlines():
        bits=line.split()
        if len(bits)==4 and bits[3] in names: symbols[bits[3]]=(int(bits[0],16),int(bits[1],16))
    if len(symbols)!=len(names): raise SystemExit('ELF missing monitor symbols: rebuild current firmware')
    return symbols

def openocd_argv(openocd,commands):
    args=[str(openocd),'-s',str(Path(openocd).parent.parent/'share/openocd/scripts'),'-f','openocd_dap.cfg']
    for command in commands: args.extend(['-c',command])
    return args

def dump_ram(out,symbols,interval,openocd,elf,timeout=40):
    """Dump the reset vector plus one snapshot, or two when interval is given."""
    out.mkdir(parents=True,exist_ok=True)
    tags=('a','b') if interval is not None else ('a',)
    commands=['gdb_port disabled','tcl_port disabled','telnet_port disabled','init',
              'targets stm32h723.cpu0',f'dump_image {{{(out/"vectors.bin").as_posix()}}} 0x08000000 256']
    for tag in tags:
        if tag=='b': commands.append(f'sleep {int(interval*1000)}')
        for name,(address,size) in symbols.items():
            commands.append(f'dump_image {{{(out/(tag+"_"+name+".bin")).as_posix()}}} {address:#x} {size}')
    commands+=['catch {resume}','shutdown']  # never leave the target halted after a read
    proc=subprocess.run(openocd_argv(openocd,commands),cwd=ROOT,capture_output=True,text=True,timeout=timeout)
    log=proc.stdout+proc.stderr
    (out/'openocd.log').write_text(log,encoding='utf-8')
    if proc.returncode:
        print(log[-6000:]); raise SystemExit(proc.returncode)
    if (out/'vectors.bin').read_bytes()!=elf.with_suffix('.bin').read_bytes()[:256]:
        raise SystemExit('Board vector table differs from RL firmware; refuse to decode RAM')
    return log

def decode(out,tag):
    read=lambda name:(out/(tag+'_'+name+'.bin')).read_bytes()
    data=read('g_can_monitor'); values=struct.unpack_from('<10I',data)
    result=dict(zip(('received','other','tx_error','rx_error','bus_off','last_error','tx_queued','nonzero_torque_frames','enable_commands','status_samples'),values))
    result['nodes']=[]
    bank=read('g_motor_snapshot')
    if len(bank)!=248: raise ValueError('Review ARM MotorBank layout before decoding')
    for i,label in enumerate(NODE_LABELS):
        address,count,tick=struct.unpack_from('<3I',data,40+i*20)
        raw=data[52+i*20:60+i*20]
        result['nodes'].append({'label':label,'feedback_id':hex(address),'received':count,'last_ms':tick,
                                'encoder_counts':struct.unpack_from('>H',raw)[0],
                                'encoder_total_counts':struct.unpack_from('<q',bank,i*40+8)[0],
                                'temperature_c':raw[6],'status_byte':raw[7],'data':raw.hex(' ')})
    result['chassis']=dict(zip(('tick','inhibit','fault','cycles','policy_runs','inference_us','max_loop_us','overruns','mode','actor_selftest'),struct.unpack('<10I',read('g_chassis'))))
    remote=read('g_remote')
    result['remote']={'channels':list(struct.unpack_from('<5h',remote)),'s1':remote[10],'s2':remote[11],
                      'tick':struct.unpack_from('<I',remote,12)[0],'valid_frames':struct.unpack_from('<I',remote,16)[0],
                      'rejected':struct.unpack_from('<I',remote,20)[0],'valid':bool(remote[24])}
    imu=read('g_imu')
    result['imu']={'tick':struct.unpack_from('<I',imu,92)[0],'samples':struct.unpack_from('<I',imu,96)[0],
                   'calibration_samples':struct.unpack_from('<I',imu,100)[0],'errors':struct.unpack_from('<I',imu,104)[0],
                   'initialized':bool(imu[108]),'calibrated':bool(imu[109]),
                   'quaternion_wxyz':struct.unpack_from('<4f',imu,0),
                   'omega_body_rad_s':struct.unpack_from('<3f',imu,16),
                   'accel_body_m_s2':struct.unpack_from('<3f',imu,28),
                   'gravity_body':struct.unpack_from('<3f',imu,40),'yaw_rad':struct.unpack_from('<f',imu,52)[0]}
    if len(imu)>=124:
        result['imu'].update(zip(('motion_resets','cadence_resets','max_gap_ms'),struct.unpack_from('<3I',imu,112)))
    if len(imu)>=128: result['imu']['drdy_edges']=struct.unpack_from('<I',imu,124)[0]
    result['board_debug']=dict(zip(('init_stage','rx_starts','rx_start_failures','rx_events','uart_errors',
        'irq_count','cr1','isr','uart_clock','rx_state'),struct.unpack('<10I',read('g_board_debug'))))
    measurement=read('g_measurement')
    result['measurement']={'joint_position_rad':struct.unpack_from('<6f',measurement,0),
        'joint_velocity_rad_s':struct.unpack_from('<6f',measurement,24),
        'root_xy_m':struct.unpack_from('<2f',measurement,72),
        'body_velocity_xy_m_s':struct.unpack_from('<2f',measurement,84)}
    result['tx_failed']=struct.unpack('<I',read('g_can_tx_fail'))[0]
    result['fifo_overflow']=struct.unpack('<I',read('g_can_rx_overflow'))[0]
    rl=read('g_rl')
    if len(rl)!=1164: raise ValueError('Review ARM RlController layout before decoding')
    result['rl']={'action':list(struct.unpack_from('<6f',rl,1020)),
                  'command':list(struct.unpack_from('<3f',rl,1044)),
                  'reference_yaw':struct.unpack_from('<f',rl,1064)[0],
                  'target_q':list(struct.unpack_from('<6f',rl,1068)),
                  'target_qd':list(struct.unpack_from('<6f',rl,1092)),
                  'joint_torque':list(struct.unpack_from('<6f',rl,1116)),
                  'motor_torque':list(struct.unpack_from('<6f',rl,1140))}
    return result

def snapshot(out,interval=2.0,openocd=DEFAULT_OPENOCD,elf=None):
    """Decode one capture. interval=None reads a single snapshot without a/b."""
    elf=Path(elf) if elf else ROOT/'build/rl/wheel_leg_rl.elf'
    dump_ram(out,elf_symbols(elf),interval,openocd,elf)
    report={'vector_fingerprint_matches':True,'a':decode(out,'a')}
    if interval is not None:
        report['b']=decode(out,'b')
        report['live_ids']=[node['feedback_id'] for i,node in enumerate(report['b']['nodes'])
                            if node['received']>report['a']['nodes'][i]['received']]
    return report

def brief(out,report):
    """One-screen summary for bench operation."""
    for tag in ('a','b') if 'b' in report else ('a',):
        v=report[tag]; c=v['chassis']; rl=v['rl']
        print(f"[{tag}] tick={c['tick']} mode={c['mode']} inhibit={c['inhibit']} fault={c['fault']} "
              f"policy_runs={c['policy_runs']} inference_us={c['inference_us']} max_loop_us={c['max_loop_us']} overruns={c['overruns']}")
        print(f"     nonzero_torque_frames={v['nonzero_torque_frames']} enable_commands={v['enable_commands']} "
              f"tx_failed={v['tx_failed']} rx_overflow={v['fifo_overflow']}")
        print(f"     action      {[round(x,3) for x in rl['action']]}")
        print(f"     target_q    {[round(x,3) for x in rl['target_q']]}")
        print(f"     motor_torque{[round(x,3) for x in rl['motor_torque']]} Nm")
        print(f"     q           {[round(x,3) for x in v['measurement']['joint_position_rad']]}")
        print(f"     qd          {[round(x,3) for x in v['measurement']['joint_velocity_rad_s']]}")
        print(f"     root_xy     {[round(x,3) for x in v['measurement']['root_xy_m']]} m  yaw={v['imu']['yaw_rad']:+.3f} rad")
        print(f"     imu calibrated={v['imu']['calibrated']} max_gap_ms={v['imu'].get('max_gap_ms')} errors={v['imu']['errors']}")
        rm=v['remote']
        print(f"     remote s1={rm['s1']} s2={rm['s2']} channels={rm['channels']} valid_frames={rm['valid_frames']} rejected={rm['rejected']}")
        print('     nodes '+', '.join(f"{n['label']} recv={n['received']} raw={n['encoder_counts']} "
                                     f"{n['temperature_c']}C st=0x{n['status_byte']:02X}" for n in v['nodes']))
    print('Evidence:',out/'result.json')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interval',type=float,default=2)
    p.add_argument('--brief',action='store_true',help='print a one-screen summary instead of full JSON')
    p.add_argument('--fast',action='store_true',
                   help='read only g_chassis+g_can_monitor (about 40 ms halt, keeps DM comm alive)')
    p.add_argument('--openocd',type=Path,default=DEFAULT_OPENOCD)
    a=p.parse_args()
    if not 0.1<=a.interval<=10: p.error('interval must be .1..10 s')
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    out=ROOT/'diagnostics'/('live_can_'+stamp)
    if a.fast:
        elf=ROOT/'build/rl/wheel_leg_rl.elf'
        dump_ram(out,elf_symbols(elf,('g_can_monitor','g_chassis','g_remote')),None,a.openocd,elf)
        ch=(out/'a_g_chassis.bin').read_bytes(); cm=(out/'a_g_can_monitor.bin').read_bytes()
        rm=(out/'a_g_remote.bin').read_bytes()
        c=dict(zip(('tick','inhibit','fault','cycles','policy_runs','inference_us','max_loop_us','overruns'),struct.unpack_from('<8I',ch)))
        mode=struct.unpack_from('<I',ch,32)[0]
        cmv=dict(zip(('received','other','tx_error','rx_error','bus_off','last_error','tx_queued','nonzero_torque_frames','enable_commands','status_samples'),struct.unpack_from('<10I',cm)))
        print(f"tick={c['tick']} mode={mode} inhibit={c['inhibit']} fault={c['fault']} policy_runs={c['policy_runs']} "
              f"inference_us={c['inference_us']} overruns={c['overruns']}")
        print(f"nonzero_torque_frames={cmv['nonzero_torque_frames']} enable_commands={cmv['enable_commands']} "
              f"tx_error={cmv['tx_error']} rx_error={cmv['rx_error']} bus_off={cmv['bus_off']} received={cmv['received']}")
        print(f"remote s1={rm[10]} s2={rm[11]} channels={list(struct.unpack_from('<5h',rm))} "
              f"valid_frames={struct.unpack_from('<I',rm,16)[0]} valid={bool(rm[24])}")
        return
    report=snapshot(out,a.interval,a.openocd)
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    if a.brief: brief(out,report)
    else: print(json.dumps(report,indent=2)); print('Evidence:',out/'result.json')
if __name__=='__main__': main()