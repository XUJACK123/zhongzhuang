"""DAP T0 collection: no flash/reset, motor enable, or parameter commands.

Snapshot reads are asynchronous. The optional ring export pauses the CPU while
reading an already recorded 64 ms window, then resumes it; it is not streaming.
"""
import argparse
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import monitor_can as monitor
import telemetry_log

OUT = Path((ROOT / 'real_data_collection/latest_session.txt').read_text(encoding='utf-8-sig').strip())
ELF = ROOT / 'build/rl/wheel_leg_rl.elf'
RAW = OUT / 'raw'
SYMBOLS = monitor.elf_symbols(ELF, monitor.SYMBOLS + ('g_telemetry', 'g_motor_diagnostics', 'g_imu_timing', 'g_imu_fusion'))


def utc():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def clean(obj):
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    return obj


def save(path, obj):
    path.write_text(json.dumps(clean(obj), ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def event(kind, **kwargs):
    with (OUT / 'events.jsonl').open('a', encoding='utf-8') as f:
        f.write(json.dumps({'host_utc': utc(), 'kind': kind, 'mcu_tick': None,
                            'synchronization': 'host timestamps bracket debugger operations; no shared trigger', **kwargs}, ensure_ascii=False) + '\n')


def run(name, commands, timeout=180):
    script = OUT / (name + '.tcl')
    base = ['source [find openocd_dap.cfg]',
            'stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}',
            'gdb_port disabled', 'tcl_port disabled', 'telnet_port disabled',
            'init', 'targets stm32h723.cpu0']
    script.write_text('\n'.join(base + commands + ['shutdown']) + '\n', encoding='ascii')
    args = [str(monitor.DEFAULT_OPENOCD), '-s', str(monitor.DEFAULT_OPENOCD.parent.parent / 'share/openocd/scripts'),
            '-s', str(ROOT), '-f', str(script)]
    event('debugger_start', operation=name, argv=args)
    with (OUT / (name + '_openocd.log')).open('w', encoding='utf-8') as log:
        p = subprocess.Popen(args, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        try:
            code = p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            p.kill(); p.wait()
            event('debugger_timeout', operation=name, timeout_s=timeout)
            raise
    event('debugger_end', operation=name, exit_code=code)
    if code:
        print((OUT / (name + '_openocd.log')).read_text(encoding='utf-8')[-3000:])
        raise RuntimeError(f'{name}: OpenOCD exit {code}')


def dump(tag, name):
    addr, size = SYMBOLS[name]
    return f'dump_image {{{(RAW / (tag + "_" + name + ".bin")).as_posix()}}} {addr:#x} {size}'


def identity():
    board_path = RAW / 'board_firmware_timeout_attempt1.bin'
    board = board_path.read_bytes()
    binary = ELF.with_suffix('.bin').read_bytes()
    contents = ELF.read_bytes()
    sections_text = subprocess.check_output(['arm-none-eabi-objdump', '-h', str(ELF)], text=True)
    (OUT / 'elf_sections.txt').write_text(sections_text, encoding='utf-8')
    sections = []
    covered = set()
    lines = sections_text.splitlines()
    for i, line in enumerate(lines[:-1]):
        m = re.match(r'\s*\d+\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)', line)
        if not m or 'LOAD' not in lines[i + 1] or 'ALLOC' not in lines[i + 1]:
            continue
        name, size, vma, lma, offset = m.groups()
        size, lma, offset = int(size, 16), int(lma, 16), int(offset, 16)
        if not size or not 0x08000000 <= lma < 0x08100000:
            continue
        start = lma - 0x08000000
        good = board[start:start + size] == contents[offset:offset + size]
        covered.update(range(start, start + size))
        sections.append({'section': name, 'lma': hex(lma), 'bytes': size, 'matches': good})
    differences = [i for i, (a, b) in enumerate(zip(board, binary)) if a != b]
    verified = bool(sections) and all(s['matches'] for s in sections) and len(board) == len(binary)
    result = {'captured_at_utc': utc(), 'full_bin_bytewise_match': board == binary,
              'elf_flash_load_sections_match': verified, 'sections': sections,
              'differing_bin_offsets': differences,
              'all_differences_outside_load_sections': all(i not in covered for i in differences),
              'difference_explanation': 'Erased 0xff in the four-byte section alignment gap at 0x080002cc; local BIN fills it with 0x00.',
              'board_firmware_sha256': hashlib.sha256(board).hexdigest(),
              'local_bin_sha256': hashlib.sha256(binary).hexdigest(),
              'local_elf_sha256': hashlib.sha256(contents).hexdigest(),
              'image_bytes': len(board), 'entire_physical_flash_verified': False,
              'first_process_timeout_s': 50, 'first_stdout_log_available': False,
              'symbols': SYMBOLS}
    save(OUT / 'identity.json', result)
    assert verified and result['all_differences_outside_load_sections'], 'Firmware identity not established'
    version = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    diff = subprocess.check_output(['git', 'diff', '--', '.'], cwd=ROOT, text=True)
    (OUT / 'source.diff').write_text(diff, encoding='utf-8')
    manifest = json.loads((ROOT / 'user/algorithm/model/manifest.json').read_text(encoding='utf-8'))
    session = {'experiment': 'T0 supported static, zero active motor output', 'timezone': 'Asia/Hong_Kong',
               'operator': 'user on site; Codex DAP collection', 'created_at_utc': utc(),
               'probe': {'type': 'Horco CMSIS-DAP v2', 'vid_pid': 'faed:4870', 'serial': '332107148640', 'swd_khz': 100},
               'firmware': result, 'control_version_expected': '0x20261115', 'model_version_expected': 100,
               'balance_actor_sha256': manifest['sha256'], 'getup_actor_sha256': None,
               'source_commit': version, 'source_diff': 'source.diff',
               'field_conditions': {'support': 'user confirmed reliable support', 's1': 'user confirmed down',
                                    'motion': 'user confirmed static', 'motor_power': 'user confirmed on',
                                    'measured_voltage_v': None, 'load_kg': None, 'floor': None,
                                    'ambient_temperature_c': None, 'support_force_n': None,
                                    'external_sensor_model_range_accuracy': None},
               'prohibited_actions_not_executed': ['flash', 'reset', 'motor enable', 'motion command', 'CAN parameter writes'],
               'limitations': ['SWD snapshots are asynchronous and perturb timing.',
                               'No external truth or shared timing trigger.', 'No continuous 500 Hz transport.']}
    save(OUT / 'session.json', session)
    event('identity_verified', sections_match=True, bin_padding_difference=True,
          user_confirmed_support_s1_down_static=True)
    print(json.dumps({'sections_match': verified, 'padding_offsets': differences}))


def decode_snapshot(tag):
    row = monitor.decode(RAW, tag)
    pre = struct.unpack('<10I', (RAW / f'{tag}_before_g_chassis.bin').read_bytes())
    post = struct.unpack('<10I', (RAW / f'{tag}_after_g_chassis.bin').read_bytes())
    row['tag'] = tag
    row['mcu_read_bracket'] = {'tick_before': pre[0], 'tick_after': post[0], 'elapsed_ms_u32': (post[0]-pre[0]) & 0xffffffff,
                               'not_synchronous': True}
    bank = (RAW / f'{tag}_g_motor_snapshot.bin').read_bytes()
    row['motor_bank'] = []
    for i, label in enumerate(monitor.NODE_LABELS):
        q = i * 40
        arm, vel, feedback = struct.unpack_from('<3f', bank, q + 16)
        tick, received = struct.unpack_from('<2I', bank, q + 28)
        row['motor_bank'].append({'label': label, 'raw': struct.unpack_from('<H', bank, q)[0],
                                 'counts': struct.unpack_from('<q', bank, q+8)[0], 'arm_rad': arm,
                                 'velocity_rad_s': vel, 'protocol_feedback_value': feedback,
                                 'protocol_feedback_unit': 'A' if i in (2, 5) else 'Nm',
                                 'motor_tick': tick, 'motor_received': received,
                                 'temperature_c': bank[q+36], 'state': bank[q+37], 'position_lost': bool(bank[q+38])})
    save(OUT / f'{tag}.json', row)
    return row


def snapshots(stage, count, gap):
    identity_data = json.loads((OUT/'identity.json').read_text())
    assert identity_data['elf_flash_load_sections_match']
    commands = ['echo INITIAL_CPU_STATE', 'echo [stm32h723.cpu0 curstate]',
                'if {[string equal [stm32h723.cpu0 curstate] halted]} { echo RESUME_FOR_ZERO_OUTPUT_SAMPLING; resume }',
                'sleep 2000']
    names = list(monitor.SYMBOLS)
    for i in range(count):
        tag = f'{stage}_{i:03d}'
        if i:
            commands.append(f'sleep {int(gap*1000)}')
        commands.append('echo SNAPSHOT_' + tag)
        addr, size = SYMBOLS['g_chassis']
        commands.append(f'dump_image {{{(RAW/(tag+"_before_g_chassis.bin")).as_posix()}}} {addr:#x} {size}')
        for name in names:
            commands.append(dump(tag, name))
        commands.append(f'dump_image {{{(RAW/(tag+"_after_g_chassis.bin")).as_posix()}}} {addr:#x} {size}')
    commands += ['echo FINAL_CPU_STATE', 'echo [stm32h723.cpu0 curstate]']
    event('snapshots_requested', stage=stage, count=count, requested_gap_s=gap,
          interpretation='gap is a wait after the preceding asynchronous snapshot, not the true sample period')
    run(stage, commands, timeout=max(120, count*(gap+8)))
    rows = [decode_snapshot(f'{stage}_{i:03d}') for i in range(count)]
    save(OUT / f'{stage}_snapshots.json', rows)
    pairs = []
    if stage.startswith('T0'):
        for a, b in zip(rows[::2], rows[1::2]):
            growth = [(y['received']-x['received']) & 0xffffffff for x,y in zip(a['nodes'],b['nodes'])]
            pairs.append({'a': a['tag'], 'b': b['tag'], 'received_deltas': growth,
                          'all_six_increasing': all(0<x<0x80000000 for x in growth)})
    save(OUT / f'{stage}_result.json', {'snapshot_count': len(rows), 'pairs': pairs,
                                     'six_motor_connection_pass': bool(pairs) and all(p['all_six_increasing'] for p in pairs)})
    for r in rows:
        print(json.dumps({'tag': r['tag'], 'tick':r['chassis']['tick'],'mode':r['chassis']['mode'],
                          'inhibit':r['chassis']['inhibit'],'remote':r['remote'],
                          'counts':[n['received'] for n in r['nodes']], 'nonzero':r['nonzero_torque_frames'],
                          'enables':r['enable_commands'],'imu_calibrated':r['imu']['calibrated']}))
    print('PAIR_RESULTS',json.dumps(pairs))


def ring(tag='bounded', source_stage='T0'):
    # The latest verified STOP prevents exporting by pausing an active controller.
    rows=json.loads((OUT/(source_stage+'_snapshots.json')).read_text())
    assert all(r['chassis']['mode']==0 and r['nonzero_torque_frames']==0 and r['enable_commands']==0 for r in rows)
    commands=['if {[string equal [stm32h723.cpu0 curstate] halted]} {resume}', 'sleep 2000',
              'echo RING_HALT_BEGIN',
              'set ring_error [catch { halt; wait_halt 1000; '+dump(tag, 'g_telemetry')+' } ring_message]',
              'echo RING_RESUME',
              'if {[string equal [stm32h723.cpu0 curstate] halted]} {resume}',
              'echo [stm32h723.cpu0 curstate]',
              'if {$ring_error} {error $ring_message}']
    event('ring_export_pause_requested', reason='read stable bounded window; STOP verified; user support confirmed',
          motor_output_commands_issued=False)
    run(tag+'_ring', commands, timeout=90)
    report=telemetry_log.decode((RAW/(tag+'_g_telemetry.bin')).read_bytes())
    report['summary']=telemetry_log.summarize(report)
    report['capture_method']='MCU paused only during SWD read, resumed afterwards; frame ticks precede the halt'
    report['not_continuous_60s_recording']=True
    save(OUT/(tag+'_telemetry.json'), report)
    print(json.dumps({k:report[k] for k in ('sample_count','rejected_slots','continuous_2ms_window','frozen')}))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=['identity','T0','static','ring','final','retry','static_after','ring_after'])
    args=parser.parse_args()
    if args.stage=='identity': identity()
    elif args.stage=='T0': snapshots('T0',6,2)
    elif args.stage=='static': snapshots('static',21,2)
    elif args.stage=='final': snapshots('final',2,2)
    elif args.stage=='static_after': snapshots('static_after_reconnect',21,2)
    elif args.stage=='ring_after': ring('bounded_retry1','T0_retry1')
    elif args.stage=='retry':
        event('user_reported_connection_repaired', action='right wheel power/CAN/ID checked and reconnected; remote on and S1 down',
              external_event_time_utc=None, synchronization_error_ms=None)
        snapshots('T0_retry1',6,2)
    else: ring()
