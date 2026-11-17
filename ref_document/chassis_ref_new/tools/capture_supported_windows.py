"""Capture coherent short telemetry windows in operator-confirmed supported STOP.

Only the diagnostic ring's frozen word is written; no reset, halt or motor command.
Restores live recording after each dump, including a separate recovery on failure.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time
import monitor_can as monitor
import telemetry_log as telemetry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    report = monitor.snapshot(args.out/'before', interval=1)
    (args.out/'before/result.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    state = report['b']
    if state['remote']['s1'] != 2 or not state['remote']['valid'] or state['chassis']['mode'] != 0:
        raise RuntimeError('Supported S1-down STOP required')
    elf = monitor.ROOT/'build/rl/wheel_leg_rl.elf'
    symbols = monitor.elf_symbols(elf, ('g_telemetry',))
    address, size = symbols['g_telemetry']
    flag = address+20
    prefix = ['gdb_port disabled', 'tcl_port disabled', 'telnet_port disabled',
              'init', 'targets stm32h723.cpu0']
    results = []
    for i in range(3):
        folder = args.out/f'window_{i+1}'
        folder.mkdir()
        binary = (folder/'g_telemetry.bin').resolve()
        commands = prefix+[f'mww {flag:#x} 1', 'sleep 10',
            f'dump_image {{{binary.as_posix()}}} {address:#x} {size}',
            f'mww {flag:#x} 0', 'shutdown']
        completed = False
        try:
            proc = subprocess.run(monitor.openocd_argv(monitor.DEFAULT_OPENOCD, commands),
                cwd=monitor.ROOT, capture_output=True, text=True, timeout=45)
            (folder/'openocd.log').write_text(proc.stdout+proc.stderr, encoding='utf-8')
            proc.check_returncode()
            completed = True
        finally:
            if not completed:
                recovery = subprocess.run(monitor.openocd_argv(monitor.DEFAULT_OPENOCD,
                    prefix+[f'mww {flag:#x} 0', 'shutdown']), cwd=monitor.ROOT,
                    capture_output=True, text=True, timeout=30)
                (folder/'restore.log').write_text(recovery.stdout+recovery.stderr, encoding='utf-8')
                recovery.check_returncode()
        decoded = telemetry.decode(binary.read_bytes())
        decoded['capture_method'] = 'diagnostic frozen word set to 1 for RAM read, restored to 0; CPU running'
        (folder/'result.json').write_text(json.dumps(decoded, indent=2)+'\n', encoding='utf-8')
        rows = decoded['samples']
        results.append(dict(window=i+1, count=len(rows), rejected=decoded['rejected_slots'],
            continuous_2ms_window=decoded['continuous_2ms_window'],
            mode_values=sorted({r['mode'] for r in rows}), inhibit_values=sorted({r['inhibit'] for r in rows}),
            motor_age_max_ms=[max(r['motor_age_ms'][j] for r in rows) for j in range(6)] if rows else None,
            imu_age_max_ms=max((r['imu_age_ms'] for r in rows),default=None),
            remote_age_max_ms=max((r['remote_age_ms'] for r in rows),default=None),
            abs_qd_max_rad_s=[max(abs(r['qd'][j]) for r in rows) for j in range(6)] if rows else None,
            confidence_min=min((r['odometry_confidence'] for r in rows),default=None)))
        print(json.dumps(results[-1]), flush=True)
        time.sleep(1)
    after = monitor.snapshot(args.out/'after', interval=1)
    (args.out/'after/result.json').write_text(json.dumps(after, indent=2)+'\n', encoding='utf-8')
    results.append(dict(final_mode=after['b']['chassis']['mode'], final_s1=after['b']['remote']['s1'],
        nonzero_delta=after['b']['nonzero_torque_frames']-state['nonzero_torque_frames'],
        enable_delta=after['b']['enable_commands']-state['enable_commands']))
    (args.out/'summary.json').write_text(json.dumps(results, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
