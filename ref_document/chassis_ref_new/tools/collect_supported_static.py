"""Record a supported, S1-down static session and optional MCU reset repeatability.

Each SWD snapshot is asynchronous. This is not continuous dynamic telemetry.
--resets explicitly authorizes MCU resets; no flash or motor commands are issued.
"""
import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import statistics
import struct
import subprocess
import time
import monitor_can as monitor


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def stats(values):
    return dict(mean=statistics.mean(values), std=statistics.pstdev(values),
                min=min(values), max=max(values), span=max(values)-min(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=65)
    parser.add_argument('--resets', type=int, default=0, choices=range(4))
    args = parser.parse_args()
    if args.seconds < 1 or args.seconds > 300:
        parser.error('seconds must be 1..300')
    args.out.mkdir(parents=True, exist_ok=False)
    elf = monitor.ROOT/'build/rl/wheel_leg_rl.elf'
    events = []
    def event(kind, **fields):
        row = dict(at=datetime.datetime.now(datetime.timezone.utc).isoformat(), kind=kind, **fields)
        events.append(row)
        with (args.out/'events.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row) + '\n')

    def capture(label, interval=None):
        folder = args.out/label
        report = monitor.snapshot(folder, interval=interval)
        for tag in ('a', 'b') if 'b' in report else ('a',):
            bank = (folder/(tag+'_g_motor_snapshot.bin')).read_bytes()
            report[tag]['motor_detail'] = [dict(
                arm_rad=struct.unpack_from('<f', bank, i*40+16)[0],
                velocity_rad_s=struct.unpack_from('<f', bank, i*40+20)[0],
                protocol_feedback=struct.unpack_from('<f', bank, i*40+24)[0],
                tick=struct.unpack_from('<I', bank, i*40+28)[0],
                position_lost=bool(bank[i*40+38])) for i in range(6)]
        save(folder/'result.json', report)
        value = report.get('b', report['a'])
        event('snapshot', label=label, mcu_tick=value['chassis']['tick'],
              mode=value['chassis']['mode'], s1=value['remote']['s1'])
        if value['remote']['s1'] != 2 or not value['remote']['valid'] or value['chassis']['mode'] != 0:
            raise RuntimeError('S1-down STOP required; snapshot saved, collection stopped')
        return report, value

    metadata = dict(started_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    support='operator confirmed fixed supported setup',
                    voltage_v=None, ambient_temperature_c=None, external_force_sensor=None,
                    floor=None, external_angle_reference=None, continuous_capture=False,
                    reset_count_requested=args.resets,
                    firmware_sha256=hashlib.sha256(elf.read_bytes()).hexdigest(),
                    binary_sha256=hashlib.sha256(elf.with_suffix('.bin').read_bytes()).hexdigest())
    save(args.out/'session.json', metadata)
    (args.out/'robot_config.h').write_bytes((monitor.ROOT/'user/robot_config.h').read_bytes())
    (args.out/'model_manifest.json').write_bytes((monitor.ROOT/'user/algorithm/model/manifest.json').read_bytes())
    _, before = capture('before', 2)
    baseline = (before['nonzero_torque_frames'], before['enable_commands'])
    restart = []
    for i in range(args.resets):
        argv = monitor.openocd_argv(monitor.DEFAULT_OPENOCD,
            ['gdb_port disabled', 'tcl_port disabled', 'telnet_port disabled',
             'init', 'targets stm32h723.cpu0', 'reset run', 'shutdown'])
        proc = subprocess.run(argv, cwd=monitor.ROOT, capture_output=True, text=True, timeout=40)
        (args.out/f'reset_{i+1}.log').write_text(proc.stdout+proc.stderr, encoding='utf-8')
        event('reset', number=i+1, returncode=proc.returncode)
        proc.check_returncode()
        time.sleep(3)
        _, value = capture(f'reset_{i+1}', 2)
        if value['nonzero_torque_frames'] or value['enable_commands']:
            raise RuntimeError('Unexpected command counter after reset; stop and inspect')
        restart.append(value)
        baseline = (0, 0)
        print(f'Reset {i+1}: inhibit={value["chassis"]["inhibit"]}, '
              f'position_lost={[m["position_lost"] for m in value["motor_detail"]]}', flush=True)
    samples = []
    start = time.monotonic()
    while time.monotonic()-start < args.seconds or len(samples) < 3:
        _, value = capture(f'static_{len(samples):03d}')
        samples.append(value)
        if (value['nonzero_torque_frames'], value['enable_commands']) != baseline:
            raise RuntimeError('Command counters changed during static capture')
        print(f'Static {len(samples)}: MCU={value["chassis"]["tick"]} '
              f'inhibit={value["chassis"]["inhibit"]}', flush=True)
        time.sleep(2)
    first, last = samples[0], samples[-1]
    duration = ((last['chassis']['tick']-first['chassis']['tick']) & 0xffffffff)/1000
    yaw_delta = sum(math.atan2(math.sin(b['imu']['yaw_rad']-a['imu']['yaw_rad']),
                              math.cos(b['imu']['yaw_rad']-a['imu']['yaw_rad']))
                    for a,b in zip(samples,samples[1:]))
    valid_geometry = all(not (s['chassis']['inhibit'] & (4|8|32)) for s in samples)
    result = dict(sample_count=len(samples), mcu_span_s=duration,
        wall_capture_s=time.monotonic()-start, continuous_capture=False,
        mode_values=sorted({s['chassis']['mode'] for s in samples}),
        inhibit_values=sorted({s['chassis']['inhibit'] for s in samples}),
        position_lost_any=any(m['position_lost'] for s in samples for m in s['motor_detail']),
        nonzero_frame_delta=last['nonzero_torque_frames']-first['nonzero_torque_frames'],
        enable_command_delta=last['enable_commands']-first['enable_commands'],
        node_receive_delta=[last['nodes'][i]['received']-first['nodes'][i]['received'] for i in range(6)],
        motor_temperature_c=[stats([s['nodes'][i]['temperature_c'] for s in samples]) for i in range(6)],
        motor_raw_counts=[stats([s['nodes'][i]['encoder_counts'] for s in samples]) for i in range(6)],
        gyro_rad_s=[stats([s['imu']['omega_body_rad_s'][i] for s in samples]) for i in range(3)],
        yaw_drift_deg=yaw_delta*180/math.pi, yaw_drift_deg_s=yaw_delta*180/math.pi/duration,
        tilt_deg=stats([math.degrees(math.acos(max(-1,min(1,-s['imu']['gravity_body'][2])))) for s in samples]),
        tx_fail_delta=last['tx_failed']-first['tx_failed'],
        fifo_overflow_delta=last['fifo_overflow']-first['fifo_overflow'],
        overrun_delta=last['chassis']['overruns']-first['chassis']['overruns'],
        measurement_valid_all=valid_geometry,
        joint_position_span_rad=[stats([s['measurement']['joint_position_rad'][i] for s in samples])['span'] for i in range(6)] if valid_geometry else None,
        xy_net_drift_m=[last['measurement']['root_xy_m'][i]-first['measurement']['root_xy_m'][i] for i in range(2)] if valid_geometry else None,
        reset_reference_repeatability_rad=[stats([s['measurement']['joint_position_rad'][i] for s in restart]) for i in range(6)] if restart else None,
        reset_samples_healthy=all(not(s['chassis']['inhibit'] & (4|8|32)) for s in restart),
        limitations=['SWD sequential asynchronous samples, not continuous coverage or latency measurement',
                     'Soft MCU reset only, motors remained powered',
                     'Supported pose, external angles/contact force unknown',
                     'Static gyro results include ongoing adaptive bias estimation'])
    save(args.out/'result.json', result)
    save(args.out/'artifact_manifest.json', {p.relative_to(args.out).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
        for p in args.out.rglob('*') if p.is_file() and p.name!='artifact_manifest.json'})
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
