#!/usr/bin/env python3
"""Decode bounded MCU telemetry; optionally read RAM after a supported STOP.

No flash/reset/enable commands. A RAM dump is NOT a continuous 500 Hz logger.
"""
import argparse
import ctypes as C
import datetime
import hashlib
import json
from pathlib import Path
import statistics

U, F = C.c_uint32, C.c_float
SCHEMA = 2
CAPACITY = 32
ORDER = ['right_hip_DM4', 'right_knee_DM3', 'right_wheel_M3508_3',
         'left_hip_DM1', 'left_knee_DM2', 'left_wheel_M3508_2']
SIGNS = [-1, 1, -1, 1, 1, 1]


class Frame(C.LittleEndianStructure):
    _fields_ = ([(n, U) for n in ('begin', 'tick', 'cycle', 'control_version', 'model_version',
                                 'getup_model_version', 'mode', 'phase', 'failure', 'inhibit',
                                 'fault', 'policy_age_ms', 'submitted_mask')]
                + [('motor_tick', U*6), ('motor_received', U*6), ('imu_tick', U), ('remote_tick', U),
                   ('motor_raw', U*6), ('unwrap_ambiguous', U*6), ('imu_missed_edges', U)]
                + [(n, F*k) for n, k in [('arm', 6), ('motor_velocity', 6), ('quaternion', 4),
                                        ('omega', 3), ('acceleration', 3), ('bias', 3), ('xy', 2), ('velocity', 2),
                                        ('q', 6), ('qd', 6), ('obs', 31), ('action', 6), ('target_q', 6),
                                        ('target_qd', 6), ('requested_motor_torque', 6),
                                        ('limited_motor_torque', 6), ('feedback', 6)]]
                + [(n, F) for n in ('odometry_confidence', 'wheel_residual', 'accel_weight',
                                   'imu_dt_s', 'torque_scale', 'balance_weight')]
                + [('dm', C.c_uint8*8), ('dji', C.c_uint8*8), ('end', U)])


class Ring(C.LittleEndianStructure):
    _fields_ = [(n, U) for n in ('schema', 'frame_bytes', 'capacity', 'published', 'overwritten', 'frozen')] + [('frame', Frame*CAPACITY)]


def decode(raw):
    if len(raw) != C.sizeof(Ring):
        raise ValueError(f'Wrong ring length: {len(raw)}, expected {C.sizeof(Ring)}')
    ring = Ring.from_buffer_copy(raw)
    if (ring.schema, ring.frame_bytes, ring.capacity) != (SCHEMA, C.sizeof(Frame), CAPACITY):
        raise ValueError('Unknown firmware telemetry ABI')
    rows, rejected = [], []
    for index, frame in enumerate(ring.frame):
        if frame.begin == frame.end == 0:
            continue
        if frame.begin != frame.end or frame.begin & 1:
            rejected.append(index)
            continue
        row = {}
        for name, _ in Frame._fields_:
            value = getattr(frame, name)
            row[name] = list(value) if isinstance(value, C.Array) else value
        row['motor_age_ms'] = [(frame.tick-t) & 0xffffffff for t in frame.motor_tick]
        row['imu_age_ms'] = (frame.tick-frame.imu_tick) & 0xffffffff
        row['remote_age_ms'] = (frame.tick-frame.remote_tick) & 0xffffffff
        row['active_submission'] = (frame.mode == 3 and not frame.inhibit and not frame.fault and frame.submitted_mask == 3)
        # DM feedback is protocol-reported torque; wheel feedback is current.
        row['dm_reported_canonical_nm'] = [SIGNS[i]*frame.feedback[i] for i in (0, 1, 3, 4)]
        row['wheel_reported_canonical_a'] = [SIGNS[i]*frame.feedback[i] for i in (2, 5)]
        rows.append(row)
    # Order by sequence age relative to the latest slot, including uint32 wrap.
    if rows:
        latest = min(rows, key=lambda r: ((ring.published*2-r['begin']) & 0xffffffff))['begin']
        rows.sort(key=lambda r: ((latest-r['begin']) & 0xffffffff), reverse=True)
    gaps = [((b['tick']-a['tick']) & 0xffffffff) for a, b in zip(rows, rows[1:])]
    return {'schema': 'chassis_telemetry_v2', 'evidence_kind': 'hardware_ram_window',
            'raw_sha256': hashlib.sha256(raw).hexdigest(), 'frame_bytes': ring.frame_bytes,
            'order': ORDER, 'published': ring.published, 'overwritten': ring.overwritten,
            'frozen': bool(ring.frozen), 'rejected_slots': rejected,
            'sample_count': len(rows), 'tick_gaps_ms': gaps,
            'continuous_2ms_window': bool(gaps) and all(x == 2 for x in gaps), 'samples': rows}


def summarize(report):
    active = [r for r in report['samples'] if r['active_submission']]
    ratios = {}
    for i in (0, 1, 3, 4):
        values = [SIGNS[i]*r['feedback'][i]/r['limited_motor_torque'][i]
                  for r in active if abs(r['limited_motor_torque'][i]) >= 1.0 and r['torque_scale'] >= .999]
        ratios[ORDER[i]] = {'count': len(values), 'median_reported_to_command_ratio': statistics.median(values) if values else None}
    return {'active_submission_samples': len(active), 'dm_ratios': ratios,
            'interpretation': 'Descriptive protocol feedback only, not independently measured shaft torque or identified Kt.',
            'can_submission_is_delivery_proof': False,
            'dynamic_latency_identified': False,
            'limited_or_ramped_samples': sum(any(abs(a-b) > .01 for a, b in zip(r['requested_motor_torque'], r['limited_motor_torque'])) for r in active)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    offline = sub.add_parser('decode'); offline.add_argument('binary', type=Path); offline.add_argument('--out', type=Path, required=True)
    capture = sub.add_parser('capture'); capture.add_argument('--out', type=Path, required=True)
    capture.add_argument('--openocd', type=Path)
    args = parser.parse_args()
    if args.command == 'capture':
        from monitor_can import ROOT, DEFAULT_OPENOCD, dump_ram, elf_symbols
        if args.out.exists():
            parser.error('Use a new output directory to preserve previous evidence')
        elf = ROOT/'build/rl/wheel_leg_rl.elf'
        symbols = elf_symbols(elf, ('g_telemetry',))
        if symbols['g_telemetry'][1] != C.sizeof(Ring):
            raise ValueError('ELF telemetry ABI mismatch; rebuild first')
        dump_ram(args.out, symbols, None, args.openocd or DEFAULT_OPENOCD, elf)
        binary = args.out/'a_g_telemetry.bin'; output = args.out/'result.json'
    else:
        binary, output = args.binary, args.out
    report = decode(binary.read_bytes())
    report['source_binary'] = str(binary.resolve())
    report['decoded_at'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    report['summary'] = summarize(report)
    if args.command == 'capture':
        report['elf_sha256'] = hashlib.sha256(elf.read_bytes()).hexdigest()
        report['openocd_log'] = str((args.out/'openocd.log').resolve())
        report['vector_fingerprint_matches'] = True
        report['full_flash_image_verified'] = False
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('sample_count', 'frozen', 'rejected_slots', 'continuous_2ms_window')}))


if __name__ == '__main__':
    main()
