"""Reproduce the 2026-11-17 static field report from retained raw/decoded records."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STATIC = ROOT/'real_data_collection/supported_static_20261117_01'
WINDOWS = ROOT/'real_data_collection/supported_static_windows_20261117_02'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    stats = read(STATIC/'result.json')
    windows = [read(p) for p in sorted(WINDOWS.glob('window_*/result.json'))]
    rows = [row for window in windows for row in window['samples']]
    after = read(WINDOWS/'after/result.json')['b']
    doubles = [read(STATIC/name/'result.json') for name in ['before','reset_1','reset_2','reset_3']]
    double_growth = [[r['b']['nodes'][i]['received']-r['a']['nodes'][i]['received']
                      for i in range(6)] for r in doubles]
    report = {
        'evidence_kind': 'supported_static_hardware_measurements_with_explicit_partial_coverage',
        'date': '2026-11-17', 'session': read(STATIC/'session.json'),
        'static': stats, 'double_snapshot_receive_deltas': double_growth,
        'coherent_windows': {
            'count': len(windows), 'frames': len(rows),
            'span_ms_each': [((w['samples'][-1]['tick']-w['samples'][0]['tick']) & 0xffffffff) for w in windows],
            'all_internal_gaps_2ms': all(w['continuous_2ms_window'] for w in windows),
            'continuous_across_windows': False,
            'motor_age_max_ms': [max(r['motor_age_ms'][i] for r in rows) for i in range(6)],
            'imu_age_max_ms': max(r['imu_age_ms'] for r in rows),
            'remote_age_max_ms': max(r['remote_age_ms'] for r in rows),
            'abs_joint_speed_max_rad_s': [max(abs(r['qd'][i]) for r in rows) for i in range(6)],
            'odometry_confidence_min': min(r['odometry_confidence'] for r in rows),
            'abs_forward_speed_max_m_s': max(abs(r['velocity'][0]) for r in rows),
            'abs_omega_max_rad_s': max(abs(v) for r in rows for v in r['omega']),
            'mode_values': sorted({r['mode'] for r in rows}),
            'inhibit_values': sorted({r['inhibit'] for r in rows}),
            'startup_wait_cause': 'Not isolated; eligible short windows do not cover preceding 500 ms dwell or reset events',
            'initial_unfrozen_read': 'supported_static_window_20261117_01: 32 torn slots rejected, zero valid frames; retained as failed capture',
            'recording_restored': read(WINDOWS/'recording_restored/result.json')['frozen'] == 0,
        },
        'last_observed_state': {'tick': after['chassis']['tick'], 'mode': after['chassis']['mode'],
            's1': after['remote']['s1'], 'inhibit': after['chassis']['inhibit'],
            'fault': after['chassis']['fault'], 'nonzero_frames': after['nonzero_torque_frames'],
            'enable_commands': after['enable_commands'], 'stop_loop_max_us': after['chassis']['max_loop_us'],
            'policy_runs': after['chassis']['policy_runs']},
        'progress': {
            'T0': 'partial: 4 double snapshots, 19 sparse static samples, 3 coherent 62 ms windows; no continuous 60 s stream or ACTIVE timing distribution',
            'T1': 'supported pose only; no natural unassisted rest or contact force truth',
            'T2': '3 MCU soft resets repeat exactly at sampled pose; not full power-cycle, approach-direction, or absolute reference calibration',
            'T3': 'deferred independent shaft torque; operator confirms no force/torque instrument',
            'T4': 'accepted nominal 15 MPa curve retained; not new measured force/hysteresis',
            'T5': 'partial warm stationary samples; adaptive bias enabled, no cold/dynamic/external angle truth',
            'T6': 'static estimated XY drift measured; accepted 700 mm / 2 turns and left symmetry retained without retest',
            'T7': 'no new loss in static session after reset; previous high-speed unwrap fault remains unresolved',
            'T8': 'prior supported attempt recorded, no stand achieved; final 62 ms only, no additional ACTIVE trial in this session',
        },
        'previous_active_trial': 'diagnostics/supported_trial_20261117_01/result.json',
        'limitations': ['No external force, voltage, contact, or angle measurement in this session',
            'No absolute zero or drivetrain calibration changed',
            'MCU reset clears position_lost latch but does not fix high-speed unwrap cause',
            'STOP loop time is not ACTIVE policy execution timing',
            'SWD snapshots of different structures are asynchronous; ages use coherent short frames only'],
    }
    (STATIC/'combined_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n',encoding='utf-8')
    for folder in [STATIC, WINDOWS]:
        manifest = {p.relative_to(folder).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in folder.rglob('*') if p.is_file() and p.name != 'artifact_manifest.json'}
        (folder/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'double_receive_deltas':double_growth,'windows':report['coherent_windows'],
                      'last_observed_state':report['last_observed_state']},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
