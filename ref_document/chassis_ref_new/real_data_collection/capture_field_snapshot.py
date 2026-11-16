"""Capture a stationary endpoint from the existing zero-output diagnostic.

Read-only board access: no flash, reset, halt, resume, token or CAN writes.
The on-site operator holds S1 down and stops before each requested endpoint.
"""
import argparse
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import re
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'real_data_collection'
HERE = BASE / 'identification'
sys.path.insert(0, str(HERE))
import operate as op


def capture(tag):
    if not re.fullmatch(r'[A-Za-z0-9_]+', tag):
        raise ValueError('Use an ASCII filename tag')
    out = Path((BASE / 'latest_field_session.txt').read_text(encoding='utf-8').strip())
    if (out / (tag + '.json')).exists():
        raise ValueError('Endpoint tag already exists; preserve the old reading')
    archive = Path((HERE / 'latest_trial.txt').read_text(encoding='utf-8').strip())
    op.c.OUT, op.c.RAW = out, out / 'raw'
    op.S = op.m.elf_symbols(archive / 'diagnostic.elf', op.NAMES)
    with contextlib.redirect_stdout(io.StringIO()):
        result = op.status(tag)
    bank = (out / 'raw' / (tag + '_g_motor_snapshot.bin')).read_bytes()
    imu = (out / 'raw' / (tag + '_g_imu.bin')).read_bytes()
    for i, motor in enumerate(result['motors']):
        motor['encoder_total_counts'] = struct.unpack_from('<q', bank, i * 40 + 8)[0]
    result['imu_raw_snapshot'] = {
        'quaternion_wxyz': struct.unpack_from('<4f', imu, 0),
        'omega_body_rad_s': struct.unpack_from('<3f', imu, 16),
        'accel_body_m_s2': struct.unpack_from('<3f', imu, 28),
        'tick': struct.unpack_from('<I', imu, 92)[0],
        'samples': struct.unpack_from('<I', imu, 96)[0],
    }
    gravity = result['gravity_body']
    norm = math.sqrt(sum(x * x for x in gravity))
    result['imu_tilt_deg'] = math.degrees(math.acos(max(-1, min(1, -gravity[2] / norm))))
    result['zero_output_preconditions_pass'] = (
        result['phase'] == 'IDLE' and result['request_token'] == 0
        and result['s1'] == 2 and result['can']['nonzero'] == 0
        and result['can']['enable'] == 0
    )
    result['capture_kind'] = 'asynchronous read-only SWD; stationary endpoint requested'
    op.save(out / (tag + '.json'), result)
    if not result['zero_output_preconditions_pass']:
        raise RuntimeError('Endpoint is not in the expected zero-output state; inspect saved data')
    main_path = BASE / 'real_data_full_tests_20261116_165832.json'
    main = json.loads(main_path.read_text(encoding='utf-8'))
    main['latest_field_session'].setdefault('named_snapshots', {})[tag] = result
    op.save(main_path, main)
    op.save(out / 'artifact_manifest.json', [
        {'path': str(p.relative_to(out)), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
        for p in sorted(out.rglob('*')) if p.is_file() and p.name != 'artifact_manifest.json'
    ])
    print(json.dumps({
        'tag': tag, 'phase': result['phase'], 's1': result['s1'],
        'healthy': result['healthy_at_control_step'],
        'raw': [m['raw'] for m in result['motors']],
        'total_counts': [m['encoder_total_counts'] for m in result['motors']],
        'position_lost': [m['position_lost'] for m in result['motors']],
        'imu_tilt_deg': result['imu_tilt_deg'],
        'nonzero': result['can']['nonzero'], 'enable': result['can']['enable'],
    }, ensure_ascii=True))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag', required=True)
    capture(parser.parse_args().tag)
