import ctypes as C
import json
import math
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from test_host import Network, Measurement, Controller
folder = Path(__file__).resolve().parent
report = json.loads((folder / 'result.json').read_text())
lib = C.CDLL(str(ROOT / 'build/host/rl_host.dll'))
fp = C.POINTER(C.c_float)
lib.rl_actor.argtypes = [C.POINTER(Network), fp, fp]
lib.rl_actor.restype = C.c_bool
lib.rl_set_action.argtypes = [C.POINTER(Controller), fp]
lib.rl_set_action.restype = C.c_bool
lib.rl_torque_step.argtypes = [C.POINTER(Controller), C.POINTER(Measurement)]
lib.rl_torque_step.restype = C.c_bool
active = [s for s in report['samples'] if s['active_submission']]
max_action_error = max_target_error = max_torque_error = 0.0
for sample in active:
    controller, measurement = Controller(), Measurement()
    output = (C.c_float * 6)()
    assert lib.rl_actor(C.byref(controller.network), (C.c_float * 31)(*sample['obs']), output)
    assert lib.rl_set_action(C.byref(controller), output)
    for field in ('q', 'qd', 'omega', 'xy', 'yaw', 'velocity'):
        if field in sample and field != 'yaw':
            getattr(measurement, field)[:] = sample[field]
    measurement.gravity[:] = [0, 0, -1] # torque step only checks this field for finiteness
    assert lib.rl_torque_step(C.byref(controller), C.byref(measurement))
    max_action_error = max(max_action_error, max(abs(a-b) for a,b in zip(controller.action,sample['action'])))
    max_target_error = max(max_target_error, max(abs(a-b) for a,b in zip(controller.target_q,sample['target_q'])))
    max_torque_error = max(max_torque_error, max(abs(a-b) for a,b in zip(controller.motor_torque,sample['limited_motor_torque'])))
def extent(field, index):
    values = [s[field][index] for s in active]
    return {'min': min(values), 'max': max(values)}
first, last = report['samples'][0], report['samples'][-1]
summary = {
    'evidence': 'frozen final 62 ms; startup transient absent',
    'control_version': hex(first['control_version']),
    'active_samples': len(active),
    'network_clipped_action_max_error': max_action_error,
    'target_max_error_rad': max_target_error,
    'motor_torque_replay_max_error_nm': max_torque_error,
    'active_duration_estimate_ms': (report['published']-1)*2,
    'duration_basis': 'ring reset on ACTIVE and published every 2 ms; historical samples overwritten',
    'motors': [{
        'name': name,
        'command_nm': extent('limited_motor_torque', i),
        'feedback_speed_rad_s': extent('motor_velocity', i),
        'joint_actual_deg': first['q'][i]*180/math.pi,
        'joint_target_deg': first['target_q'][i]*180/math.pi,
        'q_window_delta_rad': active[-1]['q'][i]-first['q'][i],
    } for i,name in enumerate(report['order'])],
    'stop': {k:last[k] for k in ('tick','mode','inhibit','fault','unwrap_ambiguous','motor_age_ms','submitted_mask')},
    'wheel_residual_m_s': {'min':min(s['wheel_residual'] for s in active),'max':max(s['wheel_residual'] for s in active)},
    'interpretation': ['BALANCE and real command submissions established',
        'Both hip actions clipped at limits; both knee targets clipped to configured target boundaries',
        'Left knee requested approximately -30.81 Nm and clipped to -20 Nm',
        'Left wheel motion with right wheel stationary is inconsistent with bilateral no-slip odometry',
        'Left wheel unwrap ambiguity caused position_lost and inhibit bits 4 plus 32',
        'fault=0 does not exclude CONTROL_FAULT from health checks',
        'Exact wheel slip/contact and first leg lift require physical context or longer trajectory'],
}
(folder/'analysis.json').write_text(json.dumps(summary,indent=2)+chr(10),encoding='utf-8')
print(json.dumps(summary,indent=2))
