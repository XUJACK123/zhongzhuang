"""Offline bounded-trial analysis; reconstruct PD targets with the matching C build."""
import argparse
import ctypes as C
import json
import math
from pathlib import Path
from test_host import Controller,Measurement

def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('folder',type=Path); args=p.parse_args()
    record=json.loads((args.folder/'result.json').read_text())
    rows=record['samples']; active=[r for r in rows if r['mode']==3]
    result={'sample_count':len(rows),'active_count':len(active),'continuous_2ms':record['continuous_2ms'],
        'guard':record['guard'],'entry_counts':record['entry_counts'],
        'first_tick':rows[0]['tick'] if rows else None,'last_tick':rows[-1]['tick'] if rows else None,
        'last_mode':rows[-1]['mode'] if rows else None,'last_fault':rows[-1]['fault'] if rows else None,
        'last_inhibit':rows[-1]['inhibit'] if rows else None,
        'events':[dict(tick=r['tick'],mode=r['mode'],fault=r['fault'],inhibit=r['inhibit'],entry_block=r['entry_block'])
                  for i,r in enumerate(rows) if not i or (r['mode'],r['fault'])!=(rows[i-1]['mode'],rows[i-1]['fault'])],
        'limits':['Protocol feedback is not independent shaft torque','Physical posture/contact require external observation',
                  'Retained trace may predate boot: compare reset flags, firmware archive and timestamps']}
    if active:
        root=Path(__file__).resolve().parents[1];lib=C.CDLL(str(root/'build/host/rl_host.dll'))
        fp=C.POINTER(C.c_float)
        lib.rl_set_action.argtypes=[C.POINTER(Controller),fp];lib.rl_set_action.restype=C.c_bool
        lib.rl_torque_step.argtypes=[C.POINTER(Controller),C.POINTER(Measurement)];lib.rl_torque_step.restype=C.c_bool
        targets=[];prelimit=[];error=0
        wheel_cap=record.get('software_session',{}).get('applied_wheel_cap_nm',2.0)
        for row in active:
            ctl=Controller();m=Measurement()
            m.q[:]=row['q'];m.qd[:]=row['qd'];m.gravity[:]=row['gravity'];m.omega[:]=row['omega']
            assert lib.rl_set_action(C.byref(ctl),(C.c_float*6)(*row['action']))
            assert lib.rl_torque_step(C.byref(ctl),C.byref(m))
            for index in (2,5): ctl.motor_torque[index]=max(-wheel_cap,min(wheel_cap,ctl.motor_torque[index]))
            scale=min(1,((row['tick']-active[0]['tick'])&0xffffffff)/200)
            error=max(error,max(abs(ctl.motor_torque[i]*scale-row['torque'][i]) for i in range(6)))
            targets.append(list(ctl.target_q));prelimit.append(list(ctl.motor_torque))
        def ranges(key): return [[min(r[key][i] for r in active),max(r[key][i] for r in active)] for i in range(6)]
        result.update(active_first_tick=active[0]['tick'],active_last_tick=active[-1]['tick'],
            guard_header_matches_trace=record['guard']['started']==active[0]['tick'],
            active_span_ms=(active[-1]['tick']-active[0]['tick'])&0xffffffff,
            first_q=active[0]['q'],last_q=active[-1]['q'],first_action=active[0]['action'],last_action=active[-1]['action'],
            first_target_q=targets[0],last_target_q=targets[-1],torque_range_nm=ranges('torque'),
            protocol_feedback_range=ranges('feedback'),motor_speed_range_rad_s=ranges('motor_velocity'),
            joint_delta_deg=[math.degrees(active[-1]['q'][i]-active[0]['q'][i]) for i in range(6)],
            replay_max_torque_error_nm=error,
            inference_us_range=[min(r['inference_us'] for r in active),max(r['inference_us'] for r in active)],
            loop_us_max=max(r['loop_us'] for r in active),
            motor_age_max_ms=[max(r['motor_age_ms'][i] for r in active) for i in range(6)],
            max_wheel_residual_m_s=max(r['residual'] for r in active))
    (args.folder/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
