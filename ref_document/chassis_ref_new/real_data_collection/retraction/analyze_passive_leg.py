"""Offline raw-encoder reconstruction; cached joint q may be stale after wheel loss."""
import argparse
import json
import math
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('trace',type=Path)
p.add_argument('--out',type=Path,required=True)
a=p.parse_args()
d=json.loads((a.trace/'result.json').read_text(encoding='utf-8'))
rows=d['samples']
assert rows and d['frozen'] and not d['rejected_slots']
assert all(r['mode']==0 and all(t==0 for t in r['torque']) for r in rows)
refs=[2.7511945431,2.4643401357,0,1.0385049934,2.8731460157,0]
arm_refs=[3.1500981078,1.5822922061,0,3.1500981078,1.5822922061,0]
signs=[-1,1,-1,1,1,1]
indices=[0,1,3,4]
counts={i:[] for i in indices}
previous=None
derived=[]
def knee(alpha):
    if not -2*math.pi<alpha<0: return None
    return math.degrees(math.pi+alpha/2+math.asin(.84*math.sin(alpha/2)))
for r in rows:
    arm={};angle={}
    for i in indices:
        raw=r['raw'][i]
        if previous is None:
            value=refs[i]+math.remainder(raw*2*math.pi/8192-refs[i],2*math.pi)
            total=round(value*8192/(2*math.pi))
        else:
            delta=(raw-previous['raw'][i]+4096)%8192-4096
            total=counts[i][-1]+delta
        counts[i].append(total)
        angle[i]=total*2*math.pi/8192
        arm[i]=arm_refs[i]+signs[i]*(angle[i]-refs[i])
    candidate_knee_arm=arm_refs[1]-(angle[1]-refs[1])
    derived.append(dict(tick=r['tick'],raw=r['raw'],
        right_hip_deg=math.degrees(arm[0]),
        right_knee_current_deg=knee(arm[1]-arm[0]),
        right_knee_minus1_hypothesis_deg=knee(candidate_knee_arm-arm[0]),
        left_knee_current_deg=knee(arm[4]-arm[3])))
    previous=r

def limits(values):
    valid=[v for v in values if v is not None]
    return dict(min=min(valid),max=max(valid),peak_to_peak=max(valid)-min(valid)) if valid else None
x,y=counts[0],counts[1]
mx,my=sum(x)/len(x),sum(y)/len(y)
xx=sum((v-mx)**2 for v in x);yy=sum((v-my)**2 for v in y)
xy=sum((v-mx)*(w-my) for v,w in zip(x,y))
report=dict(kind='zero-command manual right-leg movement',
    source=str(a.trace).replace('\\','/'),frames=len(rows),span_ms=d['span_ms'],
    raw_interval_ms=sorted(set(d['tick_gaps_ms'])),
    operator_action='Raise whole robot-own right leg, hold about 2 s, return; keep knee angle fixed; left leg unchanged.',
    cached_joint_measurements_used=False,
    raw_leg_frames_max_age_ms=max(r['motor_age_ms'][i] for r in rows for i in indices),
    counter_unwrap='nearest raw-count delta at 120 ms during requested slow small manual motion; no claim of high-speed unwrap validation',
    raw_count_ranges={str(i):limits(counts[i]) for i in indices},
    knee_vs_hip_raw_slope=xy/xx if xx else None,
    knee_vs_hip_raw_correlation=xy/math.sqrt(xx*yy) if xx and yy else None,
    angle_ranges={key:limits([r[key] for r in derived]) for key in derived[0] if key not in ('tick','raw')},
    recorded_inhibit_values=sorted(set(r['inhibit'] for r in rows)),
    nominal_link_lengths_mm=[210,250],
    operator_confirmed_hip_to_wheel_mm=[412,412],
    ruler_derived_knee_deg=math.degrees(math.acos((210**2+250**2-412**2)/(2*210*250))),
    external_angle_uncertainty_deg=None,
    limitations=['manual fixed-knee assertion and ruler measurements are operator evidence, no synchronized external angle stream',
        'common raw rotation constrains the two right encoder signs to agree under fixed knee; -1 knee hypothesis assumes existing -1 hip sign. This trace alone does not uniquely determine absolute hip sign',
        'uses accepted planar 210/250 mm link lengths; lateral offsets and ruler error not identified',
        'minus1 is an offline sign hypothesis; absolute hip reference and torque direction not recalibrated',
        'raw gyro/static-statistics fields from this moving interval are not a stationary noise dataset'],
    firmware_or_calibration_modified=False,samples=derived)
a.out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('samples','limitations')},ensure_ascii=False))
