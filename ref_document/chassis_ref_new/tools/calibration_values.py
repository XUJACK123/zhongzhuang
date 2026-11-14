#!/usr/bin/env python3
"""Calculate calibration macros from a MEASURED reference pose. No hardware I/O.

Inputs use policy order: right hip DM4, right knee DM3, right wheel DJI3,
left hip DM1, left knee DM2, left wheel DJI2. Counts are raw 0..8191.
Pose angles are URDF generalized coordinates, NOT motor output-arm angles.
"""
import argparse
import json
import math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def arm_from_joint(q):
    """Joint-space reference pose to canonical motor output-arm angles."""
    arm=list(q)
    for h,sign in ((0,1),(3,-1)):
        beta=sign*q[h+1]+math.pi/2
        alpha=2*math.atan2(math.sin(beta),113.4/135+math.cos(beta))
        if not -2*math.pi+.001<alpha<-.001: raise ValueError('pose exits the assembly branch')
        arm[h]=sign*q[h]; arm[h+1]=arm[h]+alpha
    return arm
def macros(counts,signs,q):
    raw=[n*2*math.pi/8192/(15.88 if i in (2,5) else 1) for i,n in enumerate(counts)]
    return {"ROBOT_ENCODER_SIGNS":list(signs),"ROBOT_ENCODER_REFERENCE_RAD":raw,
            "ROBOT_ARM_REFERENCE_RAD":arm_from_joint(q)}
def define_line(name,values):
    return "#define "+name+" {"+", ".join(f"{v:.10f}f" for v in values)+"}"
def measured_pose(right_thigh,right_included,left_thigh,left_included):
    """Thigh: body +X toward -Z is positive. Included: knee-to-hip vs knee-to-wheel."""
    values=(right_thigh,right_included,left_thigh,left_included)
    if not all(map(math.isfinite,values)) or not 0<right_included<180 or not 0<left_included<180:
        raise ValueError('Included knee angles must be finite in (0,180) degrees')
    return [math.radians(v) for v in
            (right_thigh,right_included-270,0,-left_thigh,270-left_included,0)]
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--counts",type=int,nargs=6,required=True)
    p.add_argument("--signs",type=int,nargs=6,required=True)
    pose=p.add_mutually_exclusive_group(required=True)
    pose.add_argument("--joint-deg",type=float,nargs=6)
    pose.add_argument("--at-training-default",action="store_true",
                      help="Use ONLY if the robot is physically placed in the archived default pose")
    pose.add_argument('--measured-leg-angles',type=float,nargs=4,
                      metavar=('RIGHT_THIGH','RIGHT_INCLUDED','LEFT_THIGH','LEFT_INCLUDED'),
                      help='Measured physical leg angles in degrees; see docs/COMMISSIONING.md')
    a=p.parse_args()
    if any(n<0 or n>=8192 for n in a.counts) or any(s not in (-1,1) for s in a.signs):
        p.error("counts must be 0..8191; signs must be -1 or +1")
    manifest=json.loads((ROOT/"user/algorithm/model/manifest.json").read_text())
    if a.measured_leg_angles:
        try: q=measured_pose(*a.measured_leg_angles)
        except ValueError as e: p.error(str(e))
    elif a.at_training_default: q=manifest["default_q"]
    else: q=[math.radians(x) for x in a.joint_deg]
    if not all(map(math.isfinite,q)): p.error("nonfinite pose")
    try: values=macros(a.counts,a.signs,q)
    except ValueError as e: p.error(str(e))
    for name in ("ROBOT_ENCODER_SIGNS","ROBOT_ENCODER_REFERENCE_RAD","ROBOT_ARM_REFERENCE_RAD"):
        print(define_line(name,values[name]))
    print("/* Measured reference pose only. Commissioning/output flags remain 0. */")
if __name__=="__main__": main()
