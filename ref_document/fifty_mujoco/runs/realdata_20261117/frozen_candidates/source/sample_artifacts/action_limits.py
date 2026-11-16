"""Optional wider controller requests; knee actions and physical motor limits stay fixed."""
import math

def validate_wheel_limit(value):
    if type(value) not in (int,float) or not math.isfinite(value) or not 1<=value<=2.5:
        raise ValueError('Normalized wheel action limit must be finite in [1,2.5]')
    return float(value)

def validate_hip_limit(value):
    if type(value) not in (int,float) or not math.isfinite(value) or not 1<=value<=2:
        raise ValueError('Normalized hip action limit must be finite in [1,2]')
    return float(value)

def clip(ops,actions,wheel_limit,hip_limit=1.):
    result=ops.clip(actions,-1.,1.)
    if hip_limit!=1.:
        result[...,0]=ops.clip(actions[...,0],-hip_limit,hip_limit)
        result[...,3]=ops.clip(actions[...,3],-hip_limit,hip_limit)
    if wheel_limit!=1.:
        result[...,2]=ops.clip(actions[...,2],-wheel_limit,wheel_limit)
        result[...,5]=ops.clip(actions[...,5],-wheel_limit,wheel_limit)
    return result
