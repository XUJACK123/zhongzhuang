"""Ground get-up lifecycle and measured stability, shared by both simulators."""
import math


def validate(cfg):
    positive=('target_clearance_m','deadline_s','stable_hold_s','height_tolerance_m',
              'linear_speed_m_s','angular_speed_rad_s','tilt_deg','support_fraction')
    if any(not math.isfinite(cfg[k]) or cfg[k]<=0 for k in positive):raise ValueError('Invalid get-up criteria')
    if cfg['stable_hold_s']>=cfg['deadline_s']:raise ValueError('Stability hold must fit inside get-up deadline')
    if cfg['target_clearance_m']<=cfg['height_tolerance_m']:raise ValueError('Ground contact cannot count as standing')
    if not 0<cfg['support_fraction']<=1:raise ValueError('Invalid required wheel support')
    if len(cfg['reset_joint_angles_rad'])!=6 or not all(math.isfinite(q) for q in cfg['reset_joint_angles_rad']):
        raise ValueError('Six finite policy-order reset angles required')
    stage=cfg.get('compression_before_hip')
    if stage:
        if not 23.85<stage['included_angle_deg']<90 or not 0<stage['hold_s']<cfg['deadline_s']:
            raise ValueError('Invalid knee compression transition')


def initial_state(template):
    zero=template*0
    return dict(released=zero>0,hold_s=zero,first_stable_s=zero-1,
                compressed=zero>0,compression_hold_s=zero,first_compressed_s=zero-1)


def advance(state,elapsed,height,gravity,linear,angular,wheel_vertical,base_contact,position_error,weight_n,cfg,dt,ops,q=None):
    """No state teleport and no grace after lift; initial floor support is explicit."""
    speed=(linear*linear).sum(-1)**.5;angular_speed=(angular*angular).sum(-1)**.5
    both_wheels=(wheel_vertical>1).all(-1)
    stage=cfg.get('compression_before_hip');compression_progress=height*0
    if stage:
        if q is None:raise ValueError('Joint states required for knee-first recovery')
        right=(q[...,1]+1.5*math.pi)*180/math.pi
        left=(-q[...,4]+1.5*math.pi)*180/math.pi
        angle=ops.maximum(right,left)
        small=(angle<=stage['included_angle_deg'])&(ops.minimum(right,left)>=23.85-.006)
        state['compression_hold_s']=ops.where(small,state['compression_hold_s']+dt,height*0)
        state['compressed']=state['compressed']|(state['compression_hold_s']>=stage['hold_s']-1e-6)
        state['first_compressed_s']=ops.where((state['first_compressed_s']<0)&state['compressed'],elapsed,state['first_compressed_s'])
        start=(cfg['reset_joint_angles_rad'][1]+1.5*math.pi)*180/math.pi
        compression_progress=ops.clip((start-angle)/(start-stage['included_angle_deg']),0.,1.)
        compression_progress=ops.where(state['compressed'],compression_progress*0+1,compression_progress)
    stable=((ops.abs(height-cfg['target_clearance_m'])<=cfg['height_tolerance_m'])
        &(-gravity[...,2]>=math.cos(math.radians(cfg['tilt_deg'])))
        &(speed<=cfg['linear_speed_m_s'])&(angular_speed<=cfg['angular_speed_rad_s'])
        &both_wheels
        &(wheel_vertical.sum(-1)>=cfg['support_fraction']*weight_n)
        &(base_contact<=10)&(position_error<=.05))
    if stage:stable=stable&state['compressed']
    state['released']=state['released']|(height>.02)
    state['hold_s']=ops.where(stable,state['hold_s']+dt,state['hold_s']*0)
    reached=state['hold_s']>=cfg['stable_hold_s']-1e-6
    state['first_stable_s']=ops.where((state['first_stable_s']<0)&reached,elapsed,state['first_stable_s'])
    missed=(elapsed>=cfg['deadline_s']-1e-6)&((state['first_stable_s']<0)|(state['first_stable_s']>cfg['deadline_s']+1e-6))
    fail=((height<-.005)|(-gravity[...,2]<math.cos(math.radians(30)))
        |(state['released']&(base_contact>10))|missed
        |((state['first_stable_s']>=0)&(height<cfg['target_clearance_m']-.02)))
    return dict(stable=stable,failed=fail,progress=ops.clip(height/cfg['target_clearance_m'],0.,1.),
                first_stable_s=state['first_stable_s'],released=state['released'],compression_progress=compression_progress)
