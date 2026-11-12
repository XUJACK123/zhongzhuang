"""Bounded pose-error velocity targets for rewards, never actuator commands."""
import math

KEYS=('position_gain_per_s','heading_gain_per_s','lateral_gain_per_m2',
      'max_speed_correction_m_s','max_yaw_correction_rad_s',
      'speed_limit_m_s','yaw_limit_rad_s','yaw_sigma_rad_s')

def validate(settings):
    if settings is None:return None
    if not isinstance(settings,dict) or set(settings)!=set(KEYS):
        raise ValueError('Explicit complete pose-feedback reward settings required')
    if any(type(settings[k]) not in (float,int) or not math.isfinite(settings[k]) or settings[k]<=0 for k in KEYS):
        raise ValueError('Pose-feedback reward values must be positive finite numbers')
    return dict(settings)

def targets(forward,yaw_rate,position,reference,yaw,reference_yaw,settings,ops):
    error=reference-position
    c,s=ops.cos(yaw),ops.sin(yaw)
    ex=c*error[...,0]+s*error[...,1]
    ey=-s*error[...,0]+c*error[...,1]
    angle=ops.arctan2(ops.sin(reference_yaw-yaw),ops.cos(reference_yaw-yaw))
    dv=forward*(ops.cos(angle)-1)+settings['position_gain_per_s']*ex
    dw=settings['heading_gain_per_s']*angle+forward*settings['lateral_gain_per_m2']*ey
    v=forward+ops.clip(dv,-settings['max_speed_correction_m_s'],settings['max_speed_correction_m_s'])
    w=yaw_rate+ops.clip(dw,-settings['max_yaw_correction_rad_s'],settings['max_yaw_correction_rad_s'])
    return ops.clip(v,-settings['speed_limit_m_s'],settings['speed_limit_m_s']),ops.clip(w,-settings['yaw_limit_rad_s'],settings['yaw_limit_rad_s'])
