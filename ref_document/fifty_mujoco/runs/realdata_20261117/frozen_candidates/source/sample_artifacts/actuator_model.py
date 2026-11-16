"""Chassis-mounted coupled leg actuators; ideal virtual-work transmission.

The speed envelope and 15.88 gearbox rescaling are explicit approximations.
Catalog output torque already includes the DM8009P's internal 9:1 gearbox.
"""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ORDER=['rf0_Joint','rf1_Joint','r_wheel_Joint','lf0_Joint','lf1_Joint','l_wheel_Joint']


def load_hardware(path=None):
    path=path or Path(__file__).resolve().parents[1]/'config/actuator_hardware.json'
    cfg=json.loads(Path(path).read_text(encoding='utf-8'))
    if cfg['supply_voltage_v']!=24 or not cfg['user_confirmed']['both_leg_motors_fixed_to_chassis']:
        raise ValueError('This actuator profile requires the confirmed 24 V chassis-mounted arrangement')
    if cfg['training_envelope']['mode']!='rated_torque_only':
        raise ValueError('Uncalibrated peak/thermal mode is unsupported')
    if not 0<cfg['linkage']['active_arm_m']<cfg['linkage']['passive_arm_m']:
        raise ValueError('Invalid linkage dimensions')
    return cfg


def bend_from_alpha(alpha, cfg, ops=math):
    r=cfg['linkage']['active_arm_m']/cfg['linkage']['passive_arm_m']
    return alpha/2+ops.asin(r*ops.sin(alpha/2))


def alpha_from_bend(beta, cfg, ops=math):
    r=cfg['linkage']['active_arm_m']/cfg['linkage']['passive_arm_m']
    return 2*ops.atan2(ops.sin(beta),r+ops.cos(beta))


def knee_ratio(beta,cfg,ops=math):
    a=alpha_from_bend(beta,cfg,ops)
    r=cfg['linkage']['active_arm_m']/cfg['linkage']['passive_arm_m']
    return .5+r*ops.cos(a/2)/(2*ops.sqrt(1-r*r*ops.sin(a/2)**2))


def motor_ratings(cfg):
    leg=cfg['leg_motor']; wheel=cfg['wheel_motor']; ref=wheel['reference_p19']
    ratio=wheel['total_reduction_ratio']/(ref['reduction_ratio_numerator']/ref['reduction_ratio_denominator'])
    return dict(leg_torque_nm=leg['rated_torque_nm'],leg_rated_rad_s=leg['rated_speed_rpm_at_24v']*math.pi/30,
                leg_no_load_rad_s=leg['no_load_speed_rpm_at_24v']*math.pi/30,
                wheel_torque_nm=ref['rated_output_torque_nm']*ratio,
                wheel_rated_rad_s=ref['rated_output_speed_rpm']/ratio*math.pi/30,
                wheel_no_load_rad_s=ref['no_load_output_speed_rpm']/ratio*math.pi/30)


def joint_envelope_bounds(params,cfg):
    ratings=motor_ratings(cfg); efforts={}; velocities={}; ratios={}
    for side,sign in (('r',1),('l',-1)):
        hip,knee=side+'f0_Joint',side+'f1_Joint'
        lo,hi=params['joint_limits_rad'][knee]
        # k decreases with |alpha| on the confirmed assembly branch.
        betas=[sign*q+math.pi/2 for q in (lo,hi)]
        if any(not -2*math.pi<alpha_from_bend(beta,cfg)<0 for beta in betas):
            raise ValueError('Knee range exits the supplied linkage assembly branch')
        ks=[knee_ratio(beta,cfg) for beta in betas]
        ratios[knee]=[min(ks),max(ks)]
        efforts[hip]=2*ratings['leg_torque_nm']  # generalized torque, not one motor's rating
        efforts[knee]=ratings['leg_torque_nm']/min(ks)
        velocities[hip]=ratings['leg_no_load_rad_s']
        velocities[knee]=2*max(ks)*ratings['leg_no_load_rad_s']
    for wheel in ('r_wheel_Joint','l_wheel_Joint'):
        efforts[wheel]=ratings['wheel_torque_nm']; velocities[wheel]=ratings['wheel_no_load_rad_s']
    return dict(effort_nm=efforts,velocity_rad_s=velocities,knee_ratio_range=ratios,
                note='Component-wise outer bounds only; runtime coupled motor constraints are stricter.')


def validate_hardware_asset(params,urdf_path,cfg):
    if abs(params['wheel_radius_m']-cfg['wheel_diameter_m']/2)>1e-12:
        raise ValueError('Asset wheel radius differs from confirmed hardware; rebuild the URDF')
    expected=joint_envelope_bounds(params,cfg)
    root=ET.parse(urdf_path).getroot()
    if cfg.get('hip_continuous',False):
        for hip in ('rf0_Joint','lf0_Joint'):
            joint=root.find("joint[@name='%s']" % hip)
            if params['joint_limits_rad'][hip] is not None or joint.get('type')!='continuous':
                raise ValueError('Hip must allow continuous rotation')
    for name in ORDER:
        limit=root.find("joint[@name='%s']/limit" % name)
        for attribute,key in (('effort','effort_nm'),('velocity','velocity_rad_s')):
            if limit is None or abs(float(limit.get(attribute))-expected[key][name])>1e-7:
                raise ValueError('Stale actuator %s limit for %s; rebuild the URDF' % (attribute,name))
    return expected


class CoupledActuators:
    def __init__(self,torch,cfg,dof_names):
        if list(dof_names)!=ORDER:
            raise ValueError('Actuator model requires canonical right-leg-first policy order')
        self.torch,self.cfg=torch,cfg
        self.ratings=motor_ratings(cfg)
        self._limits={}

    def to_motor(self,joint_torque,q,qd):
        motor_torque=joint_torque.clone(); motor_velocity=qd.clone(); ratios=[]
        for h,k,sign in ((0,1,1),(3,4,-1)):
            ratio=knee_ratio(sign*q[...,k]+math.pi/2,self.cfg,self.torch)
            ratios.append(ratio)
            motor_torque[...,h]=sign*(joint_torque[...,h]-ratio*joint_torque[...,k])
            motor_torque[...,k]=sign*ratio*joint_torque[...,k]
            motor_velocity[...,h]=sign*qd[...,h]
            motor_velocity[...,k]=sign*(qd[...,h]+qd[...,k]/ratio)
        return motor_torque,motor_velocity,ratios

    def from_motor(self,torque,ratios):
        joint=torque.clone()
        for (h,k,sign),ratio in zip(((0,1,1),(3,4,-1)),ratios):
            joint[...,h]=sign*(torque[...,h]+torque[...,k])
            joint[...,k]=sign*torque[...,k]/ratio
        return joint

    def apply(self,requested,q,qd,output_gain=None):
        torch=self.torch
        motor,velocity,ratios=self.to_motor(requested,q,qd)
        key=(requested.device,requested.dtype)
        if key not in self._limits:
            r=self.ratings
            self._limits[key]=tuple(torch.tensor([r['leg_'+suffix],r['leg_'+suffix],r['wheel_'+suffix]]*2,
                device=requested.device,dtype=requested.dtype) for suffix in ('torque_nm','rated_rad_s','no_load_rad_s'))
        torque,rated,free=self._limits[key]
        positive=torque*torch.clamp((free-velocity)/(free-rated),0,1)
        negative=-torque*torch.clamp((free+velocity)/(free-rated),0,1)
        applied=torch.maximum(torch.minimum(motor,positive),negative)
        if output_gain is not None:
            # Delivered motor error, not independent generalized joint gains.
            # Keep the original speed-dependent hardware envelope after error.
            applied=torch.maximum(torch.minimum(applied*output_gain,positive),negative)
        self.motor_torque_requested=motor; self.motor_torque_applied=applied; self.motor_velocity=velocity
        return self.from_motor(applied,ratios)
