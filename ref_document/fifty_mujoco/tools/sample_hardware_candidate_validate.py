#!/usr/bin/env python3
"""Validate generated C observation/target/PD behavior against the candidate model."""
import argparse,ctypes as c,json,math,sys
from pathlib import Path
import numpy as np
PROJECT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PROJECT/'src'))
from fifty_mujoco.contracts.sample_actor31 import build_observation,ramp_commands,action_targets,advance_reference
from fifty_mujoco.envs.sample_env31_hardware_guarded import GuardedHardwareEnv31


class Measurement(c.Structure):
    _fields_=[('q',c.c_float*6),('qd',c.c_float*6),('omega',c.c_float*3),('gravity',c.c_float*3),
              ('xy',c.c_float*2),('yaw',c.c_float),('velocity',c.c_float*2)]


class Controller(c.Structure):
    _fields_=[('network',c.c_float*224),('obs',c.c_float*31),('action',c.c_float*6),
        ('command',c.c_float*3),('reference_xy',c.c_float*2),('reference_yaw',c.c_float),
        ('target_q',c.c_float*6),('target_qd',c.c_float*6),('joint_torque',c.c_float*6),('motor_torque',c.c_float*6)]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    manifest=json.loads((a.bundle/'manifest.json').read_text());kd=manifest['wheel_kd']
    env=GuardedHardwareEnv31(wheel_kd=kd,knee_margin_rad=manifest['environment_contract']['knee_margin_rad'])
    lib=c.CDLL(str((a.bundle/'host_actor.so').resolve()));ptr=c.POINTER(c.c_float)
    lib.rl_reset.argtypes=[c.POINTER(Controller),c.POINTER(Measurement)]
    lib.rl_policy_step.argtypes=[c.POINTER(Controller),c.POINTER(Measurement),ptr,c.c_float];lib.rl_policy_step.restype=c.c_bool
    lib.rl_torque_step.argtypes=[c.POINTER(Controller),c.POINTER(Measurement)];lib.rl_torque_step.restype=c.c_bool
    cf=lambda v:(c.c_float*len(v))(*v)
    rng=np.random.default_rng(20261115);obs_error=target_error=torque_error=ref_error=0.
    for _ in range(200):
        m=Measurement();ctrl=Controller()
        m.q[:]=env.source.defaults+rng.uniform(-.04,.04,6);m.qd[:]=rng.uniform(-4,4,6)
        m.omega[:]=rng.uniform(-.3,.3,3);m.gravity[:]=[.02,-.03,-.999]
        m.xy[:]=[.3,-.2];m.yaw=.4;m.velocity[:]=[.2,-.05]
        lib.rl_reset(c.byref(ctrl),c.byref(m));ctrl.reference_xy[:]=[.4,-.1];ctrl.reference_yaw=.6
        ctrl.action[:]=rng.uniform(-.8,.8,6)
        previous=list(ctrl.action);reference=list(ctrl.reference_xy);heading=ctrl.reference_yaw
        request=[float(rng.uniform(-.3,.3)),float(rng.uniform(-.3,.3)),.1];dt=c.c_float(.01).value
        applied=ramp_commands(list(ctrl.command),request,dt)
        expected=build_observation(angular_velocity_body=list(m.omega),projected_gravity_body=list(m.gravity),
            applied_commands=applied,joint_position=list(m.q),joint_velocity=list(m.qd),previous_clipped_action=previous,
            position_xy=list(m.xy),reference_xy=reference,yaw=m.yaw,reference_yaw=heading,body_velocity_xy=list(m.velocity))
        assert lib.rl_policy_step(c.byref(ctrl),c.byref(m),cf(request),dt)
        obs_error=max(obs_error,float(np.max(np.abs(np.array(ctrl.obs)-expected))))
        targets=action_targets(list(ctrl.action),env.target_limits)
        target_error=max(target_error,float(np.max(np.abs(np.array(ctrl.target_q)-targets['position']))))
        position,yaw=advance_reference(reference,heading,applied[0],applied[1],dt)
        ref_error=max(ref_error,max(abs(x-y) for x,y in zip(position,ctrl.reference_xy)),abs(yaw-ctrl.reference_yaw))
        assert lib.rl_torque_step(c.byref(ctrl),c.byref(m))
        q=np.asarray(m.q,dtype=np.float64);qd=np.asarray(m.qd,dtype=np.float64)
        error=np.asarray(ctrl.target_q)-q;error[[0,3]]=np.arctan2(np.sin(error[[0,3]]),np.cos(error[[0,3]]))
        request=np.array([40,60,0]*2)*error+np.array([2,2,kd]*2)*(np.asarray(ctrl.target_qd)-qd)
        env.fast_actuators.apply(request,q,qd)
        torque_error=max(torque_error,float(np.max(np.abs(np.array(ctrl.motor_torque)-env.fast_actuators.command))))
    assert obs_error<3e-6 and target_error<1e-6 and ref_error<1e-6 and torque_error<3e-4
    physical=list((c.c_float*4).in_dll(lib,'rl_physical_knee_limits'))
    assert np.allclose(physical,env.limits.reshape(-1),atol=3e-7,rtol=0)
    report={'tool':'candidate-c-controller-validation','status':'passed','vectors':200,
        'observation_max_abs_error':obs_error,'target_max_abs_error':target_error,
        'reference_max_abs_error':ref_error,'motor_command_max_abs_error_nm':torque_error,
        'wheel_kd':kd,'target_knee_limits_rad':env.target_limits,'physical_knee_limits_rad':physical,
        'physical_and_action_limits_are_separate':True,'host_c_verified':True,'arm_cross_build_verified':False,
        'arm_cross_build_reason':'arm-none-eabi-gcc not found in available toolchain paths',
        'hardware_tested':False,'bundle':str(a.bundle.resolve())}
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
