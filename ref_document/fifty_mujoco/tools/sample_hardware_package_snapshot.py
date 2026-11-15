#!/usr/bin/env python3
"""Attach the exact reconstruction inputs and code to a generated candidate."""
import argparse,hashlib,json,shutil,sys
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PROJECT/'src'))
from fifty_mujoco.envs.sample_env31_hardware_guarded import GuardedHardwareEnv31


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',type=Path,required=True);a=p.parse_args()
    manifest=json.loads((a.bundle/'manifest.json').read_text());contract=manifest['environment_contract']
    e=GuardedHardwareEnv31(wheel_kd=manifest['wheel_kd'],knee_margin_rad=contract['knee_margin_rad'])
    (a.bundle/'simulation_snapshot.xml').write_text(e.source.xml+'\n')
    snapshot={'schema':'hardware_reconstruction_snapshot_v1','physics_dt_s':e.model.opt.timestep,
        'external_pd_dt_s':e.source.external_dt,'policy_dt_s':e.source.policy_dt,'estimator_dt_s':.005,
        'hardware_profile':e.source.profile,'actuator_hardware':e.source.hardware,
        'effective_motor_envelope':e.actuators.inner.ratings,'gas_spring_resolved':e.source.spring_config,
        'action_target_knee_limits_rad':e.target_limits,'physical_knee_limits_rad':e.limits.tolist(),
        'source_reset_environment':e.source.env,'default_q':e.source.defaults.tolist(),
        'code_required':'XML alone does not apply gas spring/actuator forces or the training reset pose; use the matching environment code.'}
    (a.bundle/'simulation_physics.json').write_text(json.dumps(snapshot,indent=2)+'\n')
    paths=['envs/sample_env31.py','envs/sample_env31_hardware.py','envs/sample_env31_hardware_fast.py',
        'envs/sample_env31_hardware_guarded.py','transfer/sample_source.py','transfer/sample_target.py',
        'contracts/sample_actor31.py','learning/sample_ppo.py']
    for name in paths:
        target=a.bundle/'simulation_code'/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(PROJECT/'src/fifty_mujoco'/name,target)
    for name in ['sample_hardware_profile.json','sample_target_profile.json']:
        target=a.bundle/'simulation_code/configs'/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(PROJECT/'configs'/name,target)
    manifest['simulation_inputs_and_code']={str(q.relative_to(a.bundle)):sha(q) for q in a.bundle.rglob('*')
        if q.is_file() and (q.name.startswith('simulation_') or 'simulation_code' in q.parts)}
    (a.bundle/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    readme=a.bundle/'README.md';s=readme.read_text()
    if 'simulation_snapshot.xml' not in s:
        s+='\n`simulation_snapshot.xml` / `simulation_physics.json` 与 `simulation_code/` 保存本次几何、惯量、气弹簧、电机和控制实现快照。\nXML 本身不施加外部气弹簧/电机力，也不包含训练 reset；复现需配套环境代码，不能仅用 XML 默认 qpos 播放。\n'
        readme.write_text(s)
    print(json.dumps({'files':len(manifest['simulation_inputs_and_code']),'bundle':str(a.bundle)}))


if __name__=='__main__':main()
