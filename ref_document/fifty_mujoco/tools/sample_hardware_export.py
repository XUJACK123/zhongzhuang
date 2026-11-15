#!/usr/bin/env python3
"""Generate a standalone candidate C actor bundle; never touch active firmware.

Uses the firmware exporter for the exact float32 actor and source constants,
then generates the experiment's explicit wheel Kd array. The source robot's
output flags, calibration and CAN handling are not part of this bundle.
"""
import argparse
import collections
import ctypes
import hashlib
import json
import re
import difflib
import subprocess
import sys
from pathlib import Path
import numpy as np
import torch

PROJECT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT/'src'))
FIRMWARE=PROJECT.parent/'chassis_ref_new'
sys.path.insert(0,str(FIRMWARE/'tools'))
import export_policy
from fifty_mujoco.learning.sample_ppo import SamplePolicy31
from fifty_mujoco.transfer.sample_source import verified_artifact


def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--wheel-kd-override',type=float,help='explicit post-training deployment control tuning')
    args=parser.parse_args()
    torch.set_num_threads(1)
    if FIRMWARE.resolve() in (args.out.resolve(), *args.out.resolve().parents):
        parser.error('candidate output must be outside the active firmware tree')
    checkpoint=torch.load(args.checkpoint,map_location='cpu',weights_only=True)
    contract=checkpoint['contract']
    if contract['environment'] not in ('sample_commissioned_cpu31_v3','sample_commissioned_guarded_cpu31_v4'):
        raise ValueError('expected the commissioned hardware v3 contract')
    training_kd=contract['wheel_kd']
    kd=training_kd if args.wheel_kd_override is None else args.wheel_kd_override
    if not 0<kd<=.3:raise ValueError('deployment wheel kd must be within (0,.3]')
    args.out.mkdir(parents=True,exist_ok=False)
    policy=SamplePolicy31(verified_artifact('model_3050.pt'))
    policy.load_state_dict(checkpoint['model'],strict=True)
    state=collections.OrderedDict((f'actor.{k}',v.detach().cpu().contiguous()) for k,v in policy.actor.state_dict().items())
    actor_checkpoint=args.out/'actor31.pt'
    torch.save({'model_state_dict':state},actor_checkpoint)
    _, manifest=export_policy.export(actor_checkpoint,args.out)
    generated=args.out/'model.c'
    text=generated.read_text()
    literal=', '.join(export_policy.literal(v) for v in (2.,2.,kd,2.,2.,kd))
    text,count=re.subn(r'const float rl_kd\[6\] = \{[^}]*\};',
                      'const float rl_kd[6] = {'+literal+'};',text)
    if count!=1:raise ValueError('could not generate wheel kd array')
    if 'action_knee_limits' in contract:
        from fifty_mujoco.envs.sample_env31_hardware import SampleHardwareEnv31
        target_limits=sum(contract['action_knee_limits'], [])
        physical_limits=SampleHardwareEnv31().limits.reshape(-1).tolist()
        target_literal=', '.join(export_policy.literal(v) for v in target_limits)
        text,n=re.subn(r'const float rl_knee_limits\[4\] = \{[^}]*\};',
                      'const float rl_knee_limits[4] = {'+target_literal+'};',text)
        if n!=1:raise ValueError('could not generate guarded target limits')
        text+='\nconst float rl_physical_knee_limits[4] = {'+', '.join(export_policy.literal(v) for v in physical_limits)+'};\n'
        header=args.out/'model.h'
        header.write_text(header.read_text().replace('#endif','extern const float rl_physical_knee_limits[4];\n#endif'))
        original=(FIRMWARE/'user/chassis.c').read_text()
        if original.count('rl_knee_limits')!=4:raise ValueError('unexpected chassis guard references')
        candidate=original.replace('rl_knee_limits','rl_physical_knee_limits')
        (args.out/'chassis_candidate.c').write_text(candidate)
        patch=''.join(difflib.unified_diff(original.splitlines(True),candidate.splitlines(True),
                    fromfile='a/user/chassis.c',tofile='b/user/chassis.c'))
        (args.out/'physical_state_guard.patch').write_text(patch)
        manifest['target_knee_limits_rad']=contract['action_knee_limits']
        manifest['physical_knee_limits_rad']=physical_limits
        manifest['requires_chassis_state_guard_patch']=True
    original_config=(FIRMWARE/'user/robot_config.h').read_text()
    candidate_config=original_config
    for macro,value in [('ROBOT_REMOTE_MAX_FORWARD','0.15f'),('ROBOT_REMOTE_MAX_YAW','0.30f')]:
        candidate_config,n=re.subn(r'^#define '+macro+r' [^\n]+',
            '#define '+macro+' '+value+' /* candidate evaluated request cap */',candidate_config,flags=re.M)
        if n!=1:raise ValueError('cannot generate request cap '+macro)
    (args.out/'robot_config_candidate.h').write_text(candidate_config)
    (args.out/'evaluated_command_caps.patch').write_text(''.join(difflib.unified_diff(
        original_config.splitlines(True),candidate_config.splitlines(True),
        fromfile='a/user/robot_config.h',tofile='b/user/robot_config.h')))
    manifest['candidate_remote_caps']={'forward_m_s':.15,'yaw_rad_s':.3,
        'reason':'Conservative caps from the evaluated nominal commands and transition protocols; boundary scan is not a full-domain acceptance.'}
    generated.write_text(text)
    # Execute the actual firmware actor linked to candidate weights on host.
    wrapper=args.out/'check_actor.c'
    wrapper.write_text('#include "algorithm/rl.h"\nint candidate_actor(const float *o,float *a) { RlNetwork n;return rl_actor(&n,o,a); }\n')
    library=args.out/'host_actor.so'
    subprocess.run(['cc','-O2','-shared','-fPIC','-I'+str(FIRMWARE/'user'),
        str(wrapper),str(FIRMWARE/'user/algorithm/rl.c'),str(generated),'-lm','-o',str(library)],check=True)
    lib=ctypes.CDLL(str(library.resolve()));ptr=ctypes.POINTER(ctypes.c_float)
    lib.candidate_actor.argtypes=[ptr,ptr];lib.candidate_actor.restype=ctypes.c_int
    rng=np.random.default_rng(20261115)
    observations=np.concatenate([np.zeros((1,31)),rng.uniform(-2,2,(255,31))]).astype('float32')
    with torch.no_grad(): reference=policy.actor(torch.from_numpy(observations)).numpy()
    actual=[]
    for obs in observations:
        out=np.zeros(6,dtype='float32')
        if not lib.candidate_actor(obs.ctypes.data_as(ptr),out.ctypes.data_as(ptr)):raise ValueError('C actor failed')
        actual.append(out)
    actual=np.stack(actual)
    if not np.allclose(reference,actual,atol=1e-5,rtol=1e-4):raise ValueError('C vs Torch parity failed')
    manifest.update({'source_training_checkpoint':str(args.checkpoint.resolve()),
        'source_training_checkpoint_sha256':digest(args.checkpoint),'source_training_update':checkpoint['update'],
        'environment_contract':contract,'training_wheel_kd':training_kd,'wheel_kd':kd,
        'control_retuned_after_training':kd!=training_kd,'external_pd_dt_s':.002,'estimator_dt_s':.005,
        'source_exporter_sha256':digest(Path(export_policy.__file__)),
        'generated_sha256':{name:digest(args.out/name) for name in ('model.c','model.h','actor31.pt','chassis_candidate.c','physical_state_guard.patch','robot_config_candidate.h','evaluated_command_caps.patch') if (args.out/name).exists()},
        'host_c_parity':{'vectors':len(observations),'max_abs_error':float(np.max(np.abs(actual-reference))),
                         'atol':1e-5,'rtol':1e-4,'passed':True},
        'physical_deployment_executed':False,'real_robot_accepted':False,
        'requires_control_parameter_sync':kd!=.3})
    (args.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (args.out/'README.md').write_text(f'''# 仿真候选策略（未烧录）

模型来自 `{args.checkpoint.name}`（update {checkpoint['update']}）。网络结构和 31 维输入不变。
训练时轮 Kd={training_kd}，部署候选 Kd={kd}；若不同，属于训练后的控制参数整定，验收按后者进行。
此包的轮速度 Kd 为 **{kd}**，必须与训练同步，PD 2 ms、估计器 5 ms、策略 10 ms。
若有 `physical_state_guard.patch`，动作目标使用带余量的 `rl_knee_limits`，状态保护使用
独立的 `rl_physical_knee_limits`；需配套 `chassis_candidate.c`，不能只替换权重文件。
`robot_config_candidate.h` / `evaluated_command_caps.patch` 把遥控上限限定为前进0.15 m/s、
yaw±0.30 rad/s；高速度/高角速边界扫描存在跟踪超标，未接受原固件全部遥控范围。
候选配置保留原校准与输出标志，未把未验证标志改为已验证。
轮半径 55 mm / 总减速比 15.88 / 电机空间上限腿 20 Nm、轮 2 Nm。

`model.c/.h` 与 `actor31.pt` 是可复核候选产物；原固件、放行标志与实物均未修改。
主机 C 与 Torch 的 256 个输入对拍通过，不能替代 MCU 时序、启动或实车平衡验证。
实测支撑姿态直接释放仍是独立失败项；右轮反馈和有效力矩需要现场复核。
''')
    print(json.dumps({'output':str(args.out),'wheel_kd':kd,'host_c_parity':manifest['host_c_parity']}))


if __name__=='__main__':main()
