#!/usr/bin/env python3
"""Generate isolated 31D actor C assets and compare Torch/C inference.

Exports carry candidate defaults and metadata, and never register, install or
flash a get-up actor into the active firmware.
"""
import argparse
import collections
import ctypes as c
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import numpy as np
import torch

from realdata_train import PROJECT,SamplePolicy31,verified_artifact,RealDataHardwareEnv31,digest
FIRMWARE=PROJECT.parent/'chassis_ref_new'
sys.path.insert(0,str(FIRMWARE/'tools'))
import export_policy


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--checkpoint',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args();torch.set_num_threads(1)
    if FIRMWARE.resolve() in (args.out.resolve(),*args.out.resolve().parents):
        raise ValueError('isolated output outside active firmware required')
    saved=torch.load(args.checkpoint,map_location='cpu',weights_only=True)
    cfg=saved.get('configuration',saved.get('contract',{}).get('configuration'))
    if not cfg:raise ValueError('candidate physical configuration missing')
    policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cpu');policy.load_state_dict(saved['model'])
    env=RealDataHardwareEnv31(config=cfg)
    args.out.mkdir(exist_ok=False,parents=True)
    state=collections.OrderedDict(('actor.'+k,v.detach().cpu().contiguous()) for k,v in policy.actor.state_dict().items())
    actor=args.out/'actor31.pt';torch.save({'model_state_dict':state},actor)
    _,manifest=export_policy.export(actor,args.out)
    arrays={'rl_default_q':cfg['standing_default_rad'],'rl_kd':[2,2,cfg['wheel_kd']]*2,
            'rl_knee_limits':sum(env.target_limits,[]),'rl_physical_knee_limits':env.limits.reshape(-1).tolist()}
    generated=args.out/'model.c';source=generated.read_text()
    for name,values in arrays.items():
        replacement=f'const float {name}[{len(values)}] = '+'{'+', '.join(map(export_policy.literal,values))+'};'
        source,count=re.subn(r'const float '+name+r'\[\d+\] = \{[^}]*\};',replacement,source)
        if count!=1:raise ValueError('generated array mismatch: '+name)
    generated.write_text(source)
    runtime=args.out/'c_runtime'
    names=('state_estimator.c','state_estimator.h','robot_config.h','algorithm/rl.c','algorithm/rl.h')
    for name in names:
        target=runtime/name;target.parent.mkdir(exist_ok=True,parents=True)
        shutil.copy2(FIRMWARE/'user'/name,target)
    header=runtime/'robot_config.h';text=header.read_text()
    for macro,value in [('ROBOT_WHEEL_RADIUS_M',cfg['wheel_radius_m']),('ROBOT_HEIGHT_COMMAND_M',cfg['standing_clearance_m'])]:
        text,count=re.subn(r'(#define '+macro+r') [^\n]+',lambda m:m[1]+' '+export_policy.literal(value)+' /* offline candidate */',text)
        if count!=1:raise ValueError('candidate macro mismatch: '+macro)
    header.write_text(text)
    model=runtime/'algorithm/model';model.mkdir()
    for name in ('model.c','model.h'):shutil.copy2(args.out/name,model/name)
    wrapper=args.out/'check_actor.c'
    wrapper.write_text('#include "algorithm/rl.h"\nint candidate_actor(const float* o,float* a){RlNetwork n;return rl_actor(&n,o,a);}\n')
    library=args.out/'host_actor.so'
    subprocess.run(['cc','-std=c11','-O2','-shared','-fPIC','-I'+str(runtime),str(wrapper),
                    str(runtime/'algorithm/rl.c'),str(runtime/'state_estimator.c'),str(model/'model.c'),'-lm','-o',str(library)],check=True)
    lib=c.CDLL(str(library.resolve()));fp=c.POINTER(c.c_float)
    lib.candidate_actor.argtypes=[fp,fp];lib.candidate_actor.restype=c.c_int
    rng=np.random.default_rng(20261117);obs=rng.uniform(-2,2,(1024,31)).astype('float32');obs[0]=0
    actual=np.empty((len(obs),6),dtype='float32')
    for o,a in zip(obs,actual):
        if not lib.candidate_actor(o.ctypes.data_as(fp),a.ctypes.data_as(fp)):raise RuntimeError('C actor rejected input')
    with torch.no_grad():expected=policy.actor(torch.tensor(obs)).numpy()
    error=float(np.abs(expected-actual).max())
    if not np.allclose(expected,actual,atol=2e-5,rtol=1e-4):raise ValueError(f'C parity failed: {error}')
    manifest.update(schema='realdata_offline_candidate_export_v1',source_checkpoint=str(args.checkpoint.resolve()),
                    source_checkpoint_sha256=digest(args.checkpoint),configuration=cfg,
                    default_q=cfg['standing_default_rad'],target_knee_limits_rad=env.target_limits,
                    physical_knee_limits_rad=env.limits.tolist(),host_c_parity={'vectors':len(obs),'passed':True,'max_abs_error':error},
                    real_robot_accepted=False,installed=False,flashed=False,
                    lifecycle='No firmware actor registration or transition integration. Candidate inference assets only.',
                    physical_model=env.snapshot())
    manifest['generated_sha256']={str(p.relative_to(args.out)):digest(p) for p in args.out.rglob('*') if p.is_file() and p.name!='manifest.json'}
    (args.out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    (args.out/'README.md').write_text('''# 离线仿真候选导出

本包仅保存标准31维网络、匹配候选站姿/轮径的C推理副本和主机对拍结果；没有改动正常固件、注册起身actor或烧录。`manifest.json`保存来源、物理假设和哈希。

编码器参考位、弹簧轴力/滞回、惯量、实际时序和实车起身尚未验证。C推理一致不代表接管成功或可直接上车。以同目录上级的独立评估报告判断训练结果。
''')
    print(json.dumps({'out':str(args.out),'vectors':len(obs),'max_abs_error':error}),flush=True)


if __name__=='__main__':main()
