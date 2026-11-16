#!/usr/bin/env python3
"""Freeze selected simulation candidates, source and acceptance specification."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
from zoneinfo import ZoneInfo
import mujoco
import torch
from realdata_train import PROJECT,RealDataHardwareEnv31,digest,verified_artifact


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for role in ['near','full']:
        ap.add_argument('--'+role,type=Path,required=True)
        ap.add_argument('--'+role+'-balance',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--plan',type=Path,required=True)
    ap.add_argument('--test-log',type=Path)
    args=ap.parse_args();args.out.mkdir(exist_ok=False,parents=True);torch.set_num_threads(1)
    manifest={'schema':'realdata_frozen_candidates_v1','frozen_at':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
              'selection_split':'development','heldout_seen_before_selection':False,'candidates':{},'source_sha256':{},
              'hardware_accepted':False,'hardware_deployed':False}
    for role in ['near','full']:
        checkpoint=getattr(args,role);balance=getattr(args,role+'_balance')
        saved=torch.load(checkpoint,map_location='cpu',weights_only=True)
        cfg=saved.get('configuration',saved.get('contract',{}).get('configuration'))
        dest=args.out/role;dest.mkdir()
        shutil.copy2(checkpoint,dest/'getup.pt');shutil.copy2(balance,dest/'balance.pt')
        (dest/'configuration.json').write_text(json.dumps(cfg,indent=2))
        env=RealDataHardwareEnv31(config=cfg)
        mujoco.mj_saveLastXML(str(dest/'model_snapshot.xml'),env.model)
        (dest/'physics.json').write_text(json.dumps(env.snapshot(),indent=2))
        manifest['candidates'][role]={'source_checkpoint':str(checkpoint.resolve()),'source_balance':str(balance.resolve()),
            'getup_sha256':digest(dest/'getup.pt'),'balance_sha256':digest(dest/'balance.pt'),
            'configuration_sha256':digest(dest/'configuration.json'),'source_update':saved.get('update'),
            'status':'Frozen for evaluation; no success presumed.'}
    files=list((PROJECT/'tools').glob('realdata_*.py'))+list((PROJECT/'configs').glob('realdata_*.json'))
    files+=list((PROJECT/'configs').glob('sample_*.json'))
    files.append(PROJECT/'configs/hardware31_next_training_20261115.json')
    files += [PROJECT/'tests'/name for name in ['test_realdata_candidate.py','test_getup31_offline.py','test_sample_hardware_fast.py']]
    for name in ['envs/sample_env31.py','envs/sample_env31_hardware.py','envs/sample_env31_hardware_guarded.py',
                 'envs/sample_env31_qualified.py','envs/sample_env31_realdata.py','envs/sample_env31_supported.py',
                 'envs/realdata_warp.py','envs/realdata_warp_getup.py','contracts/getup31.py','contracts/sample_actor31.py',
                 'learning/sample_ppo.py','learning/realdata_ppo.py','transfer/sample_source.py','transfer/firmware_runtime.py',
                 'transfer/sample_target.py','paths.py','math3d.py','assets/urdf_io.py']:
        path=PROJECT/'src/fifty_mujoco'/name
        if path.exists():files.append(path)
    for path in files:
        name=path.relative_to(PROJECT);dest=args.out/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,dest);manifest['source_sha256'][str(name)]=digest(path)
    firmware=PROJECT.parent/'chassis_ref_new'
    for name in ['user/state_estimator.c','user/state_estimator.h','user/robot_config.h','user/algorithm/rl.c',
                 'user/algorithm/rl.h','user/algorithm/model/model.c','user/algorithm/model/model.h','tools/export_policy.py']:
        path=firmware/name;dest=args.out/'source/firmware'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,dest);manifest['source_sha256']['firmware/'+name]=digest(path)
    archive=verified_artifact('env_cfg.json').parent
    for path in archive.iterdir():
        if not path.is_file() or (path.suffix not in ('.py','.json','.xml','.urdf') and path.name!='model_3050.pt'):continue
        dest=args.out/'source/sample_artifacts'/path.name;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,dest);manifest['source_sha256']['sample_artifacts/'+path.name]=digest(path)
    shutil.copy2(archive.parent/'manifest.json',args.out/'source/sample_artifacts_manifest.json')
    shutil.copy2(args.plan,args.out/'acceptance_plan.json')
    if args.test_log:
        shutil.copy2(args.test_log,args.out/'before_freeze_regression.log')
    manifest['acceptance_plan_sha256']=digest(args.out/'acceptance_plan.json')
    manifest['generated_sha256']={str(p.relative_to(args.out)):digest(p) for p in args.out.rglob('*') if p.is_file()}
    (args.out/'manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest['candidates']),flush=True)


if __name__=='__main__':main()
