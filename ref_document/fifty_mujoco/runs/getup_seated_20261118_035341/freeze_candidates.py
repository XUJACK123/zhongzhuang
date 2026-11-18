"""Freeze final weights and generate paired, untouched-seed CPU evaluations."""

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil

import torch

run = Path(__file__).resolve().parent
project = run.parent.parent


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('primary','standard','balance','configuration'):
        p.add_argument(f'--{name}',type=Path,required=True)
    p.add_argument('--standard-balance', type=Path)
    args = p.parse_args()
    out = run/'frozen'
    out.mkdir(exist_ok=False)
    weight_files = {}
    originals = {}
    for label, path in [('primary',args.primary),('standard_mlp',args.standard),('balance',args.balance),
                        ('standard_balance',args.standard_balance or args.balance)]:
        path = path.resolve()
        destination = out/f'{label}.pt'
        shutil.copy2(path,destination)
        weight_files[label] = {'path':str(destination),'sha256':digest(destination)}
        originals[label] = str(path)
    config = out/'configuration.json'
    shutil.copy2(args.configuration,config)
    manifest = {'frozen_at':datetime.now().astimezone().isoformat(),
                'weights':weight_files,'originals':originals,
                'configuration_sha256':digest(config),'input_manifest_sha256':digest(run/'input_manifest.json')}
    paths = ['src/fifty_mujoco/envs/sample_env31_realdata.py','src/fifty_mujoco/envs/sample_env31_supported.py',
             'src/fifty_mujoco/envs/sample_env31_qualified.py','src/fifty_mujoco/contracts/getup31.py',
             'src/fifty_mujoco/transfer/firmware_runtime.py','tools/realdata_train.py','tools/realdata_evaluate.py',
             'tools/realdata_parallel_train.py','tools/realdata_sampler.py','tools/realdata_batch_evaluate.py',
             'tools/realdata_seated_state_teacher.py','tools/realdata_seated_progress_fit.py']
    manifest['sources'] = {}
    for path in paths:
        dest = out/'source'/path
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(project/path,dest)
        manifest['sources'][path] = digest(dest)
    from fifty_mujoco.transfer.firmware_runtime import RUNTIME_SOURCES
    for path in RUNTIME_SOURCES:
        dest = out/'source/firmware'/path
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(project.parent/'chassis_ref_new'/path,dest)
        manifest['sources']['firmware/'+path] = digest(dest)
    specs = []
    for role in ('primary','standard_mlp'):
        balance = weight_files['balance' if role == 'primary' else 'standard_balance']
        for group, domain, start in [('nominal',False,1011000), ('joint',True,1012000)]:
            current = start
            for i, (duration,count) in enumerate([(1,17),(1,17),(2,17),(2,16),(3,17),(3,16)]):
                specs.append({'label':f'{role}_{group}_{i}', 'split':'heldout',
                              'checkpoint':weight_files[role]['path'],'balance':balance['path'],
                              'expected_checkpoint_sha256':weight_files[role]['sha256'],
                              'expected_balance_sha256':balance['sha256'],
                              'config':str(config),'reset_range':[0.,0.],'settle_seconds':duration,
                              'seed_base':current,'count':count,'domain':domain,'record_traces':True})
                current += count
            assert current-start == 100
    protocol = out/'heldout_spec.json'
    protocol.write_text(json.dumps(specs,indent=2)+'\n')
    manifest.update(heldout_spec_sha256=digest(protocol), preferred_primary_schema=torch.load(
        weight_files['primary']['path'],map_location='cpu',weights_only=True)['schema'])
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'frozen':str(out),'tasks':len(specs),'trials':sum(x['count'] for x in specs)}))


if __name__ == '__main__':
    main()
