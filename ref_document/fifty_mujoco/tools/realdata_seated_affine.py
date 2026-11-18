#!/usr/bin/env python3
"""Fine-tune the standard 31D actor output layer against closed-loop cohorts."""

import argparse
import copy
import json
from pathlib import Path
import time

import torch

from fifty_mujoco.envs.realdata_warp_getup import WarpSupportedGetup
from realdata_seated_state_teacher import load_policy
from realdata_train import digest, save_checkpoint


def six_channels(parameters):
    hip, knee, wheel, hip_bias, knee_bias, wheel_bias = parameters.unbind(-1)
    scale = torch.stack((hip, knee, wheel, hip, knee, wheel), dim=-1)
    bias = torch.stack((hip_bias, knee_bias, wheel_bias, -hip_bias, -knee_bias, wheel_bias), dim=-1)
    return scale, bias


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('init', 'balance', 'config', 'out'):
        parser.add_argument(f'--{name}', type=Path, required=True)
    parser.add_argument('--population', type=int, default=16)
    parser.add_argument('--replicates', type=int, default=8)
    parser.add_argument('--iterations', type=int, default=20)
    parser.add_argument('--seconds', type=float, default=1500.)
    parser.add_argument('--seed', type=int, default=811500)
    parser.add_argument('--domain', action='store_true')
    args = parser.parse_args()
    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    cfg = json.loads(args.config.read_text())
    device = torch.device('cuda')
    actor = load_policy(args.init, device)
    balance = load_policy(args.balance, device)
    prototype = torch.load(args.init, map_location='cpu', weights_only=True)
    final_weight = next(k for k in reversed(prototype['model']) if k.startswith('actor.') and k.endswith('.weight'))
    final_bias = final_weight.removesuffix('weight') + 'bias'
    size = args.population*args.replicates
    env = WarpSupportedGetup(balance, size, config=cfg, settle_seconds=1., domain=args.domain)
    target = args.out
    target.mkdir(parents=True, exist_ok=False)
    center = torch.tensor([1.,1.,1.,0.,0.,0.], device=device)
    spread = torch.tensor([.012,.015,.018,.008,.012,.017], device=device)
    floor = spread*.2
    low = torch.tensor([.85,.85,.85,-.10,-.12,-.15], device=device)
    high = torch.tensor([1.15,1.15,1.15,.10,.12,.15], device=device)
    elites = center[None].clone()
    best_so_far = -1.e12
    start = time.monotonic()
    horizon = round((1 + cfg['success']['getup_timeout_s'] + cfg['success']['balance_s']
                     + cfg['success']['blend_s'] + .2)/.01)
    with torch.no_grad(), (target/'rows.jsonl').open('w') as stream:
        for update in range(1, args.iterations+1):
            if time.monotonic()-start >= args.seconds:
                break
            candidates = (center+spread*torch.randn((args.population, 6), device=device)).clamp(low, high)
            candidates[:len(elites)] = elites
            candidates[-1] = candidates[-1].new_tensor([1.,1.,1.,0.,0.,0.])
            scale, bias = six_channels(candidates.repeat_interleave(args.replicates, dim=0))
            env.reset()
            env.repeat_initial_cohort(args.replicates)
            dwell = torch.zeros(size, device=device)
            height = torch.zeros_like(dwell)
            for tick in range(horizon):
                env.step(actor(env.observation)*scale+bias)
                dwell = torch.maximum(dwell, env.stable)
                valid = (env.phase > 0) & (env.phase < 4)
                height = torch.maximum(height, torch.where(valid, env.plant.clearance, 0.))
                if tick % 25 == 0 and bool((env.phase >= 4).all()):
                    break
            wins = (env.phase==5).reshape(args.population, args.replicates).sum(-1)
            score = (200*wins + (9*env.balance_time + 4*dwell + 10*height).reshape(args.population,args.replicates).mean(-1))
            values, indices = score.nan_to_num(-1.e6).topk(max(3, args.population//4))
            elites = candidates[indices]
            center = .45*center + .55*elites.mean(0)
            spread = torch.maximum(floor, .55*spread + .45*elites.std(0))
            row = {'update':update,'worlds':size,'best_score':float(values[0]),'replicates':args.replicates,
                   'best_successes':int(wins[indices[0]]),'unmodified_successes':int(wins[-1]),
                   'all_successes':int(wins.sum()),'max_balance_s':float(env.balance_time.max()),
                   'wall_s':time.monotonic()-start}
            pos = int(indices[0])*args.replicates
            perturb = (env.plant.motor_gain[pos:pos+args.replicates]-1.).abs().amax(-1) > 1e-6
            row['best_perturbed'] = int(perturb.sum())
            row['best_perturbed_successes'] = int((perturb & (env.phase[pos:pos+args.replicates]==5)).sum())
            stream.write(json.dumps(row)+'\n')
            stream.flush()
            print(json.dumps(row), flush=True)
            if row['best_score'] > best_so_far or update%5 == 0:
                best_so_far = max(best_so_far, row['best_score'])
                for rank, params in enumerate(elites[:3]):
                    scaling, offset = six_channels(params.cpu())
                    model = copy.deepcopy(prototype['model'])
                    model[final_weight] = model[final_weight]*scaling[:,None]
                    model[final_bias] = model[final_bias]*scaling+offset
                    save_checkpoint(target/f'update_{update:04d}_rank{rank}.pt', {'schema':'seated_affine_refinement_v1',
                        'model':model,'configuration':cfg,'initial_sha256':digest(args.init),
                        'balance_sha256':digest(args.balance),'seed':args.seed,'training':row,
                        'parameters':params.tolist()})
    (target/'summary.json').write_text(json.dumps({'best_training_score':best_so_far,
                                                  'wall_s':time.monotonic()-start},indent=2)+'\n')


if __name__ == '__main__':
    main()
