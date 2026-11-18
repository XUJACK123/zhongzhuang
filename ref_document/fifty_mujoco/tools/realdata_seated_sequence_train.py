#!/usr/bin/env python3
"""Train observed-state rise timing and targets across randomized physics cohorts."""

import argparse
import json
from pathlib import Path
import time

import torch

from fifty_mujoco.envs.realdata_warp_getup import WarpSupportedGetup
from realdata_seated_state_teacher import ContactSequenceActor, load_policy
from realdata_train import digest, save_checkpoint


def pack(motion, thresholds):
    return torch.as_tensor([value for line in motion for value in line[:3]] + list(thresholds),
                           device='cuda', dtype=torch.float32)


def unpack(parameters):
    packed = parameters[..., :15].reshape(*parameters.shape[:-1], 5, 3)
    h, k, w = packed.unbind(-1)
    return torch.stack((h, k, w, -h, -k, w), dim=-1), parameters[..., 15:]


def train(args):
    cfg = json.loads(args.config.read_text())
    prior = json.loads(args.prior.read_text())
    torch.manual_seed(args.seed)
    device = torch.device('cuda')
    elite_source = prior['rows'][:max(1, min(args.population//3, 6))]
    center = pack(prior['motion'], elite_source[0]['thresholds'])
    elites = torch.stack([pack(prior['motion'], row['thresholds']) for row in elite_source])
    spread = torch.tensor([.05, .06, .10]*5 + [.004,.12,.025,.028,.025,.025], device=device)
    minimum_spread = spread*.17
    low = torch.tensor([-1., -1., -2.]*5 + [-.04,.25,.32,.22,.40,-.10], device=device)
    high = torch.tensor([1., 1., 2.]*5 + [-.0005,1.95,.61,.56,.64,.20], device=device)
    size = args.population*args.replicates
    teacher = ContactSequenceActor(load_policy(args.tracking, device), *unpack(center)).to(device)
    paths = args.out
    paths.mkdir(parents=True, exist_ok=False)
    balance = load_policy(args.balance, device)
    env = WarpSupportedGetup(balance, size, config=cfg, settle_seconds=args.settle_seconds, domain=True)
    horizon = round((args.settle_seconds + cfg['success']['getup_timeout_s']
                     + cfg['success']['balance_s'] + cfg['success']['blend_s'] + .2)/.01)
    start = time.monotonic()
    overall = -1.e20
    frozen_source = {'config_sha256': digest(args.config), 'tracking_sha256': digest(args.tracking),
                     'balance_sha256': digest(args.balance), 'prior_sha256': digest(args.prior)}
    with torch.no_grad(), (paths/'rows.jsonl').open('w') as stream:
        for iteration in range(1, args.iterations+1):
            if time.monotonic()-start >= args.seconds:
                break
            candidates = (center+spread*torch.randn((args.population,len(center)), device=device)).clamp(low, high)
            candidates[:len(elites)] = elites
            active = candidates.repeat_interleave(args.replicates,0)
            teacher.motions, teacher.thresholds = unpack(active)
            env.reset()
            env.repeat_initial_cohort(args.replicates)
            peak = torch.zeros(size, device=device)
            dwell = torch.zeros_like(peak)
            for tick in range(horizon):
                env.step(teacher(env.observation))
                clearance = env.plant.clearance
                tilt = env.plant.rpy[:, :2].abs().amax(-1)
                leg = env.observation[:, 9:13].abs().amax(-1)
                velocity = env.plant.observer.velocity[:, 0].abs()
                movement = torch.linalg.vector_norm(env.plant.qpos[:, :2]-env.origin, dim=-1)
                height_error = (clearance-cfg['standing_clearance_m']).abs()
                quality = 10 - 38*height_error - 5*tilt - 4*leg - 5*velocity - 8*movement
                peak = torch.maximum(peak, torch.where((env.phase > 0)&(env.phase < 4), quality, 0.))
                dwell = torch.maximum(dwell, env.stable)
                if tick%25==0 and bool((env.phase >= 4).all()):
                    break
            successful = env.phase==5
            perturbed_world = (env.plant.motor_gain-1.).abs().amax(-1) > 1e-6
            wins = successful.reshape(args.population, args.replicates).sum(-1)
            perturb_wins = (successful & perturbed_world).reshape(args.population, args.replicates).sum(-1)
            score = ((peak + 5*dwell + 6*env.balance_time).reshape(args.population, args.replicates).mean(-1)
                     + 150*wins + args.perturbed_bonus*perturb_wins)
            best_values, indices = score.nan_to_num(-1.e6).topk(max(3,args.population//5))
            elites = candidates[indices]
            center = .4*center + .6*elites.mean(0)
            spread = torch.maximum(minimum_spread, .5*spread + .5*elites.std(0))
            row = {'iteration': iteration,'resets':size,'best_score':float(best_values[0]),
                   'top_successes':int(wins[indices[0]]),'replicates':args.replicates,
                   'all_successes':int(wins.sum()),'longest_balance_s':float(env.balance_time.max()),
                   'max_dwell_s':float(dwell.max()),'wall_s':time.monotonic()-start}
            winner = slice(int(indices[0])*args.replicates, (int(indices[0])+1)*args.replicates)
            perturbed = (env.plant.motor_gain[winner] - 1.).abs().amax(-1) > 1e-6
            row['perturbed_replicates'] = int(perturbed.sum())
            row['perturbed_successes'] = int((perturbed & (env.phase[winner] == 5)).sum())
            stream.write(json.dumps(row)+'\n')
            stream.flush()
            print(json.dumps(row),flush=True)
            improved = row['best_score'] > overall
            if improved or iteration%3 == 0:
                for rank, candidate in enumerate(elites[:3]):
                    motion, thresholds = unpack(candidate)
                    actor = ContactSequenceActor(load_policy(args.tracking, 'cpu'), motion.cpu(), thresholds.cpu())
                    state = {'schema':'seated_contact_sequence_v1',
                        'configuration':cfg,'model':actor.state_dict(),'motion':motion.tolist(),
                        'thresholds':thresholds.tolist(),'training_row':row, 'source':frozen_source}
                    save_checkpoint(paths/f'iteration_{iteration:03d}_rank{rank}.pt', state)
                    if improved:
                        save_checkpoint(paths/f'best_rank{rank}.pt', state)
            if improved:
                overall = row['best_score']
                (paths/'best.json').write_text(json.dumps({'training_row':row,'motion':unpack(elites[0])[0].tolist(),
                      'thresholds':unpack(elites[0])[1].tolist(),'source':frozen_source},indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('tracking','balance','prior','config','out'):
        parser.add_argument(f'--{name}', required=True, type=Path)
    parser.add_argument('--population',type=int,default=24)
    parser.add_argument('--replicates',type=int,default=6)
    parser.add_argument('--iterations',type=int,default=24)
    parser.add_argument('--seconds',type=float,default=1800)
    parser.add_argument('--settle-seconds',type=float,default=1.)
    parser.add_argument('--seed',type=int,default=811400)
    parser.add_argument('--perturbed-bonus',type=float,default=130.)
    args = parser.parse_args()
    torch.set_num_threads(1)
    train(args)


if __name__ == '__main__':
    main()
