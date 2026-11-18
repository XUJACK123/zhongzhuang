#!/usr/bin/env python3
"""Search for an unassisted seated rise through the unchanged six-action limits."""

import argparse
import json
from pathlib import Path
import time

import torch

from realdata_train import SamplePolicy31, verified_artifact


def search(args, config, balance, rise, tracking):
    from fifty_mujoco.envs.realdata_warp_getup import WarpSupportedGetup

    torch.manual_seed(args.seed)
    device = torch.device('cuda')
    env = WarpSupportedGetup(balance.actor, args.envs, config=config, settle_seconds=1., domain=False)
    plant = env.plant
    mean = torch.zeros((args.segments, 3), device=device)
    mean[0] = torch.tensor([-.7, -.7, 0.], device=device)
    spread = torch.ones_like(mean)*.7
    output = args.out
    output.mkdir(parents=True, exist_ok=False)
    best_score = -1.e9
    elites = None
    if args.warm_start:
        saved = json.loads(args.warm_start.read_text())
        elites = torch.tensor(saved['candidates'], device=device)
        if elites.shape[1:] != mean.shape or saved['segment_seconds'] != args.segment_seconds:
            raise ValueError('warm-start trajectory layout mismatch')
        elites = elites[:args.envs-1]
        mean.copy_(elites[0])
        spread.fill_(.3)
    started = time.monotonic()
    segment_ticks = round(args.segment_seconds/.01)
    fixed_ticks = args.segments*segment_ticks
    rollout_ticks = fixed_ticks + round(args.feedback_seconds/.01)

    with torch.no_grad(), (output/'search.jsonl').open('w') as log:
        for iteration in range(args.iterations):
            if time.monotonic() - started >= args.seconds:
                break
            env.reset()
            plant.motor_gain[:, [0, 1, 3, 4]] = args.gain
            params = (mean + spread*torch.randn((args.envs, *mean.shape), device=device)).clamp(-1, 1)
            params[0] = mean
            if elites is not None:
                params[1:len(elites)+1] = elites
            best_pose = torch.full((args.envs,), -1000., device=device)
            terminal_pose = best_pose.clone()
            max_height = torch.zeros_like(best_pose)
            max_stable = torch.zeros_like(best_pose)
            capture_x = torch.zeros_like(best_pose)
            captured = torch.zeros(args.envs, device=device, dtype=torch.bool)
            previous_action = torch.zeros((args.envs, 6), device=device)
            for tick in range(100 + rollout_ticks):
                if tick < 100 + fixed_ticks:
                    phase_tick = max(0, tick - 100)
                    current = params[:, phase_tick//segment_ticks]
                    hip, knee, wheel = current.unbind(-1)
                    target = torch.stack((hip, knee, 2*wheel, -hip, -knee, 2*wheel), dim=-1)
                    if tick >= 100:
                        previous_action.lerp_(target, args.slew)
                    action = previous_action
                else:
                    if rise is None:
                        action = tracking.actor(env.observation)
                    else:
                        new_capture = ~captured & (plant.clearance >= .12) & (plant.rpy[:, :2].abs().amax(-1) < .2618)
                        capture_x = torch.where(new_capture, torch.minimum(plant.qpos[:, 0]+.02, env.origin[:, 0]+.20), capture_x)
                        captured |= new_capture
                        teacher_obs = env.observation.clone()
                        if args.brake:
                            teacher_obs[:, 25] = (5*(capture_x-plant.qpos[:, 0])).clamp(-2, 2)
                            teacher_obs[:, 29] = 2*plant.qvel[:, 0]
                        captured_action = balance.actor(teacher_obs)
                        captured_action[:, 2] += args.brake*plant.qvel[:, 0]
                        captured_action[:, 5] += args.brake*plant.qvel[:, 0]
                        action = torch.where(captured[:, None], captured_action, rise.actor(env.observation))
                env.step(action)
                if tick < 100:
                    continue
                dx = torch.linalg.vector_norm(plant.qpos[:, :2] - env.origin, dim=-1)
                clearance = plant.clearance.clamp(-.02, .28)
                rp = plant.rpy[:, :2].abs().amax(-1)
                error = plant.obs[:, 9:13].abs().amax(-1)
                velocity = plant.observer.velocity[:, 0].abs()
                active = (env.phase >= 1) & (env.phase <= 3)
                quality = (35*clearance - 3*rp - 2*error - 2*velocity.clamp(max=2)
                           - 15*(dx-.14).clamp(min=0)
                           + .5*(env.wheel_load >= .02*env.bodyweight).all(-1))
                best_pose = torch.maximum(best_pose, torch.where(active, quality, -1000.))
                terminal_pose = torch.where(active & (dx < .3), quality, terminal_pose)
                max_height = torch.maximum(max_height, torch.where(active & (dx < .3), clearance, 0.))
                max_stable = torch.maximum(max_stable, env.stable)
                if tick % 25 == 0 and bool((env.phase >= 4).all()):
                    break
            score = (best_pose + .25*terminal_pose + 8*max_height + 20*max_stable
                     + 100*(env.phase >= 3).float()*(env.first_failure == 0) + 30*env.balance_time)
            score = torch.nan_to_num(score, nan=-1.e6)
            values, indices = score.topk(min(max(4, args.envs//12), args.envs-1))
            elites = params[indices].clone()
            mean = .3*mean + .7*elites.mean(0)
            spread = (.3*spread + .7*elites.std(0)).clamp(min=.10, max=.85)
            row = {'iteration': iteration+1, 'best_score': float(values[0]),
                   'best_height_m': float(max_height.max()), 'best_dwell_s': float(max_stable.max()),
                   'max_balance_s': float(env.balance_time.max()),
                   'valid_resets': int((env.first_failure != 16).sum()), 'wall_s': time.monotonic()-started}
            log.write(json.dumps(row)+'\n')
            log.flush()
            print(json.dumps(row), flush=True)
            if float(values[0]) > best_score:
                best_score = float(values[0])
                (output/'best.json').write_text(json.dumps({
                    'gain': args.gain, 'segment_seconds': args.segment_seconds,
                    'slew': args.slew,
                    'capture_brake': args.brake, 'rise': str(args.rise) if args.rise else None,
                    'iterations': iteration+1, 'score': best_score,
                    'candidates': elites.cpu().tolist(), 'metrics': row}, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--balance', type=Path, required=True)
    parser.add_argument('--rise', type=Path)
    parser.add_argument('--tracking', type=Path)
    parser.add_argument('--brake', type=float, default=0.)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--envs', type=int, default=128)
    parser.add_argument('--seed', type=int, default=810100)
    parser.add_argument('--seconds', type=float, default=600.)
    parser.add_argument('--iterations', type=int, default=15)
    parser.add_argument('--gain', type=float, default=1.)
    parser.add_argument('--segments', type=int, default=12)
    parser.add_argument('--segment-seconds', type=float, default=.20)
    parser.add_argument('--feedback-seconds', type=float, default=3.)
    parser.add_argument('--warm-start', type=Path)
    parser.add_argument('--slew', type=float, default=1.)
    args = parser.parse_args()
    torch.set_num_threads(1)
    config = json.loads(args.config.read_text())
    balance = SamplePolicy31(verified_artifact('model_3050.pt'), 'cuda')
    balance.load_state_dict(torch.load(args.balance, map_location='cuda', weights_only=True)['model'])
    rise = None
    if args.rise:
        rise = SamplePolicy31(verified_artifact('model_3050.pt'), 'cuda')
        rise.load_state_dict(torch.load(args.rise, map_location='cuda', weights_only=True)['model'])
    tracking = balance
    if args.tracking:
        tracking = SamplePolicy31(verified_artifact('model_3050.pt'), 'cuda')
        tracking.load_state_dict(torch.load(args.tracking, map_location='cuda', weights_only=True)['model'])
    search(args, config, balance, rise, tracking)


if __name__ == '__main__':
    main()
