#!/usr/bin/env python3
"""Search a compact observation-based get-up teacher for the 31D actor."""

import argparse
import json
from pathlib import Path
import time

import torch

from realdata_train import SamplePolicy31, verified_artifact


def mixed_action(observation, tracking_action, parameters):
    """Use the existing leg and gravity observations to time the lift."""
    hip, knee, wheel, pitch_gate, hip_gate, knee_gate, leg_rate = parameters.unbind(-1)
    pose_hip = (observation[:, 9]-observation[:, 11])*.5
    hip = (hip + leg_rate*(pose_hip-.35)).clamp(-1, 1)
    pivot = torch.stack((hip, knee, wheel, -hip, -knee, wheel), dim=-1)
    gates = torch.stack((pitch_gate, hip_gate, knee_gate), dim=-1)
    return gated_action(observation, pivot, tracking_action, gates)


def gated_action(observation, lift, tracking, gates):
    pitch_gate, hip_gate, knee_gate = gates.unbind(-1)
    pose_hip = (observation[:, 9] - observation[:, 11])*.5
    pose_knee = (observation[:, 10] - observation[:, 12])*.5
    transition = torch.minimum(torch.minimum(pitch_gate-observation[:, 3]-.064*observation[:, 1], pose_hip-hip_gate),
                               pose_knee-knee_gate)
    weight = (transition/.08 + .5).clamp(0, 1)
    weight = weight.square()*(3-2*weight)
    return weight[:, None]*lift + (1-weight[:, None])*tracking


class GatedActor(torch.nn.Module):
    def __init__(self, lift, tracking, gates):
        super().__init__()
        self.lift = lift
        self.tracking = tracking
        self.register_buffer('gates', torch.as_tensor(gates, dtype=torch.float32))

    def forward(self, observations):
        return gated_action(observations, self.lift(observations), self.tracking(observations), self.gates)


class ContactSequenceActor(torch.nn.Module):
    """Trigger each stage from the existing policy observations and previous action."""

    def __init__(self, tracking, actions, thresholds):
        super().__init__()
        self.tracking = tracking
        self.register_buffer('motions', torch.as_tensor(actions, dtype=torch.float32))
        self.register_buffer('thresholds', torch.as_tensor(thresholds, dtype=torch.float32))

    def forward(self, observation):
        wheel_start, reverse_ref, return_hip, push_pitch, catch_pitch, catch_wheel = self.thresholds.unbind(-1)
        pitch = observation[:, 3]
        hip = (observation[:, 9] - observation[:, 11])*.5
        knee = (observation[:, 10] - observation[:, 12])*.5
        wheel = (observation[:, 15] + observation[:, 18])*.5
        previous_hip = (observation[:, 19] - observation[:, 22])*.5
        previous_knee = (observation[:, 20] - observation[:, 23])*.5
        previous_wheel = (observation[:, 21] + observation[:, 24])*.5
        cold = observation[:, 19:25].abs().amax(-1) < .001
        reverse = cold | (previous_wheel < -.9)
        load = cold | ((previous_knee > .53) & (wheel > wheel_start))
        reverse_finished = (observation[:, 25] > reverse_ref) & (wheel < -.55)
        return_wheel = (reverse & reverse_finished) | ((previous_hip > -.32) & (previous_wheel > .35) & (hip > return_hip))
        catch = (pitch > push_pitch) | ((previous_hip < -.80) & (previous_knee > .15) & (previous_knee < .55))
        tracking = ((pitch > catch_pitch) & (hip < -.25) & (wheel < catch_wheel)) | ((knee < .43) & (hip < .32))
        index = torch.where(catch, 4, 3)
        index = torch.where(return_wheel, 2, index)
        index = torch.where(reverse & ~reverse_finished, torch.where(load, 0, 1), index)
        if self.motions.ndim == 3:
            action = self.motions[torch.arange(len(index), device=index.device), index]
        else:
            action = self.motions[index]
        return torch.where((~reverse & ~return_wheel & tracking)[:, None], self.tracking(observation), action)


def load_policy(path, device):
    policy = SamplePolicy31(verified_artifact('model_3050.pt'), device)
    policy.load_state_dict(torch.load(path, map_location=device, weights_only=True)['model'])
    return policy.actor.requires_grad_(False).eval()


def search(args, config):
    from fifty_mujoco.envs.realdata_warp_getup import WarpSupportedGetup

    device = torch.device('cuda')
    torch.manual_seed(args.seed)
    balance = load_policy(args.balance, device)
    tracking = load_policy(args.tracking, device)
    env = WarpSupportedGetup(balance, args.envs, config=config, settle_seconds=1.)
    args.out.mkdir(parents=True, exist_ok=False)

    low = torch.tensor([-1., -.3, -.5, .08, -.28, .14, -3.], device=device)
    high = torch.tensor([.4, 1., 2., .6, .50, .60, 3.], device=device)
    scale = (high-low)*.32
    mean = torch.tensor([-.95, .70, 1.9, .33, .04, .36, 0.], device=device)
    best_score = -1.e9
    elites = None
    start = time.monotonic()
    with torch.no_grad(), (args.out/'search.jsonl').open('w') as stream:
        for update in range(args.iterations):
            if time.monotonic()-start >= args.seconds:
                break
            env.reset()
            env.plant.motor_gain[:, [0, 1, 3, 4]] = args.gain
            fixed = (mean + torch.randn((args.envs, len(mean)), device=device)*scale).clamp(low, high)
            if elites is not None:
                fixed[:len(elites)] = elites
            best_pose = torch.full((args.envs,), -5., device=device)
            max_stable = torch.zeros_like(best_pose)
            for step in range(round((1 + args.horizon)/.01)):
                obs = env.observation
                env.step(mixed_action(obs, tracking(obs), fixed))
                good = (env.phase > 0) & (env.phase < 4)
                plant = env.plant
                distance = torch.linalg.vector_norm(plant.qpos[:, :2]-env.origin, dim=-1)
                clearance = plant.clearance
                height = config['standing_clearance_m']
                velocity = plant.observer.velocity[:, 0].abs()
                error = obs[:, 9:13].abs().amax(-1)
                tilt = plant.rpy[:, :2].abs().amax(-1)
                value = 6. - 35*(clearance-height).abs() - 5*error - 3*velocity - 4*tilt - 5*distance
                best_pose = torch.maximum(best_pose, torch.where(good, value, -5.))
                max_stable = torch.maximum(max_stable, env.stable)
                if step % 25 == 0 and bool((env.phase >= 4).all()):
                    break
            score = (best_pose + 12*max_stable + 12*env.balance_time + 100*(env.phase == 5).float()).nan_to_num(-1.e5)
            values, indices = score.topk(max(3, args.envs//16))
            elites = fixed[indices]
            mean = .4*mean + .6*elites.mean(0)
            scale = (.5*scale + .5*elites.std(0)).clamp(min=(high-low)*.045)
            row = {'update': update+1, 'max_score': float(values[0]), 'wall_s': time.monotonic()-start,
                   'completed': int((env.phase == 5).sum()),
                   'max_dwell_s': float(max_stable.max()), 'max_balance_s': float(env.balance_time.max())}
            stream.write(json.dumps(row)+'\n')
            stream.flush()
            print(json.dumps(row), flush=True)
            if row['max_score'] > best_score:
                best_score = row['max_score']
                (args.out/'best.json').write_text(json.dumps({'metrics': row, 'gain': args.gain,
                    'state_parameters': elites.cpu().tolist()}, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--balance', type=Path, required=True)
    parser.add_argument('--tracking', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=810800)
    parser.add_argument('--envs', type=int, default=128)
    parser.add_argument('--seconds', type=float, default=900.)
    parser.add_argument('--horizon', type=float, default=15.)
    parser.add_argument('--iterations', type=int, default=12)
    parser.add_argument('--gain', type=float, default=1.)
    args = parser.parse_args()
    torch.set_num_threads(1)
    search(args, json.loads(args.config.read_text()))


if __name__ == '__main__':
    main()
