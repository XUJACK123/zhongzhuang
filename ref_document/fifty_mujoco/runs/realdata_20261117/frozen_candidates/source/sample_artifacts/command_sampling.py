"""Mix exact command endpoints with the existing uniform interior samples."""
import math


def endpoint_probability(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 1:
        raise ValueError('Command endpoint probability must be finite and in [0, 1]')
    return float(value)


def apply_endpoints(commands, env_ids, ranges, probability, torch):
    # Legacy profiles consume no additional random numbers and retain exactly
    # their original commands. Operate only on the environments being resampled.
    if probability == 0 or not len(env_ids):
        return
    bounds = torch.stack([ranges[key][env_ids] for key in
                          ('lin_vel_x', 'ang_vel_yaw', 'height')], dim=1)
    mask = torch.rand((len(env_ids), 3), device=commands.device) < probability
    upper = torch.rand((len(env_ids), 3), device=commands.device) < .5
    endpoints = torch.where(upper, bounds[:, :, 1], bounds[:, :, 0])
    commands[env_ids, :3] = torch.where(mask, endpoints, commands[env_ids, :3])
