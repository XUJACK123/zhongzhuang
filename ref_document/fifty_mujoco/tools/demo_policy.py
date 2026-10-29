#!/usr/bin/env python3
"""Render a rollout of a trained checkpoint to a PNG contact sheet.

Usage::

    uv run python tools/demo_policy.py --checkpoint runs/demo_run/model_best.pt

The rollout runs on the *visual* MJCF (same physics, real meshes) through the
normal policy contract, so what you see is what the policy actually does.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import mujoco  # noqa: E402
from PIL import Image  # noqa: E402

from fifty_mujoco.config import load_configs  # noqa: E402
from fifty_mujoco.contracts import names  # noqa: E402
from fifty_mujoco.envs.fifty_env import EnvOptions, FiftyVecEnv  # noqa: E402
from fifty_mujoco.learning.actor_critic import ActorCriticSequence, NetworkDims  # noqa: E402


def load_policy(path: Path, num_privileged: int) -> ActorCriticSequence:
    policy = ActorCriticSequence(NetworkDims(num_privileged=num_privileged))
    payload = torch.load(path, map_location="cpu", weights_only=False)
    policy.load_state_dict(payload["policy"])
    policy.eval()
    return policy


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default="runs/demo_run/model_best.pt")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--frame-every", type=float, default=1.0, help="seconds between frames")
    parser.add_argument("--out", default="reports/demo_frames.png")
    parser.add_argument(
        "--video",
        default="",
        help="also write an mp4 here (needs imageio + ffmpeg)",
    )
    parser.add_argument("--fps", type=int, default=25, help="mp4 frame rate")
    parser.add_argument("--width", type=int, default=560)
    parser.add_argument("--height", type=int, default=420)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument(
        "--no-follow",
        action="store_true",
        help="keep the camera fixed instead of tracking the base (the policy "
        "drifts a little, so a fixed camera loses the robot)",
    )
    args = parser.parse_args()

    checkpoint = Path(args.checkpoint)
    if not checkpoint.is_file():
        print(f"checkpoint not found: {checkpoint}")
        return 1

    config = load_configs()
    env = FiftyVecEnv(
        config,
        EnvOptions(
            num_envs=1,
            variant="visual",
            seed=1,
            episode_length_s=max(args.seconds + 2.0, 20.0),
            fixed_command=(args.vx, 0.0, 0.0),
        ),
    )
    env.commands.values[:] = (args.vx, 0.0, env.nominal_height)
    policy = load_policy(checkpoint, env.num_privileged_obs)

    renderer = mujoco.Renderer(env.single.model, height=args.height, width=args.width)
    option = mujoco.MjvOption()
    for group in range(len(option.sitegroup)):
        option.sitegroup[group] = 1
    for group in range(len(option.geomgroup)):
        option.geomgroup[group] = 1 if group < 3 else 0  # hide collision primitives
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth = 90.0
    camera.elevation = -2.0
    camera.distance = 1.9
    camera.lookat[:] = (0.0, 0.0, 0.40)

    obs, history = env.get_observations()
    frames: list[tuple[float, np.ndarray]] = []
    next_frame = 0.0
    t = 0.0
    up = 1.0
    fell_at = None
    height = env.spawn_z
    start_xy = env.data[0].qpos[0:2].copy()
    last_xy = start_xy.copy()
    steps = int(round(args.seconds / env.dt))

    video_frames: list[np.ndarray] | None = [] if args.video else None
    video_stride = max(1, int(round(1.0 / (env.dt * args.fps))))

    def _shoot() -> np.ndarray:
        if not args.no_follow:
            # keep the base centred: the policy creeps forward ~0.2 m/s, which
            # would otherwise walk it out of a fixed camera
            base_xy = env.data[0].qpos[0:2]
            camera.lookat[:] = (float(base_xy[0]), float(base_xy[1]), 0.40)
        renderer.update_scene(env.data[0], camera=camera, scene_option=option)
        return renderer.render().copy()

    for k in range(steps):
        with torch.no_grad():
            mean, _ = policy.act_inference(
                torch.as_tensor(obs, dtype=torch.float32),
                torch.as_tensor(history, dtype=torch.float32),
            )
        obs, privileged, reward, done, infos, history = env.step(mean.numpy())
        t = (k + 1) * env.dt
        # env.step() respawns finished worlds immediately, so the qpos of a done
        # world is already the fresh spawn -- keep the last live sample instead
        if not done[0]:
            height = float(env.data[0].qpos[2])
            rotation = env._base_rotation(0)
            up = float(rotation[2, 2])
            last_xy = env.data[0].qpos[0:2].copy()
        if done[0] and fell_at is None:
            fell_at = t
            print(f"  terminated at {t:.2f} s: {infos.get('terminations')}")
            break
        image: np.ndarray | None = None
        want_sheet = t >= next_frame
        want_video = video_frames is not None and k % video_stride == 0
        if want_sheet or want_video:
            image = _shoot()
        if want_sheet:
            frames.append((t, image))
            next_frame += args.frame_every
        if want_video:
            video_frames.append(image)  # type: ignore[union-attr]
    renderer.close()

    if not frames:
        print("no frames captured")
        return 1

    columns = min(4, len(frames))
    rows = (len(frames) + columns - 1) // columns
    sheet = Image.new("RGB", (columns * args.width, rows * args.height))
    try:
        from PIL import ImageDraw

        draw = ImageDraw.Draw(sheet)
    except Exception:  # pragma: no cover
        draw = None
    for index, (stamp, image) in enumerate(frames):
        col, row = index % columns, index // columns
        sheet.paste(Image.fromarray(image), (col * args.width, row * args.height))
        if draw is not None:
            draw.text((col * args.width + 8, row * args.height + 6), f"t={stamp:.1f}s", fill=(255, 255, 0))
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out)

    video_path = None
    if video_frames:
        video_path = Path(args.video)
        video_path.parent.mkdir(parents=True, exist_ok=True)
        import imageio.v2 as imageio

        with imageio.get_writer(
            str(video_path), fps=args.fps, macro_block_size=None
        ) as writer:
            for image in video_frames:
                writer.append_data(image)

    print(f"checkpoint    : {checkpoint}")
    print(f"frames        : {len(frames)} -> {out}")
    print(f"simulated     : {t:.2f} s")
    print(f"first fall    : {('%.2f s' % fell_at) if fell_at else 'did not fall'}")
    print(f"final height  : {height:.4f} m")
    print(f"final up.Z    : {up:+.4f}")
    drift = float(np.linalg.norm(last_xy - start_xy))
    print(f"xy drift      : {drift:.3f} m in {t:.1f} s ({drift / max(t, 1e-9):.3f} m/s)")
    if video_path is not None:
        print(f"video         : {len(video_frames)} frames @ {args.fps} fps -> {video_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
