#!/usr/bin/env python3
"""Render a recorded CPU trajectory from the seated get-up experiment."""

import argparse
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from fifty_mujoco.envs.sample_env31_realdata import RealDataHardwareEnv31


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--trace', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--label', default='CPU TEACHER REPLAY')
    parser.add_argument('--cad', action='store_true')
    args = parser.parse_args()
    env = RealDataHardwareEnv31(config=json.loads(args.config.read_text()))
    model, data = env.model, env.data
    options = mujoco.MjvOption()
    if args.cad:
        from sample_qualified_play import display_model

        original = mujoco.MjModel.from_xml_string(env.source.xml)
        model, data, _ = display_model(SimpleNamespace(source=env.source, model=original))
        options.geomgroup[3] = 0
    else:
        for i in range(model.ngeom):
            name = model.geom(i).name
            color = [.80, .82, .85, 1.] if name == 'floor' else (
                [.12, .13, .15, 1.] if 'wheel' in name else [.95, .50, .12, 1.])
            model.geom_rgba[i] = color
    with np.load(args.trace) as trace, mujoco.Renderer(model, height=480, width=640) as renderer:
        # Fixed, nearly side-on camera so movement can be judged against the floor.
        camera = mujoco.MjvCamera()
        camera.distance = 1.65
        camera.azimuth = 88.
        camera.elevation = -12.
        camera.lookat[:] = [0., 0., .24]
        states, metrics, phases = trace['qpos'], trace['metrics'], trace['phase']
        with subprocess.Popen(['ffmpeg', '-v', 'error', '-n', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                               '-s', '640x480', '-r', '25', '-i', 'pipe:0', '-an', '-c:v', 'libx264',
                               '-threads', '2', '-crf', '20', '-pix_fmt', 'yuv420p', str(args.out)],
                              stdin=subprocess.PIPE) as video:
            for i in range(0, len(states), 4):
                data.qpos[:] = states[i]
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera, scene_option=options)
                frame = Image.fromarray(renderer.render())
                draw = ImageDraw.Draw(frame)
                draw.rectangle((0, 0, 640, 66), fill=(18, 24, 33))
                columns = [0, 1, 3, 4, 5] if metrics.shape[1] >= 18 else [0, 1, 2, 3, 4]
                t, h, displacement, roll, pitch = metrics[i, columns]
                draw.text((10, 8), f'{args.label} | {phases[i]}', fill='white')
                draw.text((10, 27), f't={t:.2f}s   clearance={h:.3f}m   travel={displacement:.3f}m', fill='white')
                draw.text((10, 46), f'pitch={pitch:+.2f}deg   roll={roll:+.2f}deg   support=0 N / 0 Nm', fill='white')
                if i == 0:
                    frame.save(args.out.with_suffix('.png'))
                video.stdin.write(frame.tobytes())
            video.stdin.close()
            if video.wait():
                raise RuntimeError('ffmpeg failed')


if __name__ == '__main__':
    main()
