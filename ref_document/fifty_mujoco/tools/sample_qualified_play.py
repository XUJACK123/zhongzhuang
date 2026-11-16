#!/usr/bin/env python3
"""Play the qualified balance actor with CAD visuals and the original physics.

The display model only receives simulated states; it is never stepped. Meshes
come from the existing corrected CAD visual asset, not from collision hulls.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/'src'))


def display_model(env):
    import mujoco
    import numpy as np

    template_path = PROJECT/'assets/mjcf/fifty_visual.xml'
    template = ET.parse(template_path).getroot()
    root = ET.fromstring(env.source.xml)
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    asset = ET.SubElement(root, 'asset')
    mesh_dir = (template_path.parent/template.find('compiler').get('meshdir')).resolve()
    for original in template.find('asset'):
        element = copy.deepcopy(original)
        if element.tag == 'mesh':
            element.set('file', str(mesh_dir/element.get('file')))
        asset.append(element)
    ET.SubElement(asset, 'texture', name='play_grid', type='2d', builtin='checker',
                  rgb1='.20 .24 .29', rgb2='.32 .37 .43', width='512', height='512')
    ET.SubElement(asset, 'material', name='play_floor', texture='play_grid',
                  texrepeat='20 20', texuniform='true', reflectance='.05')
    bodies = {b.get('name'): b for b in root.iter('body')}
    count = 0
    for body in template.iter('body'):
        target = bodies[body.get('name')]
        for original in body.findall('geom'):
            if original.get('type') != 'mesh':
                continue
            geom = copy.deepcopy(original)
            geom.attrib.pop('class', None)
            geom.set('name', body.get('name')+'_cad')
            geom.attrib.update(group='2', contype='0', conaffinity='0', mass='0', density='0')
            target.append(geom)
            count += 1
    for geom in root.find('worldbody').iter('geom'):
        if geom.get('name') == 'floor':
            geom.attrib.update(group='0', material='play_floor')
        elif geom.get('type') != 'mesh':
            geom.attrib.update(group='3', rgba='.9 .4 .1 .25')
    world = root.find('worldbody')
    ET.SubElement(world, 'light', name='play_key', pos='1 -2 3', dir='-.3 .4 -1',
                  diffuse='.4 .4 .4', directional='true')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    # Keep state layout and all body/joint transforms aligned with the plant.
    assert (model.nq, model.nv, model.nbody) == (env.model.nq, env.model.nv, env.model.nbody)
    for field in ('body_pos', 'body_quat', 'body_mass', 'body_inertia', 'body_ipos',
                  'jnt_pos', 'jnt_axis', 'jnt_range', 'jnt_qposadr', 'jnt_dofadr'):
        np.testing.assert_allclose(getattr(model, field), getattr(env.model, field), atol=1e-12)
    model.vis.headlight.ambient[:] = .25
    model.vis.headlight.diffuse[:] = .4
    return model, mujoco.MjData(model), count


def sync_state(env, model, data):
    import mujoco
    data.qpos[:] = env.data.qpos
    data.qvel[:] = env.data.qvel
    data.time = env.data.time
    mujoco.mj_forward(model, data)


def demo_command(seconds):
    segment = int(seconds % 24)//4
    return [(0., 0., .1), (.15, 0., .1), (0., 0., .1),
            (0., .3, .1), (-.15, 0., .1), (0., -.3, .1)][segment]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint', type=Path, default=PROJECT/'runs/qualified_combined_v1_20261116/update_000400.pt')
    p.add_argument('--scenario', choices=['demo', 'stand', 'forward', 'turning'], default='demo')
    p.add_argument('--seconds', type=float, default=120.)
    p.add_argument('--seed', type=int, default=24000000)
    p.add_argument('--headless', action='store_true', help='run without opening a desktop window')
    p.add_argument('--screenshot', type=Path, help='save the final CAD view')
    p.add_argument('--report', type=Path, help='save playback diagnostics')
    args = p.parse_args()
    if not 0 < args.seconds <= 3600:
        p.error('--seconds must be in (0, 3600]')
    os.environ.setdefault('MUJOCO_GL', 'osmesa' if args.headless else 'glfw')
    import mujoco
    import numpy as np
    import torch
    from fifty_mujoco.envs.sample_env31_qualified import QualifiedHardwareEnv31, ENV_SCHEMA
    from fifty_mujoco.learning.sample_ppo import SamplePolicy31
    from fifty_mujoco.transfer.sample_source import verified_artifact

    torch.set_num_threads(1)
    saved = torch.load(args.checkpoint, map_location='cpu', weights_only=True)
    contract = saved['contract']
    if contract['environment'] != ENV_SCHEMA:
        p.error('a qualified balance training checkpoint is required')
    policy = SamplePolicy31(verified_artifact('model_3050.pt'))
    policy.load_state_dict(saved['model'])
    policy.eval()
    env = QualifiedHardwareEnv31(episode_seconds=args.seconds, wheel_kd=contract['wheel_kd'],
                                knee_margin_rad=contract['knee_margin_rad'])
    model, data, meshes = display_model(env)
    camera = mujoco.MjvCamera()
    camera.distance, camera.azimuth, camera.elevation = 1.5, 120, -18
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0
    controls = {'manual': None, 'paused': False, 'reset': False, 'collisions': False}
    lock = threading.Lock()

    def keypress(key):
        commands = {ord('W'): [.15, 0., .1], ord('S'): [-.15, 0., .1],
                    ord('A'): [0., .3, .1], ord('D'): [0., -.3, .1], ord('X'): [0., 0., .1]}
        with lock:
            if key in commands:
                controls['manual'] = commands[key]
                print('命令:', commands[key], flush=True)
            elif key == ord('M'):
                controls['manual'] = None
            elif key == ord(' '):
                controls['paused'] = not controls['paused']
            elif key == ord('R'):
                controls['reset'] = True
            elif key == ord('C'):
                controls['collisions'] = not controls['collisions']

    def command():
        return demo_command(env.data.time) if args.scenario == 'demo' else {
            'stand': (0., 0., .1), 'forward': (.15, 0., .1), 'turning': (0., .3, .1)}[args.scenario]

    env.reset(args.seed, command())
    sync_state(env, model, data)
    viewer = None
    if not args.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data, key_callback=keypress)
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.5, 120, -18
        viewer.opt.geomgroup[3] = 0
    print('CAD外观 + 最新平衡策略。默认每4秒切换站立/前进/停车/左转/后退/右转。', flush=True)
    print('W前进 S后退 A左转 D右转 X停车 M恢复演示 空格暂停 R重置 C碰撞层；鼠标旋转/缩放。', flush=True)
    start_xy = env.data.qpos[:2].copy()
    max_distance = 0.
    last_print, tick, info = -4., 0, None
    try:
        while viewer is None or viewer.is_running():
            started = time.monotonic()
            with lock:
                state = controls.copy()
                controls['reset'] = False
            if state['reset']:
                env.reset(args.seed, state['manual'] or command())
                controls['paused'] = False
                state['paused'] = False
            if not state['paused']:
                requested = state['manual'] or command()
                env.request(requested)
                obs = env.observation()
                with torch.inference_mode():
                    action = policy.actor(torch.tensor(obs)[None])[0].numpy()
                _, _, done, truncated, info = env.step(action)
                max_distance = max(max_distance, float(np.linalg.norm(env.data.qpos[:2]-start_xy)))
                if info['time_s']-last_print >= 4-1e-6:
                    print(f"t={info['time_s']:.1f}s command={list(requested)} vx={info['point_velocity_xy'][0]:+.3f}m/s xy={env.data.qpos[:2].round(3)}", flush=True)
                    last_print = info['time_s']
                if done or truncated:
                    print('本段结束:', info['failure'] or '达到播放时长', flush=True)
                    if args.headless:
                        break
                    with lock:
                        controls['paused'] = True
            if tick % 4 == 0 or state['paused']:
                sync_state(env, model, data)
                if viewer is not None:
                    viewer.cam.lookat[:] = env.data.qpos[:3]
                    viewer.opt.geomgroup[3] = int(state['collisions'])
                    viewer.sync()
            tick += 1
            if not args.headless:
                time.sleep(max(0., env.source.policy_dt-(time.monotonic()-started)))
    finally:
        if viewer is not None:
            viewer.close()
    sync_state(env, model, data)
    if args.screenshot:
        from PIL import Image
        args.screenshot.parent.mkdir(parents=True, exist_ok=True)
        camera.lookat[:] = env.data.qpos[:3]
        with mujoco.Renderer(model, height=720, width=960) as renderer:
            renderer.update_scene(data, camera=camera, scene_option=options)
            Image.fromarray(renderer.render()).save(args.screenshot)
    report = {'visual_meshes': meshes, 'scenario': args.scenario, 'simulation_seconds': float(env.data.time),
              'maximum_displacement_m': max_distance, 'first_failure': info['failure'] if info else None,
              'physics_model_unchanged': True, 'cad_source': 'assets/mjcf/fifty_visual.xml',
              'scope': 'CAD appearance driven by qualified balance simulation; not hardware video or get-up'}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
