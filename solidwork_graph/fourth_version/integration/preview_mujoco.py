"""Preview the URDF using MuJoCo; this is not the Fudan training environment.

Starts paused in the configured stance. Space: free fall/pause; R: reset.
Uses a temporary MJCF conversion, leaving the source URDF unchanged.
"""
import argparse
import json
import os
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--seconds', type=float, default=5.0,
                        help='Duration of headless uncontrolled physics test')
    parser.add_argument('--screenshot', type=Path, help='Save the initial stance as PNG')
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    if args.headless:
        os.environ.setdefault('MUJOCO_GL', 'osmesa')
    import mujoco
    import numpy as np

    params = json.loads((PACKAGE / 'config/model_parameters.json').read_text())
    urdf = ET.parse(PACKAGE / 'urdf/fourth_version.urdf').getroot()
    for link in urdf.findall('link'):
        for kind in ('visual', 'collision'):
            for i, geom in enumerate(link.findall(kind)):
                geom.set('name', f'{link.get("name")}_{kind}_{i}')
    extension = ET.SubElement(urdf, 'mujoco')
    ET.SubElement(extension, 'compiler', discardvisual='false', fusestatic='false')
    imported = mujoco.MjModel.from_xml_string(ET.tostring(urdf, encoding='unicode'))
    with tempfile.TemporaryDirectory(prefix='fourth_preview_') as tmp:
        path = Path(tmp) / 'import.xml'
        mujoco.mj_saveLastXML(str(path), imported)
        scene = ET.parse(path).getroot()
    scene.remove(scene.find('statistic'))
    ET.SubElement(scene, 'option', timestep='0.002', gravity='0 0 -9.81')
    visual = ET.SubElement(scene, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    world = scene.find('worldbody')
    base = world.find("body[@name='base_link']")
    base.set('pos', f'0 0 {params["spawn_height_m"]}')
    ET.SubElement(base, 'freejoint', name='floating_base')
    # Match the adapter's disabled self-collision setting, but allow ground contact.
    for geom in base.iter('geom'):
        if '_collision_' in geom.get('name', ''):
            geom.set('contype', '1')
            geom.set('conaffinity', '2')
            geom.set('group', '3')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='3 3 0.1',
                  rgba='0.3 0.34 0.38 1', contype='2', conaffinity='1',
                  friction='0.8 0.005 0.0001')
    ET.SubElement(world, 'light', pos='1 -2 3', dir='-0.3 0.3 -1', directional='true')
    model = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding='unicode'))
    data = mujoco.MjData(model)

    def reset():
        mujoco.mj_resetData(model, data)
        for name, angle in params['default_joint_angles_rad'].items():
            data.qpos[model.joint(name).qposadr[0]] = angle
        mujoco.mj_forward(model, data)

    reset()
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
             for i in range(model.njnt)]
    if names[1:] != params['expected_runtime_dof_order']:
        raise RuntimeError(f'Unexpected joint mapping: {names}')
    if model.nbody != 8 or model.nv != 12:
        raise RuntimeError('Expected world + 7 robot bodies, 6 base + 6 joint velocities')
    print('MuJoCo URDF preview: 7 robot bodies, 6 moving joints.', flush=True)
    print('No trained policy or balance controller. Space: free fall/pause; R: reset.', flush=True)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, 0.22]
    camera.distance = 1.7
    camera.azimuth = 135
    camera.elevation = -18
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0  # Hide duplicate collision surfaces in the appearance view.
    if args.screenshot:
        from PIL import Image
        args.screenshot.parent.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(model, height=720, width=960)
        try:
            renderer.update_scene(data, camera=camera, scene_option=option)
            Image.fromarray(renderer.render()).save(args.screenshot)
        finally:
            # MuJoCo 2.3 has no Renderer context manager / public close method.
            if hasattr(renderer, 'close'):
                renderer.close()
            del renderer
        print(f'Saved initial stance: {args.screenshot}', flush=True)
    if args.headless:
        max_contacts = 0
        for _ in range(int(np.ceil(args.seconds / model.opt.timestep))):
            mujoco.mj_step(model, data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise RuntimeError('Non-finite physics state')
            if any(w.number for w in data.warning):
                raise RuntimeError('MuJoCo reported a simulation warning')
            max_contacts = max(max_contacts, data.ncon)
        print(json.dumps({'simulated_seconds': data.time, 'finite_states': True,
                          'max_contacts': max_contacts,
                          'final_base_height_m': float(data.qpos[2]),
                          'controlled_standing': False, 'isaacgym_tested': False}, indent=2))
        return

    import mujoco.viewer
    state = {'paused': True, 'reset': False}

    def key_callback(key):
        if key == 32:
            state['paused'] = not state['paused']
        elif key in (82, 114):
            state['reset'] = True

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance = camera.distance
            viewer.cam.azimuth = camera.azimuth
            viewer.cam.elevation = camera.elevation
            viewer.opt.geomgroup[3] = 0
        while viewer.is_running():
            started = time.monotonic()
            with viewer.lock():
                if state['reset']:
                    reset()
                    state.update(paused=True, reset=False)
                if not state['paused']:
                    for _ in range(5):
                        mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, 0.01 - (time.monotonic() - started)))


if __name__ == '__main__':
    main()
