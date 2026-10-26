"""Dependency-free structural and kinematic validation of the saved URDF."""
from pathlib import Path
import json
import math
import random
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]

def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(len(B))) for j in range(len(B[0]))] for i in range(len(A))]

def tf(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
            [-sp, cp*sr, cp*cr, xyz[2]], [0, 0, 0, 1]]

def vec(s):
    return [float(x) for x in s.split()]

def validate():
    r = ET.parse(PACKAGE / 'urdf/fourth_version.urdf').getroot()
    p = json.loads((PACKAGE / 'config/model_parameters.json').read_text(encoding='utf-8'))
    links, joints = r.findall('link'), r.findall('joint')
    names = [l.get('name') for l in links]
    assert len(set(names)) == len(names) == 7
    assert [j.get('name') for j in joints] == p['expected_runtime_dof_order']
    children = [j.find('child').get('link') for j in joints]
    assert len(set(children)) == 6 and set(names) - set(children) == {'base_link'}
    mass = sum(float(l.find('inertial/mass').get('value')) for l in links)
    assert abs(mass - p['original_mass_kg']) < 1e-10
    for l in links:
        assert float(l.find('inertial/mass').get('value')) > 0
        for tag in ['visual', 'collision']:
            geom = l.find(tag + '/geometry')
            assert geom is not None and len(geom) == 1 and geom[0].tag in ['box', 'cylinder']
            for val in geom[0].attrib.values():
                assert all(x > 0 for x in vec(val))
    for j in joints:
        assert j.find('parent').get('link') in names
        assert vec(j.find('axis').get('xyz')) == [0, 0, 1]
        assert j.get('type') == ('continuous' if 'wheel' in j.get('name') else 'revolute')
        lim = j.find('limit')
        assert float(lim.get('effort')) > 0 and float(lim.get('velocity')) > 0
        if j.get('type') == 'revolute':
            assert float(lim.get('lower')) < p['default_joint_angles_rad'][j.get('name')] < float(lim.get('upper'))

    random.seed(12)
    max_error = 0.0
    for _ in range(100):
        q = {j.get('name'): random.uniform(-1, 1) for j in joints}
        poses = {'base_link': tf([0, 0, 0], [0, 0, 0])}
        for j in joints:
            o = j.find('origin')
            T = tf(vec(o.get('xyz')), vec(o.get('rpy')))
            poses[j.find('child').get('link')] = mul(mul(poses[j.find('parent').get('link')], T), tf([0, 0, 0], [0, 0, q[j.get('name')]]))
        for side, sign in [('l', 1), ('r', -1)]:
            a, b = p['leg_dimensions'][side]['thigh_planar_m'], p['leg_dimensions'][side]['shin_planar_m']
            t1, t2 = sign*q[side+'f0_Joint'], sign*q[side+'f1_Joint']+math.pi/2
            x = a*math.cos(t1)+b*math.cos(t1+t2)
            down = a*math.sin(t1)+b*math.sin(t1+t2)
            H, W = poses[side+'f0_Link'], poses[side+'_wheel_Link']
            max_error = max(max_error, abs(x-(W[0][3]-H[0][3])), abs(down+(W[2][3]-H[2][3])))
    assert max_error < 1e-10
    return {'links': 7, 'moving_joints': 6, 'mass_kg': mass,
            'external_mesh_files_required': 0, 'fudan_fk_cases': 100,
            'max_planar_fk_error_m': max_error, 'isaacgym_runtime': 'not tested by this static validator'}

if __name__ == '__main__':
    print(json.dumps(validate(), indent=2))
