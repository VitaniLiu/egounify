"""Offline URDF composite inertia calculation. Never writes robot configuration.

Results use the URDF root frame, all movable joints at zero, and SI units.
They exclude the mounting adapter and are NOT controller ToolD arrays.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def origin(element):
    t = np.eye(4)
    if element is None:
        return t
    t[:3, 3] = list(map(float, element.get('xyz', '0 0 0').split()))
    r, p, y = map(float, element.get('rpy', '0 0 0').split())
    cr, cp, cy = np.cos([r, p, y]); sr, sp, sy = np.sin([r, p, y])
    t[:3, :3] = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return t


def combine(parts):
    mass = sum(m for m, _, _ in parts)
    if mass <= 0:
        raise ValueError('Nonpositive total mass')
    com = sum(m * c for m, c, _ in parts) / mass
    inertia = np.zeros((3, 3))
    for m, c, i in parts:
        d = c - com
        inertia += i + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return mass, com, inertia


def derive(path):
    tree = ET.parse(path).getroot()
    links = {l.get('name'): l for l in tree.findall('link')}
    joints = tree.findall('joint')
    children = [j.find('child').get('link') for j in joints]
    roots = set(links) - set(children)
    if len(roots) != 1 or len(children) != len(set(children)):
        raise ValueError('Invalid URDF tree')
    root = roots.pop(); transforms = {root: np.eye(4)}; todo = list(joints)
    while todo:
        previous = len(todo)
        for j in todo[:]:
            parent = j.find('parent').get('link'); child = j.find('child').get('link')
            if parent not in transforms:
                continue
            if j.get('type') not in ('fixed', 'revolute', 'continuous', 'prismatic') or j.find('mimic') is not None:
                raise ValueError('Unsupported joint')
            limit = j.find('limit')
            if limit is not None and not float(limit.get('lower', '0')) <= 0 <= float(limit.get('upper', '0')):
                raise ValueError('Zero posture violates joint limits')
            transforms[child] = transforms[parent] @ origin(j.find('origin'))
            todo.remove(j)
        if len(todo) == previous:
            raise ValueError('Disconnected or cyclic tree')
    parts = []
    for name, link in links.items():
        inertial = link.find('inertial')
        if inertial is None:
            raise ValueError('Missing inertia: ' + name)
        mass = float(inertial.find('mass').get('value'))
        v = inertial.find('inertia').attrib
        i = np.array([[float(v['ixx']), float(v['ixy']), float(v['ixz'])], [float(v['ixy']), float(v['iyy']), float(v['iyz'])], [float(v['ixz']), float(v['iyz']), float(v['izz'])]])
        t = transforms[name] @ origin(inertial.find('origin')); r = t[:3, :3]
        if mass < 0 or not np.isfinite(i).all() or np.linalg.eigvalsh(i).min() < -1e-10:
            raise ValueError('Invalid inertia: ' + name)
        parts.append((mass, t[:3, 3], r @ i @ r.T))
    mass, com, inertia = combine(parts)
    if not np.isfinite(com).all() or not np.isfinite(inertia).all():
        raise ValueError('Nonfinite result')
    return dict(root_frame=root, posture='all URDF joints zero; not an issued command',
                model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), links=len(parts),
                mass_kg=mass, com_m=com.tolist(), inertia_about_com_kg_m2=inertia.tolist(),
                principal_moments_kg_m2=np.linalg.eigvalsh(inertia).tolist())


def self_check():
    # Two equal point masses: analytic center and parallel-axis inertia.
    m, c, i = combine([(2., np.array([-1., 0., 0.]), np.zeros((3, 3))), (2., np.array([1., 0., 0.]), np.zeros((3, 3)))])
    assert m == 4 and np.allclose(c, 0) and np.allclose(i, np.diag([0., 4., 4.]))
    t = origin(ET.fromstring('<origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>'))
    assert np.allclose(t[:3, :3] @ [1, 0, 0], [0, 1, 0])
    assert np.allclose(t[:3, 3], [1, 2, 3])


if __name__ == '__main__':
    self_check()
    root = Path(__file__).parent
    result = dict(status='MODEL_ONLY_NOT_FOR_HARDWARE', controller_parameters_written=False,
                  missing=['flange-to-hand-root transform', 'adapter inertial properties',
                           'model revision confirmation', 'controller ToolD unit/order verification'],
                  arms={})
    for index, side, serial in [(0, 'left', 'CE50903ECE53'), (1, 'right', 'CC50933FCC53')]:
        result['arms']['arm'+str(index)] = dict(side=side, serial=serial,
            **derive(root/'models'/f'{side}_sharpa_wave.urdf'))
    (root/'model_payload.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2, allow_nan=False))
