"""Pose validation: microduck and milli at the same joint angles, the left foot planted, side by side.

  python tools/validate_poses.py [--duck PATH]   -> reports/compare/poses_<name>.png, reports/pose_validation.md

microduck's own poses (its scene.xml keyframes STAND, SIT, FOLD) and single-joint moves are applied to both robots.
Each microduck joint maps to milli's of the same role; the angle's sign comes from the two joints' world axes at the
zero pose (both robots' zero pose is microduck's stance). With the left foot where it stands at the zero pose, the
landmarks (head, body, both knees, the right foot) are compared, microduck scaled by milli's height over its own.
The model and the pictures stay local (microduck's meshes are CC BY-NC-SA); the table is ours.
"""
import argparse
import os
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import compare as CP  # noqa: E402
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

DUCK_DIR = os.path.dirname(CP.DUCK)
MAP = {  # microduck joint -> milli joint
    'left_hip_yaw': 'L_hip_yaw', 'left_hip_roll': 'L_hip_roll', 'left_hip_pitch': 'L_hip_pitch', 'left_knee': 'L_knee',
    'left_ankle': 'L_ankle_pitch', 'right_hip_yaw': 'R_hip_yaw', 'right_hip_roll': 'R_hip_roll',
    'right_hip_pitch': 'R_hip_pitch', 'right_knee': 'R_knee', 'right_ankle': 'R_ankle_pitch',
    'neck_pitch': 'neck_pitch', 'head_pitch': 'head_pitch', 'head_yaw': 'head_yaw', 'head_roll': 'head_roll'}
# landmarks: the joints themselves (name, microduck joints averaged, milli joints averaged)
MARKS = [('head (roll joint)', ['head_roll'], ['head_roll']),
         ('body (hip yaws)', ['left_hip_yaw', 'right_hip_yaw'], ['L_hip_yaw', 'R_hip_yaw']),
         ('L knee', ['left_knee'], ['L_knee']), ('R knee', ['right_knee'], ['R_knee']),
         ('R ankle', ['right_ankle'], ['R_ankle_pitch'])]
MOVES = {
    'hip pitch +0.5 (leg forward)': {'left_hip_pitch': 0.5},
    'knee +0.6': {'left_knee': 0.6},
    'ankle +0.5': {'left_ankle': 0.5},
    'hip roll +0.3, hip yaw +0.3': {'left_hip_roll': 0.3, 'left_hip_yaw': 0.3},
    'neck pitch +0.6, head pitch -0.6': {'neck_pitch': 0.6, 'head_pitch': -0.6},
    'head yaw +1.0, head roll +0.3': {'head_yaw': 1.0, 'head_roll': 0.3},
}


def duck_model(path):
    spec = mujoco.MjSpec.from_file(path)
    spec.visual.global_.offwidth = spec.visual.global_.offheight = 1600
    return spec.compile()


def keyframes(scene_path, m):
    """{name: {joint: angle}} from microduck's scene.xml keyframes (its ctrl lists the 14 actuated joints in order)."""
    sm = mujoco.MjModel.from_xml_path(scene_path)
    out = {}
    for k in range(sm.nkey):
        q = {}
        for j in range(sm.njnt):
            if sm.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
                q[sm.joint(j).name] = float(sm.key_qpos[k][sm.jnt_qposadr[j]])
        out[sm.key(k).name] = q
    return out


def signs(m, d, spec):
    mujoco.mj_forward(m, d)
    J = {j['name']: j for j in spec['joints']}
    s = {}
    for a, b in MAP.items():
        s[a] = float(np.sign(d.xaxis[m.joint(a).id] @ G.AXIS[J[b]['axis']]))
    return s


def duck_pose(m, d, q):
    d.qpos[:] = 0
    d.qpos[2], d.qpos[3] = 0.12, 1.0                    # its model's standing base height (scene.xml INIT)
    for n, v in q.items():
        try:
            d.qpos[m.jnt_qposadr[m.joint(n).id]] = v
        except KeyError:
            pass
    mujoco.mj_kinematics(m, d)
    return d


def duck_marks(m, d, q, foot0):
    duck_pose(m, d, q)
    f = m.body('ankle_left').id
    R = d.xmat[f].reshape(3, 3)
    T = foot0[1] @ R.T                                  # rotation taking the posed foot back to its zero pose
    p0 = d.xpos[f]
    at = lambda js: np.mean([d.xanchor[m.joint(j).id] for j in js], axis=0)
    return {n: T @ (at(js) - p0) + foot0[0] for n, js, _ in MARKS}


def milli_marks(R, q):
    M = R.fk(q, base=np.linalg.inv(R.fk(q)['L_foot']))
    at = lambda js: np.mean([G.apply(M[R.joints[j]['parent']], np.array(R.joints[j]['origin'])[None])[0] for j in js],
                            axis=0)
    return {n: at(js) for n, _, js in MARKS}, M


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duck', default=CP.DUCK)
    a = ap.parse_args()
    spec = G.load()
    R = G.Robot(spec)
    sc = RP.Scene(spec)
    m = duck_model(a.duck)
    d = mujoco.MjData(m)
    sg = signs(m, d, spec)
    duck_pose(m, d, {})
    f = m.body('ankle_left').id
    foot0 = (d.xpos[f].copy(), d.xmat[f].reshape(3, 3).copy())
    zero_d = duck_marks(m, d, {}, foot0)
    zero_m, _ = milli_marks(R, {})
    tg = __import__('json').load(open(os.path.join(G.ROOT, 'spec', 'targets.json'), encoding='utf-8'))['model']
    height = C.measure(spec, sc.S)['height_m']
    k = height / tg['height_m']
    poses = dict(keyframes(os.path.join(DUCK_DIR, 'scene.xml'), m), **MOVES)
    rows = ['# Pose validation: microduck and milli at the same joint angles\n',
            f"microduck scaled x {k:.2f}; the left foot planted. Each landmark's move from the zero pose, microduck's vs "
            f"milli's (mm), and the gap between them. Joint signs from the world axes: "
            + ', '.join(f'{n} {int(v):+d}' for n, v in sg.items() if v < 0) + ' (the rest +1).\n',
            '| pose | landmark | microduck moves | milli moves | gap |', '|---|---|---|---|---|']
    for name, qd in poses.items():
        qd = {n: v for n, v in qd.items() if n in MAP}
        qm = {MAP[n]: sg[n] * v for n, v in qd.items()}
        lim = {j['name']: (j['lower'], j['upper']) for j in spec['joints']}
        clipped = [f'{n} {v:+.2f}' for n, v in qm.items() if not lim[n][0] - 1e-6 <= v <= lim[n][1] + 1e-6]
        qm = {n: float(np.clip(v, *lim[n])) for n, v in qm.items()}
        md = duck_marks(m, d, qd, foot0)
        mm, M = milli_marks(R, qm)
        for n in md:
            dv = (md[n] - zero_d[n]) * k
            mv = mm[n] - zero_m[n]
            rows.append(f"| {name} | {n} | {np.linalg.norm(dv) * 1000:.0f} | {np.linalg.norm(mv) * 1000:.0f} | "
                        f"{np.linalg.norm(dv - mv) * 1000:.0f} |")
        if clipped:
            rows.append(f"| {name} | (milli's limits) | | | clipped: {', '.join(clipped)} |")
        # pictures: side and front, both posed, left foot planted
        tiles = []
        for view in ('side', 'front'):
            duck_pose(m, d, qd)
            img_d = render_posed_duck(m, d, view, 0.36 / k, 0.17 / k, foot0)
            img_m = sc.render(qm, view, 600, 600, 0.36, 0.17)
            t = Image.new('RGB', (1200, 630), 'white')
            t.paste(img_d, (0, 30))
            t.paste(img_m, (600, 30))
            ImageDraw.Draw(t).text((8, 8), f'{name} -- microduck (left), milli (right)', fill=(0, 0, 0))
            tiles.append(t)
        sheet = Image.new('RGB', (2400, 630), 'white')
        sheet.paste(tiles[0], (0, 0))
        sheet.paste(tiles[1], (1200, 0))
        sheet.save(os.path.join(CP.DIR, 'poses_' + ''.join(c if c.isalnum() else '_' for c in name) + '.png'))
        print(name, flush=True)
    out = os.path.join(G.ROOT, 'reports', 'pose_validation.md')
    open(out, 'w', encoding='utf-8').write('\n'.join(rows) + '\n')
    print(out)


def render_posed_duck(m, d, view, pic_h, look_z, foot0):
    """microduck posed in d, moved so its left foot sits where it does at the zero pose, rendered like milli."""
    f = m.body('ankle_left').id
    R = d.xmat[f].reshape(3, 3)
    T = foot0[1] @ R.T
    # apply the planting transform to the free base
    base_R = d.xmat[1].reshape(3, 3)
    new_R = T @ base_R
    new_p = T @ (d.xpos[1] - d.xpos[f]) + foot0[0]
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, new_R.flatten())
    d.qpos[0:3] = new_p
    d.qpos[3:7] = q
    mujoco.mj_kinematics(m, d)
    import math
    az, el = (math.radians(x) for x in RP.VIEWS[view])
    dv = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0, 0, look_z + floor_z(m, d)]
    cam.distance = 2.0
    cam.azimuth = math.degrees(math.atan2(-dv[1], -dv[0]))
    cam.elevation = -math.degrees(el)
    m.vis.global_.orthographic = 1
    m.vis.global_.fovy = pic_h
    opt = mujoco.MjvOption()
    for g in range(6):
        opt.geomgroup[g] = 1
    r = mujoco.Renderer(m, 600, 600)
    r.update_scene(d, cam, opt)
    img = r.render().copy()
    r.enable_segmentation_rendering()
    r.update_scene(d, cam, opt)
    seg = r.render()[:, :, 0]
    r.close()
    img[seg < 0] = 255
    return Image.fromarray(img)


_FLOOR = {}


def floor_z(m, d):
    """The zero pose's lowest point (microduck's floor)."""
    if 'z' not in _FLOOR:
        d2 = mujoco.MjData(m)
        d2.qpos[3] = 1.0
        d2.qpos[2] = 0.12
        mujoco.mj_kinematics(m, d2)
        z = min(float((m.mesh_vert[m.mesh_vertadr[m.geom_dataid[g]]:m.mesh_vertadr[m.geom_dataid[g]] +
                                   m.mesh_vertnum[m.geom_dataid[g]]] @ d2.geom_xmat[g].reshape(3, 3).T)[:, 2].min()
                        + d2.geom_xpos[g][2]) for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH)
        _FLOOR['z'] = z
    return _FLOOR['z']


if __name__ == '__main__':
    main()
