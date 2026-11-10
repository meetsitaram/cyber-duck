"""A trained walk in MuJoCo's classic blue scene, in milli's own colours, the camera following it round: python tools/render_walk.py WALK.npz OUT.mp4 [PNG]

WALK.npz: from the trainer's scripts/milli_eval.py --traj (the robot's root and joints in the trainer's names). This replays what was recorded,
so it shows the policy's motion, not a new simulation. Each link is a mocap body placed by the spec's forward kinematics; the beak stays shut, as it is
locked in training. PNG: also a contact sheet to look at. Plain python.
"""
import os
import sys
import tempfile

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402
import make_rl_model as RL  # noqa: E402
import render_pose as RP  # noqa: E402

W, H = 1280, 720
AZIMUTH = (150, 65)                                # from the front three-quarter round to the side
FPS = 50


def model(scene, tmp):
    assets, bodies = [], {}
    for k, (ln, v, f, color, role) in enumerate(scene.parts):
        if len(f) == 0:
            continue
        RP.stl(os.path.join(tmp, f'p{k}.stl'), v, f)
        assets.append(f'<mesh name="p{k}" file="p{k}.stl"/>')
        mat = 'gloss' if color in RP.GLOSSY else 'matte'
        rgba = ' '.join(f'{c:.3f}' for c in RP.rgba(color))
        bodies.setdefault(ln, []).append(f'<geom type="mesh" mesh="p{k}" material="{mat}" rgba="{rgba}" contype="0" conaffinity="0"/>')
    xml = f"""<mujoco><compiler meshdir="{tmp}"/>
<visual><global offwidth="{W}" offheight="{H}" fovy="30"/><headlight ambient=".3 .3 .3" diffuse=".6 .6 .6" specular="0 0 0"/>
<rgba haze=".15 .25 .35 1"/><quality shadowsize="4096" offsamples="8"/><map shadowclip="0.6" znear="0.01"/></visual>
<statistic extent="1" center="0 0 0.2"/>
<asset><material name="matte" specular=".12" shininess=".25"/><material name="gloss" specular=".7" shininess=".9"/>
<texture type="skybox" builtin="gradient" rgb1=".3 .5 .7" rgb2="0 0 0" width="512" height="3072"/>
<texture type="2d" name="floor" builtin="checker" mark="edge" rgb1=".2 .3 .4" rgb2=".1 .2 .3" markrgb=".8 .8 .8" width="300" height="300"/>
<material name="floor" texture="floor" texrepeat="5 5" texuniform="true" reflectance=".2"/>
{''.join(assets)}</asset>
<worldbody><geom type="plane" size="0 0 .05" material="floor" contype="0" conaffinity="0"/>
<body name="rig" mocap="true"><light directional="true" pos=".25 -.35 1.4" dir="-.18 .25 -1" diffuse=".5 .5 .5" specular=".15 .15 .15" castshadow="true"/></body>
{''.join(f'<body name="{ln}" mocap="true">{"".join(g)}</body>' for ln, g in bodies.items())}
</worldbody></mujoco>"""
    path = os.path.join(tmp, 'scene.xml')
    open(path, 'w', encoding='utf-8').write(xml)
    return mujoco.MjModel.from_xml_path(path), list(bodies)


def main():
    Z = np.load(sys.argv[1])
    out = sys.argv[2]
    q, names = Z['q'], [str(n) for n in Z['names']]
    sign = {rl: (spec_name, s) for spec_name, (rl, s) in RL.JOINT.items()}
    scene = RP.Scene()
    frames = []
    with tempfile.TemporaryDirectory() as tmp:
        m, links = model(scene, tmp)
    d = mujoco.MjData(m)
    rig = m.body_mocapid[m.body('rig').id]
    mids = {ln: m.body_mocapid[m.body(ln).id] for ln in links}
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.elevation, cam.distance = -9, 1.0
    n = len(q)
    follow = q[:, :2].copy()
    k = np.ones(25) / 25                           # the camera follows the walk's average, not each sway
    for a_ in range(2):
        follow[:, a_] = np.convolve(np.pad(q[:, a_], 12, mode='edge'), k, mode='valid')
    for i in range(n):
        Rw = np.zeros(9)
        mujoco.mju_quat2Mat(Rw, q[i, 3:7])
        B = np.eye(4)
        B[:3, :3] = Rw.reshape(3, 3)
        B[:3, 3] = q[i, :3] - B[:3, :3] @ [0, 0, RL.ROOT_Z]    # the trainer's root is the spec's point (0, 0, ROOT_Z)
        qd = {}
        for jn, v in zip(names, q[i, 7:]):
            sn, s = sign[jn]
            qd[sn] = s * float(v)
        M = scene.R.fk(qd, base=B)
        for ln, mid in mids.items():
            d.mocap_pos[mid] = M[ln][:3, 3]
            mujoco.mju_mat2Quat(d.mocap_quat[mid], M[ln][:3, :3].flatten())
        d.mocap_pos[rig] = [follow[i, 0], follow[i, 1], 0]
        mujoco.mj_forward(m, d)
        cam.azimuth = AZIMUTH[0] + (AZIMUTH[1] - AZIMUTH[0]) * i / max(1, n - 1)
        cam.lookat[:] = [follow[i, 0], follow[i, 1], 0.17]
        r.update_scene(d, cam)
        frames.append(r.render().copy())
    r.close()
    imageio.mimsave(out, frames, fps=FPS, codec='libx264', quality=9, macro_block_size=1)
    print(out, n, 'frames')
    if len(sys.argv) > 3:
        pick = np.linspace(n * 0.15, n - 1, 8).astype(int)
        t = [np.array(Image.fromarray(frames[i]).resize((640, 360))) for i in pick]
        Image.fromarray(np.vstack([np.hstack(t[:4]), np.hstack(t[4:])])).save(sys.argv[3])
    return 0


if __name__ == '__main__':
    sys.exit(main())
