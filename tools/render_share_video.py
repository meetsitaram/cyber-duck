"""One video for sharing, under 30 s: the small parts at speed, the big connections (legs, body, head) slowly and the mouth moving, in a dark studio and marked as
animation; then the robot stands still while the scene fades slowly to MuJoCo's classic blue, and only there does the trained walk start, so nobody takes the
animation for the policy. Parts, order and paths are tools/assembly.py's.

  python tools/render_share_video.py WALK.npz OUT.mp4 [SHEET.png]

WALK.npz: from the trainer's scripts/milli_eval.py --traj. The build and mouth are animation; the walk replays a policy's motion in simulation, and says so on screen.
Plain python (MuJoCo lives in the user site-packages).
"""
import math
import sys

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402
import make_rl_model as RL  # noqa: E402
import render_assembly as RA  # noqa: E402
import render_build_video as RB  # noqa: E402

FPS, W, H = RB.FPS, RB.W, RB.H
BUILD_S, MOUTH_S, BLEND_S, TOTAL_S = 9.2, 2.4, 3.0, 29.0
ARROW_S, ARROW_M_PER_N = 0.7, 0.045                # an arrow stays longer than its push, to be seen; the caption gives the real numbers
FOVY = 30.0
HIDE = (1000, 1000, -100)                          # past the far plane
BENCH_FLOOR = -0.14                                # under the shoes on the bench; it rises as the legs go on
DARK_BG = (44, 49, 58)
DARK_HAZE = (44 / 255, 49 / 255, 58 / 255, 1)
VISUAL = f"""<visual><global offwidth="{W}" offheight="{H}" fovy="{FOVY}"/><headlight ambient=".3 .3 .3" diffuse=".6 .6 .6" specular="0 0 0"/>
<rgba haze=".15 .25 .35 1"/><quality shadowsize="4096" offsamples="8"/><map shadowclip="0.6" znear="0.01" zfar="60"/></visual>
<statistic extent="1" center="0 0 0.2"/>"""
ASSET = """<texture type="skybox" builtin="gradient" rgb1=".3 .5 .7" rgb2="0 0 0" width="512" height="3072"/>
<texture type="2d" name="floor" builtin="checker" mark="edge" rgb1=".2 .3 .4" rgb2=".1 .2 .3" markrgb=".8 .8 .8" width="300" height="300"/>
<material name="floor" texture="floor" texrepeat="5 5" texuniform="true" reflectance=".2"/>
<material name="darkfloor" rgba=".2 .22 .26 1" reflectance=".12" specular=".1"/>"""
EXTRA = """<body name="floor" mocap="true"><geom type="plane" size="0 0 .05" material="floor" contype="0" conaffinity="0"/></body>
<body name="darkfloor" mocap="true"><geom type="plane" size="0 0 .05" material="darkfloor" contype="0" conaffinity="0"/></body>
<body name="rig" mocap="true"><light directional="true" pos=".25 -.35 1.4" dir="-.18 .25 -1" diffuse=".5 .5 .5" specular=".15 .15 .15" castshadow="true"/></body>"""


def smooth(t):
    t = min(1.0, max(0.0, t))
    return t * t * (3 - 2 * t)


def is_slow(text, mv):
    """A connection: a finished section carried in, not a part or a screw."""
    return any(a is not None and len(items) > 1 for items, a, _ in mv)


def text_on(img, caption, badge=None, say=None):
    """badge: what the viewer is looking at, always on screen. say: (text, opacity), in the middle of the lower third."""
    im = Image.fromarray(img).convert('RGBA')
    ov = Image.new('RGBA', im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    if badge:
        f_ = RB.font(24)
        w_ = d.textlength(badge[0], font=f_)
        d.rounded_rectangle((W - w_ - 62, 26, W - 30, 70), 10, fill=(*badge[1], 225))
        d.text((W - w_ - 46, 34), badge[0], fill=(255, 255, 255, 255), font=f_)
    if say:
        f_ = RB.font(36)                           # beside the robot, not over it
        y_ = H // 2 - 26 * len(say[0])
        for line in say[0]:
            d.text((64, y_), line, fill=(255, 255, 255, int(255 * say[1])), font=f_, stroke_width=3, stroke_fill=(8, 14, 22, int(220 * say[1])))
            y_ += 52
    if caption:
        d.text((46, H - 70), caption, fill=(255, 255, 255, 235), font=RB.font(30), stroke_width=2, stroke_fill=(10, 20, 30, 200))
    return np.asarray(Image.alpha_composite(im, ov).convert('RGB'))


def main():
    Z = np.load(sys.argv[1])
    out_path = sys.argv[2]
    sheet_path = sys.argv[3] if len(sys.argv) > 3 else None
    spec = G.load()
    R = G.Robot(spec)
    sc = RA.Scene(spec, extra=EXTRA, visual=VISUAL, asset=ASSET)
    floor_mc = sc.m.body_mocapid[sc.m.body('floor').id]
    dark_mc = sc.m.body_mocapid[sc.m.body('darkfloor').id]
    # each scene its own renderer: haze and sky are fixed when a renderer starts. A floor is hidden by its material, as anything moved away still shows in the other's reflection
    classic_haze = sc.m.vis.rgba.haze.copy()
    sc.m.vis.rgba.haze[:] = DARK_HAZE
    r_dark = mujoco.Renderer(sc.m, H, W)
    sc.m.vis.rgba.haze[:] = classic_haze
    r_dark.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = 0
    mat = {True: sc.m.material('floor').id, False: sc.m.material('darkfloor').id}
    mat_a = {k: sc.m.mat_rgba[v, 3] for k, v in mat.items()}
    ANIM, SIM = ('Animation', (92, 96, 106)), ('Trained policy, MuJoCo simulation', (22, 110, 190))
    rig_mc = sc.m.body_mocapid[sc.m.body('rig').id]
    tl = RB.timeline(sc)
    ibox = {n: RA.bounds(sc, [n]) for n in sc.names}
    link_of = {n: sc.B.idx[sc.B.parts[n]['geoms'][0]][0] if n in sc.B.parts else sc.B.screws[n][0] for n in sc.names}
    n_slow = sum(is_slow(t, mv) for t, mv in tl)
    n_fast = len(tl) - n_slow
    fast = 3 if n_fast * 3 < BUILD_S * FPS * 0.6 else 2
    slow = int(np.clip((BUILD_S * FPS - n_fast * fast) / max(1, n_slow), 18, 42))
    print(f'{n_fast} fast beats at {fast} frames, {n_slow} slow at {slow}: build {(n_fast * fast + n_slow * slow) / FPS:.1f} s')
    writer = imageio.get_writer(out_path, fps=FPS, codec='libx264', quality=8, macro_block_size=1, pixelformat='yuv420p')
    frames_for_sheet, count = [], 0
    r = sc.r
    cam = mujoco.MjvCamera()

    def scene_image(mix, z, arrow=None):
        """mix 0: the dark studio. 1: MuJoCo's classic scene. Between: both, the same camera, faded."""
        imgs = []
        for classic in ([False] if mix <= 0 else [True] if mix >= 1 else [False, True]):
            sc.d.mocap_pos[floor_mc] = sc.d.mocap_pos[dark_mc] = (0, 0, z)
            mujoco.mj_forward(sc.m, sc.d)                  # (not mj_kinematics: it leaves the lights where they were, and the scene goes dark)
            for k_, v in mat.items():
                sc.m.mat_rgba[v, 3] = mat_a[k_] if k_ == classic else 0.0
            rd = r if classic else r_dark
            rd.update_scene(sc.d, cam)
            if arrow is not None:
                g = rd.scene.geoms[rd.scene.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9), np.array([1.0, 0.16, 0.1, 1.0], np.float32))
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.012, arrow[0], arrow[1])
                rd.scene.ngeom += 1
            img = rd.render().copy()
            if not classic:
                img[img.sum(axis=2) == 0] = DARK_BG
            imgs.append(img)
        if len(imgs) == 1:
            return imgs[0]
        return (imgs[0] * (1 - mix) + imgs[1] * mix).astype(np.uint8)

    def shot(shown, pos, quats, lookat, size, az, el, caption, mix=0.0, badge=None, say=None, arrow=None):
        nonlocal count
        for n, b in sc.bid.items():
            mc = sc.m.body_mocapid[b]
            sc.d.mocap_pos[mc] = pos.get(n, (0, 0, 0)) if n in shown else HIDE
            sc.d.mocap_quat[mc] = quats[n] if quats else (1, 0, 0, 0)
        sc.d.mocap_pos[rig_mc] = [lookat[0], lookat[1], 0]
        cam.lookat[:] = lookat
        cam.distance = size / (2 * math.tan(math.radians(FOVY / 2)))
        cam.azimuth, cam.elevation = az, el
        img = text_on(scene_image(mix, floor_z, arrow), caption, badge, say)
        writer.append_data(img)
        count += 1
        frames_for_sheet.append((count, img))

    off, shown = {}, set()
    cen, size = None, None

    def target():
        lo, hi = np.full(3, 9.0), np.full(3, -9.0)
        for n in shown:
            o = np.array(off.get(n, (0, 0, 0)))
            lo, hi = np.minimum(lo, ibox[n][0] + o), np.maximum(hi, ibox[n][1] + o)
        return (lo + hi) / 2, max(float(np.linalg.norm(hi - lo)) * 1.05, 0.24)

    floor_z = BENCH_FLOOR
    total_build = n_fast * fast + n_slow * slow
    done = 0
    for text, mv in tl:
        s_ = is_slow(text, mv)
        n_f = slow if s_ else fast
        rising = text == 'Legs onto the body' and s_
        for k in range(n_f):
            t = min(1.0, k / max(1, n_f - 1))
            e = smooth(t)
            for items, a, b_ in mv:
                for n in items:
                    shown.add(n)
                    off[n] = (0, 0, 0) if a is None else tuple(a + (b_ - a) * e)
            if rising:
                floor_z = BENCH_FLOOR * (1 - e)
            c, s2 = target()
            rate = 0.05 if s_ else 0.12
            cen = c if cen is None else cen + rate * (c - cen)
            size = s2 if size is None else size + rate * (s2 - size)
            az = 200 + 15 * done / total_build
            shot(shown, off, None, cen.tolist(), size, az, -14, text if s_ else 'Small parts, at speed', badge=ANIM)
            done += 1
    floor_z = 0.0
    astray = [n for n in sc.names if n not in shown or np.linalg.norm(off[n]) > 1e-9]
    if astray:
        raise SystemExit(f'not in place at the end of the build: {astray[:8]}')

    def placed(M):
        pos, qu = {}, {}
        for n in sc.names:
            m_ = M[link_of[n]]
            pos[n] = tuple(m_[:3, 3])
            qu[n] = RB.quat(m_[:3, :3])
        return pos, qu

    # the mouth, standing in the zero pose, looking ahead
    zero = RB.link_poses(R, {}, 'L', 0.0)
    az0, c0, s0 = 215.0, np.array(cen), float(size)
    head_c = np.array([zero['trunk'][0, 3] + 0.02, 0, 0.2])
    n_mouth = int(MOUTH_S * FPS)
    for k in range(n_mouth):
        t = k / FPS
        beak = 0.4 * (0.5 - 0.5 * math.cos(2 * math.pi * t / 0.75)) if t < MOUTH_S - 0.3 else 0.0
        P = RB.link_poses(R, dict(beak=beak), 'L', 0.0)
        pos, qu = placed(P)
        u = smooth(k / 30)
        shot(shown, pos, qu, (c0 + (head_c - c0) * u).tolist(), s0 + (0.48 - s0) * u, az0, -14, 'Its mouth', badge=ANIM)

    # the walk: turned so it starts ahead, and with its left foot where the zero pose has it
    q, names, dt = Z['q'], [str(n) for n in Z['names']], float(Z['dt'])
    sign = {rl: (sn, s) for sn, (rl, s) in RL.JOINT.items()}

    def walk_frame(i):
        Rw = np.zeros(9)
        mujoco.mju_quat2Mat(Rw, q[i, 3:7])
        B = np.eye(4)
        B[:3, :3] = Rw.reshape(3, 3)
        B[:3, 3] = q[i, :3] - B[:3, :3] @ [0, 0, RL.ROOT_Z]
        qd = {sign[jn][0]: sign[jn][1] * float(v) for jn, v in zip(names, q[i, 7:])}
        return B, qd

    B0, q0 = walk_frame(0)
    yaw0 = math.atan2(B0[1, 0], B0[0, 0])
    T = np.eye(4)
    T[:2, :2] = [[math.cos(-yaw0), -math.sin(-yaw0)], [math.sin(-yaw0), math.cos(-yaw0)]]
    foot = R.fk(q0, base=T @ B0)['L_foot']
    T[:2, 3] = zero['L_foot'][:2, 3] - foot[:2, 3]
    B0T = T @ B0
    # easing from the zero pose into the walk's first pose
    n_blend = int(BLEND_S * FPS)
    zt = zero['trunk']
    joints = set(q0)
    tgt0 = R.fk(q0, base=B0T)['trunk']
    for k in range(n_blend):
        e = smooth(k / max(1, int(n_blend * 0.45)))    # settles into its stance, then stands still while the scene changes
        qd = {j: q0[j] * e for j in joints}
        qa, qb = RB.quat(zt[:3, :3]), RB.quat(tgt0[:3, :3])
        qm = qa * (1 - e) + qb * e
        qm /= np.linalg.norm(qm)
        Mz = np.eye(4)
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, qm)
        Mz[:3, :3] = rot.reshape(3, 3)
        Mz[:3, 3] = zt[:3, 3] * (1 - e) + tgt0[:3, 3] * e
        M = R.fk(qd)
        P = {ln: Mz @ np.linalg.inv(M['trunk']) @ m_ for ln, m_ in M.items()}
        pos, qu = placed(P)
        tt = k / max(1, n_blend - 1)
        say = (('Everything so far', 'was animation.', '', 'Now: the trained', 'policy, in MuJoCo', 'simulation.'), min(smooth(tt / 0.2), smooth((1 - tt) / 0.12)))
        shot(shown, pos, qu, head_c.tolist(), 0.48, az0, -14, '', mix=smooth((tt - 0.1) / 0.75), badge=ANIM if tt < 0.5 else SIM, say=say)
    # the pushes, from the recording: real forces on the trunk during the simulation
    forces = Z['force'] if 'force' in Z.files else np.zeros((len(q), 3))
    on = np.linalg.norm(forces, axis=1) > 0
    starts = [i for i in range(len(on)) if on[i] and (i == 0 or not on[i - 1])]
    pushes = []
    for i0 in starts:
        n_on = 0
        while i0 + n_on < len(on) and on[i0 + n_on]:
            n_on += 1
        pushes.append((i0, T[:3, :3] @ forces[i0], n_on * dt))
    print('pushes:', [(round(i0 * dt, 2), np.round(f, 1).tolist(), round(d_, 2)) for i0, f, d_ in pushes])
    # the walk, the camera easing out and round to follow it
    n_walk = int(TOTAL_S * FPS) - count
    n_walk = min(n_walk, int((len(q) - 1) * dt * FPS))
    xy = []
    for k in range(n_walk):
        B, _ = walk_frame(int(k / FPS / dt))
        xy.append((T @ B)[:2, 3])
    xy = np.array(xy)
    avg = np.ones(31) / 31
    fx = np.stack([np.convolve(np.pad(xy[:, a_], 15, mode='edge'), avg, mode='valid') for a_ in range(2)], axis=1)
    for k in range(n_walk):
        B, qd = walk_frame(int(k / FPS / dt))
        P = R.fk(qd, base=T @ B)
        pos, qu = placed(P)
        u = smooth(k / (3.5 * FPS))
        look = head_c + (np.array([fx[k, 0], fx[k, 1], 0.165]) - head_c) * u
        az = az0 + 50 * smooth(k / max(1, n_walk - 1))
        arrow, cap = None, ''
        for i0, f, d_ in pushes:
            if i0 * dt <= k / FPS < i0 * dt + ARROW_S:
                tip = (P['trunk'] @ np.array([-0.01, 0.0, 0.2, 1.0]))[:3] - f / np.linalg.norm(f) * 0.055
                arrow = (tip - f * ARROW_M_PER_N, tip)
                cap = f'Push: {np.linalg.norm(f):.0f} N for {d_:.2f} s'
        shot(shown, pos, qu, look.tolist(), 0.48 + 0.14 * u, az, -14 + 5 * u, cap, mix=1.0, badge=SIM, arrow=arrow)
    writer.close()
    print(f'{count / FPS:.1f} s, {count} frames ->', out_path)
    if sheet_path:
        pick = np.linspace(0, count - 1, 24).astype(int)
        by = dict(frames_for_sheet)
        ims = [Image.fromarray(by[i + 1]).resize((320, 180)) for i in pick]
        S = Image.new('RGB', (320 * 6, 180 * 4))
        for j, im in enumerate(ims):
            S.paste(im, ((j % 6) * 320, (j // 6) * 180))
        S.save(sheet_path)


if __name__ == '__main__':
    sys.exit(main())
