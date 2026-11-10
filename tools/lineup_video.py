"""The whole family to scale, standing in a row from the smallest to the biggest, in echo's living room: one film and one still.

  python tools/lineup_video.py                         -> docs/images/family_lineup.mp4 and docs/images/family_lineup.png
  python tools/lineup_video.py --still-only            the still only
  python tools/lineup_video.py --scale 0.5 --fps 15 --out draft.mp4        a quick draft
  python tools/lineup_video.py --frames 3,9,20 --out frames.png            stills at these seconds, for a look

Plain python (MuJoCo is not in the isolated interpreter's path). The room is echo's (github.com/meetsitaram/echo, tools/scene_living_room.py,
Poly Haven CC0 textures): ECHO_RENDER points at a checkout of it, ../echo-render by default.

(2026-11-06, the user: "all the ducks in one video but to scale standing next to each other, smaller on the left and big duck on the right most.
Slowly move camera from left to right having one focused in the middle ... adjust the camera view so that droid fits the screen. Showcase each one
looking straight, turn it right sideways 60 slowly, on to next one ... The left slowly turns back to straight forward looking. Add a tag on the top
highlighting servos used, total height, and total weight"; "highlight height and width"; "realistic lighting, shadows, nice background to contrast
the white body"; "nice wooden floor texture, with textured wall and posters in the background. May be reuse the same living room ... with shades
and reflections"; "add colors to the ducks like we did in our previous showcase video"; "camera can be glass textured"; "all screw heads can be
steel shiny"; "with lens effect as well"; "while droids waving their heads, make them also move the jaw"; "keep the moves natural".)

Each duck's links are mocap bodies placed by its spec's forward kinematics, its legs at the zero pose (the family's bent-knee stance) on the floor,
its neck, head and beak moving. One MuJoCo scene, so shadows and overlaps are real; then echo's lens in numpy: depth of field from the depth
buffer, focused on the duck in the middle, a little bloom, a vignette. Colours: each duck's white shell in the family video's colour
(docs/images/family/duck_family.mp4), the big duck white; screw heads polished steel; the eye's lens dark glass.
Heights and widths are measured from the posed meshes (the width across the front); weights are tools/mass_model.py's, from materials:
nothing has been printed or weighed.
"""
import argparse
import collections
import json
import math
import os
import pickle
import subprocess
import sys
import tempfile

import imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

ECHO = os.environ.get('ECHO_RENDER', os.path.join(os.path.dirname(ROOT), 'echo-render'))

# smallest to biggest: (recipe, spec file, name on the tag, servos on the tag, shell colour)
DUCKS = [
    ('nano', 'robot_nano', 'nano duck', '9 x SG90 + 4 x MG90S', '#A98CEB'),
    ('dynamixel', 'robot_dynamixel', 'all-Dynamixel duck', '15 x Dynamixel XL330-M288', '#F2C640'),
    ('hd1910', 'robot_hd1910', 'HD-1910 duck', '15 x Feetech HD-1910', '#5BA6E0'),
    ('hybrid', 'robot', 'hybrid duck', '10 x Feetech STS3215 + 5 x Dynamixel XL/XC330', '#78D47E'),
    ('hobby', 'robot_hobby', 'hobby-servo duck', '9 x MG996R + 3 x DS3218 + 3 x MG90S', '#3FC1C4'),
    ('feetech', 'robot_feetech', 'all-Feetech duck', '15 x Feetech STS3215', '#F08CC0'),
    ('big', 'robot_big', 'big duck', '12 x RobStride RS04 + 2 x RS03 + 1 x RS00', None),
]
WHITES = ('#ECEBE6', '#F2F1EC')
TURN = math.radians(-60.0)                         # to its right: clockwise seen from above
FOV = 36.0                                         # the room's lens (echo's clip camera)
FILL = 1.62                                        # the frame's height over the focused duck's: room for the tag over it and the width under it
X_ROW = 3.2                                        # the row; the ducks face the camera (-x)
WALL_X = X_ROW + 2.4                               # the wall behind the row
HOLD, TURN_T, HOLD2, MOVE = 1.8, 2.8, 1.0, 2.8     # seconds: facing, turning, turned, to the next
INTRO, OUTRO, CARD = 1.4, 1.2, 3.0
EL = -5.0                                          # the camera a little above, looking down
FONT, FONT_R = 'arialbd.ttf', 'arial.ttf'


def mass_of(recipe):
    env = dict(os.environ, MILLI_RECIPE=recipe, PYTHONUTF8='1')
    out = subprocess.run([sys.executable, '-I', os.path.join(HERE, 'mass_model.py')], capture_output=True, text=True, env=env, cwd=ROOT).stdout
    for line in out.splitlines():
        if line.startswith('total'):
            return float(line.split()[1])
    raise RuntimeError(f'no mass for {recipe}')


def material_of(color, tint):
    c = color.upper()
    if c == RP.G.__dict__.get('_none', '') or False:
        pass
    if c in WHITES:
        return 'shell', tint or color
    if c == '#C9CDD2':
        return 'steel', '#B9BEC4'
    if c == '#0A1420':
        return 'glass', '#0B1626'
    if c in RP.GLOSSY:
        return 'gloss', color
    return 'matte', color


def duck_meshes(spec_file, tint):
    """Per link, its parts merged by material and colour, in the link's own frame; and the shift that puts the zero pose on the floor,
    its plan's middle on the duck's axis. Cached."""
    path = os.path.join(ROOT, 'spec', f'{spec_file}.json')
    key = f'{spec_file}_{int(os.path.getmtime(path))}_{tint}_v4'
    cache = os.path.join(tempfile.gettempdir(), f'milli_lineup_{key}.pkl')
    if os.path.exists(cache):
        return pickle.load(open(cache, 'rb'))
    spec = json.load(open(path, encoding='utf-8'))
    sc = RP.Scene(spec)
    M = sc.R.fk({})
    base = np.linalg.inv(M['L_foot']) if 'L_foot' in M else np.eye(4)
    M = sc.R.fk({}, base=base)
    groups = collections.defaultdict(lambda: ([], []))
    allv = []
    for ln, v, f, color, role in sc.parts:
        if len(f) == 0:
            continue
        mat, col = material_of(color, tint)
        vs, fs = groups[(ln, mat, col)]
        fs.append(f + sum(len(x) for x in vs))
        vs.append(v)
        allv.append(G.apply(M[ln], v))
    parts = {k: (np.vstack(vs), np.vstack(fs)) for k, (vs, fs) in groups.items()}
    allv = np.vstack(allv)
    lo, hi = allv.min(0), allv.max(0)
    # the duck turns about the point between its feet, as one would (about its plan's middle, the big duck's feet swung half a metre off: its
    # head reaches 0.6 m ahead of them)
    if 'L_foot' in M and 'R_foot' in M:
        fc = 0.5 * (M['L_foot'][:3, 3] + M['R_foot'][:3, 3])
        shift = np.array([fc[0], fc[1], lo[2]])
    else:
        shift = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
    bc = ((lo + hi) / 2 - shift)[:2]               # the plan's middle, from the feet's
    res = dict(parts=parts, size=hi - lo, shift=shift, base=base, bc=bc, pts=(allv[::7] - shift).astype(np.float32))
    pickle.dump(res, open(cache, 'wb'))
    return res


def cx(d):
    """The middle of the duck's plan in the room, facing the camera (its feet's middle is at (X_ROW, y))."""
    return X_ROW - d['mesh']['bc'][0]


def cy(d):
    return d['y'] - d['mesh']['bc'][1]


def reach(d, angles, zcap, side):
    """How far the duck reaches across the row (the room's y) toward `side` (+1 / -1), over the turn's angles, counting only its parts below zcap."""
    P = d['mesh']['pts']
    P = P[P[:, 2] < zcap]
    out = 0.0
    for a in angles:
        th = math.pi + a
        y = math.sin(th) * P[:, 0] + math.cos(th) * P[:, 1]
        out = max(out, float((side * y).max()))
    return out


def half_width(size, angles):
    """Half the plan's width across the row (the room's y) over the turn's angles."""
    L, W = size[0], size[1]
    return max(0.5 * (abs(L * math.sin(a)) + abs(W * math.cos(a))) for a in angles)


def import_room():
    """echo's living room, without its other modules (some share milli's tools' names)."""
    tools = os.path.join(ECHO, 'tools')
    if not os.path.exists(os.path.join(tools, 'scene_living_room.py')):
        raise SystemExit(f'echo\'s living room not found: set ECHO_RENDER to an echo-render checkout (looked in {ECHO})')
    saved = {k: sys.modules.pop(k) for k in ('geom', 'checks', 'render_pose', 'render_motion') if k in sys.modules}
    sys.path.insert(0, tools)
    try:
        import scene_living_room as LR
    finally:
        sys.path.remove(tools)
        for k in ('geom', 'checks', 'render_pose', 'render_motion'):
            sys.modules.pop(k, None)
        sys.modules.update(saved)
    return LR


def studio(ms, LR, tmp, ducks):
    """A quiet room (the user: the living room was "too cluttered"): echo's wooden floor, one long plastered wall behind the row with a skirting
    board (no posters: the user, "no posters please"), a ceiling, daylight from a window out of the picture. Echo's CC0 textures and room helpers."""
    R = LR.Room(ms)
    ms.add_texture(name='sky', type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                   rgb1=[0.95, 0.96, 0.97], rgb2=[0.86, 0.88, 0.90], width=256, height=1536)
    floor = R.material('floor', LR.recolour(LR.CC0 / 'wood_floor.jpg', (124, 92, 64), 1.2), tile=2.0, plane=True, reflectance=0.12, specular=0.35, shininess=0.55)
    wall = R.material('wall', LR.recolour(LR.CC0 / 'plastered_wall_04.jpg', (206, 200, 190), 1.3), tile=2.5, specular=0.03, emission=0.0)
    ceil = R.material('ceil', LR.recolour(LR.CC0 / 'white_planks_clean.jpg', (240, 240, 238), 1.0), tile=1.2, specular=0.02, emission=0.3)
    trim = R.material('trim', rgba=(0.93, 0.92, 0.90, 1), specular=0.25, shininess=0.5)
    ms.worldbody.add_geom(name='room floor', type=mujoco.mjtGeom.mjGEOM_PLANE, pos=[X_ROW, 0, 0], size=[12, 12, 0.05], material=floor,
                          contype=0, conaffinity=0)
    yc = 0.5 * (ducks[0]['y'] + ducks[-1]['y'])
    R.wall((WALL_X, yc - 9), (WALL_X, yc + 9), 0, 3.6, wall, t=0.12)
    # (no side walls or ceiling: never in the picture, their shadows drew diagonals across the wall)
    R.box((WALL_X - 0.012, yc, 0.06), (0.012, 9.0, 0.06), trim)                       # the skirting board
    # a sofa beside the big duck, away from the others (the user: "a sofa to the big duck's side ... can illustrate its true size"): 2.0 m long,
    # its seat 0.45 m up and its back 0.85 m, a common sofa's, in echo's velvet
    big = ducks[-1]
    linen = R.material('linen', LR.recolour(LR.CC0 / 'velour_velvet.jpg', (54, 70, 98), 1.6), tile=0.4, specular=0.06, emission=0.02)   # deep blue velvet: the big duck is white (the user)
    sl, sd = 1.0, 0.45
    sy = big['y'] - reach(big, np.linspace(0, TURN, 13), 9.0, -1) - 0.35 - sl
    sx = WALL_X - 0.12 - sd
    R.soft((sx, sy, 0.25), (sd, sl, 0.13), linen, e=0.12)
    for k in range(2):
        c = sy - sl + 0.15 + (k + 0.5) * (2 * sl - 0.3) / 2
        R.soft((sx - 0.05, c, 0.42), (sd - 0.12, (2 * sl - 0.3) / 4 - 0.005, 0.075), linen, e=0.3)
        R.soft((sx + sd - 0.18, c, 0.66), (0.13, (2 * sl - 0.3) / 4 - 0.005, 0.21), linen, q=LR.quat_xyz(ry=0.12), e=0.35)
    for e_ in (sy - sl + 0.075, sy + sl - 0.075):
        R.soft((sx, e_, 0.43), (sd, 0.075, 0.19), linen, e=0.3)
    leg = R.material('sofa leg', rgba=(0.05, 0.045, 0.04, 1), specular=0.5, shininess=0.8)
    for ex in (sx - sd + 0.06, sx + sd - 0.06):
        for ey in (sy - sl + 0.06, sy + sl - 0.06):
            R.cyl((ex, ey, 0.05), 0.02, 0.05, leg)
    sun = ms.worldbody.add_light(name='window sun', dir=[0.55, -0.55, -0.62], diffuse=[0.24, 0.235, 0.22], specular=[0.08, 0.08, 0.08], castshadow=True,
                                 pos=[X_ROW - 4, yc + 6, 6])
    sun.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    wash = ms.worldbody.add_light(name='wall wash', pos=[WALL_X - 2.0, yc + 1.0, 3.3], dir=[0.6, -0.15, -0.6], diffuse=[0.12, 0.115, 0.11], specular=[0, 0, 0],
                                  castshadow=False, cutoff=90, exponent=12)          # (a wide, soft fall-off: no cone's edge on the wall)
    wash.type = mujoco.mjtLightType.mjLIGHT_SPOT
    return R


def build(tmp, ducks, W, H):
    LR = import_room()
    ms = mujoco.MjSpec()
    ms.compiler.degree = False
    R = studio(ms, LR, tmp, ducks)
    v = ms.visual
    v.global_.offwidth, v.global_.offheight, v.global_.fovy = W, H, FOV
    v.quality.shadowsize, v.quality.offsamples = 8192, 8
    ms.stat.extent = 10.0                          # (MuJoCo's near and far planes scale with it: 2 cm and 60 m, a depth fine enough at nano's 0.7 m)
    v.map.znear, v.map.zfar = 0.002, 6.0
    v.map.shadowclip = 0.32                       # the sun's shadow map over the row (3.2 m), not the whole room: no banding on small faces
    v.headlight.ambient = [0.30, 0.30, 0.30]
    v.headlight.diffuse = [0.20, 0.20, 0.20]
    v.headlight.specular = [0.02, 0.02, 0.02]
    # a soft key light from the camera's side, high, with shadows, and a warm rim from the windows
    span_y = [d['y'] for d in ducks]
    key = ms.worldbody.add_light(name='duck key', pos=[X_ROW - 3.2, 0.5 * (span_y[0] + span_y[-1]) - 1.2, 3.0], diffuse=[0.55, 0.53, 0.50],
                                 specular=[0.25, 0.25, 0.25], castshadow=True, cutoff=50, exponent=2, attenuation=[1, 0, 0])
    key.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL           # (a spot's cone drew a hard diagonal edge across the wall)
    tgt = np.array([X_ROW, 0.5 * (span_y[0] + span_y[-1]), 0.4])
    dvec = tgt - np.array(key.pos)
    key.dir = list(dvec / np.linalg.norm(dvec))
    mats = dict(shell=dict(specular=0.5, shininess=0.6), matte=dict(specular=0.18, shininess=0.3), gloss=dict(specular=0.8, shininess=0.9),
                steel=dict(specular=1.0, shininess=0.97, reflectance=0.0), glass=dict(specular=1.0, shininess=1.0, reflectance=0.0))
    for n, kw in mats.items():
        ms.add_material(name=f'duck_{n}', **kw)
    for k, d in enumerate(ducks):
        d['bodies'] = []
        for j, ((ln, mat, col), (vv, ff)) in enumerate(d['mesh']['parts'].items()):
            n = f'd{k}_{j}'
            RP.stl(os.path.join(tmp, f'{n}.stl'), vv, ff)
            ms.add_mesh(name=n, file=os.path.join(tmp, f'{n}.stl'))
        links = sorted({ln for (ln, _, _) in d['mesh']['parts']})
        for ln in links:
            b = ms.worldbody.add_body(name=f'd{k}:{ln}', mocap=True)
            for j, ((ln2, mat, col), _) in enumerate(d['mesh']['parts'].items()):
                if ln2 != ln:
                    continue
                c = RP.rgba(col)
                b.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f'd{k}_{j}', material=f'duck_{mat}', rgba=[c[0], c[1], c[2], 1.0],
                           contype=0, conaffinity=0, group=2)
            d['bodies'].append(ln)
    m = ms.compile()
    return m


def ease(t):
    t = min(1.0, max(0.0, t))
    return t * t * (3 - 2 * t)


def smooth_noise(t, seed, freqs=(0.11, 0.23, 0.41)):
    """A slow wander in about -1 .. 1: a few sines at unrelated rates, phases by the seed."""
    rng = np.random.default_rng(seed)
    ph = rng.uniform(0, 2 * math.pi, len(freqs))
    w = np.array([1.0, 0.55, 0.3])[:len(freqs)]
    return float(sum(wi * math.sin(2 * math.pi * f * t + p) for wi, f, p in zip(w, freqs, ph)) / w.sum())


def head_pose(d, k, t, focus, turned):
    """A small nod and one bite (the user: "make a small head nod and open and close jaw"): about 5 deg of nod, a little more in focus, each duck a little out of step with the others. The head turns back toward the camera as
    the body turns away. Inside each joint's limits with a margin."""
    J = d['limits']
    live = 0.6 + 0.4 * focus
    ph = 1.3 * k
    nod = 0.09 * live * (0.5 - 0.5 * math.cos(2 * math.pi * 0.55 * t + ph))        # down and back up, a nod about every 1.8 s
    q = {'neck_pitch': 0.04 + 0.35 * nod, 'head_pitch': nod}
    if 'head_yaw' in J:
        q['head_yaw'] = -0.5 * turned
    if 'head_roll' in J:
        q['head_roll'] = 0.0
    # one bite each (the user: "we just need one bite for each duck"): open and shut once as the camera settles on it, shut the rest of the time
    c = (t - d['bite_at']) / 0.8 if d.get('bite_at') is not None else -1.0
    q['beak'] = (math.sin(math.pi * c) ** 2 if 0.0 < c < 1.0 else 0.0) * 0.7 * J['beak'][1]
    for n, v in list(q.items()):
        lo, hi = J[n]
        q[n] = min(hi - 0.02, max(lo + 0.02, v)) if n != 'beak' else min(hi - 0.02, max(lo, v))   # (the beak shuts right to its stop)
    return q


class Camera:
    """The camera looks along +x at the row (the ducks face -x, toward it), slightly from above."""

    def __init__(self, W, H):
        self.W, self.H = W, H

    def fit(self, y, h):
        frame = FILL * h
        dist = frame / 2 / math.tan(math.radians(FOV) / 2)
        return dict(y=y, dist=dist, z=0.43 * frame)

    def basis(self, p):
        el = math.radians(EL)
        fwd = np.array([math.cos(el), 0.0, math.sin(el)])
        eye = np.array([X_ROW, p['y'], p['z']]) - p['dist'] * fwd
        right = np.cross(fwd, [0, 0, 1.0])
        right /= np.linalg.norm(right)
        return eye, fwd, right, np.cross(right, fwd)

    def set(self, cam, p):
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [X_ROW, p['y'], p['z']]
        cam.distance = p['dist']
        cam.azimuth = 0.0
        cam.elevation = EL

    def project(self, p, pt):
        eye, fwd, right, up = self.basis(p)
        d = np.asarray(pt, float) - eye
        z = d @ fwd
        f = self.H / 2 / math.tan(math.radians(FOV) / 2)
        return self.W / 2 + f * (d @ right) / z, self.H / 2 - f * (d @ up) / z


def timeline(n):
    segs, t = [], INTRO
    for k in range(n):
        a = t
        t += HOLD + TURN_T + HOLD2
        segs.append((a, t))
        if k < n - 1:
            t += MOVE
    return segs, t + OUTRO, t + OUTRO + CARD


def state(t, segs, ducks, camera):
    """The camera, each duck's turn, how much each is in focus, the tag (index, opacity) and the dimension lines' opacity."""
    n = len(ducks)
    fits = [camera.fit(cy(d), d['size'][2]) for d in ducks]
    k = 0
    for i, (a, b) in enumerate(segs):
        if t >= a:
            k = i
    a, b = segs[k]
    yaw, focus = [0.0] * n, [0.0] * n
    if t < a:                                       # the intro: on the first duck
        p, tag, dims = fits[0], (0, 0.0), 0.0
        focus[0] = 0.4
    elif t <= b or k == n - 1:
        p = fits[k]
        yaw[k] = TURN * ease((t - a - HOLD) / TURN_T)
        tag = (k, min(1.0, (t - a + 0.3) / 0.6))
        dims = min(1.0, (t - a) / 0.5) * (1.0 - ease((t - a - HOLD + 0.3) / 0.6))
        focus[k] = 1.0
    else:
        s = ease((t - b) / MOVE)
        f0, f1 = fits[k], fits[k + 1]
        lerp = lambda u, v_: math.exp(math.log(u) + (math.log(v_) - math.log(u)) * s)
        p = dict(y=f0['y'] + (f1['y'] - f0['y']) * s, dist=lerp(f0['dist'], f1['dist']), z=lerp(f0['z'], f1['z']))
        yaw[k] = TURN * (1 - ease((t - b) / (MOVE * 1.1)))   # turns back to the front as the camera leaves
        focus[k], focus[k + 1] = 1 - s, s
        tag = (k, max(0.0, 1 - (t - b) / 0.45)) if s < 0.5 else (k + 1, max(0.0, min(1.0, (t - b - MOVE * 0.82) / 0.4)))   # (in once the zoom has settled: not over a head passing behind)
        dims = 0.0
    return p, yaw, focus, tag, max(0.0, dims)


def place(m, dd, ducks, t, yaw, focus):
    """Each duck turned on the spot (a turntable, about the point between its feet: the user was fine with that), its neck, head and beak
    moving (sitting was tried and left out for now: the user)."""
    for k, d in enumerate(ducks):
        q = head_pose(d, k, t, focus[k], yaw[k])
        M = d['robot'].fk(q, base=d['mesh']['base'])
        a = math.pi + yaw[k]
        T = np.eye(4)
        T[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
        T[:3, 3] = [X_ROW, d['y'], 0.0]
        S = np.eye(4)
        S[:3, 3] = -d['mesh']['shift']
        TS = T @ S
        for ln in d['bodies']:
            Wm = TS @ M[ln]
            bid = m.body(f'd{k}:{ln}').mocapid[0]
            dd.mocap_pos[bid] = Wm[:3, 3]
            qq = np.zeros(4)
            mujoco.mju_mat2Quat(qq, np.ascontiguousarray(Wm[:3, :3]).flatten())
            dd.mocap_quat[bid] = qq


def lens(img, depth, focus_d, W, duck=None):
    """echo's lens (tools/render_quality.py), as echo's renderer does it: the robots drawn sharp over a room that softens with its distance from
    the focus. No bloom (it smeared the ducks' edges); a mild vignette and a touch of warmth."""
    H_ = img.shape[0]
    u, v = np.meshgrid((np.arange(W) + 0.5) / W * 2 - 1, (np.arange(H_) + 0.5) / H_ * 2 - 1)
    room = img.astype(float) * (1 - 0.12 * np.clip(u * u + v * v - 0.45, 0, 1.4))[..., None] * np.array([1.015, 1.0, 0.98])
    if depth is None:
        return np.clip(room, 0, 255).astype(np.uint8)
    cmax = 6.0 * W / 1920
    slab = 0.2 * focus_d
    coc = cmax * np.clip((np.abs(depth - focus_d) - slab) / (0.55 * focus_d), 0, 1)
    sig = np.array([0.0, 1.2, 2.6, 4.2, 6.0]) * W / 1920
    # the room blurred with the ducks cut out (a normalised blur), so no duck colour bleeds into the wall round it as a halo
    bgw = 1.0 - (duck.astype(float) if duck is not None else 0.0)
    bgw = np.broadcast_to(bgw, room.shape[:2]) if np.ndim(bgw) else np.ones(room.shape[:2])
    def soft(s_):
        num = ndimage.gaussian_filter(room * bgw[..., None], (s_, s_, 0))
        den = ndimage.gaussian_filter(bgw, s_)[..., None]
        return np.where(den > 1e-3, num / np.maximum(den, 1e-3), room)
    stack = np.stack([room] + [soft(s_) for s_ in sig[1:]])
    pos = np.interp(coc, sig, np.arange(len(sig)))
    lo = np.floor(pos).astype(int)
    fr = (pos - lo)[..., None]
    hi_ = np.minimum(lo + 1, len(sig) - 1)
    yy, xx = np.indices(lo.shape)
    out = stack[lo, yy, xx] * (1 - fr) + stack[hi_, yy, xx] * fr
    if duck is not None:                           # the ducks are never blurred: only the room behind them
        a_ = ndimage.gaussian_filter(duck.astype(float), 0.6)[..., None]
        out = room * a_ + out * (1 - a_)
    return np.clip(out, 0, 255).astype(np.uint8)


def overlay(img, camera, p, ducks, tag, dims, W, H, sc):
    im = Image.fromarray(img).convert('RGBA')
    lay = Image.new('RGBA', im.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(lay)
    fb, fm, fr = ImageFont.truetype(FONT, int(40 * sc)), ImageFont.truetype(FONT, int(30 * sc)), ImageFont.truetype(FONT_R, int(26 * sc))
    k, alpha = tag
    d = ducks[k]
    a = int(255 * max(0.0, min(1.0, alpha)))
    hl = (196, 82, 20)                              # a deep orange: height and width, on the tag and the duck
    if a > 0:
        # no box, as the family video's labels: the name big in the duck's own colour, the rest under it, each with a soft shadow
        # dark on the light wall (the user: "some of them hard to read. May be use gray or black"), the duck's colour as a short bar under its name
        tint = tuple(int(255 * c) for c in RP.rgba(d['tint'])[:3]) if d['tint'] else (150, 150, 150)
        fn = ImageFont.truetype(FONT, int(58 * sc))

        def put(xy, txt, font, col):                  # a light outline keeps it readable over a dark neighbour
            dr.text(xy, txt, font=font, fill=col + (a,), stroke_width=max(1, int(2 * sc)), stroke_fill=(246, 244, 240, int(a * 0.85)))
        y = 26 * sc
        nw = dr.textlength(d['name'], font=fn)
        put((W / 2 - nw / 2, y), d['name'], fn, (28, 30, 34))
        dr.rounded_rectangle([W / 2 - 60 * sc, y + 70 * sc, W / 2 + 60 * sc, y + 77 * sc], radius=3 * sc, fill=tint + (a,))
        y += 90 * sc
        put((W / 2 - dr.textlength(d['servos'], font=fm) / 2, y), d['servos'], fm, (70, 74, 82))
        y += 44 * sc
        stats = [('height', f"{d['size'][2] * 100:.1f} cm"), ('width', f"{d['size'][1] * 100:.1f} cm"),
                 ('weight', f"{d['mass']:.2f} kg" if d['mass'] < 10 else f"{d['mass']:.0f} kg")]
        sw = [dr.textlength(v, font=fm) + dr.textlength(n_, font=fr) + 10 * sc for n_, v in stats]
        x = W / 2 - (sum(sw) + 2 * 44 * sc) / 2
        for (n_, v), w_ in zip(stats, sw):
            put((x, y + 4 * sc), n_, fr, (100, 104, 112))
            put((x + dr.textlength(n_, font=fr) + 10 * sc, y), v, fm, hl if n_ != 'weight' else (28, 30, 34))
            x += w_ + 44 * sc
    if dims > 0.02:
        a2 = int(255 * dims)
        L, Wd, Hd = d['size']
        col, lw = hl + (a2,), max(2, int(3 * sc))
        shadow = (255, 255, 255, int(a2 * 0.35))
        yr = cy(d) - Wd / 2 - 0.08 * Hd                     # beside the duck, on screen right (the room's -y)
        p0, p1 = camera.project(p, [X_ROW, yr, 0.0]), camera.project(p, [X_ROW, yr, Hd])
        segs = [[p0, p1], [(p0[0] - 12 * sc, p0[1]), (p0[0] + 12 * sc, p0[1])], [(p1[0] - 12 * sc, p1[1]), (p1[0] + 12 * sc, p1[1])]]
        zw = Hd * 1.07                                        # the width over its head, under the labels
        q0, q1 = camera.project(p, [cx(d), cy(d) + Wd / 2, zw]), camera.project(p, [cx(d), cy(d) - Wd / 2, zw])
        yy = min(q0[1], q1[1])
        segs += [[(q0[0], yy), (q1[0], yy)], [(q0[0], yy - 12 * sc), (q0[0], yy + 12 * sc)], [(q1[0], yy - 12 * sc), (q1[0], yy + 12 * sc)]]
        for s_ in segs:
            dr.line([(x_ + 2, y_ + 2) for x_, y_ in s_], fill=shadow, width=lw)
        for s_ in segs:
            dr.line(s_, fill=col, width=lw)
        for txt, xy in ((f"{Hd * 100:.1f} cm", (p0[0] + 16 * sc, (p0[1] + p1[1]) / 2 - 18 * sc)),
                        (f"{Wd * 100:.1f} cm", ((q0[0] + q1[0]) / 2 - dr.textlength(f"{Wd * 100:.1f} cm", font=fm) / 2, yy - 44 * sc))):
            dr.text((xy[0] + 2, xy[1] + 2), txt, font=fm, fill=shadow)
            dr.text(xy, txt, font=fm, fill=col)
    # (no footer: the user, "remove the trained in mujoco and other details in the footer for now")
    return np.asarray(Image.alpha_composite(im, lay).convert('RGB'))


def card(img, W, H, sc, alpha):
    im = Image.fromarray(img).convert('RGBA')
    lay = Image.new('RGBA', im.size, (8, 9, 12, int(220 * alpha)))
    dr = ImageDraw.Draw(lay)
    a = int(255 * alpha)
    f1, f2 = ImageFont.truetype(FONT, int(96 * sc)), ImageFont.truetype(FONT_R, int(34 * sc))
    for txt, f, y, c in (('duck-droids', f1, H * 0.40, (232, 100, 42)), ('seven ducks, one family, from 21 cm to 1.8 m', f2, H * 0.55, (235, 235, 235))):
        dr.text((W / 2 - dr.textlength(txt, font=f) / 2, y), txt, font=f, fill=c + (a,))
    return np.asarray(Image.alpha_composite(im, lay).convert('RGB'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(ROOT, 'docs', 'images', 'family_lineup.mp4'))
    ap.add_argument('--still', default=os.path.join(ROOT, 'docs', 'images', 'family_lineup.png'))
    ap.add_argument('--still-only', action='store_true')
    ap.add_argument('--frames', help='seconds, comma separated: a sheet of those frames to --out (png)')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--scale', type=float, default=1.0)
    a = ap.parse_args()
    W, H = int(1920 * a.scale) // 2 * 2, int(1080 * a.scale) // 2 * 2
    ducks = []
    for rec, f, name, servos, tint in DUCKS:
        spec = json.load(open(os.path.join(ROOT, 'spec', f'{f}.json'), encoding='utf-8'))
        d = dict(rec=rec, name=name, servos=servos, tint=tint, mesh=duck_meshes(f, tint), mass=mass_of(rec), robot=G.Robot(spec),
                 limits={j['name']: (j['lower'], j['upper']) for j in spec['joints']})
        d['size'] = d['mesh']['size']
        ducks.append(d)
        print(f"{name}: {d['size'][2] * 100:.1f} cm tall, {d['size'][1] * 100:.1f} cm wide, {d['mass']:.2f} kg", flush=True)
    angles = np.linspace(0, TURN, 13)
    off = 0.0
    for k, d in enumerate(ducks):
        if k:
            prev = ducks[k - 1]
            # only what stands below the shorter duck's head can meet it: the big duck's head turns high over its neighbour's
            zc = 1.15 * min(prev['size'][2], d['size'][2])
            off += reach(prev, angles, zc, -1) + reach(d, angles, zc, +1) + 0.12 * min(prev['size'][2], d['size'][2])
        d['off'] = off
    for d in ducks:                                 # left to right on screen is +y to -y; the big duck clear of the armchair
        d['y'] = 0.6 + (ducks[-1]['off'] - d['off'])
    camera = Camera(W, H)
    with tempfile.TemporaryDirectory() as tmp:
        m = build(tmp, ducks, W, H)
        dd = mujoco.MjData(m)
        r = mujoco.Renderer(m, H, W)
        cam = mujoco.MjvCamera()
        ducks_only = mujoco.MjvOption()
        ducks_only.geomgroup[:] = 0
        ducks_only.geomgroup[2] = 1

        def frame(t, p, yaw, focus, focus_d):
            place(m, dd, ducks, t, yaw, focus)
            mujoco.mj_forward(m, dd)
            camera.set(cam, p)
            r.update_scene(dd, cam)
            img = r.render().copy()
            r.enable_depth_rendering()
            r.update_scene(dd, cam)
            dep = r.render().copy()
            r.update_scene(dd, cam, scene_option=ducks_only)     # the ducks alone: their pixels stay sharp
            duck = r.render() < 50.0
            r.disable_depth_rendering()
            return lens(img, dep, focus_d, W, duck)

        segs, t_end, t_all = timeline(len(ducks))
        for k, d in enumerate(ducks):
            d['bite_at'] = segs[k][0] + 0.5            # its one bite, just after the camera arrives
        if a.frames:
            ts = [float(x) for x in a.frames.split(',')]
            ims = []
            for t in ts:
                p, yaw, focus, tag, dims = state(t, segs, ducks, camera)
                ims.append(overlay(frame(t, p, yaw, focus, p['dist']), camera, p, ducks, tag, dims, W, H, a.scale))
            cols = 2
            sheet = Image.new('RGB', (W * cols, H * ((len(ims) + cols - 1) // cols)))
            for i, im in enumerate(ims):
                sheet.paste(Image.fromarray(im), (W * (i % cols), H * (i // cols)))
            sheet.save(a.out)
            print(a.out)
            return
        # the still: everyone facing front, the whole row in the frame
        mid = 0.5 * (cy(ducks[0]) + cy(ducks[-1]))
        span = ducks[0]['y'] - ducks[-1]['y'] + ducks[-1]['size'][1] + ducks[0]['size'][1]
        hfit = max(ducks[-1]['size'][2] * 1.22, span / (W / H) * 1.10)
        ps = dict(y=mid, dist=hfit / 2 / math.tan(math.radians(FOV) / 2), z=0.40 * hfit)
        img = frame(1.0, ps, [0.0] * len(ducks), [0.3] * len(ducks), ps['dist'])
        im = Image.fromarray(img).convert('RGBA')
        dr = ImageDraw.Draw(im)
        fr = ImageFont.truetype(FONT, int(22 * a.scale))
        for k, d in enumerate(ducks):                # staggered, so neighbours' names don't run together; each over its duck's feet
            x, _ = camera.project(ps, [X_ROW, d['y'], 0.0])                 # between its feet
            _, yb = camera.project(ps, [cx(d) - d['size'][0] / 2, cy(d), 0.0])
            lbl = d['name'].replace(' duck', '')
            y_ = min(yb + (10 if k % 2 == 0 else 36) * a.scale, H - 34 * a.scale)
            dr.text((x - dr.textlength(lbl, font=fr) / 2, y_), lbl, font=fr, fill=(28, 30, 34), stroke_width=max(1, int(2 * a.scale)), stroke_fill=(246, 244, 240))
        im.convert('RGB').save(a.still)
        print(a.still, flush=True)
        if a.still_only:
            return
        n = int(t_all * a.fps)
        wr = imageio.get_writer(a.out, fps=a.fps, codec='libx264', quality=8, pixelformat='yuv420p', macro_block_size=2,
                                ffmpeg_params=['-movflags', '+faststart'])
        for i in range(n):
            t = i / a.fps
            tt = min(t, t_end)
            p, yaw, focus, tag, dims = state(tt, segs, ducks, camera)
            img = overlay(frame(t, p, yaw, focus, p['dist']), camera, p, ducks, tag, dims, W, H, a.scale)
            if t > t_end:
                img = card(img, W, H, a.scale, ease((t - t_end) / 0.8))
            wr.append_data(img)
            if i % 150 == 0:
                print(f'{i}/{n}', flush=True)
        wr.close()
        r.close()
    print(a.out, f'{t_all:.1f} s')


if __name__ == '__main__':
    main()
