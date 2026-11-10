"""The big duck's two videos, offline in MuJoCo (no Fusion time): the 30-second reveal and the 5-minute build. docs/images/assembly/big/
tools/render_pov_video.py and cad/fusion/reveal_frames.py make the small ducks'; the big duck's parts, screws and order are tools/big_assembly.py's.

  MILLI_RECIPE=big python tools/render_big_video.py build          the build, build_5min.mp4 (and reports/big/build_sections.json, for tools/cut_clips.py)
  MILLI_RECIPE=big python tools/render_big_video.py build --test   a contact sheet of the build instead, reports/big/pov_frames.png
  MILLI_RECIPE=big python tools/render_big_video.py reveal         the reveal, reveal_30s.mp4
  MILLI_RECIPE=big python tools/render_big_video.py reveal --test  a contact sheet of the reveal, reports/big/reveal_frames.png

Build: the order is big_assembly.full_order(), and each part's last stretch in is the way and travel that order checked (PATH); each screw goes in along its own axis
from its length out (SCREW). Before that, a part comes in from out of the picture along the same line, as far back as it's clear of what's fitted (tested here
the same way, on the same solids), and from the side only where that line is blocked. Subassemblies are built on the bench, alone in the picture, then go on as one.
The printed panels aren't in it: their fixings aren't designed, so big_assembly has no order for them.
Reveal: every part appears in place, from the ground up, motors then skeleton then electronics then covers, on a slow turn that ends on the face.

The actuators, boards and battery are the spec's shapes (RobStride's own models are in Fusion only). Plain python (MuJoCo may be in the user site-packages).
"""
import hashlib
import json
import math
import os
import re
import sys
import tempfile
import time

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import big_assembly as BA  # noqa: E402
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

assert G.RECIPE == 'big', 'MILLI_RECIPE=big'
FPS = 30
W, H = 720, 1280                                   # the build, portrait
RW, RH = 1080, 1920                                # the reveal, portrait
HIDE = (0.0, 0.0, -400.0)                          # past the far plane
BG = (.95, .95, .94)
SCREW_RGBA = (.40, .44, .50, 1)                    # mid-grey: shows on the pale plates and the black actuators
EXT_MAX, EXT_STEP = 0.9, 0.02                      # how far back along its line a part may start, and the step that's tested
CAVEAT = ['A design study: not safe to build.', 'Strength unchecked, panels have no fixings,', 'chain, tensioner and beak drive not designed,', '14 of 22 harnesses don\'t reach.']
ASSET = f"""<texture type="skybox" builtin="flat" rgb1="{BG[0]} {BG[1]} {BG[2]}" rgb2="{BG[0]} {BG[1]} {BG[2]}" width="32" height="32"/>"""
VISUAL = """<visual><global offwidth="1920" offheight="1920"/><map znear="0.004" zfar="60"/><headlight ambient=".44 .44 .44" diffuse=".38 .38 .38" specular=".06 .06 .06"/><quality shadowsize="0"/></visual>
<worldbody><light directional="true" diffuse=".24 .24 .24" specular=".05 .05 .05" dir="-0.5 0.3 -1" castshadow="false"/></worldbody>"""
_EL = (math.radians(40), math.radians(-25))
VIEWS = [np.array([math.cos(a) * math.cos(e), math.sin(a) * math.cos(e), math.sin(e)]) for e in _EL for a in np.radians(np.arange(0, 360, 45))] + [np.array([.05, -.05, 1.0]), np.array([.05, -.05, -1.0])]
SEEN_PX = 12


def ease(t):
    t = min(1.0, max(0.0, t))
    return t * t * (3 - 2 * t)


def font(size):
    try:
        return ImageFont.truetype('arialbd.ttf', size)
    except OSError:
        return ImageFont.load_default()


def way(s):
    return np.array(BA.way(s), float)


# ---------------------------------------------------------------- the parts, as MuJoCo bodies
class Model:
    """big_assembly's parts and screws. Each item is one or more pieces (one per link and four-bar group), each a mocap body placed from the item's pose."""

    def __init__(self, panels):
        t0 = time.time()
        self.spec = G.load()
        self.S = C.Solids(self.spec)
        self.P = BA.Parts(self.spec, self.S)
        self.R = G.Robot(self.spec)
        self.joint = {j['name']: j for j in self.spec['joints']}
        self.mech = {j['name']: j for j in self.spec['mech']['joints']}
        self.samples = self.spec['mech']['samples']
        geoms = lambda ln, i: self.spec['links'][ln]['geoms'][i]
        pieces = {}                                # (item, link, group) -> {colour: [(v, f)]}
        self.box, self.solid = {}, {}
        cloud = []
        in_order = set()

        def walk(steps):
            for st in steps:
                walk(st['steps']) if 'bench' in st else in_order.update(st['parts'])
        walk(BA.full_order())
        for p, ks in self.P.geoms.items():
            if self.P.kind[p] == 'panel' and not panels and p not in in_order:     # (the build has the panels its order fits: the shoes, soles and thigh covers)
                continue
            for ln, i in ks:
                g = geoms(ln, i)
                m = self.S.wall[(ln, i)] if (ln, i) in self.S.wall else self.S.geom[(ln, i)]
                if m is None:
                    continue
                mesh = m.to_mesh()
                v, f = np.asarray(mesh.vert_properties[:, :3], float), np.asarray(mesh.tri_verts, int)
                if len(f):
                    pieces.setdefault((p, ln, g.get('group')), {}).setdefault(g['color'], []).append((v, f))
                    cloud.append(v[::5])
            self.solid[p], self.box[p] = self.P.solid[p], self.P.box[p]
        for s in self.P.screws:
            ln, i = s['hidx']
            mesh = s['solid'].to_mesh()
            pieces[(s['name'], ln, geoms(ln, i).get('group'))] = {'screw': [(np.asarray(mesh.vert_properties[:, :3], float), np.asarray(mesh.tri_verts, int))]}
            self.solid[s['name']], self.box[s['name']] = s['solid'], s['box']
        self.screw = {s['name']: s for s in self.P.screws}
        self.cloud = np.vstack(cloud)              # points of every part, for framing the whole duck
        tmp = tempfile.mkdtemp()
        assets, bodies, self.pieces = [], [], []
        k = 0
        for bi, ((item, ln, grp), cols) in enumerate(pieces.items()):
            gs = ''
            for col, vfs in cols.items():
                vs, fs, n = [], [], 0
                for v, f in vfs:
                    vs.append(v)
                    fs.append(f + n)
                    n += len(v)
                RP.stl(os.path.join(tmp, f'm{k}.stl'), np.vstack(vs), np.vstack(fs))
                assets.append(f'<mesh name="m{k}" file="m{k}.stl"/>')
                rgba = SCREW_RGBA if col == 'screw' else RP.rgba(col)
                gs += f'<geom type="mesh" mesh="m{k}" rgba="{" ".join(f"{c:.3f}" for c in rgba)}"/>'
                k += 1
            bodies.append(f'<body name="b{bi}" mocap="true">{gs}</body>')
            self.pieces.append((item, ln, grp))
        xml = f"""<mujoco><compiler meshdir="{tmp}"/>{VISUAL}<asset>{ASSET}{''.join(assets)}</asset><worldbody>{''.join(bodies)}</worldbody></mujoco>"""
        path = os.path.join(tmp, 'scene.xml')
        open(path, 'w').write(xml)
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        self.mc = np.array([self.m.body_mocapid[self.m.body(f'b{i}').id] for i in range(len(self.pieces))])
        self.geom_item = {}
        self.of_item = {}
        for i, (item, _, _) in enumerate(self.pieces):
            self.of_item.setdefault(item, []).append(i)
            bid = self.m.body(f'b{i}').id
            for gi in np.where(self.m.geom_bodyid == bid)[0]:
                self.geom_item[int(gi)] = item
        self.geom_items = np.array([self.geom_item.get(gi, '') for gi in range(self.m.ngeom)], dtype=object)
        print(f'model: {len(self.of_item)} items, {len(self.pieces)} pieces, {self.m.ngeom} geoms, {time.time() - t0:.0f} s')

    # ---- the four-bar: the knee's cranks and rods at a knee angle, from the spec's samples (as tools/big_sweep.py poses them)
    def bar(self, knee):
        ks = [s['knee'] for s in self.samples]
        return {k: float(np.interp(knee, ks, [s[k] for s in self.samples])) for k in ('crank', 'rod_abs')}

    def local(self, grp, q):
        """The group's transform within its link, at the knee angle in q."""
        T = np.eye(4)
        if not grp:
            return T
        side, t = grp[0], grp[-1]
        b = self.bar(q.get(f'{side}_knee', 0.0))
        o = np.array(self.mech[f'{side}_knee_crank_{t}']['origin'])
        if '_crank_' in grp:
            R_ = G.rot('y', b['crank'])
            T[:3, :3], T[:3, 3] = R_, o - R_ @ o
            return T
        a = np.array(self.mech[f'{side}_knee_rod_{t}']['origin'])
        Rc = G.rot('y', b['crank'])
        shift = (o + Rc @ (a - o)) - a
        R_ = G.rot('y', b['rod_abs'])
        T[:3, :3], T[:3, 3] = R_, a - R_ @ a + shift
        return T

    def place(self, shown, off, q):
        """Mocap poses: shown items at their place under the joints q, moved by off; the rest hidden."""
        Ms = self.R.fk(q)
        quat = np.zeros(4)
        for i, (item, ln, grp) in enumerate(self.pieces):
            mc = self.mc[i]
            if item not in shown:
                self.d.mocap_pos[mc] = HIDE
                continue
            T = Ms[ln] @ self.local(grp, q) if (q or grp) else np.eye(4)
            mujoco.mju_mat2Quat(quat, T[:3, :3].flatten())
            self.d.mocap_pos[mc] = T[:3, 3] + off.get(item, 0.0)
            self.d.mocap_quat[mc] = quat
        mujoco.mj_forward(self.m, self.d)


def check_bar(M):
    """The rods' far ends must stay on the levers' pins as the knee turns: else the four-bar's posing is wrong."""
    worst = 0.0
    for side in 'LR':
        for t in 'IO':
            pin = f'{side} knee lever pin {t}'
            c = (M.box[pin][0] + M.box[pin][1]) / 2
            for kq in np.linspace(-0.9, 0.28, 7):
                q = {f'{side}_knee': kq}
                Ms = M.R.fk(q)
                want = G.apply(Ms[f'{side}_knee_link'], c)
                got = G.apply(Ms[f'{side}_hip_pitch_link'] @ M.local(f'{side}_knee_rod_{t}', q), c)
                worst = max(worst, float(np.linalg.norm(want - got)))
    print(f'four-bar posing: rod ends off their pins by {worst * 1000:.1f} mm at most')
    return worst


# ---------------------------------------------------------------- the order, as events
SECTIONS = [('deck plate', 'body', 'The body'), ('L hip yaw inner retainer', 'leg_L', 'The left leg'), ('R hip yaw inner retainer', 'leg_R', 'The right leg (mirror)'),
            ('neck output L', 'neck', 'The neck'), ('head_yaw actuator (buy) RS03', 'head', 'The head')]
LOW_FROM = 'neck'                                  # labels at the bottom from here on, off the head


def events(M):
    """big_assembly.Fitter.run's walk, without its tests: what goes on, when, along what, and the screws as soon as all they join is there."""
    P = M.P
    done, out = set(), []
    first = {a: (k, t) for a, k, t in SECTIONS}

    def screws(names):
        new = [s['name'] for s in P.screws if s['name'] not in done and s['needs'] <= names]
        done.update(new)
        names.update(new)
        return new

    def run(steps, names, depth):
        here = []
        for st in steps:
            if depth == 0:
                lead = st['bench'] if 'bench' in st else st['parts'][0]
                if lead in first:
                    out.append(dict(kind='section', key=first[lead][0], title=first[lead][1]))
            if 'bench' in st:
                out.append(dict(kind='bench', name=st['bench'], depth=depth + 1))
                sub = set()
                parts = run(st['steps'], sub, depth + 1)
                out.append(dict(kind='join', name=st['bench'], items=parts, way=st['way'], travel=st['travel'], path=st['path'], depth=depth))
            else:
                parts = [p for p in st['parts'] if p in P.solid]
                out.append(dict(kind='part', name=parts[0], items=parts, way=st['way'], travel=st['travel'], path=st['path'], depth=depth))
            names |= set(parts)
            here += parts
            sc = screws(names)
            if sc:
                out.append(dict(kind='screws', items=sc, depth=depth))
                here += sc
        return here

    run(BA.full_order(), set(), 0)
    left = [s['name'] for s in P.screws if s['name'] not in done]
    assert not left, f'screws never fitted: {left[:5]}'
    return out


def plan_paths(M, evs):
    """For each part and subassembly: how far back along its line it can start, clear of what's fitted in its context, and, if that's short of the picture's
    edge, the clear way to come at that start from. The last `travel` is big_assembly's own PATH check; this tests the rest the same way."""
    key = hashlib.md5(open(os.path.join(G.ROOT, 'spec', 'robot_big.json'), 'rb').read() + json.dumps([(e['kind'], e.get('items'), e.get('way')) for e in evs]).encode()).hexdigest()[:12]
    cache = os.path.join(tempfile.gettempdir(), f'big_video_paths_{key}.json')
    if os.path.exists(cache):
        return {int(k): v for k, v in json.load(open(cache)).items()}
    t0 = time.time()
    out, ctx = {}, [[]]
    for i, e in enumerate(evs):
        if e['kind'] == 'bench':
            ctx.append([])
            continue
        if e['kind'] == 'screws':
            ctx[-1] += e['items']
            continue
        if e['kind'] != 'part' and e['kind'] != 'join':
            continue
        if e['kind'] == 'join':
            ctx.pop()
        placed = [(n, M.solid[n], M.box[n]) for n in ctx[-1]]
        grp = C.union([M.solid[n] for n in e['items']])
        w = way(e['way'])
        tr = e['travel'] if e['path'] and e['travel'] > 0 else 0.02

        def clear(off):
            m = grp.translate(list(off))
            b = C.bbox(m)
            for n, s, bb in placed:
                if BA.near(b, bb, 0.0) and C.overlap(m, s) > BA.TINY:
                    return False
            return True
        if not (e['path'] and e['travel'] > 0) and not all(clear(-w * t) for t in np.arange(0.004, tr + 1e-9, 0.004)):
            out[i] = dict(ext=0.0, tr=0.0, side=None, appear=True)       # not a path the order checked, and not clear: it's simply there
            ctx[-1] += e['items']
            continue
        ext = tr
        for t in np.arange(tr + EXT_STEP, EXT_MAX + 1e-9, EXT_STEP):
            if not clear(-w * t):
                break
            ext = t
        side = None
        if ext < 0.45:
            a = -w * ext
            c_ctx = np.mean([(M.box[n][0] + M.box[n][1]) / 2 for n in ctx[-1]], axis=0) if ctx[-1] else np.zeros(3)
            lo, hi = C.bbox(grp)
            out_ = (lo + hi) / 2 - c_ctx
            cands = []
            for u in [out_ * [1, 1, 0], out_, np.array([0, 0, 1.0]), np.array([0, 0, -1.0]), np.array([1.0, 0, 0]), np.array([-1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, -1.0, 0])]:
                if np.linalg.norm(u) < 1e-6 or abs(float(u @ w) / np.linalg.norm(u)) > 0.95:
                    continue
                cands.append(u / np.linalg.norm(u))
            best, best_l = None, -1.0
            for u in cands:
                l = 0.0
                for s_ in np.arange(0.03, 0.75, 0.03):
                    if not clear(a + u * s_):
                        break
                    l = s_
                if l > best_l:
                    best, best_l = u, l
                if l >= 0.72:
                    break
            side = dict(u=best.tolist(), clear=best_l)
        out[i] = dict(ext=float(ext), tr=float(tr), side=side)
        ctx[-1] += e['items']
    json.dump(out, open(cache, 'w'))
    print(f'paths planned in {time.time() - t0:.0f} s -> {cache}')
    return out


# ---------------------------------------------------------------- labels
BEARING = {'hip yaw bearing': '61824 bearing', 'knee bearing': '6908 bearing', 'knee far bearing': '6908 bearing', 'ankle bearing': '6206 bearing', 'head pitch L bearing': '6908 bearing', 'head pitch R bearing': '6908 bearing'}
MAT = {'al6061': '6061 aluminium', 'al7075': '7075 aluminium', 'steel': 'steel', 'petg': 'printed, PETG', 'tpu': 'printed, TPU'}
BOXES = {'battery 13S': '13S pack: a block in the spec', 'battery BMS, fuse and case, allowance': 'space kept for them', 'computer and power boards': 'a block in the spec', 'head boards and camera': 'a placeholder'}


def part_name(n):
    n = re.sub(r' \(buy\).*', '', n).replace('_', ' ')
    n = re.sub(r'^L ', 'Left ', re.sub(r'^R ', 'Right ', n))
    return n[0].upper() + n[1:]


def part_kind(M, n):
    if '(buy) RS' in n:
        return 'RobStride ' + n.split()[-1] + (', a placeholder' if 'beak' in n else '')
    if 'clip (buy)' in n:
        return 'E-clip, DIN 6799, 8 mm'
    m = re.search(r'\(buy\) (\d+T) (#\d+)', n)
    if m:
        return f'sprocket, {m.group(1)}, {m.group(2)} chain'
    if 'chain' in n:
        return '#40 roller chain'
    for k, v in BEARING.items():
        if n.endswith(k):
            return v
    if n in BOXES:
        return BOXES[n]
    mats = {M.spec['links'][ln]['geoms'][i].get('material') for ln, i in M.P.geoms.get(n, [])}
    mats.discard(None)
    return ', '.join(MAT.get(x, x) for x in sorted(mats)) or None


def screw_label(M, names):
    sz = {}
    for n in names:
        s = M.screw[n]
        k = (s['size'], round(s['L'] * 1000), bool(s.get('csk')))
        sz[k] = sz.get(k, 0) + 1
    big = ', '.join(f'{c} x {a} x {b} mm' for (a, b, _), c in sorted(sz.items()))
    kinds = {k[2] for k in sz}
    return big + (': countersunk, hex key' if kinds == {True} else ': socket head, hex key' if kinds == {False} else ': hex key')


# ---------------------------------------------------------------- the build
TESTS = {'L hip yaw beam': [('L_hip_yaw', 'the hip yaw')], 'L hip roll cleat B': [('L_hip_roll', 'the hip roll')], 'L hip pitch bridge': [('L_hip_pitch', 'the hip pitch')],
         'L knee lever pin O clip (buy) DIN 6799 8': [('L_knee', 'the knee\'s four-bar')], 'L foot': [('L_ankle_pitch', 'the ankle')],
         'neck spine': [('neck_pitch', 'the neck')], 'head pitch bridge': [('head_pitch', 'the head pitch')], 'head yaw plate': [('head_yaw', 'the head yaw')],
         'head frame': [('head_roll', 'the head roll')]}
FINAL = ['L_hip_yaw', 'R_hip_yaw', 'L_hip_roll', 'R_hip_roll', 'L_hip_pitch', 'R_hip_pitch', 'L_knee', 'R_knee', 'L_ankle_pitch', 'R_ankle_pitch', 'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll']


class Film:
    def __init__(self, M, test=False, dry=False, every=60):
        self.M, self.m, self.d = M, M.m, M.d
        self.test, self.dry, self.every = test, dry, every
        self.r = None if dry else mujoco.Renderer(self.m, H, W)
        self.small = None
        self.m.vis.global_.orthographic = 0
        self.m.vis.global_.fovy = 62
        self.shown, self.off, self.q = set(), {}, {}
        self.cam_t, self.cam_dir, self.cam_d = np.array([0.0, 0.1, 0.86]), np.array([0.6, -0.6, 0.6]), 1.2
        self.title, self.detail, self.note, self.caveat, self.low = '', '', '', [], False
        self.lab, self.lab_age = None, 0
        self.n, self.sheet, self.seen, self.marks = 0, [], [], {}
        self.empty, self.outside, self.full = [], [], []
        self.writer = None
        self.pace = 1.0
        self.blocked = 0                           # approaches whose way in from the side isn't clear all the way out of the picture
        self.turned = 0                            # approaches whose line came at the camera: in from the side instead, untested

    def open(self, path):
        self.path = path
        self.writer = imageio.get_writer(path, fps=FPS, codec='libx264', quality=8, macro_block_size=1, pixelformat='yuv420p')

    def centre(self, names):
        lo, hi = np.full(3, 9.0), np.full(3, -9.0)
        for n in names:
            o = np.asarray(self.off.get(n, 0.0)) * np.ones(3)
            lo, hi = np.minimum(lo, self.M.box[n][0] + o), np.maximum(hi, self.M.box[n][1] + o)
        return (lo + hi) / 2, float(np.linalg.norm(hi - lo))

    def cam(self, aim, dist, dv):
        c = mujoco.MjvCamera()
        c.type = mujoco.mjtCamera.mjCAMERA_FREE
        c.lookat[:], c.distance = aim, dist
        dv = dv / np.linalg.norm(dv)
        c.azimuth = math.degrees(math.atan2(-dv[1], -dv[0]))
        c.elevation = -math.degrees(math.asin(np.clip(dv[2], -1, 1)))
        return c

    def best_view(self, group, dist, fallback, aim=None, way_=None):
        """The side that shows most of what's being added, in place with the work around it: counted in pixels. Not along its way in, where it flew at the camera."""
        if self.dry:
            return fallback
        if self.small is None:
            self.small = mujoco.Renderer(self.m, 320, 180)
        was = {n: self.off.pop(n, None) for n in group}
        self.M.place(self.shown | set(group), self.off, self.q)
        aim = self.centre(group)[0] if aim is None else aim
        mask = np.isin(self.M.geom_items, list(group))
        here = self.cam_dir / np.linalg.norm(self.cam_dir)
        best, best_s, best_px = fallback, -1.0, 0
        self.small.enable_segmentation_rendering()
        for v in VIEWS:
            self.small.update_scene(self.d, self.cam(aim, dist, v))
            seg = self.small.render()
            ok = (seg[..., 1] == mujoco.mjtObj.mjOBJ_GEOM) & (seg[..., 0] >= 0)
            px = int(mask[seg[..., 0][ok]].sum())
            sc = px * (1 + 0.35 * float(v @ here)) * (1.12 if v[2] > 0 else 1.0)
            if way_ is not None:
                sc *= 1 - 0.8 * float(v @ way_) ** 2
            if sc > best_s:
                best, best_s, best_px = v, sc, px
        self.small.disable_segmentation_rendering()
        for n, o in was.items():
            if o is not None:
                self.off[n] = o
        self.seen.append((best_px, self.title, group[0]))
        return np.array(best, float)

    def frame(self, aim=None, dist=None, dv=None, rate=0.12, watch=None):
        if self.dry:
            self.n += 1
            return
        rate = min(0.6, rate * max(1.0, self.pace))
        if aim is not None:
            self.cam_t = self.cam_t + rate * (np.asarray(aim, float) - self.cam_t)
        if dist is not None:
            self.cam_d += rate * (dist - self.cam_d)
        if dv is not None:
            self.cam_dir = self.cam_dir + rate * 1.3 * (np.asarray(dv, float) - self.cam_dir)
        self.M.place(self.shown, self.off, self.q)
        self.r.update_scene(self.d, self.cam(self.cam_t, self.cam_d, self.cam_dir))
        img = self.r.render().copy()
        bg = np.abs(img.astype(int) - np.array(BG) * 255).sum(-1) > 24
        if bg.mean() < 0.004:
            self.empty.append(self.n)
        if bg.mean() > 0.9:                        # a part across the whole picture: something passing the camera
            self.full.append(self.n)
        if watch:                                  # is what's moving in the picture?
            lo, hi = np.full(3, 9.0), np.full(3, -9.0)
            c, _ = self.centre(watch)
            self.outside.append((self.n, self.inside(c)))
        self.emit(img)

    def inside(self, p):
        """Is point p in the picture (a margin in from its edge)?"""
        dv = self.cam_dir / np.linalg.norm(self.cam_dir)
        eye = self.cam_t + dv * self.cam_d
        f = -dv
        r = np.cross(f, [0, 0, 1.0])
        r /= np.linalg.norm(r) + 1e-9
        u = np.cross(r, f)
        v = p - eye
        z = v @ f
        if z <= 0:
            return False
        th = math.tan(math.radians(31))
        y, x = (v @ u) / (z * th), (v @ r) / (z * th * W / H)
        return abs(x) < 0.95 and abs(y) < 0.95

    def emit(self, img):
        im = Image.fromarray(img).convert('RGBA')
        ov = Image.new('RGBA', im.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)
        ink, grey = (24, 28, 30), (90, 96, 100)
        if self.note:
            big, sub, ink = self.note, '', (20, 115, 80)
        elif self.detail:
            big, _, sub = self.detail.partition(': ')
        else:
            big, sub = self.title, ''
        size = 50
        while d.textlength(big, font=font(size)) > W - 50 and size > 26:
            size -= 2
        rows = [(big, font(size), ink, size + 12)]
        if sub:
            f27 = font(27)
            while d.textlength(sub, font=f27) > W - 50 and f27.size > 20:
                f27 = font(f27.size - 1)
            rows.append((sub, f27, grey, 36))
        rows += [(t, font(26), grey, 33) for t in self.caveat]
        key = (big, sub)
        if key != self.lab:
            self.lab, self.lab_age = key, 0
        self.lab_age += 1
        al = int(255 * min(1.0, self.lab_age / 8))
        y = H - 90 - sum(r[3] for r in rows) if self.low else 44
        for t, fnt, col, step in rows:
            d.text((W / 2 - d.textlength(t, font=fnt) / 2, y), t, fill=col + (al,), font=fnt, stroke_width=3, stroke_fill=(245, 245, 243, al))
            y += step
        out = np.asarray(Image.alpha_composite(im, ov).convert('RGB'))
        if self.writer:
            self.writer.append_data(out)
        if self.test and self.n % self.every == 0:
            self.sheet.append((Image.fromarray(out).resize((W // 3, H // 3)), self.n))
            Image.fromarray(out).save(os.path.join(TEST_DIR, f'b{self.n:05d}.png'))
        self.n += 1

    # ---- what happens
    def dist_for(self, size, least=0.5):
        """Far enough that the part fills about half the picture's narrow width, with the work round it (at 2.2 x its size, a plate filled the picture)."""
        return float(np.clip(size * 3.0, least, 2.4))

    def bring(self, items, plan, w, speed):
        """In from out of the picture, along its own line (or from the side, where that's blocked), and the last `travel` along the checked path."""
        w = np.asarray(w, float)
        c, size = self.centre(items)
        dist = self.dist_for(size)
        dv = self.best_view(items, dist, np.array([0.6, 0.55 if c[1] >= 0 else -0.55, 0.55]), c, w)
        ext, tr = plan['ext'], plan['tr']
        du = dv / np.linalg.norm(dv)
        toward = -float(w @ du)                    # how much of the way back along its line comes at the camera
        if toward > 0.3 and not plan.get('appear'):
            lim = 0.35 * dist / toward             # never nearer the camera than this, or it fills the picture on its way past
            if max(ext, 0.42 * dist + 0.5 * size) > lim:
                ext = max(tr, min(ext, lim))
                side = np.cross(du, [0, 0, 1.0])
                side = side if np.linalg.norm(side) > 1e-3 else np.cross(du, [1.0, 0, 0])
                plan = dict(plan, ext=ext, side=dict(u=(side / np.linalg.norm(side)).tolist(), clear=0.0))
        a = -w * ext
        pts = []
        edge = 0.42 * dist + 0.5 * size
        if plan.get('appear'):
            pts = [np.zeros(3)] * 4
        elif plan['side'] and ext < edge:
            u = np.array(plan['side']['u'])
            pts.append(a + u * max(edge, 0.25))
            self.blocked += plan['side']['clear'] < max(edge, 0.25) - 0.03 and plan['side']['clear'] > 0
            self.turned += plan['side']['clear'] == 0.0
        else:
            pts.append(-w * max(edge, ext))
        if not plan.get('appear'):
            pts += [a, -w * tr, np.zeros(3)]
        if not self.first_seen and not plan.get('appear'):     # a section's or bench's first part: in the picture from the start, on its own line (from outside it, the frame was empty for half a second)
            a = -w * max(tr, min(ext, 0.3 * dist))
            pts = [a, a, -w * tr, np.zeros(3)]
        self.shown.update(items)
        sp = speed * self.pace
        segs = [max(2, int(16 / sp)), max(2, int(12 / sp)), max(3, int(14 / sp))]
        if np.linalg.norm(pts[0] - pts[1]) < 1e-6:
            segs[0] = 0
        if not self.first_seen:                    # a section's or bench's first part: in view at once, not an empty frame first
            self.cam_t, self.cam_d, self.cam_dir = c.copy(), dist, dv.copy()
            self.first_seen = True
        for s_, ((p0, p1), n_) in enumerate(zip(zip(pts, pts[1:]), segs)):
            for k in range(n_):
                t = ease((k + 1) / n_) if n_ > 1 else 1.0
                for x in items:
                    self.off[x] = p0 + (p1 - p0) * t
                self.frame(c, dist, dv, watch=items if s_ == 2 else None)     # (in from outside the picture on purpose till the checked stretch)
        for x in items:
            self.off.pop(x, None)
        for _ in range(max(2, int(6 / sp))):
            self.frame(c, dist, dv)

    def drive(self, names, speed):
        """A step's screws, together for each direction, each along its own axis, from its length out."""
        by = {}
        for n in names:
            by.setdefault(tuple(np.round(self.M.screw[n]['d'], 2)), []).append(n)
        sp = speed * self.pace
        for d_, group in by.items():
            d = np.array(d_, float)
            d /= np.linalg.norm(d)
            c, spread = self.centre(group)
            dist = self.dist_for(max(spread * 1.1, 0.12), 0.36)
            dv = self.best_view(group, dist, -d * 0.6 + np.array([0.3, 0.3, 0.5]), c, d)
            self.detail = screw_label(self.M, group)
            self.shown.update(group)
            m = max(3, int(20 / sp))
            for k in range(m):
                t = ease(k / max(1, m - 1))
                for n in group:
                    L = self.M.screw[n]['L']
                    self.off[n] = -d * (L + 0.0015) * (1 - t)     # (big_assembly's SCREW: from its length out)
                self.frame(c, dist, dv, watch=group)
            for n in group:
                self.off.pop(n, None)
        for _ in range(max(1, int(4 / sp))):
            self.frame()

    def test_joint(self, j, words, speed, view=None, aim=None, dist=None):
        J = self.M.joint[j]
        lo, hi = J['lower'] * 0.7, J['upper'] * 0.7
        if 'knee' in j:                            # the four-bar's sampled range
            lo, hi = max(lo, -0.85), min(hi, 0.26)
        self.note = f'Test: {words} moves freely'
        n = max(10, int(48 / (speed * self.pace)))
        keys = [(0.0, 0.0), (0.3, hi), (0.65, lo), (1.0, 0.0)]
        sub = set(self.M.R.subtree(J['child']))
        child = [x for x in self.shown if any(self.M.pieces[i][1] in sub for i in self.M.of_item[x])]
        Ms0 = self.M.R.fk({})
        c, size = self.centre(child or list(self.shown))
        dist = dist or self.dist_for(size * 1.05)
        aim = c if aim is None else aim
        if view is None:                           # across the joint's axis, from the side it's on (the best-pixel view looked up the knee from below, its four-bar hidden)
            sg = -1.0 if j.startswith('R_') else 1.0
            view = {'y': np.array([0.2, sg, 0.25]), 'x': np.array([1.0, 0.3 * sg, 0.25]), 'z': np.array([0.7, 0.4 * sg, 0.6])}[J['axis']]
        dv = view
        for k in range(n):
            t = k / max(1, n - 1)
            for (t0, a), (t1, b) in zip(keys, keys[1:]):
                if t <= t1:
                    self.q = {j: a + (b - a) * ease((t - t0) / (t1 - t0))}
                    break
            self.frame(aim, dist, dv)
        self.q = {}
        self.note = ''


TEST_DIR = os.path.join(tempfile.gettempdir(), 'big_video_test')


def build(M, evs, plans, test=False, dry=False, pace=1.0):
    f = Film(M, test, dry)
    f.pace = pace
    if not dry and not test:
        p = os.path.join(G.ROOT, 'docs', 'images', 'assembly', 'big', 'build_5min.mp4')
        os.makedirs(os.path.dirname(p), exist_ok=True)
        f.open(p)
    f.ctx = [[]]
    f.first_seen = False
    stack = []
    speed, key = 1.0, None
    pending = []
    upto = int(os.environ.get('BIG_EVENTS', 0)) or len(evs)       # a quick look at the start only
    for i, e in enumerate(evs[:upto]):
        k = e['kind']
        if k == 'section':
            if key:
                f.marks[key][1] = f.n
            key = e['key']
            f.marks[key] = [f.n, None]
            f.title = e['title']
            f.low = f.low or key == LOW_FROM
            speed = 3.0 if key == 'leg_R' else 1.0
            f.first_seen = False
            continue
        if k != 'screws' and pending:
            for j, words in pending:
                f.test_joint(j, words, speed)
            pending = []
        if k == 'bench':
            stack.append((f.title, set(f.shown)))
            f.title = 'Bench: ' + part_name(e['name'])
            f.shown = set()                        # built alone on the bench
            f.ctx.append([])
            f.first_seen = False
            continue
        if k == 'join':
            items = e['items']
            f.ctx.pop()
            f.title, f.shown = stack.pop()
            f.detail = part_name(e['name']) + ': built on the bench'
            f.bring(items, plans[i], way(e['way']), speed)
            f.ctx[-1] += items
            if e['depth'] == 0 and e['name'] in TESTS and key != 'leg_R':
                pending = TESTS[e['name']]
        elif k == 'part':
            items = e['items']
            kd = part_kind(M, items[0])
            f.detail = part_name(items[0]) + (f': {kd}' if kd else '') if len(items) == 1 else part_name(items[0]) + (' and its bushes' if 'rod' in items[0] else '') + (f': {kd}' if kd else '')
            f.bring(items, plans[i], way(e['way']), speed)
            f.ctx[-1] += items
            if e['depth'] == 0 and items[0] in TESTS and key != 'leg_R':
                pending = TESTS[items[0]]
        elif k == 'screws':
            f.drive(e['items'], speed * (1.5 if len(e['items']) > 8 else 1.0))
            f.ctx[-1] += e['items']
            for j, words in pending:
                f.test_joint(j, words, speed)
            pending = []
        f.detail = ''
    for j, words in pending:
        f.test_joint(j, words, speed)
    f.marks[key][1] = f.n
    # the finished frame: every joint, then the whole duck from the front
    f.marks['finished'] = [f.n, None]
    f.shown = {x for e in evs if e['kind'] in ('part', 'join', 'screws') for x in e['items']}     # (all of it by now; with BIG_EVENTS, a quick look at the end)
    f.title, f.detail = 'Finished: try every joint', ''
    f.low = True
    for j in FINAL:
        J = M.joint[j]
        sg = -1.0 if j.startswith('R_') else 1.0
        leg = j[:2] in ('L_', 'R_')
        v = {'y': np.array([0.12, sg, 0.15]), 'x': np.array([1.0, 0.25 * sg, 0.2]), 'z': np.array([0.8, 0.35 * sg, 0.55])}[J['axis']]
        # the whole duck, from the frame's narrow width: at 2 m on the hips, a side view cut off the head and toes, and closer on the head, the neck's swing left the picture
        aim = np.array([0.24, 0.0, 0.8]) if leg else np.array([0.3, 0.0, 0.95])
        f.test_joint(j, j.replace('_', ' ').replace('L ', 'left ').replace('R ', 'right '), 2.0, v, aim, 2.15 if leg else 2.3)
    f.title, f.detail = 'The big duck, 1.8 m', f'The big duck, 1.8 m: 22 RobStride actuators, {len(M.screw)} screws'
    f.caveat = CAVEAT
    nf = 150
    for k in range(nf):
        a = math.radians(-80 + 102 * ease(min(1.0, k / (nf * 0.55))))       # round to the front, then hold on the face
        f.frame(np.array([0.22, 0.0, 0.70]), 2.25, np.array([math.cos(a), math.sin(a), 0.22]), rate=0.06)    # (at 2.75 m it filled half the picture)
    f.marks['finished'][1] = f.n
    if dry:
        return f.n
    if f.writer:
        f.writer.close()
        print(f.path, os.path.getsize(f.path) // 1024, 'kB')
    print(f'{f.n} frames, {f.n / FPS:.0f} s, pace {pace:.2f}; {f.blocked} approaches from the side not clear all the way out of the picture, {f.turned} turned to come in from the side as their line came at the camera')
    print('least visible at 180 x 320 (px, section, part):', sorted(f.seen)[:12])
    out = [n for n, ok in f.outside if not ok]
    print(f'moving part out of the picture in {len(out)} of {len(f.outside)} frames' + (f', first {out[:12]}' if out else ''))
    for what, ns in (('empty', f.empty), ('filled by one part', f.full)):
        runs, cur = [], []
        for n in ns:
            if cur and n != cur[-1] + 1:
                runs.append(cur)
                cur = []
            cur.append(n)
        if cur:
            runs.append(cur)
        print(f'{what}, stretches (first frame, length):', [(r[0], len(r)) for r in runs if len(r) > 5] or 'none')
    if not test:
        with open(G.out_path('reports', 'build_sections.json'), 'w', encoding='utf-8') as fo:
            json.dump(dict(fps=FPS, frames=f.n, sections=f.marks), fo, indent=0)
    else:
        sheet(f.sheet, W // 3, H // 3, G.out_path('reports', 'pov_frames.png'))
    return f.n


def sheet(frames, w, h, path, cols=8):
    rows = (len(frames) + cols - 1) // cols
    im = Image.new('RGB', (cols * w, rows * h), 'white')
    d = ImageDraw.Draw(im)
    for i, (x, n) in enumerate(frames):
        im.paste(x, ((i % cols) * w, (i // cols) * h))
        d.text(((i % cols) * w + 4, (i // cols) * h + 4), str(n), fill=(200, 30, 30))
    im.save(path)
    print('a sheet of portrait frames ->', path)


def fit(M, evs, plans, seconds):
    lo, hi = 0.2, 12.0
    for _ in range(10):
        mid = (lo * hi) ** 0.5
        if build(M, evs, plans, dry=True, pace=mid) / FPS > seconds:
            lo = mid
        else:
            hi = mid
    return hi


# ---------------------------------------------------------------- the reveal
N = 900
STAGES = [('motors', 0, 150), ('skeleton', 150, 330), ('electronics', 360, 480), ('covers', 510, 650)]
FRONT_AZ = 25
HOLD, FADE = 30, 18


def stage_of(M, n):
    if n in M.screw:
        return 'skeleton'
    if '(buy) RS' in n:
        return 'motors'
    k = M.P.kind[n]
    return 'electronics' if k == 'box' else 'covers' if k == 'panel' else 'skeleton'


def reveal(M, test=False):
    m, d = M.m, M.d
    m.vis.global_.orthographic = 1
    r = mujoco.Renderer(m, RH, RW)
    items = list(M.of_item)
    show = {}
    counts = {}
    for name, a, b in STAGES:
        grp = sorted((x for x in items if stage_of(M, x) == name), key=lambda x: M.box[x][0][2])
        counts[name] = len(grp)
        for k, x in enumerate(grp):
            show[x] = a + (b - a) * k / max(1, len(grp) - 1)
    print('reveal stages:', counts)
    n_act = {}
    for x in items:
        if '(buy) RS' in x:
            n_act[x.split()[-1]] = n_act.get(x.split()[-1], 0) + 1
    motors = f"{sum(n_act.values())} RobStride: " + ', '.join(f'{v} {k}' for k, v in sorted(n_act.items(), key=lambda kv: -kv[1]))
    titles = [(0, 150, 'Motors', motors), (150, 355, 'Skeleton', f'aluminium frames, bearings, chain, {len(M.screw)} screws'),
              (360, 505, 'Electronics', 'computer, power boards, 13S battery (blocks)'), (510, 700, 'Covers', 'printed PETG panels, TPU soles'),
              (705, 900, 'Big duck, 1.8 m', '')]
    # the framing: one zoom for the whole turn, from the points of everything, so nothing leaves the picture at any angle
    verts = M.cloud
    lo, hi = verts.min(0), verts.max(0)
    tgt = np.array([(lo[0] + hi[0]) / 2, 0.0, 0.0])
    poses = []
    for f in range(N):
        t = f / (N - 1)
        turn = min(1.0, f / (N * 0.92))
        az = math.radians(FRONT_AZ - 360 * (1 - turn) ** 1.5)
        el = math.radians(14 + 6 * math.sin(t * math.pi))
        poses.append((az, el))
    top_px, bot_px, side_px = 80, 1590, 60          # the duck between the top and the titles
    # each frame's zoom and centre from what that angle needs, smoothed so it eases in and out: a side view of the beak needs more width than the front
    rel = verts[::max(1, len(verts) // 40000)] - tgt
    need, uc, vlo = [], [], []
    for az, el in poses:
        dv = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])
        rgt = np.cross([0, 0, 1.0], dv)
        rgt /= np.linalg.norm(rgt)
        up = np.cross(dv, rgt)
        u, v = rel @ rgt, rel @ up
        need.append(max((v.max() - v.min()) / ((bot_px - top_px) / RH), (u.max() - u.min()) / ((RW - 2 * side_px) / RW) * RH / RW))
        uc.append((u.max() + u.min()) / 2)
        vlo.append(v.min())

    def smooth(x, k, peak):
        x = np.asarray(x)
        if peak:                                   # never less than any frame near it needs
            x = np.array([x[max(0, i - k):i + k + 1].max() for i in range(len(x))])
        ker = np.ones(2 * k + 1) / (2 * k + 1)
        return np.convolve(np.pad(x, k, mode='edge'), ker, mode='valid')
    ext_f = smooth(need, 40, True) * 1.01
    uc_f = smooth(uc, 40, False)
    vlo_f = -smooth(-np.asarray(vlo), 40, True)     # (the lowest point near it, so the feet never sink into the titles)
    print(f'reveal framing: {ext_f.min():.2f} to {ext_f.max():.2f} m tall')
    big, small, tiny = font(76), font(36), font(29)

    def titled(im, t, s, al=255, extra=()):
        ov = Image.new('RGBA', im.size, (0, 0, 0, 0))
        dr = ImageDraw.Draw(ov)
        dr.text((im.width / 2 - dr.textlength(t, font=big) / 2, 1630), t, fill=(24, 28, 30, al), font=big)
        dr.text((im.width / 2 - dr.textlength(s, font=small) / 2, 1722), s, fill=(90, 96, 100, al), font=small)
        y = 1722 if not s else 1770
        for ln in extra:
            dr.text((im.width / 2 - dr.textlength(ln, font=tiny) / 2, y), ln, fill=(90, 96, 100, al), font=tiny)
            y += 34
        return Image.alpha_composite(im, ov)

    def render(f):
        shown = {x for x in items if f >= show[x]}
        M.place(shown, {}, {})
        az, el = poses[f]
        dv = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])
        rgt = np.cross([0, 0, 1.0], dv)
        rgt /= np.linalg.norm(rgt)
        up = np.cross(dv, rgt)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        # the look-at point: centred across, and moved along the picture's up so the duck's lowest point lands at bot_px
        px_m = RH / ext_f[f]
        m.vis.global_.fovy = ext_f[f]
        look = tgt + rgt * uc_f[f] + up * (vlo_f[f] + (bot_px - RH / 2) / px_m)
        cam.lookat[:], cam.distance = look, 6.0
        cam.azimuth = math.degrees(math.atan2(-dv[1], -dv[0]))
        cam.elevation = -math.degrees(math.asin(np.clip(dv[2], -1, 1)))
        r.update_scene(d, cam)
        return r.render().copy()

    frames = range(0, N, 30) if test else range(N)
    out_dir = os.path.join(G.ROOT, 'docs', 'images', 'assembly', 'big')
    os.makedirs(out_dir, exist_ok=True)
    w = None if test else imageio.get_writer(os.path.join(out_dir, 'reveal_30s.mp4'), fps=FPS, codec='libx264', quality=9, macro_block_size=1, pixelformat='yuv420p')
    t_end, s_end = titles[-1][2], titles[-1][3]
    last = Image.fromarray(render(N - 1)).convert('RGBA')
    first = Image.fromarray(render(0)).convert('RGBA')
    end_card = titled(last, t_end, s_end, extra=CAVEAT)
    if w:
        for _ in range(HOLD + 15):
            w.append_data(np.asarray(end_card.convert('RGB')))
        for k in range(FADE):
            a = (k + 1) / FADE
            w.append_data(np.asarray(Image.blend(end_card, first, a * a * (3 - 2 * a)).convert('RGB')))
    bad, shots = [], []
    for f in frames:
        img = render(f)
        fg = np.abs(img.astype(int) - np.array(BG) * 255).sum(-1) > 30
        edge = fg[:6].any() or fg[-6:].any() or fg[:, :6].any() or fg[:, -6:].any() or fg[1620:].any()
        if edge:
            bad.append(f)
        im = Image.fromarray(img).convert('RGBA')
        for a, b, t, s in titles:
            if a <= f < b:
                al = int(255 * min(1.0, (f - a) / 12, (b - f) / 12)) if b < N else int(255 * min(1.0, (f - a) / 12))
                im = titled(im, t, s, al, CAVEAT if b == N else ())
        rgb = np.asarray(im.convert('RGB'))
        if w:
            w.append_data(rgb)
        else:
            shots.append((Image.fromarray(rgb).resize((RW // 4, RH // 4)), f))
            Image.fromarray(rgb).save(os.path.join(TEST_DIR, f'r{f:04d}.png'))
    print(f'frames where the duck touches the edge or the titles: {len(bad)}' + (f', {bad[:10]}' if bad else ''))
    if w:
        w.close()
        p = os.path.join(out_dir, 'reveal_30s.mp4')
        print(p, os.path.getsize(p) // 1024, 'kB')
    else:
        Image.fromarray(np.asarray(end_card.convert('RGB'))).save(os.path.join(TEST_DIR, 'r_endcard.png'))
        sheet(shots, RW // 4, RH // 4, G.out_path('reports', 'reveal_frames.png'), cols=8)


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else 'build'
    test = '--test' in sys.argv
    os.makedirs(TEST_DIR, exist_ok=True)
    M = Model(panels=(what == 'reveal'))
    if what == 'reveal':
        reveal(M, test)
        return
    if check_bar(M) > 0.003:
        sys.exit('the four-bar is posed wrong')
    evs = events(M)
    plans = plan_paths(M, evs)
    secs = float(os.environ.get('BIG_SECONDS', 300))
    pace = fit(M, evs, plans, secs)
    print(f'{sum(1 for e in evs if e["kind"] in ("part", "join"))} parts and subassemblies, {sum(len(e["items"]) for e in evs if e["kind"] == "screws")} screws; pace {pace:.2f} for {secs:.0f} s')
    build(M, evs, plans, test=test, pace=pace)


if __name__ == '__main__':
    main()
