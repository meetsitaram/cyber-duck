"""The 5-minute build video, for sharing: portrait, on a plain background, parts only. docs/images/assembly/<variant>/build_5min.mp4

Order, parts and paths are tools/assembly.py's: each part's last 45 mm is the path the checks tested, so nothing is shown arriving by a way that wasn't checked. Each part and screw group is filmed from the side that shows most of it (best_view counts its pixels from 18 views), and each joint moves when a builder could first try it. The servos are the spec's plain shapes (the makers' models are in Fusion only). The rules are in droid-from-spec skill, references/assembly-videos.md.

  python tools/render_pov_video.py           the video, about 20 minutes
  python tools/render_pov_video.py --test    a contact sheet of frames instead, reports/<variant>/pov_frames.png

Plain python (MuJoCo lives in the user site-packages).
"""
import json
import math
import os
import re
import sys

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly as A  # noqa: E402
import geom as G  # noqa: E402
import render_assembly as RA  # noqa: E402

FPS, W, H = 30, 720, 1280
RA.W, RA.H = W, H
APPROACH = 0.045                                    # the checked path: tools/assembly.py TRAVEL
PICK = np.array([0.20, -0.12, 0.05])                # where the next part comes from: the tray at the builder's right
PARKED = 0.6                                        # further than this from the mat, work is parked (run()'s spots, 1-1.5 m off), not on the tray
GLOVE = '.13 .20 .38 1'
DRIVER = '.72 .73 .75 1'
GRIP = '.85 .32 .12 1'

EXTRA = f"""
<body name="handR" mocap="true"><geom type="ellipsoid" size=".038 .034 .012" pos="-.03 0 0" rgba="{GLOVE}"/>
 <geom type="capsule" fromto="-.07 0 0 -.42 0 -.02" size=".027" rgba=".22 .23 .26 1"/>
 <geom type="capsule" fromto="0 .022 0 .03 .024 -.006" size=".007" rgba="{GLOVE}"/><geom type="capsule" fromto="0 .007 0 .036 .008 -.006" size=".0075" rgba="{GLOVE}"/>
 <geom type="capsule" fromto="0 -.008 0 .033 -.009 -.006" size=".0075" rgba="{GLOVE}"/><geom type="capsule" fromto="-.004 -.023 0 .024 -.026 -.006" size=".0065" rgba="{GLOVE}"/>
 <geom type="capsule" fromto="-.045 .032 -.004 -.004 .042 -.012" size=".008" rgba="{GLOVE}"/></body>
<body name="handL" mocap="true"><geom type="ellipsoid" size=".038 .034 .012" pos="-.03 0 0" rgba="{GLOVE}"/>
 <geom type="capsule" fromto="-.07 0 0 -.42 0 -.02" size=".027" rgba=".22 .23 .26 1"/>
 <geom type="capsule" fromto="0 -.022 0 .03 -.024 -.006" size=".007" rgba="{GLOVE}"/><geom type="capsule" fromto="0 -.007 0 .036 -.008 -.006" size=".0075" rgba="{GLOVE}"/>
 <geom type="capsule" fromto="0 .008 0 .033 .009 -.006" size=".0075" rgba="{GLOVE}"/><geom type="capsule" fromto="-.004 .023 0 .024 .026 -.006" size=".0065" rgba="{GLOVE}"/>
 <geom type="capsule" fromto="-.045 -.032 -.004 -.004 -.042 -.012" size=".008" rgba="{GLOVE}"/></body>
<body name="eyeback" mocap="true"><geom type="cylinder" fromto="-.0004 0 0 .0004 0 0" size=".007" rgba=".03 .04 .06 1"/></body>
<body name="driver" mocap="true"><geom type="cylinder" fromto="0 0 0 0 0 .055" size=".0016" rgba="{DRIVER}"/><geom type="cylinder" fromto="0 0 .055 0 0 .125" size=".011" rgba="{GRIP}"/></body>
"""
# smaller hands than a real pair: at true size they hid every part they held
def _scale(m):
    return m.group(1) + '"' + ' '.join(f'{float(v) * HAND_SCALE:.4f}' for v in m.group(2).split()) + '"'


HAND_SCALE = 0.62
_i = EXTRA.index('<body name="handR"')
_j = EXTRA.index('<body name="driver"')
EXTRA = EXTRA[:_i] + re.sub(r'( (?:size|fromto|pos)=)"([^"]+)"', _scale, EXTRA[_i:_j]) + EXTRA[_j:]

# one plain colour, floor and sky alike, as the reveal's background: no pattern, no horizon (a checkered bench looked bad)
ASSET = """<texture type="skybox" builtin="flat" rgb1=".95 .95 .94" rgb2=".95 .95 .94" width="32" height="32"/><texture name="mat" type="2d" builtin="flat" rgb1=".95 .95 .94" rgb2=".95 .95 .94" width="32" height="32"/>
<material name="mat" texture="mat" reflectance="0" emission="1"/>"""

# which joints a builder would try, right after this step: (section, words in the step's caption) -> [(joint, words)]
TESTS = {
    ('leg_L', 'through the outer ear'): [('L_ankle_pitch', 'the ankle')],
    ('leg_L', '4 screws from the inside'): [('L_knee', 'the knee')],
    ('leg_L', 'roll bracket onto'): [('L_hip_pitch', 'the hip pitch')],
    ('leg_R', 'through the outer ear'): [('R_ankle_pitch', 'the ankle')],
    ('leg_R', '4 screws from the inside'): [('R_knee', 'the knee')],
    ('leg_R', 'roll bracket onto'): [('R_hip_pitch', 'the hip pitch')],
    ('body', 'yaw brackets'): [('L_hip_yaw', 'left hip yaw'), ('R_hip_yaw', 'right hip yaw')],
    ('legs', 'left leg'): [],
    ('torso', '4 screws on each horn'): [('neck_pitch', 'the neck')],
    ('headbench', 'yaw bracket'): [('head_yaw', 'the head yaw')],
    ('head', 'The jaw, up from below'): [('beak', 'the jaw')],
}
ORDERED = {('legs',): [('L_hip_roll', 'left hip roll'), ('R_hip_roll', 'right hip roll')],
           ('head',): [('head_pitch', 'the head pitch'), ('head_roll', 'the head roll')]}
FINAL = ['L_hip_yaw', 'R_hip_yaw', 'L_hip_roll', 'R_hip_roll', 'L_hip_pitch', 'R_hip_pitch', 'L_knee', 'R_knee', 'L_ankle_pitch', 'R_ankle_pitch', 'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll', 'beak']


def ease(t):
    return t * t * (3 - 2 * t)


def off_edge(home, far, size, dist):
    """Just outside the picture, on the way from `home` to a parked spot `far`: where work set aside leaves the frame and where it comes back in.
    The parked spots are well out of frame: moving all the way to them (or from them) with the camera on the mat left the frame empty for most of each move."""
    u = np.array(far, float) - home
    u[2] = 0.0
    u /= np.linalg.norm(u) + 1e-9
    # fovy 62 in portrait, looking along -y from above: at the mat the frame's half-width (x) is 0.34 x the distance, its half-height (y, foreshortened) under 0.6 x
    r = min(0.36 * dist / max(abs(u[0]), 1e-3), 0.62 * dist / max(abs(u[1]), 1e-3))
    return home + u * (r + 0.5 * size)


def font(size):
    try:
        return ImageFont.truetype('arialbd.ttf', size)
    except OSError:
        return ImageFont.load_default()


def align(x, up=(0, 0, 1)):
    """A quaternion turning local +x onto x, keeping local z near up."""
    x = np.asarray(x, float)
    x = x / (np.linalg.norm(x) + 1e-12)
    z = np.asarray(up, float) - x * (x @ np.asarray(up, float))
    if np.linalg.norm(z) < 1e-3:
        z = np.array([0, 1.0, 0]) - x * x[1]
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.c_[x, y, z].flatten())
    return q


def align_z(zv):
    zv = np.asarray(zv, float) / (np.linalg.norm(zv) + 1e-12)
    a = np.array([1.0, 0, 0]) if abs(zv[0]) < 0.9 else np.array([0, 1.0, 0])
    x = a - zv * (a @ zv)
    x /= np.linalg.norm(x)
    y = np.cross(zv, x)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.c_[x, y, zv].flatten())
    return q


_SC = {}
# even light on a pale bench, but below white: the covers are white too, and at full brightness they lost their shape
SCREW_RGBA = (.40, .44, .50, 1)    # mid-grey: shows on both the white covers and the dark frames
_EL = (math.radians(45), math.radians(-30))
VIEWS = [np.array([math.cos(a) * math.cos(e), math.sin(a) * math.cos(e), math.sin(e)]) for e in _EL for a in np.radians(np.arange(0, 360, 45))] + [np.array([.05, -.05, 1.0]), np.array([.05, -.05, -1.0])]
SEEN_PX = 12    # at the search's 180 x 320: fewer than this and it's not really in view
LEG_VIEW, HEAD_VIEW = np.array([.9, -.25, .12]), np.array([.8, -.45, .25])   # the finished duck's tests: legs from the front and low, under the covers
VISUAL = """<visual><global offwidth="1600" offheight="1600"/><headlight ambient=".46 .46 .46" diffuse=".36 .36 .36" specular=".06 .06 .06"/><quality shadowsize="0"/></visual>
<worldbody><light directional="true" diffuse=".22 .22 .22" specular=".05 .05 .05" dir="-0.5 0.3 -1" castshadow="false"/></worldbody>"""


class Film:
    def __init__(self, test=False, every=30, name='first_person_build', pace=1.0, dry=False):
        spec = G.load()
        self.spec, self.R = spec, G.Robot(spec)
        self.sc = _SC.setdefault('sc', None) or _SC.update(sc=RA.Scene(spec, extra=EXTRA, asset=ASSET, visual=VISUAL, real_folder=os.path.join(os.environ['TEMP'], 'duck_fusion_stl', G.RECIPE))) or _SC['sc']
        self.pace, self.dry = pace, dry
        self.grip = {'R': 0.0, 'L': 0.0}
        self.m, self.d = self.sc.m, self.sc.d
        self.B = self.sc.B
        self.m.vis.global_.orthographic = 0
        for n_, b_ in self.sc.bid.items():
            if n_ in self.B.screws:
                self.m.geom_rgba[self.m.geom_bodyid == b_] = SCREW_RGBA
        self.seen = []
        self.marks = {}
        self.m.vis.global_.fovy = 62       # (portrait: the height's fov, wide enough that the work fits the narrow width)
        self.link = {}
        for n in self.sc.names:
            self.link[n] = self.B.idx[self.B.parts[n]['geoms'][0]][0] if n in self.B.parts else self.B.screws[n][0]
        self.box = {n: RA.bounds(self.sc, [n]) for n in self.sc.names}
        self.off, self.shown = {}, set()
        self.q = {}
        self.cam_t, self.cam_dir, self.cam_d = np.array([0, 0, 0.05]), np.array([0, -0.75, 0.75]), 0.5
        self.hand = {'R': np.array([0.3, -0.5, 0.05]), 'L': np.array([-0.3, -0.5, 0.05])}
        self.drv = np.array([0.4, -0.5, 0.1])
        self.drv_ax = np.array([0, 0, 1.0])
        self.title, self.step, self.note, self.detail = '', '', '', ''
        self.low = False
        self.lab, self.lab_age = None, 0
        self.test, self.every, self.sheet, self.n = test, every, [], 0
        self.writer = None
        if not test and not dry:
            p = os.path.join(G.ROOT, 'docs', 'images', 'assembly', G.RECIPE, f'{name}.mp4')
            os.makedirs(os.path.dirname(p), exist_ok=True)
            self.path = p
            self.writer = imageio.get_writer(p, fps=FPS, codec='libx264', quality=8, macro_block_size=1, pixelformat='yuv420p')

    def world(self, name):
        """(pos, quat) of an item's body: its assembled pose under the joints moved so far, then where it is on the bench."""
        o = np.array(self.off.get(name, (0, 0, 0)), float)
        if not self.q or self.link[name] not in self.Ms:
            return o, np.array([1.0, 0, 0, 0])
        M = self.Ms[self.link[name]]
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, M[:3, :3].flatten())
        return o + M[:3, 3], q

    def centre(self, names):
        lo, hi = np.full(3, 9.0), np.full(3, -9.0)
        for n in names:
            o = np.array(self.off.get(n, (0, 0, 0)), float)
            lo, hi = np.minimum(lo, self.box[n][0] + o), np.maximum(hi, self.box[n][1] + o)
        return (lo + hi) / 2, float(np.linalg.norm(hi - lo))

    def pose(self):
        self.Ms = self.R.fk(self.q) if self.q else {}
        for n, b in self.sc.bid.items():
            mc = self.m.body_mocapid[b]
            if n in self.shown:
                self.d.mocap_pos[mc], self.d.mocap_quat[mc] = self.world(n)
            else:
                self.d.mocap_pos[mc] = (50, 50, 50)
        # the eye's bore goes right through to the camera, dark in the real head: here it'd show the hood's white inside
        mc = self.m.body_mocapid[self.m.body('eyeback').id]
        self.d.mocap_pos[mc] = (70, 70, 70)
        if 'eye iris' in self.shown:
            pos, quat = self.world('eye iris')
            lo, hi = self.box['eye iris']
            c, out = np.array([lo[0], (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2]), np.zeros(3)
            mujoco.mju_rotVecQuat(out, c, quat)
            self.d.mocap_pos[mc], self.d.mocap_quat[mc] = pos + out, quat

    def best_view(self, group, at, dist, fallback, alt=None):
        """The side that shows most of what's being added, in place, with the work around it: counted in pixels, not guessed from the part's direction."""
        if self.dry:
            return fallback
        keep, was = {n: self.off.get(n) for n in group}, set(self.shown)
        self.shown.update(group)
        if not hasattr(self, 'small'):
            self.small = mujoco.Renderer(self.m, 320, 180)
        for try_at in ([at] if alt is None else [at, alt]):    # enclosed once it's home? Then show it on its way in
            if try_at is not None:
                for n in group:
                    self.off[n] = try_at
            best, best_px = self.search(group, dist, fallback)
            if best_px >= SEEN_PX:
                break
        self.shown = was
        for n, o in keep.items():
            if o is None:
                self.off.pop(n, None)
            else:
                self.off[n] = o
        self.seen.append((best_px, self.title, group[0]))
        return best

    def search(self, group, dist, fallback):
        aim = self.centre(self.work + group)[0]
        self.pose()
        for k in ('handR', 'handL', 'driver'):
            self.d.mocap_pos[self.m.body_mocapid[self.m.body(k).id]] = (60, 60, 60)
        mujoco.mj_forward(self.m, self.d)
        mask = np.isin(self.m.geom_bodyid, [self.sc.bid[n] for n in group])
        here = self.cam_dir / np.linalg.norm(self.cam_dir)
        best, best_s, best_px = fallback, -1.0, 0
        self.small.enable_segmentation_rendering()
        for v in VIEWS:
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.lookat[:], cam.distance = aim, dist or self.cam_d
            cam.azimuth, cam.elevation = math.degrees(math.atan2(-v[1], -v[0])), -math.degrees(math.asin(np.clip(v[2], -1, 1)))
            self.small.update_scene(self.d, cam)
            seg = self.small.render()
            ok = (seg[..., 1] == mujoco.mjtObj.mjOBJ_GEOM) & (seg[..., 0] >= 0)
            px = int(mask[seg[..., 0][ok]].sum())
            sc_ = px * (1 + 0.3 * float(v @ here)) * (1.15 if v[2] > 0 else 1.0)     # (no needless swings; from above if it's no worse)
            if sc_ > best_s:
                best, best_s, best_px = v, sc_, px
        self.small.disable_segmentation_rendering()
        return best, best_px

    def frame(self, aim=None, aim_d=None, aim_dir=None, rate=0.09):
        if self.dry:
            self.n += 1
            return
        rate = min(0.5, rate * max(1.0, self.pace))
        if aim is not None:
            self.cam_t = self.cam_t + rate * (np.asarray(aim) - self.cam_t)
        if aim_d is not None:
            self.cam_d += rate * (aim_d - self.cam_d)
        if aim_dir is not None:
            self.cam_dir = self.cam_dir + rate * 1.3 * (np.asarray(aim_dir) - self.cam_dir)
        self.pose()
        dv = self.cam_dir / np.linalg.norm(self.cam_dir)
        eye = self.cam_t + dv * self.cam_d
        # the builder's shoulders: below the frame, either side of the eye
        side = np.cross(dv, [0, 0, 1.0])
        side /= np.linalg.norm(side) + 1e-9
        for k, sg in (('R', -1), ('L', 1)):
            sh = eye + side * sg * 0.28 - np.array([0, 0, 0.30]) - dv * 0.05
            to = self.hand[k] - sh
            tip = self.hand[k] - to / (np.linalg.norm(to) + 1e-9) * self.grip[k]     # fingertips at the near edge of what it holds, not over it
            mc = self.m.body_mocapid[self.m.body(f'hand{k}').id]
            self.d.mocap_pos[mc] = (60, 60, 60)            # (the user: no hands, focus on the parts)
        mc = self.m.body_mocapid[self.m.body('driver').id]
        self.d.mocap_pos[mc] = (60, 60, 60)
        mujoco.mj_forward(self.m, self.d)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = self.cam_t
        cam.distance = self.cam_d
        cam.azimuth = math.degrees(math.atan2(-dv[1], -dv[0]))
        cam.elevation = -math.degrees(math.asin(np.clip(dv[2], -1, 1)))
        self.sc.r.update_scene(self.d, cam)
        img = self.sc.r.render().copy()
        self.emit(img)

    def emit(self, img):
        im = Image.fromarray(img).convert('RGBA')
        ov = Image.new('RGBA', im.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)
        # As the reveal's titles: two short centred lines on the pale bench, no band, fading in when they change.
        # What's in hand and what it is; the instructions are in the written guide.
        # At the top, until the head starts: then at the bottom, where they don't cover the head.
        m_ = re.match(r'(\d+/\d+)\s+(.*)', self.step)
        ink, grey = (24, 28, 30), (90, 96, 100)
        if self.note:
            big, sub, ink = self.note, '', (20, 115, 80)
        elif self.detail:
            big, _, sub = self.detail.partition(': ')
        else:
            big, sub = self.title, '' if m_ else self.step

        def wrap(t, fnt):
            lines, cur = [], ''
            for wd in t.split():
                if d.textlength(f'{cur} {wd}', font=fnt) > W - 60 and cur:
                    lines.append(cur)
                    cur = wd
                else:
                    cur = f'{cur} {wd}'.strip()
            return (lines + [cur])[:3] if cur else lines
        size = 50
        while d.textlength(big, font=font(size)) > W - 50 and size > 28:
            size -= 2
        rows = [(big, font(size), ink, size + 12)] + [(ln, font(27), grey, 36) for ln in wrap(sub, font(27))[:2]]
        key = (big, sub)
        if key != self.lab:
            self.lab, self.lab_age = key, 0
        self.lab_age += 1
        al = int(255 * min(1.0, self.lab_age / 8))
        y = H - 90 - sum(r[3] for r in rows) if self.low else 44     # (bottom: above a phone's own controls)
        for t, fnt, col, step in rows:
            # a thin pale edge, so the text still reads where a part passes behind it
            d.text((W / 2 - d.textlength(t, font=fnt) / 2, y), t, fill=col + (al,), font=fnt, stroke_width=3, stroke_fill=(245, 245, 243, al))
            y += step
        out = np.asarray(Image.alpha_composite(im, ov).convert('RGB'))
        if self.writer:
            self.writer.append_data(out)
        if self.test and self.n % self.every == 0:
            self.sheet.append((Image.fromarray(out).resize((W // 3, H // 3)), self.n))
        self.n += 1

    def rest(self, k):
        d = self.hand[k]
        self.hand[k] = d + 0.12 * (np.array([0.42 * (1 if k == 'R' else -1), -0.55, 0.04]) - d)

    # ------------------------------------------------------------- what happens
    def view_for(self, way):
        """The builder looks along the way the part goes, from above: as if they'd turned the work toward them."""
        base = np.array([0.0, -0.70, 0.78])
        if way is None:
            return base
        w = np.array(way, float)
        v = base - 0.75 * w
        v[2] = max(v[2], 0.28)
        return v

    def bring(self, group, target, way, speed=1.0):
        """Part from the tray in an arc to the start of its checked path, then straight along it."""
        way_ = np.array(way if way is not None else (0, 0, -1.0), float)
        start = target - way_ * APPROACH
        speed = speed * self.pace
        n1, n2, n3 = max(2, int(20 / speed)), max(2, int(16 / speed)), int(8 / speed)
        self.grip['R'] = min(0.06, self.centre(group)[1] * 0.5 + 0.01)
        origin = np.array(self.off.get(group[0], PICK), float) if group[0] in self.shown else PICK - self.box[group[0]][0]
        first = not self.work and group[0] not in self.shown
        if first:                                   # a new section's first part: in view at once, never an empty frame
            origin = target + np.array([0.05, -0.04, 0.05])
        self.shown.update(group)
        for n in group:
            self.off.setdefault(n, origin)
        grp_c = lambda: self.centre(group)[0]
        # with nothing on the mat yet, look where the part will be: `target` is an offset, not a point (looking at it put a new section's first part out of frame, the neck column for 1 s)
        c_w = self.centre(self.work)[0] if self.work else grp_c() - np.array(self.off[group[0]], float) + target
        d_ = max(0.15, min(0.50, max(self.centre(group)[1] * 2.6, self.centre(self.work + group)[1] * 0.95)))
        if np.linalg.norm(origin - target) > PARKED:     # work built earlier, parked out of frame: in from the edge, not from its spot
            origin = off_edge(target, origin, self.centre(group)[1], d_)
            for n in group:
                self.off[n] = origin
        vd = self.best_view(group, target, d_, self.view_for(way), start)
        if first:
            self.cam_t, self.cam_d, self.cam_dir = np.array(self.centre(group)[0], float), d_, np.array(vd, float)
        for k in range(n1):
            t = ease(k / max(1, n1 - 1))
            p = origin + (start - origin) * t + np.array([0, 0, 0.04]) * math.sin(math.pi * t)
            for n in group:
                self.off[n] = p
            self.hand['R'] += 0.3 * (grp_c() - self.hand['R'])
            self.rest('L') if not self.work else self.hold_work()
            self.frame(c_w + (target + self.centre(group)[0] - np.array(self.off[group[0]], float) - c_w) * 0.6, d_, vd)
        for k in range(n2):
            t = ease(k / max(1, n2 - 1))
            p = start + (target - start) * t
            for n in group:
                self.off[n] = p
            self.hand['R'] += 0.3 * (grp_c() - self.hand['R'])
            self.hold_work()
            self.frame(c_w + (grp_c() - c_w) * 0.7, d_ * 0.9, vd)
        for k in range(n3):
            self.rest('R')
            self.hold_work()
            self.frame()

    def hold_work(self):
        if self.work:
            c, sz = self.centre(self.work)
            self.grip['L'] = min(0.09, sz * 0.5 + 0.015)
            self.hand['L'] += 0.14 * (c - self.hand['L'])
        else:
            self.rest('L')

    def drive(self, names, target, speed=1.0):
        """A step's screws together, each along its own axis: the same screws, driven alike, needn't be shown one at a time. The driver goes to the first."""
        by = {}
        for n in names:
            by.setdefault(tuple(np.round(self.B.screws[n][1]['fast']['d'], 2)), []).append(n)
        speed = speed * self.pace
        self.grip['R'] = 0.0
        for d_, group in by.items():
            d = np.array(d_, float)
            d /= np.linalg.norm(d)
            f0 = self.B.screws[group[0]][1]['fast']
            p = np.array(f0['p'], float) + target
            c = np.mean([np.array(self.B.screws[n][1]['fast']['p'], float) for n in group], axis=0) + target
            self.drv_ax = -d
            self.shown.update(group)
            m = max(3, int(22 / speed))
            vd = self.best_view(group, target, None, self.view_for(d), target - d * 0.022)
            for k in range(m):
                t = ease(k / max(1, m - 1))
                for n in group:
                    self.off[n] = target - d * 0.022 * (1 - t)
                self.drv += 0.4 * (p - d * 0.022 * (1 - t) - self.drv)
                self.hand['R'] += 0.45 * (self.drv + self.drv_ax * 0.09 - self.hand['R'])
                self.hold_work()
                self.frame(c - d * 0.01, max(0.13, self.cam_d * 0.97), vd, rate=0.07)
        self.drv = self.drv + self.drv_ax * 0.12

    def test_joint(self, j, words, speed=1.0, view=None, aim_z=None, dist=None):
        """Swing it by hand: to most of one limit, the other, and back."""
        J = next(x for x in self.spec['joints'] if x['name'] == j)
        lo, hi = J['lower'] * 0.7, J['upper'] * 0.7
        self.note = f'Test: {words} moves freely'
        speed = speed * self.pace
        n = max(10, int(44 / speed))
        self.grip['R'] = 0.018
        keys = [(0.0, 0.0), (0.3, hi), (0.65, lo), (1.0, 0.0)]
        child = [x for x in self.work if self.link[x] == J['child']] or self.work
        d_ = dist or max(0.24, min(0.80, self.centre(self.work)[1] * 1.4))    # portrait is narrow: far enough that the whole work stays in frame
        vd = view if view is not None else self.best_view(child, None, d_, np.array([0.25, -0.75, 0.55]))
        for k in range(n):
            t = k / max(1, n - 1)
            for (t0, a), (t1, b) in zip(keys, keys[1:]):
                if t <= t1:
                    self.q = {j: a + (b - a) * ease((t - t0) / (t1 - t0))}
                    break
            self.Ms = self.R.fk(self.q)
            pos = [self.world(x)[0] + (self.box[x][0] + self.box[x][1]) / 2 for x in child[:3]]
            self.hand['R'] += 0.25 * (np.mean(pos, axis=0) - self.hand['R'])
            self.hold_work()
            c = np.array(self.centre(self.work)[0], float)
            if aim_z is not None:
                c[2] = aim_z
            self.frame(c, d_, vd)
        self.q = {}
        self.note = ''


QUICK_TESTS = {'L_knee', 'L_hip_pitch', 'L_hip_yaw', 'neck_pitch', 'head_yaw', 'beak'}


HEAD_SECTIONS = ('headbench', 'framebench', 'jawbench', 'head')     # from here on the labels are at the bottom
TALL = 1.29 if G.RECIPE == 'feetech' else 1.0       # these views were framed for a 29.5 cm duck: at 38 cm, the hood's top left the picture
_PARTS = json.load(open(G.parts_path(), encoding='utf-8'))


def part_kind(n):
    """What the builder picks up: the servo's model, the bought part, or what it's printed in."""
    if n in _PARTS['parts']:
        return 'printed, ' + _PARTS['parts'][n].get('material', 'pla').upper()
    if 'servo' in n and 'bus' not in n:
        m = re.search(r'\(buy\) (\S+)', ' '.join(_PARTS['groups'].get(n, [])))
        return m.group(1).replace('_', '-') if m else None
    if G.RECIPE == 'hybrid':
        dx = n.startswith(('neck', 'head', 'beak'))
        for k, v in (('ankle bearing', '6700ZZ'), ('bearing', ('6703ZZ' if G.RECIPE == 'dynamixel' else 'MR2216ZZ') if dx else '6704ZZ'), ('rear horn', 'FPX330-H101 idler' if dx else 'from the servo box'), ('horn', 'HNX330-N102' if dx else 'from the servo box'), ('battery', '2S LiPo, 850 mAh'),
                     ('L 5 V regulator', '8 A UBEC'), ('regulator', '5 V USB converter'), ('IMU', 'BNO085 board'), ('camera', 'Arducam IMX219'), ('standoff', 'hex, M2.5'),
                     ('leg bus board', 'Bus Servo Adapter (A), on UART'), ('Waveshare', 'Bus Servo Adapter (A), on USB'), ('splice', 'lead joined in heat-shrink'), ('Pi Zero', 'bought')):
            if k in n:
                return v
        return None
    fe_ = G.RECIPE == 'feetech'
    for k, v in (('ankle bearing', '6700ZZ'), ('bearing', '6704ZZ' if fe_ else '6703ZZ' if G.RECIPE == 'dynamixel' else 'MR2216ZZ'), ('rear horn', 'on the servo' if G.RECIPE == 'hd1910' else 'from the servo box' if fe_ else 'FPX330-H101 idler'), ('horn', 'on the servo' if G.RECIPE == 'hd1910' else 'from the servo box'), ('battery', '2S LiPo, 850 mAh' if fe_ else '2S LiPo, 450 mAh'),
                 ('regulator', '5 V USB converter'), ('IMU', 'BNO085 board'), ('camera', 'Arducam IMX219'), ('foot switch', 'Omron D2F-01F'), ('standoff', 'hex, M2.5'),
                 ('Waveshare', 'Bus Servo Adapter (A)'), ('Pi Zero', 'bought')):
        if k in n:
            return v
    return None


def part_label(n):
    nm = re.sub(r'^L ', 'Left ', re.sub(r'^R ', 'Right ', n))
    nm = nm[0].upper() + nm[1:]
    k = part_kind(n)
    return f'{nm}: {k}' if k else nm


def run(test=False, name='first_person_build', pace=1.0, quick=False, dry=False):
    f = Film(test, every=70, name=name, pace=pace, dry=dry)
    B = f.B
    secs = A.sections_for(B)
    done, park = {}, {}
    spots = [np.array(p) * 5 for p in ((-0.30, 0.18, 0), (0.30, 0.18, 0), (-0.32, -0.02, 0), (0.32, 0.0, 0), (-0.30, -0.2, 0), (0.0, 0.30, 0))]   # out of frame, not under the labels
    f.work = []

    def all_items(sec):
        out = [p for o in sec['on'] for p in done[o]]
        for cap, its, pats in sec['steps']:
            for it, _ in its:
                out += done[it[1:]] if it.startswith('@') else [it]
            for pt in pats:
                out += [n for n in B.screws if re.search(pt, n)]
        return out

    TITLE = {'thigh_L': 'Left thigh', 'thigh_R': 'Right thigh (mirror)', 'shin_L': 'Left shin', 'shin_R': 'Right shin (mirror)', 'shoe_L': 'Left shoe', 'shoe_R': 'Right shoe (mirror)', 'leg_L': 'Join the left leg', 'leg_R': 'Join the right leg',
             'body': 'Chassis and hips', 'legs': 'Legs onto the hips', 'column': 'Neck column', 'torso': 'Neck and body shell', 'headbench': 'Head yaw and roll', 'framebench': 'Head frame and beak servo', 'jawbench': 'The jaw', 'head': 'The head'}
    for sec in secs:
        key = sec['key']
        f.title = TITLE.get(key, sec['title'])
        f.detail = ''
        f.marks[key] = [f.n, None]
        f.low = f.low or key in HEAD_SECTIONS
        speed = 3.0 if key.endswith('_R') else 1.0
        items = all_items(sec)
        lo, hi = RA.bounds(f.sc, items)
        home = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
        if quick and key.endswith('_R'):             # the overview doesn't build the mirror side: it's simply there, set aside
            spot = spots[len(park) % len(spots)]
            f.shown.update(items)
            for n in items:
                f.off[n] = home + spot
            done[key] = list(items)
            park[key] = spot
            continue
        # what this section builds on comes to the middle of the mat
        carried = [p for o in sec['on'] for p in done[o]]
        if carried:
            f.step = 'Bring the work to the mat'
            start = np.array(f.off.get(carried[0], home), float)
            for n in carried:
                f.off[n] = home
            c_end, size = f.centre(carried)
            if np.linalg.norm(start - home) > PARKED:     # from the edge of the picture, with the camera already on the mat: not from its spot, the camera chasing it there
                start = off_edge(home, start, size, 0.45)
            nk = max(3, int(16 / (speed * f.pace)))
            for k in range(nk):
                t = 1 - (1 - k / max(1, nk - 1)) ** 2       # (in at speed, slowing onto the mat)
                for n in carried:
                    f.off[n] = start + (home - start) * t
                f.hand['R'] += 0.2 * (f.centre(carried)[0] - f.hand['R'])
                f.frame(c_end, 0.45, f.view_for(None))
            for o in sec['on']:
                park.pop(o, None)
        f.work = list(carried)
        total = len(sec['steps'])
        for i, (cap, its, pats) in enumerate(sec['steps'], 1):
            f.step = f'{i}/{total}  {cap}'
            f.detail = ''
            for it, way in its:
                group = done[it[1:]] if it.startswith('@') else [it]
                if it.startswith('@'):
                    park.pop(it[1:], None)
                f.detail = TITLE.get(it[1:], it[1:]) + ': built earlier' if it.startswith('@') else part_label(it)
                f.bring(group, home, way, speed)
                f.work += group
            names = [n for pt in pats for n in B.screws if re.search(pt, n)]
            if names:
                sz = {}
                for n_ in names:
                    fx = B.screws[n_][1]['fast']
                    sz[(fx['size'], round(fx['L'] * 1000))] = sz.get((fx['size'], round(fx['L'] * 1000)), 0) + 1
                f.detail = ', '.join(f"{c} x {a} x {b} mm" for (a, b), c in sorted(sz.items())) + (': Phillips' if all(a.startswith('M2') for a, _ in sz) else '')
                f.drive(names, home, speed * (1.6 if len(names) > 6 else 1.0))
                f.work += names
            for k in range(int(6 / (speed * f.pace))):
                f.rest('R')
                f.hold_work()
                f.frame()
            f.detail = ''
            for (kk, w), joints in TESTS.items():
                if kk == key and w in cap:
                    for j, words in joints:
                        if not quick or j in QUICK_TESTS:
                            f.test_joint(j, words, speed)
            for tup, joints in ORDERED.items():
                if key == 'legs' and key in tup and '4 screws' in cap:
                    jn = joints[0] if i <= 2 else joints[1]
                    if not quick:
                        f.test_joint(jn[0], jn[1], speed)
                if key == 'head' and key in tup and '4 screws on each horn' in cap and not quick:
                    jn = joints[0] if i <= 2 else joints[1]
                    f.test_joint(jn[0], jn[1], speed)
        done[key] = list(items)
        f.marks[key][1] = f.n
        # park it, unless the very next section carries it on
        nxt = next((s for s in secs[secs.index(sec) + 1:]), None)
        if nxt and key not in nxt['on'] and secs.index(sec) < len(secs) - 1:
            spot = spots[len(park) % len(spots)]
            f.step = 'Set it aside'
            f.detail = ''
            start = np.array(f.off.get(items[0], home), float)
            out = off_edge(home, home + spot, f.centre(items)[1], 0.5)     # to the edge of the picture, then to its spot unseen: the rest of the way was frames of an empty mat
            nk = max(3, int(14 / (speed * f.pace)))
            for k in range(nk):
                t = (k / max(1, nk - 1)) ** 2               # (off at speed: gone as the move ends)
                for n in items:
                    f.off[n] = start + (out - start) * t
                f.hand['R'] += 0.25 * (f.centre(items)[0] - f.hand['R'])
                f.rest('L')
                f.frame(np.array([0, 0, 0.04]), 0.5, f.view_for(None))
            for n in items:
                f.off[n] = home + spot
            park[key] = spot
        f.work = []
    # the finished duck: every joint, one by one, then a look at it
    f.detail = ''
    f.marks['finished'] = [f.n, None]
    f.title = 'Finished: try the joints' if quick else 'Finished: try every joint'
    f.work = list(f.sc.names)
    astray = [n for n in f.sc.names if n not in f.shown]
    if astray:
        print('not shown:', astray[:10])
    for j in (['L_hip_pitch', 'L_knee', 'head_yaw', 'beak'] if quick else FINAL):
        f.step = j.replace('_', ' ')
        leg = j[:2] in ('L_', 'R_')
        f.test_joint(j, re.sub(r'^R ', 'right ', re.sub(r'^L ', 'left ', j.replace('_', ' '))), 2.2, LEG_VIEW if leg else HEAD_VIEW, (0.12 if leg else 0.21) * TALL, (0.46 if leg else 0.50) * TALL)
    f.title, f.step = 'The hybrid duck' if G.RECIPE == 'hybrid' else 'The all-Feetech duck' if G.RECIPE == 'feetech' else 'The Feetech HD-1910 duck' if G.RECIPE == 'hd1910' else 'The all-Dynamixel duck', ''
    nf = 75 if quick else 120
    for k in range(nf):
        a = math.radians(-90 + 90 * ease(min(1.0, k / (nf * 0.6))))     # round to the front, then hold on the face
        f.rest('R')
        f.rest('L')
        f.frame(np.array([0, 0, 0.15 * TALL]), 0.46 * TALL, np.array([math.cos(a) * 0.8, math.sin(a) * 0.8, 0.25]), rate=0.05)
    print(f'{name}: pace {pace:.2f}, {f.n} frames, {f.n / FPS:.0f} s')
    f.marks['finished'][1] = f.n
    globals()['LAST_MARKS'] = f.marks
    if name == 'build_5min':
        with open(G.out_path('reports', 'build_sections.json'), 'w', encoding='utf-8') as fo:
            json.dump(dict(fps=FPS, frames=f.n, sections=f.marks), fo, indent=0)
    if not dry:
        print('least visible at 180 x 320 (px, section, part):', sorted(f.seen)[:16])
    if dry:
        return f.n
    if f.writer:
        f.writer.close()
        print(f.path, os.path.getsize(f.path) // 1024, 'kB')
    if test and not dry:
        cols = 5
        rows = (len(f.sheet) + cols - 1) // cols
        im = Image.new('RGB', (cols * W // 3, rows * H // 3), 'white')
        for i, (x, _) in enumerate(f.sheet):
            im.paste(x, ((i % cols) * W // 3, (i // cols) * H // 3))
        p = G.out_path('reports', 'pov_frames.png')
        im.save(p)
        print(p)


def fit(seconds, quick):
    """The pace that lands on this length: dry runs, as the beats have floors and it isn't quite proportional."""
    lo, hi = 0.2, 12.0
    for _ in range(9):
        mid = (lo * hi) ** 0.5
        n = run(False, 'dry', mid, quick, True)
        if n / FPS > seconds:
            lo = mid
        else:
            hi = mid
    return hi


if __name__ == '__main__':
    if '--test' in sys.argv:
        run(True, pace=fit(120, False), dry=False)
    else:
        run(False, 'build_5min', fit(300, False), False, False)
