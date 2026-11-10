"""Cyber Duck: the duck lies face down and is a wedge truck (Cybertruck-like). The head pitches back so its crown is
the windshield and its face the underside of the nose; the body is the cab, under the roof's peak; the legs, stretched
back, are the bed (each leg a half, split down the middle so it still walks); the feet fold flat into the tall tail.

The shells are made in the truck pose, from one side profile (nose -> peak -> tail, two straight lines fitted just
above the parts beneath) and one front section (slab sides, the top edges leaning in), then cut into a piece per
link and carried back into that link's frame: the same pieces cover the duck standing.

  MILLI_RECIPE=dynamixel python tools/cyber_duck.py   -> reports/<recipe>/cyber_duck/ (renders, report.txt, STLs)

Shape only (the user, 2026-11-10: electronics and the inner skeleton are tuned later): the head's own insides are
reported where they meet a shell, not designed round. Round 1's sitting tuck (legs forward, head folded over them)
made a 1.8 : 1 box; lying down uses the duck's height as the truck's length. Kinematic, no physics. Collisions as
checks.py (rigid parts, shell walls). Plain python.
"""
import itertools
import math
import os
import sys

import manifold3d as mf
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

OUT = G.out_path('reports', 'cyber_duck')
STEEL, GLASS, BLACK, LIGHT, RED, TYRE, HUB = '#B9BDC1', '#1E2227', '#2A2C30', '#F4F6FF', '#C8102E', '#121315', '#5E6368'
WALL, GAP, SEAM = 0.002, 0.003, 0.002
NEST, OVER = WALL + 0.0008, 0.008           # a nested panel's depth under its neighbour's skin; how far it reaches under       # shell wall; clearance over the parts beneath; gap between pieces
PEAK = 0.42                                 # the roof's peak, as a fraction of the length from the nose (Cybertruck ~0.4)


def Ry(t):
    c, s = math.cos(t), math.sin(t)
    M = np.eye(4)
    M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return M


# truck mode: lying face down (the body's up is forward), legs back and tipped 0.25 to the floor (the bed's slope),
# feet folded flat under the tail, head pitched back (its crown about 20 deg: the windshield)
TRUCK_BASE = Ry(math.pi / 2)
TRUCK = {f'{s}_{k}': v for s in 'LR' for k, v in (('hip_pitch', -0.4), ('knee', 0.0), ('ankle_pitch', 1.53))}   # limit 1.535
TRUCK.update(neck_pitch=0.0, head_pitch=-1.2, L_flap=1.0, R_flap=1.0)   # flaps: 1 spread (truck), 0 tucked
# the tail flaps (the user, 2026-11-10: "fold the panels like a duck would tuck in wings"): a panel per side, hinged
# on the shin across the top of the leg at the ankle; spread it closes the tail, tucked it folds over the back of the
# shin, clear of the floor and the feet. A small servo each (an XL330 on the same bus, or a micro servo).
PHI_TUCK = math.pi
FLAP_LIFT = 0.007                           # the flaps' ground clearance in the truck: their corner dips 6 mm as they swing
WHEELS = (0.17, 0.74)                       # the rear wheel clear of the ankle's arc (at 0.80 the arc sliced it)                       # the wheel centres, as fractions of the length from the nose
JAW_POSES = [{'jaw': 0.4}, {'jaw': 0.4, 'neck_pitch': 0.3, 'head_yaw': 1.0}, {'jaw': 0.4, 'head_roll': 0.24}]


def ease(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


# getting up (the user, 2026-11-10: "when getting up we will need the head and beak support until it can come up
# standing"): from the truck the flaps tuck and the feet unfold, the head tips down until the bill rests on the floor
# ahead, the chest rises on that prop while the legs fold the feet in under the body, then the head lifts and it stands.
# Keyframes over u (0 truck .. 1 standing): (u, the body's lean from upright, joints)
_LEGS = lambda hip, knee, ankle: {f'{s_}_{k}': v for s_ in 'LR' for k, v in (('hip_pitch', hip), ('knee', knee), ('ankle_pitch', ankle))}
GET_UP = [                  # from u 0.36: solved by tools/cyber_duck_getup_solve.py (statics, slow moves; margins in its log)
    (0.00, math.pi / 2, dict(_LEGS(-0.4, 0.0, 1.53), neck_pitch=0.0, head_pitch=-1.2, L_flap=1.0, R_flap=1.0)),
    (0.14, math.pi / 2, dict(_LEGS(-0.4, 0.0, 1.53), neck_pitch=0.0, head_pitch=-1.2, L_flap=0.0, R_flap=0.0)),
    (0.289, 1.500, dict(_LEGS(-0.767, -0.002, 0.001), neck_pitch=0.053, head_pitch=-0.714)),      # chest lifting on bill and legs
    (0.323, 1.420, dict(_LEGS(-1.083, 0.001, 0.001), neck_pitch=0.043, head_pitch=-0.127)),
    (0.36, 1.330, dict(_LEGS(-1.49, 0.0, 0.0), neck_pitch=0.35, head_pitch=0.12)),                # up on the bill and feet
    (0.41, 1.220, dict(_LEGS(-1.22, 0.78, -0.05), neck_pitch=0.43, head_pitch=0.20)),
    (0.47, 1.100, dict(_LEGS(-1.553, 1.570, -0.123), neck_pitch=0.653, head_pitch=0.273)),
    (0.54, 0.950, dict(_LEGS(-1.50, 1.25, -0.15), neck_pitch=1.16, head_pitch=0.14)),
    (0.575, 0.875, dict(_LEGS(-1.296, 1.089, -0.167), neck_pitch=1.321, head_pitch=0.076)),
    (0.61, 0.800, dict(_LEGS(-1.555, 0.927, -0.171), neck_pitch=1.549, head_pitch=0.001)),
    (0.653, 0.720, dict(_LEGS(-1.570, 0.890, -0.053), neck_pitch=1.600, head_pitch=0.315)),       # the bill lifting
    (0.69, 0.650, dict(_LEGS(-1.57, 0.86, 0.05), neck_pitch=1.60, head_pitch=0.59)),
    (0.723, 0.580, dict(_LEGS(-1.569, 0.835, 0.142), neck_pitch=1.600, head_pitch=0.865)),
    (0.76, 0.500, dict(_LEGS(-1.55, 0.81, 0.23), neck_pitch=0.33, head_pitch=1.18)),
    (0.83, 0.360, dict(_LEGS(-1.57, 0.94, 0.41), neck_pitch=-0.38, head_pitch=0.60)),
    (0.89, 0.220, dict(_LEGS(-1.566, 1.076, 0.605), neck_pitch=-1.050, head_pitch=0.026)),         # head back: a counterweight
    (0.95, 0.110, dict(_LEGS(-0.95, 0.56, 0.43), neck_pitch=-0.59, head_pitch=-0.05)),
    (1.00, 0.000, dict(_LEGS(-0.299, 0.052, 0.254), neck_pitch=-0.107, head_pitch=-0.122)),        # standing
]


def transform_pose(t):
    """t 0 (robot) .. 1 (truck): the body's lean and the joints, eased between GET_UP's keyframes (run backwards it
    is lying down: the same moves, the bill still the prop)."""
    u = 1.0 - min(max(t, 0.0), 1.0)
    for (u0, l0, q0), (u1, l1, q1) in zip(GET_UP, GET_UP[1:]):
        if u0 <= u <= u1:
            w = ease((u - u0) / (u1 - u0))
            q = {k: (1 - w) * q0.get(k, 0.0) + w * q1.get(k, 0.0) for k in set(q0) | set(q1)}
            return Ry((1 - w) * l0 + w * l1), q
    return Ry(0.0), {}


def contacts(md, q, base, tol=0.003):
    """The links within tol of the floor, the robot lowered onto whatever is lowest."""
    M = md.fk(q, base)
    return sorted(ln for ln in md.links if md.keep.get(ln) is not None and C.bbox(C.posed(md.keep[ln], M[ln]))[0][2] < tol)


def attach_flaps(md, hinges, jaw=None):
    md.R = FlapRobot(md.R, hinges, jaw)
    md.links += ['L_flap', 'R_flap'] + (['jaw'] if jaw is not None else [])
    for n_ in ('L_flap', 'R_flap', 'jaw'):
        md.keep[n_] = None
        md.frame[n_] = None


class FlapRobot:
    """The spec's kinematics plus the two tail flaps, as links 'L_flap' / 'R_flap' on the shins."""

    def __init__(self, R, hinges, jaw=None):
        self.R, self.hinges = R, hinges                  # side -> (point, axis) in the shin's zero-pose frame
        self.jaw = jaw                                   # (point, axis) in the head's zero-pose frame: the front hub

    def __getattr__(self, n):
        return getattr(self.R, n)

    def H(self, side, phi):
        return self.Hpa(*self.hinges[side], phi)

    @staticmethod
    def Hpa(p, a, phi):
        a = a / np.linalg.norm(a)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        Rm = np.eye(3) + math.sin(phi) * K + (1 - math.cos(phi)) * K @ K
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = p - Rm @ p
        return T

    def fk(self, q=None, base=None):
        q = q or {}
        M = self.R.fk(q, base=base)
        for sd in self.hinges:
            M[f'{sd}_flap'] = M[f'{sd}_knee_link'] @ self.H(sd, PHI_TUCK * (1 - q.get(f'{sd}_flap', 0.0)))
        if self.jaw is not None:                         # 'jaw' rad open, 0 shut (the truck)
            M['jaw'] = M['head'] @ self.Hpa(*self.jaw, q.get('jaw', 0.0))
        return M

    def adjacent(self, a, b):
        for sd in self.hinges:
            if {a, b} == {f'{sd}_flap', f'{sd}_knee_link'}:
                return True
        if {a, b} == {'jaw', 'head'}:
            return True
        return self.R.adjacent(a, b)
# which link carries which piece of the truck, nose to tail
PIECES = [('head', 'head', None), ('trunk', 'trunk', None),
          ('L thigh', 'L_hip_pitch_link', +1), ('R thigh', 'R_hip_pitch_link', -1),
          ('L shin', 'L_knee_link', +1), ('R shin', 'R_knee_link', -1)]       # the shins run on over the feet to the tail


def hidden(ln, g):
    """Today's covers that the cyber shells replace, and the beak (the truck's nose has none)."""
    if ln == 'beak_link':
        return True
    if g['role'] in ('shell', 'decal') and ln in ('head', 'trunk', 'L_hip_pitch_link', 'R_hip_pitch_link'):
        return True
    return ln == 'head' and any(w in g['name'] for w in ('hood', 'bill', 'lower shell', 'mouth'))


ELECTRONIC = ('switch', 'pi zero', 'board', 'battery', 'lipo', 'camera', 'buck', 'imu', 'usb', 'cable', 'lead', 'wire',
              'sd card', 'header', 'connector', 'regulator')
NOSE, TAIL = 0.55, 0.72
CHIN, CHIN_Z, CORNER = 0.010, 0.45, 0.013     # the chin's set-back (m) and where it starts (of the nose's height); the front corner chamfer (m)                     # the Cybertruck's nose and tail heights, as fractions of the peak's


def inside(ln, g):
    """Parts the shape is not fitted round: the head's insides and the electronics, laid out later."""
    return ln in ('head', 'beak_link') or any(w in g['name'].lower() for w in ELECTRONIC)


def extrude_profile(pts, y0, y1):
    """A side profile [(x, z)] extruded across y0..y1."""
    m = mf.Manifold.extrude(mf.CrossSection([pts]), y1 - y0).rotate([90, 0, 0])      # (x, -e, z2d)
    return m.translate([0, y1, 0])


def extrude_front(pts, x0, x1):
    """A front section [(y, z)] extruded along x0..x1."""
    m = mf.Manifold.extrude(mf.CrossSection([pts]), x1 - x0).rotate([90, 0, 90])
    lo, _ = C.bbox(m)
    return m.translate([x0 - lo[0], 0, 0])


def halfspace(n, d, size=2.0):
    """{p : n.p <= d} as a big box."""
    n = np.asarray(n, float) / np.linalg.norm(n)
    cube = mf.Manifold.cube([size, size, size], center=True).translate([0, 0, -size / 2])
    z = np.array([0, 0, 1.0])
    v, c = np.cross(z, n), float(z @ n)
    sn = np.linalg.norm(v)
    if sn < 1e-9:
        R = np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    else:
        k = v / sn
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + sn * K + (1 - c) * K @ K
    return cube.transform(np.hstack([R, (n * d)[:, None]]))


def box(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return mf.Manifold.cube((hi - lo).tolist()).translate(lo.tolist())


def fit_line(xs, zs, x_a, x_b, z_b, gap):
    """The lowest straight line from (x_b, z_b) out to x_a that stays gap above every (x, z) between them."""
    best = -1e9
    for x, z in zip(xs, zs):
        if min(x_a, x_b) <= x <= max(x_a, x_b) and abs(x - x_b) > 1e-6:
            best = max(best, z_b + (z + gap - z_b) * (x_a - x_b) / (x - x_b))
    return best


def bake(m):
    """A plain mesh again: manifold's booleans are lazy, and a shell carrying hundreds of cuts replays them all each
    time it is moved and cut again (2026-11-10: the feet's carving hit the 6 GB cap that way)."""
    return mf.Manifold(m.to_mesh())


def mesh_parts(m):
    mesh = m.to_mesh()
    return np.asarray(mesh.vert_properties[:, :3], float), np.asarray(mesh.tri_verts, int)


class Model:
    def __init__(self):
        self.spec = G.load()
        self.S, self.R = C.Solids(self.spec), G.Robot(self.spec)
        self.links = list(self.spec['links'])
        self.keep = {}                                    # link -> the rigid parts that stay
        for ln, l in self.spec['links'].items():
            ms = [self.S.geom[(ln, i)] for i, g in enumerate(l['geoms'])
                  if self.S.geom[(ln, i)] is not None and g['role'] in G.RIGID and not hidden(ln, g)]
            self.keep[ln] = C.union(ms)
        self.frame = {}                                   # what the shape must clear: kept, not inside
        for ln, l in self.spec['links'].items():
            ms = [self.S.geom[(ln, i)] for i, g in enumerate(l['geoms'])
                  if self.S.geom[(ln, i)] is not None and g['role'] in G.RIGID and not hidden(ln, g) and not inside(ln, g)]
            self.frame[ln] = C.union(ms)

    def fk(self, q, base=None):
        """Poses with the lowest kept part on the floor."""
        base = np.eye(4) if base is None else base
        M = self.R.fk(q, base=base)
        low = min(C.bbox(C.posed(self.keep[ln], M[ln]))[0][2] for ln in self.links if self.keep[ln] is not None)
        lift = np.eye(4)
        lift[2, 3] = -low
        return self.R.fk(q, base=lift @ base)


def build(md, rep):
    M = md.fk(TRUCK, TRUCK_BASE)
    posed = {ln: C.posed(md.keep[ln], M[ln]) for ln in md.links if md.keep[ln] is not None}
    lo = np.min([C.bbox(m)[0] for m in posed.values()], axis=0)
    hi = np.max([C.bbox(m)[1] for m in posed.values()], axis=0)
    pts = np.vstack([mesh_parts(C.posed(md.frame[ln], M[ln]))[0] for ln in md.links if md.frame[ln] is not None])
    xs = np.linspace(lo[0], hi[0], 200)
    dx = xs[1] - xs[0]
    env = np.array([pts[np.abs(pts[:, 0] - x) < dx][:, 2].max(initial=0.0) for x in xs])

    x_nose, x_tail = hi[0] + GAP + WALL, lo[0] - GAP - WALL
    L = x_nose - x_tail
    # the peak's place and height: closest to the Cybertruck's profile, every line clear of the parts beneath
    g, z0, best = GAP + WALL, env.max() + GAP + WALL, None
    for f in np.linspace(0.32, 0.55, 24):
        for zp in z0 + np.linspace(0.0, 0.03, 31):
            xp = x_nose - f * L
            zn = max(fit_line(xs, env, x_nose, xp, zp, g), 0.40 * zp)
            zt = max(fit_line(xs, env, x_tail, xp, zp, g), TAIL * zp)
            err = abs(zn / zp - NOSE) + abs(zt / zp - TAIL) + 0.5 * abs(f - PEAK) + 1.5 * (zp - z0) / z0
            if best is None or err < best[0]:
                best = (err, xp, zp, zn, zt)
    _, x_peak, z_peak, z_nose, z_tail = best
    floor = 0.0
    half_w = max(abs(lo[1]), abs(hi[1])) + GAP + WALL
    belt = 0.55                                         # the beltline, where the sides start to lean in
    lean = 0.20 * half_w

    def solid(d):
        """The truck's outer solid, shrunk by d (d = WALL: the inside of the walls)."""
        # the nose: its face stands from the light bar down to CHIN_Z, then the chin slopes back CHIN to the floor
        prof = [(x_tail + d, floor + d), (x_nose - CHIN - d, floor + d), (x_nose - d, CHIN_Z * z_nose),
                (x_nose - d, z_nose - d), (x_peak, z_peak - d), (x_tail + d, z_tail - d)]
        hw = half_w - d
        front = [(-hw, floor + d), (hw, floor + d), (hw, belt * z_peak), (hw - lean, z_peak + 0.02),
                 (-hw + lean, z_peak + 0.02), (-hw, belt * z_peak)]
        # plan view: the front corners cut off at 45 deg (the triangular facets either side of the nose)
        c = CORNER
        plan = [(x_tail - 0.01, -hw), (x_nose - c - d, -hw), (x_nose - d, -hw + c), (x_nose - d, hw - c),
                (x_nose - c - d, hw), (x_tail - 0.01, hw)]
        planm = mf.Manifold.extrude(mf.CrossSection([plan]), 1.0).translate([0, 0, -0.5])
        return extrude_profile(prof, -hw, hw) ^ extrude_front(front, x_tail - 0.01, x_nose + 0.01) ^ planm

    outer, inner = solid(0.0), solid(WALL)

    # piece boundaries along x: halfway between neighbouring links' parts (nose to tail)
    order = ['head', 'trunk', 'thigh', 'shin']
    span = {'head': C.bbox(posed['head']), 'trunk': C.bbox(posed['trunk']), 'thigh': C.bbox(posed['L_hip_pitch_link']),
            'shin': C.bbox(posed['L_knee_link'])}
    cuts = {}
    for a, b in zip(order, order[1:]):
        cuts[(a, b)] = 0.5 * (span[a][0][0] + span[b][1][0])           # a is ahead (+x) of b
    big = 1.0
    xr = {'head': (cuts[('head', 'trunk')], big), 'trunk': (cuts[('trunk', 'thigh')], cuts[('head', 'trunk')]),
          'thigh': (cuts[('thigh', 'shin')], cuts[('trunk', 'thigh')]), 'shin': (-big, cuts[('thigh', 'shin')])}
    pieces = {}
    for name, ln, side in PIECES:
        part = name.split()[-1]
        x0, x1 = xr[part]
        y0, y1 = (-big, big) if side is None else ((SEAM / 2, big) if side > 0 else (-big, -SEAM / 2))
        reg = box([x0 + SEAM / 2, y0, -big], [x1 - SEAM / 2, y1, big])
        # open toward the neighbouring pieces (the joints are there), closed everywhere else, its own wall on the seam
        iy0, iy1 = (y0, y1) if side is None else ((y0 + WALL, y1) if side > 0 else (y0, y1 - WALL))
        ix0 = x0 - 0.01 if part != 'shin' else x_tail + WALL
        ix1 = x1 + 0.01 if part != 'head' else x1 - SEAM / 2 - WALL
        hollow = box([ix0, iy0, -big], [ix1, iy1, big])
        pieces[name] = (ln, (outer ^ reg) - (inner ^ hollow))

    # the knee seam: an arc round the knee axis where the leg closes as it bends (the back of the knee, the bed's top),
    # so the shin turns in place against the thigh instead of sweeping through it (round 7: the thigh shell's sweep
    # took a quarter of each shin's wall)
    top_at = lambda x: (z_tail + (z_peak - z_tail) * (x - x_tail) / (x_peak - x_tail)) if x < x_peak else         (z_nose + (z_peak - z_nose) * (x_nose - x) / (x_nose - x_peak))
    knee_r, knee_c = {}, {}
    for side, sgn in (('L', +1), ('R', -1)):
        j = next(j for j in md.spec['joints'] if j['name'] == f'{side}_knee')
        k = (M[f'{side}_hip_pitch_link'] @ np.array(j['origin'] + [1.0]))[:3]
        r = top_at(k[0]) - k[2] + 0.004
        knee_r[side] = r
        knee_c[side] = k
        disc = lambda r_: mf.Manifold.cylinder(2 * big, r_, r_, 64, True).rotate([90, 0, 0]).translate([k[0], 0, k[2]])
        ys = box([-big, SEAM / 2, -big], [big, big, big]) if sgn > 0 else box([-big, -big, -big], [big, -SEAM / 2, big])
        lt, th = pieces[f'{side} thigh']
        ls, sn = pieces[f'{side} shin']
        xc_ = cuts[('thigh', 'shin')]                    # bridge the seam's gap inside the disc: one piece, not an island
        bridge = (outer - inner) ^ disc(r) ^ ys ^ box([xc_ - SEAM, -big, -big], [xc_ + SEAM, big, big])
        pieces[f'{side} shin'] = (ls, sn + (th ^ disc(r) ^ ys) + bridge)
        pieces[f'{side} thigh'] = (lt, th - disc(r + SEAM))
    # nested thighs (the user, 2026-11-10: "the panels can be spaced a little out and in ... one tucks into the other"):
    # each thigh panel sits NEST inside the body's and shin's skins and runs OVER under the body panel, so the hip
    # swings it underneath instead of carving it away
    for side, sgn in (('L', +1), ('R', -1)):
        x0, x1 = xr['thigh']
        ys0, ys1 = (SEAM / 2, big) if sgn > 0 else (-big, -SEAM / 2)
        iy0, iy1 = (ys0 + WALL, ys1) if sgn > 0 else (ys0, ys1 - WALL)
        reg = box([x0 + SEAM / 2, ys0, -big], [x1 + OVER, ys1, big])
        hol = box([x0 - 0.01, iy0, -big], [x1 + OVER + 0.01, iy1, big])
        lt, _ = pieces[f'{side} thigh']
        k_ = knee_c[side]
        disc_ = mf.Manifold.cylinder(2 * big, knee_r[side] + SEAM, knee_r[side] + SEAM, 64, True).rotate([90, 0, 0]).translate(
            [k_[0], 0, k_[2]])
        pieces[f'{side} thigh'] = (lt, ((solid(NEST) ^ reg) - (solid(NEST + WALL) ^ hol)) - disc_)
    rep.append(f'thighs nested {1000 * NEST:.1f} mm inside the skin, {1000 * OVER:.0f} mm under the body panel')
    rep.append('knee seam arcs (radius, cm): ' + ', '.join(f'{s_} {100 * r_:.1f}' for s_, r_ in knee_r.items()))

    # the legs' panels keep only the bed's top and the flank (the user, 2026-11-10: "remove those panels so the legs
    # can make the duck walk"): no truck-floor wall (the front of the legs standing) and no centre-seam wall (between
    # the legs); the truck looks the same from outside
    for n_ in ('L thigh', 'R thigh', 'L shin', 'R shin'):
        l_, p_ = pieces[n_]
        p_ = p_ - box([-big, -big, -big], [big, big, NEST + WALL + 1e-4]) -             box([-big, -(SEAM / 2 + WALL + 1e-4), -big], [big, SEAM / 2 + WALL + 1e-4, big])
        pieces[n_] = (l_, p_)
    rep.append('legs open at the front and between them (no truck-floor or centre-seam walls on thighs and shins)')
    # (an ankle arc round the axis, tried 2026-11-10, cut away the truck's rear lower flank: dropped; the rear wheel
    # moved to 0.74 L instead, clear of the foot's carving and the flap seam)

    # the shins stop at the ankle (standing, the duck is on its feet, not its shells); each foot gets a shoe that is
    # the tail's corner in the truck
    za = next(j for j in md.spec['joints'] if j['name'] == 'L_ankle_pitch')['origin'][2]
    hinges = {}
    for side, sgn in (('L', +1), ('R', -1)):
        Ms = M[f'{side}_knee_link']
        n = Ms[:3, :3] @ np.array([0, 0, 1.0])
        d = n @ (Ms @ np.array([0, 0, za, 1.0]))[:3]
        ln, sh = pieces[f'{side} shin']
        pieces[f'{side} shin'] = (ln, sh ^ halfspace(-n, -(d + SEAM / 2)))
        # the flap: the tail behind the ankle, open at the front (the ankle) and underneath (the foot swings out)
        ys = (SEAM / 2, big) if sgn > 0 else (-big, -SEAM / 2)
        iys = (ys[0] + WALL, ys[1]) if sgn > 0 else (ys[0], ys[1] - WALL)
        reg = halfspace(n, d - SEAM / 2) ^ box([-big, ys[0], -big], [big, ys[1], big])
        hollow = (inner + box([-big, -big, -big], [big, big, WALL + 1e-4])) ^ box([-big, iys[0], -big], [big, iys[1], big]) ^             halfspace(n, d + 0.01)
        pieces[f'{side} flap'] = (f'{side}_flap', ((outer ^ reg) - hollow) - box([-big, -big, -big], [big, big, FLAP_LIFT]))
        # the hinge: across the top of the leg at the ankle plane, 1 mm above the outer surface, so the flap swings up
        # and over the leg without touching it (inside the wall, both walls sliced each other into slivers)
        top = lambda x: z_tail + (z_peak - z_tail) * (x - x_tail) / (x_peak - x_tail)
        x_h = x_tail
        for _ in range(4):
            z_h = top(x_h) + 0.001
            x_h = (d - n[2] * z_h) / n[0]
        Minv = np.linalg.inv(Ms)
        hinges[side] = ((Minv @ np.array([x_h, 0.0, z_h, 1.0]))[:3], Minv[:3, :3] @ np.array([0, 1.0, 0]))
        M[f'{side}_flap'] = Ms                            # spread, the flap is at its shin's frame
    # the jaw (the user, 2026-11-10: "this can be the mouth jaw with the wheel part being the hinge"): the lower nose,
    # below a line from the front wheel's hub to the nose, with the whole front wheel, pivoting on the hub; shut in
    # the truck, it opens as the duck's bill (the beak servo's job, 0..0.4 rad)
    xw, rw = x_nose - WHEELS[0] * L, 0.24 * z_peak
    zw = rw + 0.002
    disc = lambda r_: mf.Manifold.cylinder(2 * big, r_, r_, 64, True).rotate([90, 0, 0]).translate([xw, 0, zw])
    jaw_reg = box([xw, -big, -big], [big, big, zw]) + disc(rw + 0.003)
    jaw_cut = box([xw - SEAM, -big, -big], [big, big, zw + SEAM]) + disc(rw + 0.003 + SEAM)
    ln_h, sh_h = pieces['head']
    pieces['head'] = (ln_h, sh_h - jaw_cut)
    pieces['jaw'] = ('jaw', sh_h ^ jaw_reg)
    Mh_inv = np.linalg.inv(M['head'])
    jaw = ((Mh_inv @ np.array([xw, 0.0, zw, 1.0]))[:3], Mh_inv[:3, :3] @ np.array([0, 1.0, 0]))
    M['jaw'] = M['head']
    attach_flaps(md, hinges, jaw)
    if os.environ.get('CYBER_PIECES_ONLY'):
        for n_, (l_, p_) in pieces.items():
            print(n_, l_, p_.status(), p_.num_tri(), f'{p_.volume() * 1e6:.1f} cm3', flush=True)
        sys.exit(0)

    rep += [f"truck {100 * L:.1f} x {200 * half_w:.1f} x {100 * z_peak:.1f} cm (L x W x H): L/H {L / z_peak:.2f}, "
            f"L/W {L / (2 * half_w):.2f} (Cybertruck 5.68 x 2.03 x 1.80 m: 3.16, 2.80)",
            f"profile: nose {100 * z_nose:.1f} cm ({z_nose / z_peak:.2f} H), peak {100 * z_peak:.1f} cm at {(x_nose - x_peak) / L:.2f} L from "
            f"the nose, tail {100 * z_tail:.1f} cm ({z_tail / z_peak:.2f} H); hood/windshield "
            f"{math.degrees(math.atan2(z_peak - z_nose, x_nose - x_peak)):.0f} deg, bed {math.degrees(math.atan2(z_peak - z_tail, x_peak - x_tail)):.0f} deg",
            'pieces (PLA wall, cm3): ' + ', '.join(f'{n} {p.volume() * 1e6:.0f}' for n, (_, p) in pieces.items()),
            "truck pose: lying face down; " + ', '.join(f'{k} {v:+.2f}' for k, v in TRUCK.items() if not k.startswith('R_'))]
    shells = {name: (ln, bake(p.transform(np.linalg.inv(M[ln])[:3, :]))) for name, (ln, p) in pieces.items()}
    # carve: whatever another link's parts sweep through, in the poses the duck uses and on the way between
    carved, done = [], []

    def overlaps(c, lo_, hi_):
        a, b = C.bbox(c)
        return not (np.any(a > hi_) or np.any(b < lo_))

    import hashlib
    import psutil
    proc = psutil.Process()
    cache = os.path.join(OUT, 'carve_cache')
    os.makedirs(cache, exist_ok=True)
    # sorted: transform_pose builds its dicts from sets, whose order changes from process to process (the key did too)
    nojaw = lambda qs: [sorted((k, round(v, 6)) for k, v in q_.items() if k != 'jaw') for q_ in qs]   # the jaw moves only the jaw
    key = hashlib.sha1(repr(('frame-cutters v3 get-up', sorted((k, v) for k, v in TRUCK.items() if k != 'jaw'), nojaw(CARVE_POSES), WALK_POSES, CLEAR, GAP,
                             WALL, SEAM, PEAK, NOSE, TAIL)).encode()).hexdigest()[:10]
    fresh = 0
    for name, (own, sh) in list(shells.items()):
        before = sh.volume()
        tag = f"_{int(sh.volume() * 1e9)}"               # keyed on the piece's own geometry too
        cf = os.path.join(cache, f"{name.replace(' ', '_')}_{key}{tag}.npz")
        if os.path.exists(cf) and os.environ.get('CYBER_DIAG') != name:   # carved before with these poses and sizes
            z = np.load(cf)
            sh = mf.Manifold(mf.Mesh(vert_properties=z['v'].astype(np.float32), tri_verts=z['f'].astype(np.uint32)))
            shells[name] = (own, sh)
            carved.append(f'{name} {100 * (1 - sh.volume() / before):.0f} %')
            done.append(name)
            print('cached', name, flush=True)
            continue
        log = name.endswith('flap')
        if os.environ.get('CYBER_DIAG') == name:         # which pose and part takes how much of this piece
            v0 = sh.volume()
            labels = ['truck'] + [l_ for l_, _ in ROBOT_POSES] + [f'transform {k / 20:.2f}' for k in range(1, 20)]
            rows_ = []
            for lab, q in zip(labels, CARVE_POSES):
                Mq = md.R.fk(q)
                inv = np.linalg.inv(Mq[own])
                for b_ in md.links:
                    if b_ == own or md.keep[b_] is None:
                        continue
                    c = C.posed(md.keep[b_], inv @ Mq[b_])
                    if overlaps(c, *C.bbox(sh)):
                        v = v0 - (sh - c).volume()
                        if v > 0.05e-6:
                            rows_.append((v, lab, b_))
                for other in done:
                    c = C.posed(shells[other][1], inv @ Mq[shells[other][0]])
                    if overlaps(c, *C.bbox(sh)):
                        v = v0 - (sh - c).volume()
                        if v > 0.05e-6:
                            rows_.append((v, lab, other + ' shell'))
            for lab, q in zip(['stand', 'stride', 'stride other', 'legs apart', 'high-knee both', 'back both', 'high step L', 'high step R'], WALK_POSES):
                Mq = md.R.fk(q)
                fz = min(C.bbox(C.posed(md.keep[f], Mq[f]))[0][2] for f in ('L_foot', 'R_foot'))
                g_ = box([-0.4, -0.4, fz - 0.4], [0.4, 0.4, fz + CLEAR]).transform(np.linalg.inv(Mq[own])[:3, :])
                v = v0 - (sh - g_).volume()
                if v > 0.05e-6:
                    rows_.append((v, lab, 'floor clearance'))
            print(f'{name}: {v0 * 1e6:.1f} cm3 of wall before carving')
            for v, lab, b_ in sorted(rows_, reverse=True)[:18]:
                print(f'  {v * 1e6:5.2f} cm3  {lab:28s} {b_}')
            sys.exit(0)
        # one pose at a time, and only what comes near (2026-11-10: one union of everything grew to 103 GB)
        for k_, q in enumerate(CARVE_POSES + (JAW_POSES if name == 'jaw' else [])):
            Mq = md.R.fk(q)
            inv = np.linalg.inv(Mq[own])
            if log:
                print(f'  {name}: pose {k_}, shell {sh.num_tri()} tris, status {sh.status()}', flush=True)
            lo_, hi_ = C.bbox(sh)
            near = [c for b in md.links if b != own and md.frame.get(b) is not None     # structure only: electronics
                    for c in [C.posed(md.frame[b], inv @ Mq[b])] if overlaps(c, lo_, hi_)]  # move later

            near += [c for other in done for c in [C.posed(shells[other][1], inv @ Mq[shells[other][0]])]
                     if overlaps(c, lo_, hi_)]               # and the shells ahead of it (nose to tail, left before right)
            if sh.num_tri() > 150000:                # ballooning (2026-11-10: the flap reached 3.7 M triangles)
                rep.append(f'{name}: carving stopped at {sh.num_tri()} triangles (pose {k_}): the cuts slice it to slivers')
                print(rep[-1], flush=True)
                break
            for c in near:                          # one at a time: the shell stays small, nothing big is merged
                if overlaps(c, *C.bbox(sh)):
                    if log:
                        print(f'  {name}: cut, shell {sh.num_tri()} tris, cutter {c.num_tri()} tris, '
                              f'{proc.memory_info().rss / 2 ** 30:.2f} GB', flush=True)
                        if os.environ.get('CYBER_DEBUG'):
                            for tag, m_ in (('shell', sh), ('cutter', c)):
                                v_, f_ = mesh_parts(m_)
                                np.savez(os.path.join(cache, f'debug_{tag}.npz'), v=v_, f=f_)
                    sh = sh - c
            sh = bake(sh)
        for q in WALK_POSES:                        # the floor, wherever the lower sole is in that pose
            Mq = md.R.fk(q)
            floor_z = min(C.bbox(C.posed(md.keep[f], Mq[f]))[0][2] for f in ('L_foot', 'R_foot'))
            ground = box([-0.4, -0.4, floor_z - 0.4], [0.4, 0.4, floor_z + CLEAR])
            sh = sh - ground.transform(np.linalg.inv(Mq[own])[:3, :])
        shells[name] = (own, bake(sh))
        v_, f_ = mesh_parts(shells[name][1])
        np.savez(cf, v=v_, f=f_)
        carved.append(f'{name} {100 * (1 - sh.volume() / before):.0f} %')
        done.append(name)
        print('carved', name, flush=True)
        fresh += 1
        if fresh >= int(os.environ.get('CYBER_PIECES_PER_RUN', 99)) and name != list(shells)[-1]:
            print('more to carve: run again (the memory goes back to the PC between runs)', flush=True)
            sys.exit(3)
    rep.append('carved for clearance (share of each wall removed): ' + ', '.join(carved))
    # carving can cut a scrap of wall off from its piece (2026-11-10: a sliver floated by the ankle): keep each piece's
    # main body, drop islands (attached to nothing, they could be neither printed in place nor mounted)
    dropped = []
    for name, (own, sh) in list(shells.items()):
        parts = sorted(sh.decompose(), key=lambda m: -m.volume())
        if len(parts) > 1:
            keep_ = [m for m in parts if m.volume() > 0.25 * parts[0].volume()]
            gone = [m for m in parts if m.volume() <= 0.25 * parts[0].volume()]
            shells[name] = (own, bake(C.union(keep_)))
            dropped.append(f"{name}: {len(gone)} ({sum(m.volume() for m in gone) * 1e6:.2f} cm3)")
    rep.append('islands dropped: ' + ('; '.join(dropped) or 'none'))
    # the Cybertruck's paint, as regions of the truck (coloured per piece when drawn): glass from partway up the
    # windshield over the cab roof and down the leaning upper sides; black cladding along the bottom and the angular
    # wheel arches; the light bars across the nose and the tail
    x_g1, x_g2 = x_nose - 0.45 * (x_nose - x_peak), x_nose - 0.60 * L
    glass = box([x_g2, -big, belt * z_peak + 0.002], [x_g1, big, big])
    clad = box([-big, -big, -big], [big, big, 0.15 * z_peak])
    for f in WHEELS:
        xc, b, t, h = x_nose - f * L, 0.10 * L, 0.065 * L, 0.42 * z_peak
        arch = extrude_profile([(xc - b, -0.01), (xc + b, -0.01), (xc + t, h), (xc - t, h)], -big, big)
        clad = clad + (arch - box([-big, -half_w + 0.003, -big], [big, half_w - 0.003, big]))
    # wheels painted in the arches (the duck has none: a tyre and a hub on the side walls)
    tyre, hub = [], []
    for f in WHEELS:
        xc, r = x_nose - f * L, 0.24 * z_peak
        for rr, lst in ((r, tyre), (0.55 * r, hub)):
            disc = mf.Manifold.cylinder(2 * big, rr, rr, 48, True).rotate([90, 0, 0]).translate([xc, 0, r + 0.002])
            lst.append(disc - box([-big, -half_w + 0.003, -big], [big, half_w - 0.003, big]))
    tyre, hub = C.union(tyre), C.union(hub)
    lw = half_w - lean - 0.003
    lights = {'head': box([x_nose - 0.0005, -min(lw, half_w - CORNER - 0.002), z_nose - 0.0065],
                          [x_nose + 0.0008, min(lw, half_w - CORNER - 0.002), z_nose - 0.0025]),
              'L flap': box([x_tail - 0.0008, SEAM / 2 + 0.002, z_tail - 0.008], [x_tail + 0.0005, lw, z_tail - 0.004]),
              'R flap': box([x_tail - 0.0008, -lw, z_tail - 0.008], [x_tail + 0.0005, -SEAM / 2 - 0.002, z_tail - 0.004])}
    geo = dict(x_nose=x_nose, x_tail=x_tail, x_peak=x_peak, z_nose=z_nose, z_peak=z_peak, z_tail=z_tail,
               half_w=half_w, belt=belt, lean=lean, M=M, cuts=cuts, glass=glass, clad=clad, lights=lights, tyre=tyre, hub=hub)
    return shells, geo


ROBOT_POSES = [('robot, zero pose', {}), ('robot, floor pick', dict(RP.PICK)),
               ('robot, head turned 1.0, nod 0.3', {'neck_pitch': 0.3, 'head_yaw': 1.0}),
               ('robot, stride', {'L_hip_pitch': -0.6, 'L_knee': 0.9, 'L_ankle_pitch': -0.3, 'R_hip_pitch': 0.4, 'R_knee': 0.2}),
               ('robot, stride (other leg)', {'R_hip_pitch': -0.6, 'R_knee': 0.9, 'R_ankle_pitch': -0.3, 'L_hip_pitch': 0.4, 'L_knee': 0.2}),
               ('robot, legs apart', {'L_hip_roll': 0.25, 'R_hip_roll': -0.25})]
# walking: no shell may come within CLEAR of the floor (the feet are what touch it); the gait's extremes
CLEAR = 0.008
WALK_POSES = [{}, ROBOT_POSES[3][1], ROBOT_POSES[4][1], ROBOT_POSES[5][1],
              {f'{s}_{k}': v for s in 'LR' for k, v in (('hip_pitch', -0.5), ('knee', 1.0), ('ankle_pitch', -0.5))},
              {f'{s}_{k}': v for s in 'LR' for k, v in (('hip_pitch', 0.3), ('knee', 0.3), ('ankle_pitch', 0.5))},
              {'L_hip_pitch': -0.9, 'L_knee': 1.4, 'L_ankle_pitch': -0.5, 'R_hip_pitch': 0.3, 'R_ankle_pitch': 0.35},
              {'R_hip_pitch': -0.9, 'R_knee': 1.4, 'R_ankle_pitch': -0.5, 'L_hip_pitch': 0.3, 'L_ankle_pitch': 0.35}]
# the deep floor pick (knee -1.08, the leg's front closing) is given up as Cyber Duck: it would need a 6 cm arc at
# the knee's front; the high steps are carved for instead (the walk bends the knee the other way)
GIVEN_UP = ['robot, floor pick']
CARVE_POSES = [TRUCK] + [q for l_, q in ROBOT_POSES if l_ not in GIVEN_UP] + WALK_POSES[4:] +     [transform_pose(k / 20)[1] for k in range(1, 20)]


RIM = '#2E3236'


def wheel_parts(md, geo):
    """Wheels and fender flares as parts, from the reference photo: a tyre standing proud of the side with chunky
    all-terrain blocks round its tread, a dark rim set in with six spokes and a hub, and an angular black flare round
    each arch. Decals: drawn on the pieces they sit on, not carved for (they stand 2-4 mm off the side)."""
    out = []
    L = geo['x_nose'] - geo['x_tail']
    hw, zp = geo['half_w'], geo['z_peak']
    for f, owners in zip(WHEELS, (('jaw', 'jaw'), ('L_knee_link', 'R_knee_link'))):
        xc, r = geo['x_nose'] - f * L, 0.24 * zp
        zc = r + 0.002
        for sgn, ln in ((+1, owners[0]), (-1, owners[1])):
            y0 = sgn * hw
            cyl = lambda rr, t0, t1, n=64: mf.Manifold.cylinder(abs(t1 - t0), rr, rr, n).rotate([-90, 0, 0]).translate(
                [xc, min(t0, t1), zc])
            out_ = lambda t: y0 + sgn * t                  # t mm outward from the side
            tyre = cyl(r, out_(0.0), out_(0.0030)) - cyl(0.66 * r, out_(-0.001), out_(0.0040))
            blocks = []
            for k in range(28):                           # tread blocks round the tyre's edge, staggered
                a = 2 * math.pi * k / 28
                rr = r - 0.0012 + (0.0006 if k % 2 else 0.0)
                b = box([-0.0016, 0, -0.0013], [0.0016, 0.0042, 0.0013]).rotate([0, -math.degrees(a), 0])
                b = b.translate([xc + rr * math.cos(a), min(out_(-0.0005), out_(0.0037)), zc + rr * math.sin(a)])
                blocks.append(b)
            rim = cyl(0.66 * r, out_(0.0004), out_(0.0016)) - cyl(0.50 * r, out_(0.0010), out_(0.0030))
            spokes = []
            for k in range(6):
                a = 2 * math.pi * k / 6 + 0.2
                sp = box([-0.0045 * r / 0.025 - 0.006, 0, -0.0016], [0.0045 * r / 0.025 + 0.006, 0.0012, 0.0016])
                sp = sp.rotate([0, -math.degrees(a), 0]).translate([xc, min(out_(0.0004), out_(0.0016)), zc])
                spokes.append(sp ^ cyl(0.62 * r, out_(0.0), out_(0.002)))
            hub = cyl(0.13 * r, out_(0.0004), out_(0.0022), 24)
            # the flare: an angular band round the arch (the arch's trapezoid, 3.5 mm wide), 2.5 mm proud
            b_, t_, h_ = 0.10 * L, 0.065 * L, 0.42 * zp
            tr = lambda g: [(xc - b_ - g, -0.01), (xc + b_ + g, -0.01), (xc + t_ + g, h_ + g), (xc - t_ - g, h_ + g)]
            band = mf.CrossSection([tr(0.0035)]) - mf.CrossSection([tr(0.0)])
            flare = mf.Manifold.extrude(band, 0.0025).rotate([90, 0, 0])            # (x, -e, z)
            flare = flare.translate([0, out_(0.0025) if sgn > 0 else out_(0.0), 0]) if sgn > 0 else flare.translate([0, out_(0.0025) + 0.0025, 0])
            flare = flare - box([-1, -1, -1], [1, 1, 0.15 * zp])          # it stops at the cladding: no legs under the body
            inv = np.linalg.inv(geo['M'][ln])[:3, :]
            fl_owner = 'head' if ln == 'jaw' else ln       # the front flare stays with the head as the bill opens
            for part, col, owner in ((tyre, TYRE, ln), (C.union(blocks), TYRE, ln), (rim, RIM, ln), (C.union(spokes), RIM, ln),
                                     (hub, HUB, ln), (flare, BLACK, fl_owner)):
                inv_o = np.linalg.inv(geo['M'][owner])[:3, :]
                out.append((owner, *mesh_parts(part.transform(inv_o)), col, 'decal'))
    return out


def collisions(md, shells, q, base, label, rep):
    M = md.R.fk(q, base=base)
    hits = []
    P = {n: C.posed(sh, M[ln]) for n, (ln, sh) in shells.items()}
    for n, (own, sh) in shells.items():
        for b in md.links:
            if md.keep[b] is None or (b != own and md.R.adjacent(own, b)):
                continue
            v = C.overlap(P[n], C.posed(md.keep[b], M[b]))
            if v > C.TOUCH:
                hits.append(f'{n} / {b} {v * 1e6:.2f}')
    for a, b in itertools.combinations(shells, 2):
        v = C.overlap(P[a], P[b])
        if v > C.TOUCH:
            hits.append(f'{a} / {b} shells {v * 1e6:.2f}')
    rep.append(f"collisions, {label}: {'; '.join(hits) or 'none'}")
    return hits


class Scene(RP.Scene):
    def __init__(self, md, extra):
        super().__init__(md.spec)
        kept, k = [], 0
        for ln, l in md.spec['links'].items():          # Scene.parts follows Solids.geom's order, cuts skipped
            for i, g in enumerate(l['geoms']):
                if md.S.geom[(ln, i)] is None:
                    continue
                if not hidden(ln, g) and not any(w in g['name'].lower() for w in ELECTRONIC):
                    kept.append(self.parts[k])     # (electronics not drawn: they are laid out later)
                k += 1
        self.parts = kept + extra
        self.base = np.eye(4)

    def model(self, q, tmp, hide=()):
        M = self.R.fk(q, base=self.base)
        low = min(G.apply(M[ln], v)[:, 2].min() for ln, v, f, c, r in self.parts if len(f))
        lift = np.eye(4)
        lift[2, 3] = -low
        B = lift @ self.base
        orig = self.R.fk
        self.R.fk = lambda qq, base=None: orig(qq, base=B)
        try:
            return super().model(q, tmp, hide)
        finally:
            self.R.fk = orig


def main():
    import memcap
    memcap.guard(cap_gb=float(os.environ.get('CYBER_CAP_GB', 6)), need_free_gb=16)          # never starve the PC (a run once grew to 103 GB)
    os.makedirs(OUT, exist_ok=True)
    md = Model()
    rep = [f'Cyber Duck ({G.RECIPE})']
    shells, geo = build(md, rep)
    collisions(md, shells, TRUCK, TRUCK_BASE, 'truck', rep)
    rep.append(f'given up as Cyber Duck: {", ".join(GIVEN_UP)} (collisions below are expected)')
    for label, q in ROBOT_POSES + [('robot, bill open 0.4', {'jaw': 0.4})]:
        collisions(md, shells, q, None, label, rep)
    for k in range(1, 10):
        t = k / 10
        collisions(md, shells, transform_pose(t)[1], None, f'transform {t:.1f}', rep)
    sup = []
    for k in range(0, 11):                           # what holds it up on the way (the bill should, mid-way)
        base_, q_ = transform_pose(1 - k / 10)
        c_ = contacts(md, q_, base_)
        sup.append(f"{k / 10:.1f}: {'+'.join(x.replace('_link', '') for x in c_)}")
        for j in md.spec['joints']:                      # inside the joint limits all the way
            v = q_.get(j['name'], 0.0)
            if not j['lower'] - 1e-6 <= v <= j['upper'] + 1e-6:
                sup[-1] += f" [{j['name']} {v:+.2f} outside {j['lower']:+.2f}..{j['upper']:+.2f}]"
    rep.append('getting up, floor contacts (0 truck .. 1 standing): ' + '; '.join(sup))

    for name, (ln, sh) in shells.items():
        v, f = mesh_parts(sh)
        RP.stl(os.path.join(OUT, f"{name.replace(' ', '_')}.stl"), v, f)
    extra = []
    for name, (ln, sh) in shells.items():
        inv = np.linalg.inv(geo['M'][ln])[:3, :]
        gl, cl = geo['glass'].transform(inv), geo['clad'].transform(inv)
        hb = geo['hub'].transform(inv)
        ty = geo['tyre'].transform(inv) - hb
        rest = sh - gl - ty - hb
        for part, col in ((sh ^ gl, GLASS), (sh ^ hb, HUB), ((sh - gl) ^ ty, TYRE), (rest ^ cl, BLACK), (rest - cl, STEEL)):
            if not part.is_empty():
                extra.append((ln, *mesh_parts(part), col, 'shell'))
        if name in geo['lights']:
            extra.append((ln, *mesh_parts(geo['lights'][name].transform(inv)), LIGHT if name == 'head' else RED, 'decal'))
    extra += wheel_parts(md, geo)
    for sd in 'LR':                                   # the flaps' hinge pins, drawn so the fold reads
        p_, a_ = md.R.hinges[sd]
        pin = mf.Manifold.cylinder(0.03, 0.0025, 0.0025, 16, True)
        pin = pin.rotate([90, 0, 0]).translate(p_.tolist())          # the hinge axis is the shin's y
        pin = pin ^ box(p_ - [0.02, 0.0, 0.02] + ([0, SEAM, 0] if sd == 'L' else [0, -0.08, 0]),
                        p_ + [0.02, 0.0, 0.02] + ([0, 0.08, 0] if sd == 'L' else [0, -SEAM, 0]))
        extra.append((f'{sd}_knee_link', *mesh_parts(pin), BLACK, 'decal'))
    import pickle
    with open(os.path.join(OUT, 'painted.pkl'), 'wb') as fh:      # for tools/cyber_duck_video.py
        pickle.dump(dict(extra=extra, hinges=md.R.hinges, jaw=md.R.jaw, geo={k: v for k, v in geo.items() if isinstance(v, (float, np.ndarray))}), fh)
    sc = Scene(md, extra)
    sc.R = md.R
    RP.VIEWS.update(top=(0, 89), back34=(215, 12), front34=(35, 14))
    shots = [('truck, side', 'truck', 'side'), ('truck, front 3/4', 'truck', 'front34'),
             ('truck, rear 3/4', 'truck', 'back34'), ('truck, top', 'truck', 'top'),
             ('robot, 3/4', 'robot', 'hero'), ('robot, side', 'robot', 'side')]
    tiles = []
    cx = 0.5 * (geo['x_nose'] + geo['x_tail'])
    for title, mode, view in shots:
        sc.base = TRUCK_BASE if mode == 'truck' else np.eye(4)
        if mode == 'truck':
            im = sc.render(TRUCK, view, 640, 480, 0.40, 0.06 if view != 'top' else 0.0, 0.0, (), target_x=cx)
        else:
            im = sc.render({'jaw': 0.3}, view, 640, 480, 0.42, 0.17, 0.0, ())
        ImageDraw.Draw(im).text((8, 6), title, fill=(0, 0, 0))
        tiles.append(im)
    sheet = Image.new('RGB', (1280, 480 * math.ceil(len(tiles) / 2)), 'white')
    for k, im in enumerate(tiles):
        sheet.paste(im, ((k % 2) * 640, (k // 2) * 480))
    sheet.save(os.path.join(OUT, 'cyber_duck.png'))
    with open(os.path.join(OUT, 'report.txt'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(rep) + '\n')
    print('\n'.join(rep))


if __name__ == '__main__':
    main()
