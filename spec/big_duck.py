"""A duck taller than echo, on echo's RobStride actuators: python -I spec/big_duck.py -> spec/robot_big.json. Then MILLI_RECIPE=big for the tools.

The small duck's joints and look at 1.80 m (2026-11-03, the user: "use the powerful servos we used on echo humanoid and build me a duck bigger than echo"; "full design first...
including connection parts, bearings, screws"; "they can be metal parts if needed for the strength"; "you can't give a gigantic 3d printed part that can't fit in a Bambu printer").
Not the small duck scaled up: at this size the load goes through metal, not printed brackets.

Choices, and why:
- Frame: 6061 and 7075 plate, cut flat and bolted, so a waterjet service can make every metal part from one sheet thickness. Printed parts carry no load.
- Skin: 3 mm PETG panels, bolted to the frame, each inside a 256 mm printer bed (tools/checks.py 'printable'). Split where it shows least, along the hood's and body's lines.
- Hip roll and hip pitch: a pair of RS04s on one axis, each side of the part they turn. 240 N m peak, without gearing, and no moment on either output.
- Knee: a pair of RS04s, each with a crank, a rod and a lever on the knee, as Agility's Cassie and Digit drive theirs by pushrod. 1.54:1 at its weakest (mid stroke), 2.56:1 at the straight end,
  over 70 deg, from -0.92 (bent) to +0.30. The crank never passes dead centre. Rod load about 2 kN mid stroke, more toward the ends: a hand estimate, from before the rod's length changed.
- Head pitch: one RS04 through #40 roller chain, 13T to 25T, 1.92:1: +-1.6 rad is too far for a crank. The chain carries about 3 kN at peak against 14 kN to break it. The cost: backlash.
- No head roll (2026-11-03, the user: forget the flip, lower the upper body's weight): the head pitch went from a pair to one RS04, and the upper frames are lighter. Those masses are pocketing fills, assumed:
  no pockets are drawn and nothing has been stress-checked.
- Hip yaw: one RS04, as it turns the leg but doesn't hold it up: a 61824 thin-section bearing round the stator takes the leg's moment off the actuator's own bearing.
- Knees and head pitch ride on 6908 bearings in the frame; RS04 outputs carry only torque where a far bearing is given.
- Feet: an aluminium plate, printed TPU soles in two pieces. The stance is narrower than the small duck's in proportion, as a foot further out loads the hip roll more.

NOT SAFE TO BUILD. About 100 kg and 1.8 m, with no stress analysis, metal frames nobody has checked and joints that drop when power goes (no brakes): a failure could seriously hurt someone. See docs/big/BOM.md, "Open before ordering".

Not checked here: stress in the plates and shafts (a hand estimate only), chain tension and fatigue, heat, and how it survives a fall. Actuator numbers are the retailer's or maker's: confirm before ordering.
The real actuators (2026-11-05): RobStride's STEP files, read in Fusion and as text (they stay in %TEMP%/figrefs/robstride, not in the repo: python -I tools/fetch_servo_cad.py). The numbers in RS below come from there. A real one has not been measured.
Every stator hangs by the shoulder its rear step leaves, through a plate with a bore for the step and a window for the cable, and every output carries a flange with holes for its 3 pins. So the old "plate on a rear face" is gone. Fit is what's been checked, not strength.
"""
import copy
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.pop('MILLI_RECIPE', None)
import robot as T                                  # the small duck's shape primitives, no more
from robot import ALU, AMBER, BLACK, CELL, DARK, GREY, LENS, ORANGE, PCB, WHITE, YELLOW, box, cut, cyl, fast, loft, mirror_geom, mirror_name, plate

# echo's spec/actuators.json, where the ratings come from: retailer numbers. r and h: the case's largest radius, and from its output face to its rear. The rest of the shape is in RS.
ACT = {
    'RS00': dict(r=0.0285, h=0.0511, mass=0.31, rated_nm=5.0, peak_nm=14.0, noload_rads=32.0, gear=10.0, rotor_kgm2=1e-05, damping_nms=0.02, friction_nm=0.3, out_pcd=0.027, out_n=6, mount_pcd=0.050, mount_n=6),
    'RS03': dict(r=0.0533, h=0.0566, mass=0.88, rated_nm=20.0, peak_nm=60.0, noload_rads=20.0, gear=9.0, rotor_kgm2=1e-04, damping_nms=0.12, friction_nm=1.0, out_pcd=0.03036, out_n=6, mount_pcd=0.098, mount_n=8),
    'RS04': dict(r=0.06, h=0.0527, mass=1.42, rated_nm=40.0, peak_nm=120.0, noload_rads=21.0, gear=9.0, rotor_kgm2=2e-04, damping_nms=0.2, friction_nm=1.5, out_pcd=0.036, out_n=9, mount_pcd=0.106, mount_n=10),
}
# What each STEP file says, in mm in its own frame, axis on z through the origin (Fusion's bounding box and point-in-solid scans, and the file's own circles). `zout`: which way its output faces.
# File units: mm, as each file states. No file holds a mass, so the 1.42 kg can't be checked from it.
# Screw and pin angles are given from the cable's direction ('g'), as both patterns are mirror-symmetric about it: a right-hand actuator turned 180 deg about g has its holes where the left's are mirrored. The rotor's angle is free (its zero is set in software), so it's clocked to match.
RS = {
    # file: z -27.2 (tail) to +28.5 (pin tips). 120 across z -14..+25, chamfered 1 mm at the shoulder. 94 across z -27.2..-14 (the manual's 39 and 13.2 are right). Output: a ring, r 12.5-25, at z +25.5, 0.5 proud of the stator's face (r 30.25-60) at +25.
    # Pins: 3 x 6 mm on 35.86, tips z +28.5, so 3.0 proud of the output, not the manual's 3.3. M5 x 9 on 36: blind at z +22, so **3.5 deep, not the manual's 6.5** (6.5 is from the floor to the pin tips).
    # Mount: 10 M4 on 106, angles 21 + 36k, front face tap to z +18 (7 deep, the manual says 5), rear shoulder tap to z -8 (6 deep). Cable: a 7.3 mm sleeve at angle 75, z -20.3, to r 58.1.
    'RS04': dict(zout=1, front=0.0005, shoulder=0.0395, tail=0.0527, pin=0.003, r_tail=0.047, cable=dict(z=0.0458, r=0.00365, reach=0.0581, angle=75.0),
                 mount=(18, -18, 54, -54, 90, -90, 126, -126, 162, -162), out=(0, 30, -30, 90, -90, 120, -120, 150, -150), pins=(180, 60, -60), screw_out=('M5', 0.0035), screw_mount=('M4', 0.006)),
    # file: z 0 (output) to 56.6 (rear). Output: r 35, 2.5 proud of the stator's face. Body r 46.4, with 8 lobes to r 53.3 at the screws, 22.5 + 45k. Rear step r 39.75 from z 42: 14.6 long. Screws: 8 M4 on 98, 11 deep from the front at z 2.5. From the rear only 4 are free, at 22.5 and 157.5 either way, under 8 mm recesses from z 42: at the other 4, RobStride's own cap screws stand to z 46 (Fusion's clash check found it). So a rear plate gets 4 screws and 4 holes for those heads. The front's 8 are unused
    # Output: 6 M4 on 30.36, angles 60k, 9 deep; 3 x 4 mm holes for dowels, angles 30 + 120k, 7 deep. It has holes where the RS04 has pins. Its plugs: on the rear step's sides, at z 53, facing +-x (g here), so no cable sleeve to notch for.
    'RS03': dict(zout=-1, front=0.0025, shoulder=0.042, tail=0.0566, pin=0.0, r_tail=0.0400, cable=None, mount=(22.5, -22.5, 157.5, -157.5), caps=(67.5, -67.5, 112.5, -112.5), out=(0, 60, -60, 120, -120, 180), pins=(-30, -150, 90), screw_out=('M4', 0.009), screw_mount=('M4', 0.011)),
    # file: z -6 (output) to 46.1. 57 across, one piece with its 2 sockets. Its beak drive isn't designed, so nothing is bolted to it yet.
    'RS00': dict(zout=-1, front=0.001, shoulder=0.0521, tail=0.0521, pin=0.0, r_tail=0.0285, cable=None, mount=(), out=(), pins=(), screw_out=('M3', 0.004), screw_mount=('M3', 0.004)),
}
HEAD = {'M3': (0.0028, 0.003), 'M4': (0.0035, 0.004), 'M5': (0.00425, 0.005)}   # socket cap: head radius, height
CSK = {'M4': (0.0045, 0.0025), 'M5': (0.0050, 0.0028)}                         # ISO 10642 countersunk: head radius, height. Drawn as a cylinder sunk in a counterbore of that size
ROOT = {'M3': 0.00120, 'M4': 0.00160, 'M5': 0.00205}                             # a screw drawn at its core: it must lie in the tap hole, and clashes if it's too long or misplaced
CLEAR = {'M3': 0.00170, 'M4': 0.00225, 'M5': 0.00275}
TAP = {'M3': 0.00125, 'M4': 0.00165, 'M5': 0.00210}
CHAIN = 25 / 13                                    # #40 chain, 13T on the RS04, 25T on the joint: the head pitch
# The knee's four-bar. 2026-11-05: the rods were never given their motion, and when they were, this four-bar didn't close: at 106 mm, with the lever at 130 deg, the rod can't reach below knee -0.74, probably since ZP moved after the sweep that chose it (a comment below still had 0.634; it's 0.674). Swept again over the whole range (-0.92 to +0.30)
# with the same crank and lever: a 148 mm rod, the lever at 150 deg. Crank 121 deg of stroke, from 211 (behind) to 90 (up). At its weakest 1.54:1, as before; 1.89 at the bent end, as the walks assume; 2.56 at the straight end, not 3.35. Rod's line stays 36 mm from the stator's axis, so its 11 mm eye clears the screws (r 22). Transmission angle at least 34 deg at the crank, 43 at the lever
CRANK_R, CRANK_LEVER, CRANK_ROD, CRANK_PHI = 0.060, 0.096, 0.148, math.radians(150)
KNEE_RATIO, KNEE_RATIO_MAX = 1.54, 2.56            # crank turn per knee turn: mid stroke, and at the straight end
KNEE_RATIO_BENT = 1.89                             # on the bent side, where a swinging knee moves fastest: about 1.7 at -0.92, so a little pessimistic
KNEE_RANGE = (-0.92, 0.30)                         # bent: where the shin cover meets the thigh plate. Straight: where the walks stop


def crank_points(s, k):
    """Crank pin and lever pin (x, z) at knee angle 0, for a stator at s and the knee at k: the rod's two ends."""
    b = (k[0] + CRANK_LEVER * math.cos(CRANK_PHI), k[1] + CRANK_LEVER * math.sin(CRANK_PHI))
    dd = math.hypot(b[0] - s[0], b[1] - s[1])
    c = math.atan2(b[1] - s[1], b[0] - s[0]) - math.acos((CRANK_R ** 2 + dd ** 2 - CRANK_ROD ** 2) / (2 * CRANK_R * dd))
    return (s[0] + CRANK_R * math.cos(c), s[1] + CRANK_R * math.sin(c)), b


def four_bar(s, k, q):
    """(crank angle, rod angle) in the side view's atan2(z, x), at knee angle q. A turn about +y lowers these."""
    a = CRANK_PHI - q
    b = (k[0] + CRANK_LEVER * math.cos(a), k[1] + CRANK_LEVER * math.sin(a))
    dd = math.hypot(b[0] - s[0], b[1] - s[1])
    c = math.atan2(b[1] - s[1], b[0] - s[0]) - math.acos((CRANK_R ** 2 + dd ** 2 - CRANK_ROD ** 2) / (2 * CRANK_R * dd))
    p = (s[0] + CRANK_R * math.cos(c), s[1] + CRANK_R * math.sin(c))
    return c, math.atan2(b[1] - p[1], b[0] - p[0])
PCD13, PCD25 = 0.0530, 0.1014                      # 12.7 mm pitch sprockets
BEAR = {'61824': dict(d=0.120, D=0.150, w=0.016, mass=0.45), '6908': dict(d=0.040, D=0.062, w=0.012, mass=0.11), '6206': dict(d=0.030, D=0.062, w=0.016, mass=0.20)}
AL, ALD, PET = '#A8ACB0', '#50555B', WHITE
LINKS, JOINTS = {}, []
MATERIALS = dict(T.MATERIALS)
MATERIALS.update(al7075=dict(density=2810, note='7075-T6: plates under load'), al6061=dict(density=2700, note='6061-T6: plates and tubes'),
                 petg=dict(density=1270, note='PETG: skin panels, printed on a 256 mm bed'), steel=dict(density=7800, note='bought shafts, bearings and chain'),
                 cells=dict(density=2500, note='battery: explicit mass'), electronics=dict(density=1500, note='explicit masses'))


def link(name, geoms, note=''):
    LINKS[name] = dict(name=name, budget=0.0, geoms=list(geoms), note=note)


def metal(name, lo, hi, color=AL, mat='al6061', fill=1.0, **kw):
    return box(name, lo, hi, color, 'struct', fillet=0.002, material=mat, fill=fill, exposed=True, **kw)


def side_plate(name, y0, t, circles, color=AL, mat='al7075'):
    return dict(plate(name, y0, t, circles, color, 'struct'), material=mat, fill=1.0, exposed=True)


def axle(name, c, axis, r, h, color=ALD, mat='steel'):
    return dict(cyl(name, c, axis, r, h, color, 'struct'), material=mat, fill=1.0, exposed=True)


def disc(name, c, axis, r, h, color=AL):
    return dict(cyl(name, c, axis, r, h, color, 'struct'), material='al6061', fill=1.0, exposed=True)


AX = {'x': (1.0, 0.0, 0.0), 'y': (0.0, 1.0, 0.0), 'z': (0.0, 0.0, 1.0)}


def vec(s):
    """'+y' -> (0, 1, 0)."""
    return tuple((-1.0 if s[0] == '-' else 1.0) * v for v in AX[s[1]])


def along(p, d, t):
    return tuple(p[i] + d[i] * t for i in range(3))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def ring(c, d, g, pcd, angles):
    """Points on a circle round axis d through c, at angles from direction g."""
    w = cross(d, g)
    out = []
    for a in angles:
        ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
        out.append(tuple(c[i] + pcd / 2 * (ca * g[i] + sa * w[i]) for i in range(3)))
    return out


def axis_of(d):
    return 'xyz'[max(range(3), key=lambda i: abs(d[i]))]


def tube(name, a, d, t, r, color=BLACK, role='struct', **extra):
    """A cylinder from point a, length t along d."""
    return dict(cyl(name, along(a, d, t / 2), axis_of(d), r, t, color, role), **extra)


def rs(name, act, hub, out, g):
    """A stator, its output face at hub, facing `out` ('+y'), its cable toward g: the spec's stepped stand-in for RobStride's STEP model, which replaces it in Fusion (`real`). out: the case, the step behind it and the sleeve."""
    R, a, o, gv = RS[act], ACT[act], vec(out), vec(g)
    back = tuple(-v for v in o)
    case = tube(f'{name} (buy) {act}', along(hub, back, R['front']), back, R['shoulder'] - R['front'], a['r'], BLACK, 'actuator', exposed=True, material='actuator', stand_in=True,
                real=dict(model=act, hub=list(hub), out=list(o), g=list(gv)))
    res = [case]
    if R['tail'] > R['shoulder']:
        res.append(tube(f'{name} rear step', along(hub, back, R['shoulder'] - 0.0002), back, R['tail'] - R['shoulder'] + 0.0002, R['r_tail'], BLACK, 'struct', exposed=True, material='actuator', mass=0.0, seat=True, stand_in=True))
    if R['cable']:
        C_ = R['cable']
        p = along(along(hub, back, C_['z']), gv, R['r_tail'] - 0.001)
        res.append(tube(f'{name} cable sleeve', p, gv, C_['reach'] - R['r_tail'] + 0.001, C_['r'], BLACK, 'struct', exposed=True, material='actuator', mass=0.0, seat=True, stand_in=True))
    return res


def rotor(name, act, hub, out, g):
    """The part of an actuator that turns, in the link it drives: a marker, as Fusion places that piece of the STEP model here. No body of its own in the checks."""
    o, gv = vec(out), vec(g)
    return dict(tube(f'{name} rotor', hub, o, 0.0003, 0.002, BLACK, 'decal', material='actuator', mass=0.0), stand_in=True, linkage=True, rotor=dict(model=act, hub=list(hub), out=list(o), g=list(gv)))


def seat(name, c, axis, r=0.055):
    """Where a stator's face is bolted on: a thin ring that overlaps both, as the checks read a bare touch as two pieces. Not built in Fusion."""
    return dict(cyl(f'{name} seat', c, axis, r, 0.001, ALD, 'struct'), material='al6061', fill=1.0, exposed=True, seat=True, stand_in=True)


def screws(name, pts, d, size, L, group=None, head_h=None, csk=False, **joins):
    """Socket cap screws: head under-side at each of pts, going along d. The head, and the shank at its core diameter. Bought, so 'decal': Fusion builds them, and its clash check tests each in its tap hole.
    csk: ISO 10642 countersunk, pts on the surface it's flush with, its head sunk in below them (drawn as a cylinder in a counterbore of its size). joins: what it clamps and what it threads into, for tools/big_assembly.py:
    clamp=[parts], tap=part, or tap_hub=an actuator's hub (its own thread, in a part that has no holes in the spec)."""
    rh, hh = (CSK[size] if csk else (HEAD[size][0], head_h or HEAD[size][1]))
    out = []
    for i, p in enumerate(pts):
        back = tuple(-v for v in d)
        h = fast(tube(f'{name} {i}', p, d, hh, rh, DARK, 'decal', material='steel') if csk else tube(f'{name} {i}', p, back, hh, rh, DARK, 'decal', material='steel'),
                 size, L, p, d, csk=csk, head=[rh, hh], **{k: v for k, v in joins.items() if v is not None})
        s_ = tube(f'{name} {i} shank', along(p, d, hh) if csk else p, d, L - hh if csk else L, ROOT[size], DARK, 'decal', material='steel')    # a countersunk one's length counts its head
        h['color'] = s_['color'] = T.STEEL
        h['linkage'] = s_['linkage'] = True       # bought: sits in a clearance hole, so it joins nothing in the checks' solids
        if group:
            h['group'] = s_['group'] = group
        out += [h, s_]
    return out


def holes(name, pts, d, r, depth, *targets):
    """Round holes from each of pts, along d: cuts."""
    return [cut(tube(f'{name} {i}', along(p, d, -0.0005), d, depth + 0.001, r, ALD, 'struct'), *targets) for i, p in enumerate(pts)]


def rear_mount(name, act, hub, out, g, plates, t=None):
    """A stator bolted by the shoulder its rear step leaves: the bore for the step and the window for its cable, through the plates; the seat; the screws. Plates: named geoms from the shoulder back, t thick."""
    t = PL if t is None else t
    R, a, o, gv = RS[act], ACT[act], vec(out), vec(g)
    back = tuple(-v for v in o)
    sh = along(hub, back, R['shoulder'])
    out_ = [seat(name, sh, axis_of(o), a['r'] - 0.0045)]
    out_.append(cut(tube(f'{name} bore', along(sh, o, 0.001), back, t + 0.002, R['r_tail'] + 0.0015, ALD, 'struct'), *plates))
    if R['cable']:
        out_.append(window(f'{name} cable window', sh, back, gv, t, R['r_tail'], 0.064, 0.0045, *plates))
    if R['mount']:
        pts = ring(sh, o, gv, a['mount_pcd'], R['mount'])
        under = [along(p, back, t) for p in pts]
        out_ += holes(f'{name} screw hole', under, o, CLEAR['M4'], t, *plates)
        if R.get('caps'):
            out_ += holes(f'{name} cap screw clearance', [along(p, back, t) for p in ring(sh, o, gv, a['mount_pcd'], R['caps'])], o, 0.0042, t, *plates)
        # 8 plate + the rest in its thread. RS03's sits in a 2.5 mm recess
        L = 0.012 if act == 'RS04' else 0.016
        out_ += screws(f'{name} mount screw', under, o, 'M4', L, clamp=list(plates), tap_hub=list(hub))
    return out_


def window(name, c, back, g, t, r0, r1, half, *targets):
    """A slot in a plate, along g from r0 to r1 about c, half wide, through t going `back`. Square to the axes, so a cut box."""
    w = cross(back, g)
    a = [c[i] + back[i] * (t + 0.001) * (0.5) + g[i] * (r0 + r1) / 2 for i in range(3)]
    size = [abs(back[i]) * (t + 0.002) + abs(g[i]) * (r1 - r0) + abs(w[i]) * 2 * half for i in range(3)]
    lo = [a[i] - size[i] / 2 for i in range(3)]
    hi = [a[i] + size[i] / 2 for i in range(3)]
    return cut(box(name, lo, hi, ALD, 'struct'), *targets)


def output_holes(name, act, hub, out, g, stack, web, targets, cb_targets=None, group=None, head_h=None):
    """A flange on an output at hub: its screws' holes through `stack`, a counterbore down to `web`, holes for the pins, the screws, and what turns: the rotor. Engages 3 mm of the RS04's 3.5 mm thread."""
    R, a, o, gv = RS[act], ACT[act], vec(out), vec(g)
    size, depth = R['screw_out']
    pts = ring(hub, o, gv, a['out_pcd'], R['out'])
    in_ = tuple(-v for v in o)
    under = [along(p, o, web) for p in pts]
    res = holes(f'{name} screw hole', pts, o, CLEAR[size], web + 0.0002, *targets)
    if stack > web + 1e-6:
        res += holes(f'{name} counterbore', under, o, HEAD[size][0] + 0.0004, stack - web, *(cb_targets or targets))
    pin_r = {'RS04': 0.00305, 'RS03': 0.00200}[act]
    pin_d = {'RS04': 0.0035, 'RS03': 0.0045}[act]
    res += holes(f'{name} pin hole', ring(hub, o, gv, 0.03586 if act == 'RS04' else a['out_pcd'], R['pins']), o, pin_r, pin_d, *targets)
    L = {'RS04': round((web + 0.003) * 1000) / 1000, 'RS03': round((web + 0.004) * 1000) / 1000}[act]
    res += screws(f'{name} screw', under, in_, size, L, head_h=head_h, clamp=list(targets), tap_hub=list(hub))
    if group:
        for g_ in res:
            g_['group'] = group
    r_ = rotor(name, act, hub, out, g)
    if group:
        r_['group'] = group
    return res + [r_]


def joined(name, pts, d, size, L, through, t, *targets_tap):
    """Plate-to-part screws: through `through` (t thick), tapped in the rest."""
    res = holes(f'{name} hole', pts, d, CLEAR[size], t, *through)
    res += holes(f'{name} tap', pts, d, TAP[size], L, *targets_tap)
    return res + screws(name, pts, d, size, L, clamp=list(through), tap=targets_tap[0])


def bolt(name, pts, d, size, L, clamp, t, tap, csk=False, head_h=None, tap_hub=None):
    """Screws through `clamp` (t thick in all: clearance holes) into `tap` (a tapped hole, blind, 1 mm past the tip). pts: under each head, or for countersunk ones the surface they're flush with."""
    res = holes(f'{name} hole', pts, d, CLEAR[size], t, *clamp)
    if tap:
        res += holes(f'{name} tap', [along(p, d, t) for p in pts], d, TAP[size], L - t + 0.0005, tap)
    if csk:
        r, k = CSK[size]
        res += [cut(tube(f'{name} countersink {i}', along(p, d, -0.0005), d, k + 0.0005, r + 0.0002, ALD, 'struct'), clamp[0]) for i, p in enumerate(pts)]
    return res + screws(name, pts, d, size, L, csk=csk, head_h=head_h, clamp=list(clamp), tap=tap, tap_hub=tap_hub)


def bearing(name, joint, c, axis, part, inner, outer):
    """Two races: returns the geoms for the link holding the inner race and the one holding the outer. One bought part: tools/big_assembly.py fits both races together.
    Drawn at their nominal sizes, as are the bores and shafts they sit on, so a fit reads as a touch, not a clash: the fit itself (BOM.md) is a machining tolerance."""
    b = BEAR[part]
    race = (b['D'] - b['d']) / 2 * 0.45
    i = dict(cyl(f'{name} bearing inner race', c, axis, b['d'] / 2 + race, b['w'], ALU), exposed=True, material='steel', mass=b['mass'] / 2, on_joint=joint, part=part, seat=True, unit=f'{name} bearing')
    ic = cut(cyl(f'{name} bearing inner bore', c, axis, b['d'] / 2, b['w'] + 0.002), f'{name} bearing inner race')
    o = dict(cyl(f'{name} bearing outer race', c, axis, b['D'] / 2, b['w'], ALU), exposed=True, material='steel', mass=b['mass'] / 2, on_joint=joint, part=part, unit=f'{name} bearing')
    oc = cut(cyl(f'{name} bearing outer bore', c, axis, b['D'] / 2 - race, b['w'] + 0.002), f'{name} bearing outer race')
    return [i, ic], [o, oc]


def chain(name, p, q, y, r0, r1):
    """A #40 roller chain's two strands between sprockets at p and q (x, z), at y: bought, drawn at the zero pose."""
    dx, dz = q[0] - p[0], q[1] - p[1]
    L = math.hypot(dx, dz)
    nx, nz = -dz / L, dx / L
    out = []
    for sg, tag in ((1, 'a'), (-1, 'b')):
        a = (p[0] + sg * r0 * nx, y, p[1] + sg * r0 * nz)
        b = (q[0] + sg * r1 * nx, y, q[1] + sg * r1 * nz)
        out.append(dict(T.cap(f'{name} {tag}', a, b, 0.004, DARK, 'decal'), material='steel', mass=0.09, linkage=True))
    return out


def joint(name, parent, child, origin, axis, lim, act, count=1, ratio=1.0, out=+1, act_c=None, remote=False, interface=()):
    JOINTS.append(dict(name=name, parent=parent, child=child, origin=list(origin), axis=axis, lower=lim[0], upper=lim[1], act=act, act_c=list(act_c or origin), out=out, long='-x',
                       exposed=True, remote=remote, case_in='parent', fork_at=None, fork_r=None, post=False, fork=False, sides=[], faces=[], bearing=[], stack=0.01, face_t=None, tray_full=False,
                       capacity=dict(model=act, count=count, ratio=ratio), interface=interface))


# ---------------------------------------------------------------- anchors, metres

H4 = 0.056                                         # the lanes' width for one RS04, from its output face to behind its rear step, where the plates' old stand-in ended
R4 = ACT['RS04']['r']
S4 = RS['RS04']['shoulder']                        # output face to shoulder
ZA, ZK, XK = 0.190, 0.530, -0.235                  # ankle, knee: the small duck's stance, the shin upright
STANCE = 1.02
ZP = ZK - XK / math.tan(STANCE)                    # hip pitch and roll: 0.674
ZR = ZP
YY = 0.110                                         # hip yaw and roll axes: the two yaw drums (80 mm) 20 mm apart
ZY = ZR + 0.125                                    # the yaw plate over the roll stators
PL = 0.008                                         # the frame's plate thickness
# along y, from the middle out: the yoke, the inner upright, a stator, the crank lane, the thigh's two plates, a lane, a stator, the outer upright
YOKE_END = 0.1745                                  # the hip yoke's side, where its standoff ring meets it
# 2026-11-05: both stator lanes 2.5 mm further from the thigh, so each rod pin has room past its rod for a DIN 6799 E-clip (the pin stopped 0.3 mm short of the thigh plate, with nothing to hold the rod on)
EC = 0.0025
YSI = (0.182 - EC, 0.238 - EC)                     # inner stators' lane: the output face at its outer end
YCI = 0.244                                        # inner crank lane and hip output plate
YT = (0.250, 0.290)                                # the thigh's width, between its plates' outer faces
YCO = 0.296
YSO = (0.302 + EC, 0.358 + EC)
YC = (YT[0] + YT[1]) / 2                           # 0.270: thigh, shin, ankle and foot on one line
XQ, ZQ = -0.212, ZP - 0.015                        # the knee stators, at the thigh's top back corner, as the small duck's knee servo sits
PIN_A, PIN_B = crank_points((XQ, ZQ), (XK, ZK))
# each 12 mm lane: crank and lever in one plane, as they never come within 56 mm of each other, and the rod beside them, next to the thigh plate
LANE_I, LANE_O = YSI[1] + 0.0002, YSO[0] - 0.0062   # the 6 mm crank and lever plane's inner face, in each lane: a seat bolts the crank to the actuator's output
HUB_I, HUB_O = LANE_I, LANE_O + 0.006               # the output faces, cranks flat on them
YI = (HUB_I - S4 - PL, HUB_I - S4)                  # inner upright, or the thigh's rear plate: on the shoulder, 8 thick, the 94 mm step through it. Its tail stands 5.2 mm past. Was 0.174-0.182, flat on a rear face that isn't there
YO = (HUB_O + S4, HUB_O + S4 + PL)
ROD_I, ROD_O = LANE_I + 0.0065, LANE_O - 0.0055     # the 5 mm rod's inner face: 0.5 mm off its crank and lever, 2.8 mm off the thigh plate, where its E-clip and the pin's end are
CLIP = dict(r=0.0080, t=0.0009, gap=0.0002, past=0.0016)   # DIN 6799 size 8, for a 10 mm pin with an 8 mm groove: outer radius, thickness, its gap to the rod, and the pin's end past the rod
# ankle: one RS04 in the shin, output outward; a 6206 on the far side
YA_S = (YC - H4 / 2 - 0.006, YC + H4 / 2 - 0.006)  # its lane: the output face at the outer end
YA_M = (YA_S[1] - S4 - PL, YA_S[1] - S4)            # the plate on its shoulder
FOOT = dict(x=(XK - 0.115, XK + 0.215), y=(YC - 0.095, YC + 0.095), plate=0.040, sole=0.040)

# ---------------------------------------------------------------- body

BX, BW = (-0.300, 0.200), 0.205
BZ = (ZY + 0.030, 1.045)
XN, ZN = 0.175, 0.940                              # neck pitch, in the body's front
YN = (0.034, 0.090)                                # the neck's two stators' lane, either side of its tube
NSH = YN[0] + S4                                   # their shoulders, where the plates are
DECK = (ZY + 0.010 + H4, ZY + 0.010 + H4 + PL)     # the plate the yaw stators hang from
UPPER = (BZ[1] - 0.030, BZ[1] - 0.030 + PL)


def body_panels():
    secs = [(BZ[0], -0.05, 0, 0.235, 0.190, 6.0), (BZ[0] + 0.05, -0.05, 0, 0.250, BW, 8.0), (BZ[1] - 0.05, -0.05, 0, 0.250, BW, 8.0), (BZ[1] - 0.015, -0.05, 0, 0.230, 0.185, 5.0), (BZ[1], -0.05, 0, 0.190, 0.140, 3.5)]
    out = []
    xs = [-0.310, -0.068, 0.070, 0.205]
    ys = [-0.215, 0.0, 0.215]
    for i in range(3):
        for j in range(2):
            out.append(loft(f'body panel {"RMF"[i]}{"RL"[j]}', secs, WHITE, 'shell', open=('bottom',), clip=[((xs[i], 0, 0), (1, 0, 0)), ((xs[i + 1], 0, 0), (-1, 0, 0)), ((0, ys[j], 0), (0, 1, 0)), ((0, ys[j + 1], 0), (0, -1, 0))]))
    return [dict(g, material='petg') for g in out]


neck_slot = box('neck slot', (XN - 0.135, -0.125, ZN - 0.09), (0.26, 0.125, BZ[1] + 0.02), WHITE, 'shell')
# The hip yaw (2026-11-05, redesigned for the real case). It was a bearing round the stator's own 120 mm case, its inner ring on a sleeve the case left no room for. Now the stator hangs from the deck by its shoulder, with its cable out above the deck, in the body. A fixed housing from the deck
# round it holds the bearing's outer ring below it, and the output's flange holds the inner, so the leg's moment goes housing -> deck and only torque goes through the case
HUB_Y = ZY + 0.0265                                 # the deck's underside less 39.5
BRG_Y = ZY + 0.011                                  # 16 wide: ZY + 0.003 to 0.019
yaw_in_l, yaw_out_l = bearing('L hip yaw', 'L_hip_yaw', (0, YY, BRG_Y), 'z', '61824', 'L_hip_yaw_link', 'trunk')
yaw_out_r = [mirror_geom(g) for g in yaw_out_l]
for g in yaw_out_r:
    if g.get('on_joint'):
        g['on_joint'] = 'R_hip_yaw'
    if g.get('unit'):
        g['unit'] = 'R hip yaw bearing'


def yaw_mount(tag, y):
    """In the trunk, one side's hip yaw: housing, its retainer, the stator on the deck and the screws.
    2026-11-05: the housing has a flange at its foot, as its 6 mm wall round the bearing left nowhere to screw the retainer: 8 countersunk M4 from below, put in before the yaw flange and the beam."""
    hub = (0, y, HUB_Y)
    h, b = f'{tag} hip yaw housing', f'{tag} hip yaw retainer'
    a = f'{tag}_hip_yaw actuator'
    return [
        tube(h, (0, y, ZY + 0.003), (0, 0, 1), DECK[0] - ZY - 0.003, 0.081, ALD, 'struct', material='al6061', fill=1.0, exposed=True),
        cut(tube(f'{h} bore', (0, y, ZY + 0.002), (0, 0, 1), DECK[0] - ZY, 0.062, ALD, 'struct'), h),
        cut(tube(f'{h} bearing bore', (0, y, ZY + 0.002), (0, 0, 1), 0.017, 0.075, ALD, 'struct'), h),     # to the outer ring's top: its shoulder and the retainer clamp the ring
        tube(f'{h} flange', (0, y, ZY + 0.003), (0, 0, 1), 0.010, 0.092, ALD, 'struct', material='al6061', fill=1.0, exposed=True, part=h),
        cut(tube(f'{h} flange bore', (0, y, ZY + 0.002), (0, 0, 1), 0.012, 0.081, ALD, 'struct'), f'{h} flange'),
        tube(b, (0, y, ZY), (0, 0, 1), 0.003, 0.092, ALD, 'struct', material='al6061', fill=1.0, exposed=True),
        cut(tube(f'{b} bore', (0, y, ZY - 0.001), (0, 0, 1), 0.005, 0.068, ALD, 'struct'), b),
        *bolt(f'{b} screw', ring((0, y, ZY), (0, 0, 1), (1, 0, 0), 0.171, [22.5 + 45 * k for k in range(8)]), (0, 0, 1), 'M4', 0.010, [b], 0.003, f'{h} flange', csk=True),
        *rs(a, 'RS04', hub, '-z', '+x'),
        *rear_mount(f'{tag} hip yaw', 'RS04', hub, '-z', '+x', ['deck plate']),
        *joined(f'{tag} hip yaw housing screw', [(p[0], p[1], DECK[1]) for p in ring((0, y, DECK[1]), (0, 0, -1), (1, 0, 0), 0.143, [18 + 36 * k for k in range(10)])], (0, 0, -1), 'M5', 0.016, ['deck plate'], PL, h),
    ]


POSTS = [(-0.24, 0.13), (-0.24, -0.13), (0.045, 0.02), (0.045, -0.02)]
NECK_PLATE = [(XN, ZN, 0.072), (XN - 0.10, DECK[1] + 0.030, 0.03), (XN + 0.015, DECK[1] + 0.030, 0.03), (XN - 0.128, DECK[1] + 0.002, 0.002), (XN + 0.030, DECK[1] + 0.002, 0.002)]   # flat along the deck
XT = (XN - 0.112, XN - 0.100)                      # the neck tie, between the neck plates


def neck_frame():
    """The neck's plates on the deck: a tie between them at the back, a cleat outside each at the front. 2026-11-05: they stood 1 mm into the deck, held by nothing."""
    out = [metal('neck tie', (XT[0], -NSH, DECK[1]), (XT[1], NSH, ZN - 0.02), ALD),
           *bolt('neck tie deck screw', [(sum(XT) / 2, y, DECK[0]) for y in (0.03, -0.03)], (0, 0, 1), 'M5', 0.016, ['deck plate'], PL, 'neck tie')]
    for sg, s_ in ((1, 'L'), (-1, 'R')):
        pl, cl = f'neck plate {s_}', f'neck cleat {s_}'
        yo = sg * (NSH + PL)                       # the plate's outer face
        out += [
            dict(side_plate(pl, NSH if sg > 0 else -NSH - PL, PL, NECK_PLATE), fill=0.6),
            cut(box(f'{pl} foot', (XN - 0.15, min(0, sg) * 0.1, DECK[1] - 0.02), (XN + 0.10, max(0, sg) * 0.1, DECK[1]), ALD, 'struct'), pl),    # its 144 mm circle reached 5 mm into the deck
            *bolt(f'neck tie screw {s_}', [(sum(XT) / 2, yo, z) for z in (DECK[1] + 0.014, DECK[1] + 0.039)], (0, -sg, 0), 'M5', 0.016, [pl], PL, 'neck tie'),
            metal(cl, (XN + 0.007, min(yo, yo + sg * 0.012), DECK[1]), (XN + 0.035, max(yo, yo + sg * 0.012), DECK[1] + 0.016), ALD),
            *bolt(f'neck cleat screw {s_}', [(XN + 0.021, yo + sg * 0.012, DECK[1] + 0.008)], (0, -sg, 0), 'M5', 0.018, [cl], 0.012, pl),
            *bolt(f'neck cleat deck screw {s_}', [(XN + x, yo + sg * 0.006, DECK[0]) for x in (0.014, 0.028)], (0, 0, 1), 'M5', 0.016, ['deck plate'], PL, cl),
        ]
    return out


link('trunk', [
    # a lobe round each yaw housing: its outer screws stood 11.5 mm off the 340 mm deck (2026-11-05, tools/big_assembly.py)
    dict(plate('deck plate', DECK[0], PL, [(-0.283, 0.168, 0.002), (-0.283, -0.168, 0.002), (0.208, 0.168, 0.002), (0.208, -0.168, 0.002), (0, YY, 0.0815), (0, -YY, 0.0815)], AL, 'struct', axis='z'), material='al7075', fill=0.45, exposed=True),
    metal('upper plate', (-0.260, -0.150, UPPER[0]), (0.065, 0.150, UPPER[1]), fill=0.4),
    # (the front two were at +-0.13, into the yaw stators' rear steps, which stand 5.2 mm above the deck). Square bar, tapped at each end
    *[g_ for i, (x, y) in enumerate(POSTS) for g_ in (
        metal(f'body post {i}', (x - 0.012, y - 0.012, DECK[1]), (x + 0.012, y + 0.012, UPPER[0]), ALD),
        *bolt(f'body post {i} screw low', [(x, y, DECK[0])], (0, 0, 1), 'M5', 0.016, ['deck plate'], PL, f'body post {i}'),
        *bolt(f'body post {i} screw high', [(x, y, UPPER[1])], (0, 0, -1), 'M5', 0.016, ['upper plate'], PL, f'body post {i}'))],
    *neck_frame(),
    # the battery, computer and power board on the deck. Named 24S until 2026-11-05: a 24S pack is about 100 V full, and RobStride's manuals give 60 V as the most these actuators take. 13S is 54.6 V full, as 14S's 58.8 leaves no room
    # for what walking brakes back into the bus. The candidate to bench-test (2026-11-05, docs/big/BOM.md): Dan-Tech's 13S5P soft pack, 280 x 80 x 130, 5.2 kg, across the body between the posts. It has no BMS or case, so what was left of the
    # old box and its 8 kg holds those as an allowance, unchosen, and the walks' total stays the same. Both strapped down: HELD
    box('battery 13S', (-0.215, -0.140, DECK[1]), (-0.135, 0.140, DECK[1] + 0.130), CELL, material='cells', mass=5.2),
    box('battery BMS, fuse and case, allowance', (-0.135, -0.095, DECK[1]), (-0.075, 0.095, UPPER[0] - 0.005), PCB, material='cells', mass=2.8),
    box('computer and power boards', (-0.065, -0.085, DECK[1] + 0.006), (0.025, 0.085, DECK[1] + 0.036), PCB, material='electronics', mass=1.2),      # on standoffs: the yaw stators' rear steps pass 0.8 mm under it
    *[g_ for i, y in enumerate((0.02, -0.02)) for g_ in (
        metal(f'computer standoff {i}', (-0.055, y - 0.005, DECK[1]), (0.015, y + 0.005, DECK[1] + 0.006), ALD),
        *bolt(f'computer standoff {i} screw', [(x, y, DECK[0]) for x in (-0.045, 0.005)], (0, 0, 1), 'M3', 0.012, ['deck plate'], PL, f'computer standoff {i}'))],
    *body_panels(),
    cut(neck_slot, 'body panel FR', 'body panel FL', 'body panel MR', 'body panel ML'),
    # The hip yaw drums reach 8 mm up into the body's lower edge (Fusion's first clash check: a part in one link meeting one in another): a round clearance, 4 mm over the drum
    cut(cyl('L hip yaw drum clearance', (0, YY, ZY + 0.038), 'z', 0.085, 0.02), 'body panel ML'),
    cut(cyl('R hip yaw drum clearance', (0, -YY, ZY + 0.038), 'z', 0.085, 0.02), 'body panel MR'),
    *yaw_out_l, *yaw_out_r,
    *yaw_mount('L', YY),
    *yaw_mount('R', -YY),
    *rs('neck_pitch actuator L', 'RS04', (XN, YN[0], ZN), '-y', '-x'),
    *rs('neck_pitch actuator R', 'RS04', (XN, -YN[0], ZN), '+y', '-x'),
    *rear_mount('neck L', 'RS04', (XN, YN[0], ZN), '-y', '-x', ['neck plate L']),
    *rear_mount('neck R', 'RS04', (XN, -YN[0], ZN), '+y', '-x', ['neck plate R']),
], 'a plate frame in printed panels: the yaw actuators, battery and computer, the neck in front')

# ---------------------------------------------------------------- left leg

XRS = (0.062, 0.062 + H4)                          # roll stators, front and back, outputs inward
XRM = XRS[0] + S4                                  # the shoulders, where the cheeks are, the rear steps through them
ZB = (ZY - 0.0145, ZY - 0.0025)                    # the yaw beam: 2.5 mm under the housing's retainer (it was 0.5)
YF = 'L hip yaw flange'


def turned(name, z0, h, r, part=None, mat='al7075', color=AL):
    return tube(name, (0, YY, z0), (0, 0, 1), h, r, color, 'struct', material=mat, fill=1.0, exposed=True, **({'part': part} if part else {}))


def cheek(s_, sg):
    """A hip roll cheek, hung from the beam by a cleat outside it: 3 M5 up through the cleat into the beam, 2 through the cheek into the cleat. 2026-11-05: it was 1 mm into the beam, held by nothing."""
    c, k = f'L hip roll cheek {s_}', f'L hip roll cleat {s_}'
    x0 = sg * XRM                                  # the cheek's inner face
    xc = sg * (XRM + PL + 0.008)                   # the cleat's middle
    lo = lambda a, b: (min(a, b), max(a, b))
    return [
        metal(c, (lo(x0, x0 + sg * PL)[0], YY - 0.066, ZR - 0.070), (lo(x0, x0 + sg * PL)[1], YY + 0.066, ZB[0]), mat='al7075'),
        metal(k, (lo(xc - 0.008, xc + 0.008)[0], YY - 0.055, ZB[0] - 0.016), (lo(xc - 0.008, xc + 0.008)[1], YY + 0.055, ZB[0]), mat='al7075'),
        *bolt(f'{k} screw', [(xc, YY + dy, ZB[0] - 0.016) for dy in (-0.045, 0.0, 0.045)], (0, 0, 1), 'M5', 0.025, [k], 0.016, 'L hip yaw beam'),
        *bolt(f'{c} screw', [(x0, YY + dy, ZB[0] - 0.008) for dy in (-0.0225, 0.0225)], (sg, 0, 0), 'M5', 0.016, [c], PL, k),
    ]


link('L_hip_yaw_link', [
    # the output's flange: one turned part, a lip under the bearing's inner ring, the seat in it, and a boss up to the output face. Bolted from below, before the beam goes on
    turned('L hip yaw flange lip', ZB[1], ZY + 0.003 - ZB[1], 0.065, YF),
    turned(YF, ZY + 0.003, 0.016, 0.060),
    turned('L hip yaw flange boss', ZY + 0.019, HUB_Y - ZY - 0.019, 0.030, YF),
    # over the inner ring: 6 thick now, tapped for 8 M4 from below through the flange, as nothing can reach above it once the stator is in its housing
    turned('L hip yaw inner retainer', ZY + 0.019, 0.006, 0.0615, mat='al6061', color=ALD),      # 1.5 mm over the inner ring, inside the housing's 124 mm neck
    cut(tube('L hip yaw inner retainer bore', (0, YY, ZY + 0.018), (0, 0, 1), 0.008, 0.030, ALD, 'struct'), 'L hip yaw inner retainer'),
    *[cut(tube(f'L hip yaw retainer screw counterbore {i}', (p[0], p[1], ZB[1] - 0.0005), (0, 0, 1), ZY + 0.008 - ZB[1] + 0.0005, HEAD['M4'][0] + 0.0004, ALD, 'struct'), 'L hip yaw flange lip', YF)
      for i, p in enumerate(ring((0, YY, 0), (0, 0, 1), (1, 0, 0), 0.104, [45 * k for k in range(8)]))],
    *bolt('L hip yaw inner retainer screw', [(p[0], p[1], ZY + 0.008) for p in ring((0, YY, 0), (0, 0, 1), (1, 0, 0), 0.104, [45 * k for k in range(8)])], (0, 0, 1), 'M4', 0.016, [YF], 0.011, 'L hip yaw inner retainer'),
    metal('L hip yaw beam', (-0.130, YY - 0.066, ZB[0]), (0.130, YY + 0.066, ZB[1]), mat='al7075'),
    *output_holes('L hip yaw', 'RS04', (0, YY, HUB_Y), '-z', '+x', HUB_Y - ZB[1], 0.009, ['L hip yaw flange boss', YF, 'L hip yaw flange lip']),
    *joined('L hip yaw beam screw', ring((0, YY, ZB[0]), (0, 0, 1), (1, 0, 0), 0.090, [45 * k + 22.5 for k in range(8)]), (0, 0, 1), 'M5', 0.020, ['L hip yaw beam'], 0.012, 'L hip yaw flange lip', YF),
    *yaw_in_l,
    *cheek('F', 1), *cheek('B', -1),
    *rs('L_hip_roll actuator F', 'RS04', (XRS[0], YY, ZR), '-x', '+z'),
    *rs('L_hip_roll actuator B', 'RS04', (-XRS[0], YY, ZR), '+x', '+z'),
    *rear_mount('L hip roll F', 'RS04', (XRS[0], YY, ZR), '-x', '+z', ['L hip roll cheek F']),
    *rear_mount('L hip roll B', 'RS04', (-XRS[0], YY, ZR), '+x', '+z', ['L hip roll cheek B']),
], 'hangs from the yaw actuator; carries the two roll actuators')
UPRIGHT = [(0, ZP, 0.075), (0.012, ZP + 0.068, 0.002), (0.073, ZP + 0.068, 0.002), (0.073, ZP + 0.060, 0.002)]   # the old ribs are in it: one flat plate
ZBR = ZP + R4 + 0.002                              # the bridge's underside
link('L_hip_roll_link', [
    # the yoke: a plate on each roll output, a box between them, out to the pitch uprights. One machined part with the ring that stands off the inner upright (2026-11-05: a 109 x 90 box had no room for screws into a 150 mm ring)
    disc('L hip roll output F', (0.05825, YY, ZR), 'x', 0.040, 0.0075),
    disc('L hip roll output B', (-0.05825, YY, ZR), 'x', 0.040, 0.0075),
    metal('L hip roll yoke', (-0.0545, YY - 0.045, ZR - 0.045), (0.0545, YOKE_END, ZR + 0.045), ALD, 'al7075', 0.35),
    cut(box('L hip roll yoke hollow', (-0.047, YY - 0.035, ZR - 0.05), (0.047, YY + 0.035, ZR + 0.05), ALD, 'struct'), 'L hip roll yoke'),
    *output_holes('L hip roll F', 'RS04', (XRS[0], YY, ZR), '-x', '+z', 0.015, 0.015, ['L hip roll output F', 'L hip roll yoke']),
    *output_holes('L hip roll B', 'RS04', (-XRS[0], YY, ZR), '+x', '+z', 0.015, 0.015, ['L hip roll output B', 'L hip roll yoke']),
    dict(tube('L hip pitch standoff', (0, YOKE_END, ZP), (0, 1, 0), YI[0] - YOKE_END, 0.075, AL, 'struct', material='al7075', fill=1.0, exposed=True), part='L hip roll yoke'),
    cut(tube('L hip pitch standoff bore', (0, YOKE_END - 0.001, ZP), (0, 1, 0), YI[0] - YOKE_END + 0.002, 0.0585, ALD, 'struct'), 'L hip pitch standoff'),
    # the stator's cable comes off its rear step inside the ring: a slot over it, to the top
    cut(box('L hip pitch standoff cable slot', (-0.008, YOKE_END - 0.001, ZP + 0.050), (0.008, YI[0] + 0.001, ZP + 0.080), ALD, 'struct'), 'L hip pitch standoff'),
    side_plate('L hip pitch upright in', YI[0], PL, UPRIGHT),
    side_plate('L hip pitch upright out', YO[0], PL, UPRIGHT),
    *bolt('L hip pitch upright screw', ring((0, YI[1], ZP), (0, 1, 0), (0, 0, 1), 0.134, (60, 120, 180, 240, 300)), (0, -1, 0), 'M5', 0.020, ['L hip pitch upright in'], PL, 'L hip pitch standoff'),
    metal('L hip pitch bridge', (0.010, YI[1], ZBR), (0.075, YO[0], ZBR + PL), mat='al7075'),
    # M4 into the bridge's 8 mm edges, from outside the uprights
    *bolt('L hip pitch bridge screw in', [(x, YI[0], ZBR + PL / 2) for x in (0.058, 0.068)], (0, 1, 0), 'M4', 0.016, ['L hip pitch upright in'], PL, 'L hip pitch bridge'),
    *bolt('L hip pitch bridge screw out', [(x, YO[1], ZBR + PL / 2) for x in (0.058, 0.068)], (0, -1, 0), 'M4', 0.016, ['L hip pitch upright out'], PL, 'L hip pitch bridge'),
    *rs('L_hip_pitch actuator I', 'RS04', (0, HUB_I, ZP), '+y', '+z'),
    *rs('L_hip_pitch actuator O', 'RS04', (0, HUB_O, ZP), '-y', '+z'),
    *rear_mount('L hip pitch I', 'RS04', (0, HUB_I, ZP), '+y', '+z', ['L hip pitch upright in']),
    *rear_mount('L hip pitch O', 'RS04', (0, HUB_O, ZP), '-y', '+z', ['L hip pitch upright out']),
], 'the yoke: two roll outputs, and the two pitch actuators either side of the thigh')

# the thigh: two inner plates between the hip outputs, two outer plates behind the hip that carry the knee stators; a printed cover over each
TH = [(0.0, ZP, 0.055), (XK, ZK, 0.075), (XQ, ZQ, 0.070)]
THO = [(XQ, ZQ, 0.074), (XK, ZK + 0.055, 0.030)]
kn_in, kn_out = bearing('L knee', 'L_knee', (XK, YT[0] + PL - 0.006, ZK), 'y', '6908', None, None)
kn_in2, kn_out2 = bearing('L knee far', 'L_knee', (XK, YT[1] - PL + 0.006, ZK), 'y', '6908', None, None)
TIES = {'i': [(XQ + 0.068, ZQ - 0.012), (XQ + 0.029, ZQ - 0.0626)], 'o': [(XQ + 0.0636, ZQ - 0.0267), (XQ + 0.045, ZQ - 0.0525)]}   # 69 mm off the knee stator's axis: its case is 60   # the outer ones offset, so the screws from inside the thigh don't meet head to head


def thigh_covers():
    out = []
    pieces = (('hip', [(-0.075, ZP - 0.012, 0.035), (-0.085, ZP + 0.040, 0.022), (-0.150, ZP + 0.040, 0.022), (-0.150, ZP - 0.070, 0.022), (-0.078, ZP - 0.050, 0.030)]),
              ('rear', [(-0.160, ZP + 0.045, 0.030), (-0.285, ZP + 0.045, 0.040), (-0.300, ZP - 0.040, 0.040), (-0.160, ZP - 0.075, 0.030)]),
              ('knee', [(-0.160, ZP - 0.085, 0.020), (-0.300, ZP - 0.050, 0.030), (XK - 0.060, ZK + 0.020, 0.040), (XK + 0.040, ZK + 0.020, 0.040), (-0.115, ZP - 0.080, 0.020)]))
    for t, pts in pieces:
        out.append(dict(plate(f'L thigh cover out {t}', YO[1] + 0.015, 0.003, pts, WHITE, 'decal'), material='petg', exposed=True))      # 15 off the plate: room for the rear step's 5.2 and a 7 mm cable lying on the plate
        # No covers on the thigh's inner side (2026-11-05). Fusion's first clash check found the hip piece 8.4 cm3 into the rear hip roll actuator, and the knee piece 0.16 into the cheek. tools/big_sweep.py then found the
        # other two reaching the body panel (10.7 cm3), cheek, deck and beam as the hip moves. They're 'decal', so the offline checks never saw any of it. Between the legs they show least, and the outer ones sweep clear.
        # The cost: the frame shows from the inside
    # 2026-11-05: the pieces met by overlapping, 33 cm3 at the worst: each is cut back to where the one before it ends, so they meet along a seam
    out.append(cut(plate('L thigh cover seam rear', YO[1] + 0.014, 0.005, pieces[0][1], WHITE, 'decal'), 'L thigh cover out rear', 'L thigh cover out knee'))
    out.append(cut(plate('L thigh cover seam knee', YO[1] + 0.014, 0.005, pieces[1][1], WHITE, 'decal'), 'L thigh cover out knee'))
    # stand-offs from the rear plate: one per piece. Each held by an M4 through the cover and the stand-off into the plate
    for t, (x_, z_) in (('hip', (-0.150, ZP + 0.010)), ('rear', (-0.270, ZQ)), ('knee', (XK, ZK + 0.035))):
        p_ = f'L thigh cover post out {t}'
        out.append(dict(tube(p_, (x_, YO[1], z_), (0, 1, 0), 0.015, 0.005, ALD, 'struct', material='al6061', fill=1.0, exposed=True), spacer=True))     # round: it can turn on its screw
        out += bolt(f'L thigh cover screw {t}', [(x_, YO[1] + 0.018, z_)], (0, -1, 0), 'M4', 0.025, [f'L thigh cover out {t}', p_], 0.018, 'L thigh rear plate out')
    return out


def pin(name, lane, rod, inner, p, group):
    """A rod's 10 mm pin, pressed into its crank or lever, through the rod's bush, and an E-clip past the rod (2026-11-05: the pin stopped 0.3 mm short of the thigh plate, with nothing to hold the rod on).
    inner: the lane on the thigh's inner side, where the rod is at higher y than the crank."""
    if inner:
        y0, y1 = lane, rod + 0.005 + CLIP['past']
        cy = rod + 0.005 + CLIP['gap']
    else:
        y0, y1 = rod - CLIP['past'], lane + 0.006
        cy = rod - CLIP['gap'] - CLIP['t']
    g = dict(axle(name, (p[0], (y0 + y1) / 2, p[1]), 'y', 0.005, y1 - y0))
    c = dict(tube(f'{name} clip (buy) DIN 6799 8', (p[0], cy, p[1]), (0, 1, 0), CLIP['t'], CLIP['r'], DARK, 'decal'), material='steel', exposed=True, linkage=True)
    cb = cut(tube(f'{name} clip bore', (p[0], cy - 0.001, p[1]), (0, 1, 0), CLIP['t'] + 0.002, 0.005, DARK, 'struct'), c['name'])
    if group:
        g, c, cb = dict(g, group=group), dict(c, group=group), dict(cb, group=group)
    return [g, c, cb]


link('L_hip_pitch_link', [
    side_plate('L thigh plate in', YT[0], PL, TH),
    side_plate('L thigh plate out', YT[1] - PL, PL, TH),
    cut(cyl('L thigh bore in', (XK, YT[0] + PL - 0.006, ZK), 'y', BEAR['6908']['D'] / 2, 0.02), 'L thigh plate in'),
    cut(cyl('L thigh bore out', (XK, YT[1] - PL + 0.006, ZK), 'y', BEAR['6908']['D'] / 2, 0.02), 'L thigh plate out'),
    # spacers between the plates, countersunk M5 from outside: flush, as the cranks and rods pass just outside
    metal('L thigh spacer hip', (-0.05, YT[0] + PL, ZP - 0.07), (-0.03, YT[1] - PL, ZP + 0.03), ALD),
    metal('L thigh spacer mid', (XQ - 0.012, YT[0] + PL, ZQ - 0.03), (XQ + 0.012, YT[1] - PL, ZQ + 0.04), ALD),
    *[g_ for sp, pts in (('hip', [(-0.04, ZP - 0.03), (-0.04, ZP - 0.06)]), ('mid', [(XQ, ZQ - 0.018), (XQ, ZQ + 0.028)])) for s_, y_, d_ in (('in', YT[0], 1), ('out', YT[1], -1)) for g_ in
      bolt(f'L thigh spacer {sp} screw {s_}', [(x, y_, z) for x, z in pts], (0, d_, 0), 'M5', 0.016, [f'L thigh plate {s_}'], PL, f'L thigh spacer {sp}', csk=True)],
    tube('L hip pitch output I', (0, HUB_I, ZP), (0, 1, 0), YT[0] - HUB_I, 0.040, AL, 'struct', material='al6061', fill=1.0, exposed=True),
    tube('L hip pitch output O', (0, HUB_O, ZP), (0, -1, 0), HUB_O - YT[1], 0.040, AL, 'struct', material='al6061', fill=1.0, exposed=True),
    *output_holes('L hip pitch I', 'RS04', (0, HUB_I, ZP), '+y', '+z', YT[0] - HUB_I, 0.007, ['L hip pitch output I'], head_h=0.0035),      # low heads, DIN 7984: a full one stands 0.2 mm into the plate
    *output_holes('L hip pitch O', 'RS04', (0, HUB_O, ZP), '-y', '+z', HUB_O - YT[1], 0.007, ['L hip pitch output O'], head_h=0.0035),
    # the thigh's plates on the flanges: from inside the thigh, whose 24 mm takes a 16 mm screw
    *joined('L thigh screw I', ring((0, YT[0] + PL, ZP), (0, -1, 0), (0, 0, 1), 0.060, (0, 45, -45, 180, 135, -135)), (0, -1, 0), 'M5', 0.016, ['L thigh plate in'], PL, 'L hip pitch output I'),
    *joined('L thigh screw O', ring((0, YT[1] - PL, ZP), (0, 1, 0), (0, 0, 1), 0.060, (22.5, -22.5, 90, 157.5, -157.5)), (0, 1, 0), 'M5', 0.016, ['L thigh plate out'], PL, 'L hip pitch output O'),   # between the inner ones, so a key reaches each
    side_plate('L thigh rear plate in', YI[0], PL, THO),
    side_plate('L thigh rear plate out', YO[0], PL, THO),
    # ties between the rear plates and the thigh's: square bar tapped at both ends
    *[g_ for n, (ya, yb), rp, tp, ro, ti in (('i', (YI[1], YT[0]), 'L thigh rear plate in', 'L thigh plate in', YI[0], YT[0] + PL), ('o', (YT[1], YO[0]), 'L thigh rear plate out', 'L thigh plate out', YO[1], YT[1] - PL)) for i, (x, z) in enumerate(TIES[n]) for g_ in (
        metal(f'L thigh tie {i}{n}', (x - 0.006, ya, z - 0.006), (x + 0.006, yb, z + 0.006), ALD),
        *bolt(f'L thigh tie {i}{n} screw rear', [(x, ro, z)], (0, 1 if n == 'i' else -1, 0), 'M5', 0.016, [rp], PL, f'L thigh tie {i}{n}'),
        *bolt(f'L thigh tie {i}{n} screw front', [(x, ti, z)], (0, -1 if n == 'i' else 1, 0), 'M5', 0.016, [tp], PL, f'L thigh tie {i}{n}'))],
    *rs('L_knee actuator I', 'RS04', (XQ, HUB_I, ZQ), '+y', '+x'),
    *rs('L_knee actuator O', 'RS04', (XQ, HUB_O, ZQ), '-y', '+x'),
    *rear_mount('L knee I', 'RS04', (XQ, HUB_I, ZQ), '+y', '+x', ['L thigh rear plate in']),
    *rear_mount('L knee O', 'RS04', (XQ, HUB_O, ZQ), '-y', '+x', ['L thigh rear plate out']),
    # What the four-bar moves: in the spec they're drawn at knee angle 0 in this link, as one body can't swing within a link, and 'group' says which moving part each belongs to. Fusion makes each group its own moving link (the spec's 'mech').
    dict(side_plate('L knee crank I', LANE_I, 0.006, [(XQ, ZQ, 0.028), (PIN_A[0], PIN_A[1], 0.014)]), group='L_knee_crank_I'),
    dict(side_plate('L knee crank O', LANE_O, 0.006, [(XQ, ZQ, 0.028), (PIN_A[0], PIN_A[1], 0.014)]), group='L_knee_crank_O'),
    dict(seat('L knee crank I', (XQ, YSI[1], ZQ), 'y', 0.02), group='L_knee_crank_I'),
    dict(seat('L knee crank O', (XQ, YSO[0], ZQ), 'y', 0.02), group='L_knee_crank_O'),
    *pin('L knee crank pin I', LANE_I, ROD_I, True, PIN_A, 'L_knee_crank_I'),
    *pin('L knee crank pin O', LANE_O, ROD_O, False, PIN_A, 'L_knee_crank_O'),
    *[dict(cut(cyl(f'L knee crank pin hole {s_}', (PIN_A[0], y_ + 0.003, PIN_A[1]), 'y', 0.005, 0.008), f'L knee crank {s_}'), group=f'L_knee_crank_{s_}') for s_, y_ in (('I', LANE_I), ('O', LANE_O))],
    *output_holes('L knee I', 'RS04', (XQ, HUB_I, ZQ), '+y', '+x', 0.006, 0.005, ['L knee crank I'], group='L_knee_crank_I'),
    *output_holes('L knee O', 'RS04', (XQ, HUB_O, ZQ), '-y', '+x', 0.006, 0.005, ['L knee crank O'], group='L_knee_crank_O'),
    *kn_out, *kn_out2,
    # the rods, with a 14 x 10 bronze bush at each end. Decal: they swing with the four-bar, so the offline sweeps can't test them. tools/big_sweep.py does
    *[g_ for s_, y_ in (('I', ROD_I), ('O', ROD_O)) for g_ in (
        dict(plate(f'L knee rod {s_}', y_, 0.005, [(PIN_A[0], PIN_A[1], 0.011), (PIN_B[0], PIN_B[1], 0.011)], AL, 'decal'), material='al7075', exposed=True, linkage=True, group=f'L_knee_rod_{s_}'),
        *[dict(cut(cyl(f'L knee rod {s_} hole {e}', (p_[0], y_ + 0.0025, p_[1]), 'y', 0.007, 0.007), f'L knee rod {s_}'), group=f'L_knee_rod_{s_}') for e, p_ in (('A', PIN_A), ('B', PIN_B))],
        *[dict(cyl(f'L knee rod {s_} bush {e}', (p_[0], y_ + 0.0025, p_[1]), 'y', 0.007, 0.005, '#B08D57', 'decal'), material='steel', exposed=True, linkage=True, group=f'L_knee_rod_{s_}') for e, p_ in (('A', PIN_A), ('B', PIN_B))],
        *[dict(cut(cyl(f'L knee rod {s_} bush bore {e}', (p_[0], y_ + 0.0025, p_[1]), 'y', 0.00505, 0.007), f'L knee rod {s_} bush {e}'), group=f'L_knee_rod_{s_}') for e, p_ in (('A', PIN_A), ('B', PIN_B))],
    )],
    *thigh_covers(),
], 'two plates between the hip outputs, two behind them for the knee actuators, crank and rod down to the knee')
SX_, SY_ = 0.055, 0.045                            # the shin box, half sizes
SHIN_TOP = ZK - 0.100                              # clear of the thigh plates (75 mm) and knee sprockets as the knee turns
CAPZ = (ZA + R4 + 0.002, ZA + R4 + 0.002 + PL)     # the cap under the shin's tube, the ankle hung from it


def shin_cover():
    secs = [(ZA + R4 + 0.012, XK, YC, SX_ + 0.016, SY_ + 0.016, 4.0), (SHIN_TOP + 0.002, XK, YC, SX_ + 0.012, SY_ + 0.012, 4.0)]
    return [dict(loft(f'L shin cover {t}', secs, DARK, 'shell', open=('top', 'bottom'), clip=[((XK, 0, 0), (sg, 0, 0))]), material='petg') for t, sg in (('front', 1), ('back', -1))]


def shin_box():
    """The shin: a 5 mm 6061 sheet folded into a tube, its seam welded (one part); a plug at its top, carrying the spacer between the knee hubs, and a cap under it, carrying the ankle. M4 through the walls into the plug's edges, and into a cleat
    inside each end wall at the bottom, which M4 from under the cap reach. 2026-11-05: walls, top, cap and hubs were drawn 0.5 mm into each other, and the hubs ran through the top plate."""
    T_ = 'L shin tube'
    w = lambda n, lo, hi: dict(metal(n, lo, hi), part=T_)
    out = [
        w('L shin wall front', (XK + SX_ - 0.005, YC - SY_, CAPZ[1]), (XK + SX_, YC + SY_, SHIN_TOP)),
        w('L shin wall back', (XK - SX_, YC - SY_, CAPZ[1]), (XK - SX_ + 0.005, YC + SY_, SHIN_TOP)),
        w('L shin wall in', (XK - SX_ + 0.005, YC - SY_, CAPZ[1]), (XK + SX_ - 0.005, YC - SY_ + 0.005, SHIN_TOP)),
        w('L shin wall out', (XK - SX_ + 0.005, YC + SY_ - 0.005, CAPZ[1]), (XK + SX_ - 0.005, YC + SY_, SHIN_TOP)),
        metal('L shin top', (XK - SX_ + 0.005, YC - SY_ + 0.005, SHIN_TOP - PL), (XK + SX_ - 0.005, YC + SY_ - 0.005, SHIN_TOP), mat='al7075'),
        metal('L shin cap', (XK - SX_ - 0.008, YC - SY_ - 0.002, CAPZ[0]), (XK + SX_ + 0.008, YC + SY_ + 0.002, CAPZ[1]), mat='al7075'),
    ]
    zt = SHIN_TOP - PL / 2
    for s_, sg in (('front', 1), ('back', -1)):
        out += bolt(f'L shin top screw {s_}', [(XK + sg * SX_, YC + dy, zt) for dy in (-0.02, 0.02)], (-sg, 0, 0), 'M4', 0.012, [f'L shin wall {s_}'], 0.005, 'L shin top')
        cl = f'L shin cleat {s_}'
        x0 = XK + sg * (SX_ - 0.005)
        out.append(metal(cl, (min(x0, x0 - sg * 0.010), YC - 0.035, CAPZ[1]), (max(x0, x0 - sg * 0.010), YC + 0.035, CAPZ[1] + 0.010), ALD))
        out += bolt(f'{cl} wall screw', [(XK + sg * SX_, YC + dy, CAPZ[1] + 0.005) for dy in (-0.02, 0.02)], (-sg, 0, 0), 'M4', 0.012, [f'L shin wall {s_}'], 0.005, cl)
        out += bolt(f'{cl} cap screw', [(x0 - sg * 0.005, YC + 0.028, CAPZ[0])], (0, 0, 1), 'M4', 0.016, ['L shin cap'], PL, cl)     # one: under the rest, the ankle's stator and plates leave no room
    for s_, sg in (('in', -1), ('out', 1)):
        out += bolt(f'L shin top screw {s_}', [(XK + dx, YC + sg * SY_, zt) for dx in (-0.025, 0.025)], (0, -sg, 0), 'M4', 0.012, [f'L shin wall {s_}'], 0.005, 'L shin top')
    return out


ANK = (XK, ZA)
link('L_knee_link', [
    axle('L knee shaft', (XK, YC, ZK), 'y', 0.020, YT[1] - YT[0] + 0.032),
    side_plate('L knee lever I', LANE_I, 0.006, [(XK, ZK, 0.030), (PIN_B[0], PIN_B[1], 0.014)]),
    side_plate('L knee lever O', LANE_O, 0.006, [(XK, ZK, 0.030), (PIN_B[0], PIN_B[1], 0.014)]),
    *pin('L knee lever pin I', LANE_I, ROD_I, True, PIN_B, None),
    *pin('L knee lever pin O', LANE_O, ROD_O, False, PIN_B, None),
    *[cut(cyl(f'L knee lever pin hole {s_}', (PIN_B[0], y_ + 0.003, PIN_B[1]), 'y', 0.005, 0.008), f'L knee lever {s_}') for s_, y_ in (('I', LANE_I), ('O', LANE_O))],
    cut(cyl('L knee shaft bore', (XK, YC, ZK), 'y', 0.020, YT[1] - YT[0] + 0.034), 'L knee lever I', 'L knee lever O', 'L shin hub in', 'L shin hub out'),
    # the hubs stand on the shin's top plug, flat along it: one spacer between them, bolted down into the plug
    side_plate('L shin hub in', YC - 0.0115, 0.006, [(XK, ZK, 0.030), (XK - 0.040, SHIN_TOP + 0.002, 0.002), (XK + 0.040, SHIN_TOP + 0.002, 0.002)], AL, 'al7075'),
    side_plate('L shin hub out', YC + 0.0055, 0.006, [(XK, ZK, 0.030), (XK - 0.040, SHIN_TOP + 0.002, 0.002), (XK + 0.040, SHIN_TOP + 0.002, 0.002)], AL, 'al7075'),
    metal('L shin spacer', (XK - 0.015, YC - 0.0055, SHIN_TOP), (XK + 0.015, YC + 0.0055, ZK - 0.03), ALD),
    *bolt('L shin hub screw', [(XK, YC - 0.0115, z) for z in (ZK - 0.045, ZK - 0.080)], (0, 1, 0), 'M4', 0.022, ['L shin hub in', 'L shin spacer'], 0.017, 'L shin hub out', csk=True),
    *bolt('L shin spacer screw', [(XK + dx, YC, SHIN_TOP - PL) for dx in (-0.008, 0.008)], (0, 0, 1), 'M5', 0.020, ['L shin top'], PL, 'L shin spacer'),
    *shin_box(),
    *shin_cover(),
    # under the cap: the plate the stub and its bearing are on, a ring, and the plate on the stator's shoulder, through which the rear step goes. 5 M4 through the first two into the third, and M4 down through the cap into it.
    # Their tops are cut flat: the shoulder plate's against the cap, the others 1 mm under it. The cable goes up from a window into the shin, through the cap and the shin's top
    side_plate('L ankle plate in', YA_S[0] - PL, PL, [(XK, ZA, 0.072)]),
    tube('L ankle spacer', (XK, YA_S[0], ZA), (0, 1, 0), YA_M[0] - YA_S[0], 0.072, AL, 'struct', material='al7075', fill=1.0, exposed=True),
    cut(tube('L ankle spacer bore', (XK, YA_S[0] - 0.001, ZA), (0, 1, 0), YA_M[0] - YA_S[0] + 0.002, 0.0585, ALD, 'struct'), 'L ankle spacer'),
    side_plate('L ankle mount plate', YA_M[0], PL, [(XK, ZA, 0.072)]),
    cut(box('L ankle plates top', (XK - 0.08, YA_S[0] - PL - 0.001, CAPZ[0] - 0.001), (XK + 0.08, YA_M[0] + 0.0001, CAPZ[0] + 0.02), ALD, 'struct'), 'L ankle plate in', 'L ankle spacer'),
    cut(box('L ankle mount plate top', (XK - 0.08, YA_M[0] - 0.001, CAPZ[0]), (XK + 0.08, YA_M[1] + 0.001, CAPZ[0] + 0.02), ALD, 'struct'), 'L ankle mount plate'),
    *bolt('L ankle stack screw', ring((XK, YA_S[0] - PL, ZA), (0, 1, 0), (0, 0, 1), 0.130, (60, 120, 180, 240, 300)), (0, 1, 0), 'M4', 0.022, ['L ankle plate in', 'L ankle spacer'], PL + YA_M[0] - YA_S[0], 'L ankle mount plate'),
    *bolt('L ankle cap screw', [(XK + dx, (YA_M[0] + YA_M[1]) / 2, CAPZ[1]) for dx in (-0.026, 0.026)], (0, 0, -1), 'M4', 0.016, ['L shin cap'], PL, 'L ankle mount plate'),
    *rs('L_ankle_pitch actuator', 'RS04', (XK, YA_S[1], ZA), '+y', '+z'),
    *rear_mount('L ankle', 'RS04', (XK, YA_S[1], ZA), '+y', '+z', ['L ankle mount plate']),
    cut(tube('L ankle cable hole', (XK, YA_S[1] - 0.0458, ZA + 0.060), (0, 0, 1), SHIN_TOP - ZA - 0.058, 0.0075, ALD, 'struct'), 'L shin cap', 'L shin top'),
    # the stub: 3 M4 from its end, in counterbores, into the plate. Its bearing's inner ring is pressed on
    axle('L ankle stub', (XK, YA_S[0] - PL - 0.011, ZA), 'y', 0.015, 0.022),
    *[cut(tube(f'L ankle stub counterbore {i}', (p[0], YA_S[0] - PL - 0.0225, p[2]), (0, 1, 0), 0.0165, HEAD['M4'][0] + 0.0004, ALD, 'struct'), 'L ankle stub') for i, p in enumerate(ring((XK, 0, ZA), (0, 1, 0), (0, 0, 1), 0.018, (0, 120, -120)))],
    *bolt('L ankle stub screw', [(p[0], YA_S[0] - PL - 0.006, p[2]) for p in ring((XK, 0, ZA), (0, 1, 0), (0, 0, 1), 0.018, (0, 120, -120))], (0, 1, 0), 'M4', 0.012, ['L ankle stub'], 0.006, 'L ankle plate in'),
], 'the knee shaft on two bearings, down to the ankle actuator')
FX, FY = FOOT['x'], FOOT['y']
FC = (sum(FX) / 2, sum(FY) / 2)
SA, SB = (FX[1] - FX[0]) / 2, (FY[1] - FY[0]) / 2
an_in, an_out = bearing('L ankle', 'L_ankle_pitch', (XK, YA_S[0] - PL - 0.014, ZA), 'y', '6206', None, None)


def foot_part(name, zs, x0, x1, color, mat, **kw):
    return dict(loft(name, [(z, FC[0], FC[1], SA - d, SB - d, n) for z, d, n in zs], color, 'struct', open=(), clip=[((x0, 0, 0), (1, 0, 0)), ((x1, 0, 0), (-1, 0, 0))]), material=mat, exposed=True, **kw)


FP = (FOOT['plate'] - 0.0065, FOOT['plate'] + 0.0065)   # the foot plate
SOLE = [(0.0, 0.010, 3.0), (0.006, 0.003, 4.0), (FP[0], 0.0, 4.5)]
UP = [(FP[1], 0.0, 4.5), (FOOT['plate'] + 0.030, 0.0, 4.5), (FOOT['plate'] + 0.046, 0.012, 3.5)]
SPLIT = FC[0] + 0.015
EAR = [(XK, ZA, 0.040), (XK - 0.075, FP[1] + 0.006, 0.006), (XK + 0.085, FP[1] + 0.006, 0.006)]   # flat on the foot plate
EAR_O, EAR_I = YA_S[1] + PL + 0.004, YA_S[0] - PL - 0.022 - PL
link('L_foot', [
    metal('L foot plate', (FX[0] + 0.012, FY[0] + 0.012, FP[0]), (FX[1] - 0.012, FY[1] - 0.012, FP[1]), mat='al7075', fill=0.6),
    side_plate('L ankle ear out', EAR_O, PL, EAR),
    side_plate('L ankle ear in', EAR_I, PL, EAR),
    *[g_ for s_, y_ in (('out', EAR_O), ('in', EAR_I)) for g_ in bolt(f'L ankle ear {s_} foot screw', [(XK + dx, y_ + PL / 2, FP[0]) for dx in (-0.070, 0.005, 0.080)], (0, 0, 1), 'M4', 0.020, ['L foot plate'], FP[1] - FP[0], f'L ankle ear {s_}')],
    disc('L ankle output', (XK, YA_S[1] + 0.006, ZA), 'y', 0.040, 0.012),
    *output_holes('L ankle', 'RS04', (XK, YA_S[1], ZA), '+y', '+z', 0.012, 0.007, ['L ankle output']),
    *joined('L ankle ear screw', ring((XK, EAR_O + PL, ZA), (0, -1, 0), (0, 0, 1), 0.064, (0, 60, -60, 120, -120, 180)), (0, -1, 0), 'M5', 0.016, ['L ankle ear out'], PL, 'L ankle output'),
    metal('L ankle bearing housing', (XK - 0.040, EAR_I + PL, ZA - 0.040), (XK + 0.040, YA_S[0] - PL - 0.0065, ZA + 0.040), mat='al7075', fill=0.6),
    cut(cyl('L ankle housing bore', (XK, YA_S[0] - PL - 0.014, ZA), 'y', BEAR['6206']['D'] / 2, 0.03), 'L ankle bearing housing'),
    *bolt('L ankle housing screw', [(XK - 0.034, EAR_I, ZA - 0.012), (XK + 0.034, EAR_I, ZA - 0.012), (XK, EAR_I, ZA - 0.0355)], (0, 1, 0), 'M5', 0.016, ['L ankle ear in'], PL, 'L ankle bearing housing'),
    foot_part('L sole rear', SOLE, FX[0] - 0.01, SPLIT, YELLOW, 'tpu'),
    foot_part('L sole front', SOLE, SPLIT, FX[1] + 0.01, YELLOW, 'tpu', contact=dict(c=[FC[0], FC[1], 0.002], size=[2 * SA - 0.03, 2 * SB - 0.02, 0.004])),
    foot_part('L shoe rear', UP, FX[0] - 0.01, SPLIT, ORANGE, 'petg'),
    foot_part('L shoe front', UP, SPLIT, FX[1] + 0.01, ORANGE, 'petg'),
    cut(box('L shoe opening', (XK - 0.085, YA_S[0] - PL - 0.03, FOOT['plate'] + 0.0064), (XK + 0.095, YA_S[1] + 2 * PL + 0.012, 0.2), ORANGE, 'shell'), 'L shoe rear', 'L shoe front'),
    # the shoe's pieces: M4 up through the plate into heat-set inserts, before the soles are bonded on
    *[g_ for t, x in (('rear', FX[0] + 0.020), ('front', FX[1] - 0.030)) for g_ in bolt(f'L shoe {t} screw', [(x, YC + dy, FP[0]) for dy in (-0.07, 0.07)], (0, 0, 1), 'M4', 0.020, ['L foot plate'], FP[1] - FP[0], f'L shoe {t}')],
    # a pocket in the sole over each screw head under the plate
    *[cut(tube(f'L sole pocket {i}', (x, y, FP[0] - 0.0045), (0, 0, 1), 0.0050, 0.0042, ALD, 'struct'), 'L sole rear', 'L sole front')
      for i, (x, y) in enumerate([(XK + dx, y_ + PL / 2) for y_ in (EAR_O, EAR_I) for dx in (-0.070, 0.005, 0.080)] + [(x_, YC + dy) for x_ in (FX[0] + 0.020, FX[1] - 0.030) for dy in (-0.07, 0.07)])],
    *an_out,
], 'an aluminium plate on two ears; soles and shoe printed in two')
# the bearings' inner rings on the stub and shaft
LINKS['L_knee_link']['geoms'] += kn_in + kn_in2 + an_in
joint('L_hip_yaw', 'trunk', 'L_hip_yaw_link', (0, YY, ZY), 'z', (-0.45, 0.45), 'RS04', out=-1, act_c=(0, YY, HUB_Y + 0.02), interface=['L_hip_yaw actuator (buy) RS04', 'L hip yaw flange boss', 'L hip yaw inner retainer'])
joint('L_hip_roll', 'L_hip_yaw_link', 'L_hip_roll_link', (0.062, YY, ZR), 'x', (-0.33, 0.33), 'RS04', count=2, out=-1, act_c=((XRS[0] + XRS[1]) / 2, YY, ZR), interface=['L_hip_roll actuator F (buy) RS04', 'L_hip_roll actuator B (buy) RS04', 'L hip roll output F', 'L hip roll output B'])
joint('L_hip_pitch', 'L_hip_roll_link', 'L_hip_pitch_link', (0, YSI[1], ZP), 'y', (-1.57, 0.43), 'RS04', count=2, out=1, act_c=(0, sum(YSI) / 2, ZP), interface=['L_hip_pitch actuator I (buy) RS04', 'L_hip_pitch actuator O (buy) RS04', 'L hip pitch output I', 'L hip pitch output O'])
joint('L_knee', 'L_hip_pitch_link', 'L_knee_link', (XK, YC, ZK), 'y', KNEE_RANGE, 'RS04', count=2, ratio=KNEE_RATIO, out=1, act_c=(XQ, sum(YSI) / 2, ZQ), remote=True)
joint('L_ankle_pitch', 'L_knee_link', 'L_foot', (XK, YA_S[1], ZA), 'y', (-1.2, 1.2), 'RS04', out=1, act_c=(XK, sum(YA_S) / 2, ZA), interface=['L_ankle_pitch actuator (buy) RS04', 'L ankle output', 'L ankle stub', 'L ankle bearing housing'])

# ---------------------------------------------------------------- neck and head

def rs03_front(name, hub, out, g, base):
    """An RS03's 8 free front holes (2026-11-05: only 4 rear ones were used, carrying the head): a 6 mm ring on its front face, round its output, 8 countersunk M4 into RobStride's threads (11 deep: 8 used),
    and 4 posts back past the case, between its lobes, to the plate its rear is on: M5 from behind that plate, countersunk M4 through the ring."""
    R, a, o, gv = RS['RS03'], ACT['RS03'], vec(out), vec(g)
    back = tuple(-v for v in o)
    face = along(hub, back, R['front'])            # the stator's front face
    rg = f'{name} front ring'
    top = along(face, o, 0.006)
    base_face = along(hub, back, R['shoulder'])     # the plate's face, where the posts stand
    L_post = R['shoulder'] - R['front']
    w = cross(o, gv)
    out_ = [tube(rg, face, o, 0.006, 0.072, AL, 'struct', material='al7075', fill=1.0, exposed=True),
            cut(tube(f'{rg} bore', along(face, back, 0.001), o, 0.008, 0.042, ALD, 'struct'), rg),
            *bolt(f'{rg} screw', ring(top, back, gv, a['mount_pcd'], [22.5 + 45 * k for k in range(8)]), back, 'M4', 0.014, [rg], 0.006, None, csk=True, tap_hub=list(hub))]
    for k, ang in enumerate((45, 135, -45, -135)):
        c = ring(face, o, gv, 0.124, [ang])[0]
        pn = f'{name} front post {k}'
        lo = [c[i] - 0.006 * (abs(gv[i]) + abs(w[i])) + min(0, back[i] * L_post) for i in range(3)]
        hi = [c[i] + 0.006 * (abs(gv[i]) + abs(w[i])) + max(0, back[i] * L_post) for i in range(3)]
        out_.append(metal(pn, lo, hi, ALD))
        out_ += bolt(f'{pn} screw top', [along(c, o, 0.006)], back, 'M4', 0.014, [rg], 0.006, pn, csk=True)
        out_ += bolt(f'{pn} screw base', [along(c, back, L_post + PL)], o, 'M5', 0.020, [base], PL, pn)
    return out_


def head_rail(s_, sg):
    """A rail along the head's side: through a notch in the main frame, screwed into it sideways, and into the rear and front frames' faces. A lug for the beak's hinge on its side, countersunk, and the beak's mount screwed into it from outside."""
    r, lug = f'head rail {s_}', f'head beak lug {s_}'
    yi, yo = sg * 0.188, sg * 0.200
    lo = lambda a, b: (min(a, b), max(a, b))
    return [
        metal(r, (HBACK + 0.068, lo(yi, yo)[0], ZHEAD + 0.02), (HFRONT - 0.048, lo(yi, yo)[1], ZHEAD + 0.04), ALD, 'al6061', 0.3),
        cut(box(f'{r} notch', (XFR - 0.001, lo(yi, yo + sg * 0.001)[0], ZHEAD + 0.02), (XFR + PL + 0.001, lo(yi, yo + sg * 0.001)[1], ZHEAD + 0.04), ALD, 'struct'), 'head main frame'),
        *bolt(f'{r} screw main', [(XFR + PL / 2, yo, ZHEAD + z) for z in (0.025, 0.035)], (0, -sg, 0), 'M4', 0.020, [r], 0.012, 'head main frame'),
        *bolt(f'{r} screw rear', [(HBACK + 0.06, sg * 0.194, ZHEAD + z) for z in (0.025, 0.035)], (1, 0, 0), 'M4', 0.016, ['head rear frame'], PL, r),
        *bolt(f'{r} screw front', [(HFRONT - 0.04, sg * 0.194, ZHEAD + z) for z in (0.025, 0.035)], (-1, 0, 0), 'M4', 0.016, ['head front frame'], PL, r),
        metal(lug, (JX - 0.02, lo(yo, yo + sg * 0.0045)[0], JZ - 0.02), (JX + 0.02, lo(yo, yo + sg * 0.0045)[1], JZ + 0.02), mat='al6061'),
        *bolt(f'{lug} screw', [(JX + dx, yo + sg * 0.0045, ZHEAD + 0.03) for dx in (-0.01, 0.01)], (0, -sg, 0), 'M4', 0.012, [lug], 0.0045, r, csk=True),
        *bolt(f'head beak mount screw {s_}', [(HBACK + x, yo, ZHEAD + 0.032) for x in (0.125, 0.155)], (0, -sg, 0), 'M4', 0.020, [r], 0.012, 'head beak mount'),
    ]


ZH = ZN + 0.340                                    # head pitch
NT = (-0.030, 0.030)                               # the neck's tube, 60 wide in y
XHP = XN
ZHS = ZN + 0.135                                   # the head pitch stators, low on the neck, for the head's reach
HPH = YN[0] + 0.012                                # its output face, on the sprocket
hp_in1, hp_out1 = bearing('head pitch L', 'head_pitch', (XN, 0.022, ZH), 'y', '6908', None, None)
hp_in2, hp_out2 = bearing('head pitch R', 'head_pitch', (XN, -0.022, ZH), 'y', '6908', None, None)
hp_outer = hp_out1 + hp_out2
link('neck_link', [
    # bosses on the spine's sides, each with its 5 mm wall: screws from inside, through a window across the spine
    disc('neck output L', (XN, (NT[1] + YN[0]) / 2, ZN), 'y', 0.040, YN[0] - NT[1]),
    disc('neck output R', (XN, -(NT[1] + YN[0]) / 2, ZN), 'y', 0.040, YN[0] - NT[1]),
    metal('neck spine', (XN - 0.040, NT[0], ZN - 0.050), (XN + 0.040, NT[1], ZH + 0.040), ALD, 'al7075', 0.25),
    cut(cyl('neck spine bore', (XN, 0, ZH), 'y', BEAR['6908']['D'] / 2, 0.08), 'neck spine'),
    cut(box('neck spine window', (XN - 0.05, -0.025, ZN - 0.03), (XN + 0.05, 0.025, ZN + 0.03), ALD, 'struct'), 'neck spine'),
    *output_holes('neck L', 'RS04', (XN, YN[0], ZN), '-y', '-x', 0.009, 0.009, ['neck output L', 'neck spine']),
    *output_holes('neck R', 'RS04', (XN, -YN[0], ZN), '+y', '-x', 0.009, 0.009, ['neck output R', 'neck spine']),
    *hp_outer,
    side_plate('head pitch stator plate L', HPH + S4, PL, [(XN, ZHS, 0.072)]),
    *rs('head_pitch actuator L', 'RS04', (XN, HPH, ZHS), '-y', '-x'),
    *rear_mount('head pitch L', 'RS04', (XN, HPH, ZHS), '-y', '-x', ['head pitch stator plate L']),
    dict(axle('head pitch drive sprocket L (buy) 13T #40', (XN, 0.040, ZHS), 'y', PCD13 / 2, 0.012), linkage=True),   # on the output, so it turns by itself
    *output_holes('head pitch', 'RS04', (XN, HPH, ZHS), '-y', '-x', 0.012, 0.007, ['head pitch drive sprocket L (buy) 13T #40']),
    *chain('head pitch chain L (buy) #40', (XN, ZHS), (XN, ZH), 0.038, PCD13 / 2, PCD25 / 2),
    # the stator's plate on a tie over the stator, 40 wide between the chain's strands, held by 3 M5 x 80 through the plate and the tie into the spine (2026-11-05: 24 wide, 0.5 mm into each, held by nothing).
    # One tie only: under the stator, the trunk's neck plates sweep round the neck's axis, and in front and behind, the spine is too narrow to reach
    metal('head pitch stator tie L', (XN - 0.020, NT[1], ZHS + R4 + 0.001), (XN + 0.020, HPH + S4, ZHS + R4 + 0.011), ALD),
    *bolt('head pitch stator tie L screw', [(XN + dx, HPH + S4 + PL, ZHS + R4 + 0.006) for dx in (-0.0135, 0.0, 0.0135)], (0, -1, 0), 'M5', 0.080, ['head pitch stator plate L', 'head pitch stator tie L'], PL + HPH + S4 - NT[1], 'neck spine'),
], 'a pocketed spine on the two neck outputs; the head pitch actuator low on it, on chain to the top')
ZHY = ZH + 0.115
ZHEAD = ZHY + 0.045                                # the hood's lower edge
HK = 6.95                                          # the small duck's head, at this height: 0.85 m long
HFRONT, HCX = XN + 0.040 + 0.60, XN
HOOD_MD = T.HOOD_MD
HBACK = HFRONT - HK * HOOD_MD[0][0]
link('head_pitch_link', [
    dict(axle('head pitch shaft', (XN, 0, ZH), 'y', 0.020, 0.150), fill=0.45),      # hollow: 40 mm outside, 30 inside
    *hp_in1, *hp_in2,
    cut(cyl('head pitch shaft bore', (XN, 0, ZH), 'y', 0.020, 0.152), 'head pitch yoke', 'head pitch yoke R', 'head pitch sprocket L (buy) 25T #40'),
    # (the yokes were at 43-51: now clear of the holes for the RS03's cap screws at y = 45.3. And the RS03's hub is 14 mm nearer its shoulder than the old cylinder's end, so the bridge went up as much)
    metal('head pitch yoke', (XN - 0.070, 0.050, ZH - 0.040), (XN + 0.070, 0.058, ZHY - 0.033), mat='al7075'),
    metal('head pitch yoke R', (XN - 0.070, -0.058, ZH - 0.040), (XN + 0.070, -0.050, ZHY - 0.033), mat='al7075'),
    metal('head pitch bridge', (XN - 0.070, -0.058, ZHY - 0.033), (XN + 0.070, 0.058, ZHY - 0.025), mat='al7075'),
    *[g_ for y_, n in ((0.054, 'head pitch yoke'), (-0.054, 'head pitch yoke R')) for g_ in bolt(f'{n} screw', [(XN + dx, y_, ZHY - 0.025) for dx in (-0.055, 0.055)], (0, 0, -1), 'M4', 0.016, ['head pitch bridge'], PL, n)],
    *rs03_front('head yaw', (XN, 0, ZHY + 0.017), '+z', '+x', 'head pitch bridge'),
    dict(axle('head pitch sprocket L (buy) 25T #40', (XN, 0.038, ZH), 'y', PCD25 / 2, 0.008), fill=0.5),   # with lightening holes
    *rs('head_yaw actuator', 'RS03', (XN, 0, ZHY + 0.017), '+z', '+x'),
    *rear_mount('head yaw', 'RS03', (XN, 0, ZHY + 0.017), '+z', '+x', ['head pitch bridge']),
], 'the head pitch shaft on bearings at the neck top; the head yaw actuator on the bridge')
XHR = XN + 0.14
link('head_yaw_link', [
    disc('head yaw output', (XN, 0, ZHY + 0.021), 'z', 0.040, 0.008),
    metal('head yaw plate', (XN - 0.06, -0.06, ZHY + 0.025), (XHR + 0.016, 0.06, ZHY + 0.033), mat='al7075'),
    *output_holes('head yaw', 'RS03', (XN, 0, ZHY + 0.017), '+z', '+x', 0.008, 0.004, ['head yaw output']),
    *joined('head yaw plate screw', ring((XN, 0, ZHY + 0.033), (0, 0, -1), (1, 0, 0), 0.064, (0, 60, -60, 120, -120, 180)), (0, 0, -1), 'M4', 0.014, ['head yaw plate'], PL, 'head yaw output'),
    metal('head roll cradle', (XHR + 0.008, -0.06, ZHY + 0.033), (XHR + 0.016, 0.06, ZHY + 0.160), mat='al7075'),
    *bolt('head roll cradle screw', [(XHR + 0.012, y_, ZHY + 0.025) for y_ in (-0.04, 0.0, 0.04)], (0, 0, 1), 'M4', 0.016, ['head yaw plate'], PL, 'head roll cradle'),
    *rs03_front('head roll', (XHR + 0.058, 0, ZHY + 0.095), '+x', '+y', 'head roll cradle'),
    *rs('head_roll actuator', 'RS03', (XHR + 0.058, 0, ZHY + 0.095), '+x', '+y'),
    *rear_mount('head roll', 'RS03', (XHR + 0.058, 0, ZHY + 0.095), '+x', '+y', ['head roll cradle']),
], 'turns the head; carries the head roll actuator')
XFR = XHR + 0.068                                  # the head's main frame, on the roll output


def hood_panels():
    """The hood's arches, cut along x and across y and z so each panel prints on a 256 mm bed."""
    z0 = ZHEAD + 0.035
    s = [(HBACK - 0.03, z0, 0, 0.04, 0.10, 2.4)]
    for k, (d, h, w) in enumerate(HOOD_MD):
        s.append((HFRONT - HK * d, z0, 0, HK * h, HK * w, 2.4 + 0.6 * k / (len(HOOD_MD) - 1)))
    s.append((HFRONT + 0.08,) + s[-1][1:])
    xs = [HBACK - 0.04, HFRONT - 0.64, HFRONT - 0.42, HFRONT - 0.21, HFRONT + 0.005]
    ys = [-0.34, -0.12, 0.12, 0.34]
    zc = z0 + 0.135
    out = []
    for i in range(len(xs) - 1):
        for j in range(3):
            pl = [((xs[i], 0, 0), (1, 0, 0)), ((xs[i + 1], 0, 0), (-1, 0, 0)), ((0, ys[j], 0), (0, 1, 0)), ((0, ys[j + 1], 0), (0, -1, 0))]
            if j == 1:
                out.append(dict(loft(f'hood panel {i}c', s, WHITE, 'shell', open=(), along='x', clip=pl + [((0, 0, zc), (0, 0, 1))]), material='petg'))
            else:
                out.append(dict(loft(f'hood panel {i}{"RL"[j // 2]}u', s, WHITE, 'shell', open=(), along='x', clip=pl + [((0, 0, zc), (0, 0, 1))]), material='petg'))
                out.append(dict(loft(f'hood panel {i}{"RL"[j // 2]}l', s, WHITE, 'shell', open=(), along='x', clip=pl + [((0, 0, zc), (0, 0, -1)), ((0, 0, z0), (0, 0, 1))]), material='petg'))
    return out


JX = HFRONT - 0.30
JZ = ZHEAD + 0.02
def arch():
    return [(ZHEAD + 0.035, HK * HOOD_MD[-1][1], HK * HOOD_MD[-1][2])]


def face_panels():
    z0, a, b = arch()[0]
    secs = [(HFRONT + 0.004, z0, 0, a - 0.015, b - 0.015, 3.0), (HFRONT + 0.008, z0, 0, a - 0.015, b - 0.015, 3.0)]
    zc = z0 + 0.135
    out = []
    for j, (y0, y1) in enumerate(((-0.34, -0.12), (-0.12, 0.12), (0.12, 0.34))):
        pl = [((0, y0, 0), (0, 1, 0)), ((0, y1, 0), (0, -1, 0))]
        out.append(dict(loft(f'face panel {j}u', secs, GREY, 'decal', open=(), along='x', clip=pl + [((0, 0, zc), (0, 0, 1))]), material='petg'))
        out.append(dict(loft(f'face panel {j}l', secs, GREY, 'decal', open=(), along='x', clip=pl + [((0, 0, zc), (0, 0, -1)), ((0, 0, z0), (0, 0, 1))]), material='petg'))
    return out


def bill_panels():
    """The orange band under the hood, as the small duck's: a strip round its lower edge."""
    z0 = ZHEAD + 0.0175
    s_ = [(HBACK - 0.03, z0, 0, 0.0175, 0.14, 8.0)]
    for d, h, w in HOOD_MD:
        s_.append((HFRONT - HK * d, z0, 0, 0.0175, HK * w + 0.003, 8.0))
    s_.append((HFRONT + 0.008,) + s_[-1][1:])
    xs = [HBACK - 0.01, HFRONT - 0.64, HFRONT - 0.42, HFRONT - 0.21, HFRONT + 0.012]
    out = []
    for i in range(len(xs) - 1):
        for j, (y0, y1) in enumerate(((-0.35, -0.12), (-0.12, 0.12), (0.12, 0.35))):
            if j == 1 and i in (1, 2):                 # the rim is open here
                continue
            out.append(dict(loft(f'bill panel {i}{j}', s_, ORANGE, 'shell', open=(), along='x', clip=[((xs[i], 0, 0), (1, 0, 0)), ((xs[i + 1], 0, 0), (-1, 0, 0)), ((0, y0, 0), (0, 1, 0)), ((0, y1, 0), (0, -1, 0))]), material='petg'))
    return out


link('head', [
    disc('head roll output', (XFR - 0.005, 0, ZHY + 0.095), 'x', 0.032, 0.010),
    *output_holes('head roll', 'RS03', (XFR - 0.010, 0, ZHY + 0.095), '+x', '+y', 0.010, 0.004, ['head roll output']),
    metal('head main frame', (XFR, -0.20, ZHEAD), (XFR + PL, 0.20, ZHEAD + 0.24), mat='al6061', fill=0.25),
    *bolt('head main frame screw', ring((XFR + PL, 0, ZHY + 0.095), (-1, 0, 0), (0, 1, 0), 0.050, (30, 90, 150, -30, -90, -150)), (-1, 0, 0), 'M4', 0.016, ['head main frame'], PL, 'head roll output'),
    *[g_ for s_, sg in (('L', 1), ('R', -1)) for g_ in head_rail(s_, sg)],
    metal('head rear frame', (HBACK + 0.06, -0.20, ZHEAD), (HBACK + 0.06 + PL, 0.20, ZHEAD + 0.09), mat='al6061', fill=0.3),
    metal('head front frame', (HFRONT - 0.04 - PL, -0.20, ZHEAD), (HFRONT - 0.04, 0.20, ZHEAD + 0.20), mat='al6061', fill=0.3),
    box('head boards and camera', (HFRONT - 0.10, -0.06, ZHEAD + 0.10), (HFRONT - 0.048, 0.06, ZHEAD + 0.16), PCB, material='electronics', mass=0.6, exposed=True),
    *hood_panels(),
    *face_panels(),
    *bill_panels(),
    cut(box('bill opening rear', (HBACK + 0.07, -0.186, ZHEAD - 0.01), (HFRONT - 0.42, 0.186, ZHEAD + 0.05), ORANGE, 'shell'), *[f'bill panel {i}{j}' for i in range(4) for j in range(3) if not (j == 1 and i in (1, 2))]),
    cut(box('bill opening front', (HFRONT - 0.42, -0.225, ZHEAD - 0.01), (HFRONT - 0.03, 0.225, ZHEAD + 0.05), ORANGE, 'shell'), *[f'bill panel {i}{j}' for i in range(4) for j in range(3) if not (j == 1 and i in (1, 2))]),
    dict(cyl('eye ring', (HFRONT + 0.008, 0, ZHEAD + 0.17), 'x', 0.105, 0.016, AMBER, 'decal'), material='petg', mass=0.30),      # printed and hollow, not solid metal
    dict(cyl('eye lens', (HFRONT + 0.012, 0, ZHEAD + 0.17), 'x', 0.060, 0.012, LENS, 'decal'), material='petg', mass=0.10),
    metal('head beak mount', (HBACK + 0.11, -0.188, ZHEAD + 0.028), (HBACK + 0.17, 0.188, ZHEAD + 0.036), mat='al6061', fill=0.45),
    *rs('beak actuator', 'RS00', (HBACK + 0.14, -0.02605, ZHEAD + 0.0645), '-y', '+z'),
    seat('beak', (HBACK + 0.14, 0, ZHEAD + 0.036), 'z', 0.02),

], 'a plate-and-rail frame in printed panels')
link('beak_link', [
    metal('beak hinge L', (JX - 0.02, 0.205, JZ - 0.09), (JX + 0.02, 0.213, JZ + 0.02), mat='al6061'),
    metal('beak hinge R', (JX - 0.02, -0.213, JZ - 0.09), (JX + 0.02, -0.205, JZ + 0.02), mat='al6061'),
    metal('beak tray frame', (JX - 0.02, -0.205, JZ - 0.09), (HFRONT - 0.02, 0.205, JZ - 0.082), mat='al6061', fill=0.25),
    *[g_ for s_, sg in (('L', 1), ('R', -1)) for g_ in bolt(f'beak hinge {s_} screw', [(JX + dx, sg * 0.213, JZ - 0.086) for dx in (-0.01, 0.01)], (0, -sg, 0), 'M4', 0.016, [f'beak hinge {s_}'], PL, 'beak tray frame')],
    *[dict(loft(f'beak panel {i}', [(JX - 0.03, ZHEAD - 0.045, 0, 0.042, 0.30, 8.0), (HFRONT, ZHEAD - 0.045, 0, 0.042, 0.30, 8.0)], ORANGE, 'shell', open=(), along='x',
                clip=[((JX + 0.03 + (0.27 / 2) * (i // 3), 0, 0), (1, 0, 0)), ((JX + 0.03 + (0.27 / 2) * (i // 3 + 1), 0, 0), (-1, 0, 0)), ((0, -0.31 + 0.2067 * (i % 3), 0), (0, 1, 0)), ((0, -0.31 + 0.2067 * (i % 3 + 1), 0), (0, -1, 0))]), material='petg') for i in range(6)],
], 'the jaw: a sheet frame on two hinges, printed panels under it; the beak actuator drives it by a rod (to detail)')
joint('neck_pitch', 'trunk', 'neck_link', (XN, 0, ZN), 'y', (-0.15, 1.6), 'RS04', count=2, out=1, act_c=(XN, (YN[0] + YN[1]) / 2, ZN), interface=['neck_pitch actuator L (buy) RS04', 'neck_pitch actuator R (buy) RS04', 'neck output L', 'neck output R'])
joint('head_pitch', 'neck_link', 'head_pitch_link', (XN, 0, ZH), 'y', (-1.6, 1.6), 'RS04', count=1, ratio=CHAIN, out=1, act_c=(XN, HPH + 0.02, ZHS), remote=True)
joint('head_yaw', 'head_pitch_link', 'head_yaw_link', (XN, 0, ZHY + 0.017), 'z', (-2.5, 2.5), 'RS03', out=1, act_c=(XN, 0, ZHY), interface=['head_yaw actuator (buy) RS03', 'head yaw output'])
joint('head_roll', 'head_yaw_link', 'head', (XFR - 0.010, 0, ZHY + 0.095), 'x', (-0.38, 0.38), 'RS03', out=1, act_c=(XHR + 0.04, 0, ZHY + 0.095), interface=['head_roll actuator (buy) RS03', 'head roll output'])
joint('beak', 'head', 'beak_link', (JX, 0.205, JZ), 'y', (0.0, 0.4), 'RS00', out=1, act_c=(HBACK + 0.14, 0, ZHEAD + 0.0640), remote=True)


# Joints that aren't screws, for tools/big_assembly.py: (part, part, how). Left and middle; the right is mirrored. Every other pair of parts that touch must share a screw.
# Fits are drawn at nominal size (a touch, not a clash); the tolerance is what's named here. None of these is strength-checked.
HELD = [
    ('L hip yaw bearing', 'L hip yaw housing', 'outer ring in an H7 bore, clamped by the shoulder in the housing'),
    ('L hip yaw bearing', 'L hip yaw retainer', 'the retainer clamps the outer ring'),
    ('L hip yaw bearing', 'L hip yaw flange', 'inner ring on a j6 seat, on the lip of the flange'),
    ('L hip yaw bearing', 'L hip yaw inner retainer', 'the inner retainer clamps the inner ring'),
    ('L knee bearing', 'L thigh plate in', 'outer ring pressed into an N7 bore'),
    ('L knee far bearing', 'L thigh plate out', 'outer ring pressed into an N7 bore'),
    ('L knee bearing', 'L knee shaft', 'inner ring on a k6 seat'),
    ('L knee far bearing', 'L knee shaft', 'inner ring on a k6 seat'),
    *[('L knee shaft', p_, 'keyed (DIN 6885 A 12 x 8), a DIN 471 circlip on each end') for p_ in ('L knee lever I', 'L knee lever O', 'L shin hub in', 'L shin hub out')],
    *[(f'L knee {k} pin {s_}', f'L knee {k} {s_}', 'pressed in, r6') for k in ('crank', 'lever') for s_ in 'IO'],
    *[(f'L knee {k} pin {s_} clip (buy) DIN 6799 8', f'L knee {k} pin {s_}', 'in an 8 mm groove, put on from the side') for k in ('crank', 'lever') for s_ in 'IO'],
    ('L ankle bearing', 'L ankle stub', 'inner ring on a k6 seat'),
    ('L ankle bearing', 'L ankle bearing housing', 'outer ring pressed into an N7 bore, the ear closing it'),
    ('L sole rear', 'L foot plate', 'bonded: contact adhesive'), ('L sole front', 'L foot plate', 'bonded: contact adhesive'),
    ('L sole rear', 'L sole front', 'a seam, each bonded to the plate'), ('L shoe rear', 'L shoe front', 'a seam, each screwed to the plate'),
    ('battery 13S', 'deck plate', 'two straps through slots in the deck (not drawn)'), ('battery BMS, fuse and case, allowance', 'deck plate', 'strapped, as the pack'),
    ('battery 13S', 'battery BMS, fuse and case, allowance', 'side by side, each strapped'),
    *[('computer and power boards', f'computer standoff {i}', '4 M3 from above (not drawn: the boards are a placeholder box)') for i in (0, 1)],
    ('head pitch L bearing', 'neck spine', 'outer ring pressed into an N7 bore'), ('head pitch R bearing', 'neck spine', 'outer ring pressed into an N7 bore'),
    ('head pitch L bearing', 'head pitch shaft', 'inner ring on a k6 seat'), ('head pitch R bearing', 'head pitch shaft', 'inner ring on a k6 seat'),
    *[('head pitch shaft', p_, 'keyed (DIN 6885 A 12 x 8), circlips') for p_ in ('head pitch yoke', 'head pitch yoke R', 'head pitch sprocket L (buy) 25T #40')],
    *[(f'head pitch chain L (buy) #40 {t}', s_, 'a chain on its sprocket') for t in 'ab' for s_ in ('head pitch drive sprocket L (buy) 13T #40', 'head pitch sprocket L (buy) 25T #40')],
    ('head boards and camera', 'head front frame', '4 M3 (not drawn: a placeholder box)'),
    # parts that stand on another, held to it through a third: a bearing face, not a joint of its own
    ('neck plate L', 'deck plate', 'stands on it, held by the tie and its cleat'), ('neck plate R', 'deck plate', 'stands on it, held by the tie and its cleat'),
    ('L hip roll cheek F', 'L hip yaw beam', 'hangs under it, held by its cleat'), ('L hip roll cheek B', 'L hip yaw beam', 'hangs under it, held by its cleat'),
    ('L shin hub in', 'L shin top', 'stands on it, held through the spacer'), ('L shin hub out', 'L shin top', 'stands on it, held through the spacer'),
    ('L shin tube', 'L shin cap', 'stands on it, held by the cleats'),
    ('L ankle bearing', 'L ankle ear in', 'the ear stops the outer ring'),
    *[(f'L knee rod {s_} bush {e}', f'L knee rod {s_}', 'pressed in') for s_ in 'IO' for e in 'AB'],
    *[(f'L knee rod {s_} bush {e}', f'L knee {k} pin {s_}', 'turns on it, held on by the E-clip') for s_ in 'IO' for e, k in (('A', 'crank'), ('B', 'lever'))],
    ('beak actuator (buy) RS00', 'head beak mount', 'not designed: the beak drive is a placeholder'),
]


def mech():
    """The knee's four-bar, which a tree of joints can't hold: its cranks and rods, each a moving part in Fusion, and their angles (about +y, from zero at knee 0) at sampled knee angles. The rods' far ends aren't jointed: that would close the loop. tools/big_sweep.py checks they meet."""
    s, k = (XQ, ZQ), (XK, ZK)
    c0, r0 = four_bar(s, k, 0.0)
    samples = []
    for i in range(13):
        q = KNEE_RANGE[0] + (KNEE_RANGE[1] - KNEE_RANGE[0]) * i / 12
        c, r = four_bar(s, k, q)
        c_, r_ = (c - c0 + math.pi) % (2 * math.pi) - math.pi, (r - r0 + math.pi) % (2 * math.pi) - math.pi
        samples.append(dict(knee=q, crank=-c_, rod_abs=-r_, rod=-(r_ - c_)))
    out = []
    for side, sg in (('L', 1), ('R', -1)):
        for t, yh in (('I', HUB_I), ('O', HUB_O)):
            out.append(dict(name=f'{side}_knee_crank_{t}', parent=f'{side}_hip_pitch_link', child=f'{side}_knee_crank_{t}', origin=[XQ, sg * yh, ZQ], axis='y', lower=min(x['crank'] for x in samples) - 0.1, upper=max(x['crank'] for x in samples) + 0.1, role='crank'))
            out.append(dict(name=f'{side}_knee_rod_{t}', parent=f'{side}_knee_crank_{t}', child=f'{side}_knee_rod_{t}', origin=[PIN_A[0], sg * yh, PIN_A[1]], axis='y', lower=min(x['rod'] for x in samples) - 0.1, upper=max(x['rod'] for x in samples) + 0.1, role='rod'))
    return dict(joints=out, samples=samples)


def build():
    links, joints = {}, []
    for n, l in LINKS.items():
        links[n] = copy.deepcopy(l)
        if n.startswith('L_'):
            m = copy.deepcopy(l)
            m['name'] = mirror_name(n)
            m['geoms'] = []
            for g in l['geoms']:
                mg = mirror_geom(g)
                if mg.get('on_joint'):
                    mg['on_joint'] = mirror_name(g['on_joint'])
                if mg['name'].startswith('L_'):
                    mg['name'] = 'R_' + mg['name'][2:]
                if mg.get('group'):
                    mg['group'] = mirror_name(mg['group'])
                rn = lambda t: ('R ' + t[2:]) if t.startswith('L ') else ('R_' + t[2:]) if t.startswith('L_') else t
                for k_ in ('part', 'unit'):
                    if mg.get(k_):
                        mg[k_] = rn(mg[k_])
                if mg.get('fast'):
                    f_ = mg['fast']
                    if f_.get('clamp'):
                        f_['clamp'] = [rn(t) for t in f_['clamp']]
                    if f_.get('tap'):
                        f_['tap'] = rn(f_['tap'])
                    if f_.get('tap_hub'):
                        f_['tap_hub'][1] = -f_['tap_hub'][1]
                for k_ in ('real', 'rotor'):
                    if mg.get(k_):
                        for v_ in ('hub', 'out', 'g'):
                            mg[k_][v_][1] = -mg[k_][v_][1]
                m['geoms'].append(mg)
            links[m['name']] = m
    for j in JOINTS:
        joints.append(copy.deepcopy(j))
        if j['name'].startswith('L_'):
            m = copy.deepcopy(j)
            m['name'], m['parent'], m['child'] = mirror_name(j['name']), mirror_name(j['parent']), mirror_name(j['child'])
            m['origin'][1] = -m['origin'][1]
            m['act_c'][1] = -m['act_c'][1]
            m['interface'] = [('R_' + t[2:]) if t.startswith('L_') else ('R ' + t[2:]) if t.startswith('L ') else t for t in j['interface']]
            if m['axis'] in ('x', 'z'):
                m['lower'], m['upper'] = -j['upper'], -j['lower']
            if j['axis'] == 'y':
                m['out'] = -j['out']
            joints.append(m)
    for l in links.values():
        for g in l['geoms']:
            if g['role'] != 'cut' and 'material' not in g:
                g['material'] = 'actuator' if g['role'] == 'actuator' else ('tpu' if g['color'] == YELLOW else 'petg' if g['role'] == 'shell' else 'al6061')
    rn = lambda t: ('R ' + t[2:]) if t.startswith('L ') else ('R_' + t[2:]) if t.startswith('L_') else t
    held = [dict(a=a_, b=b_, how=h_) for a_, b_, h_ in HELD] + [dict(a=rn(a_), b=rn(b_), how=h_) for a_, b_, h_ in HELD if rn(a_) != a_ or rn(b_) != b_]
    return dict(mech=mech(), held=held, recipe='big', rl=RL, recipe_limits=dict(mass_budget_kg=115.0, target_tol=0.25, cannot=['floor pick', 'deep squat'], brief=['head forward and down, beak open', 'head tucked in']),
                frame='metres; X forward, Y left, Z up; floor z=0; zero pose standing', actuators=ACT, bearings=BEAR, materials=MATERIALS, links=links, joints=joints, root='trunk')


# For the trainer (tools/make_rl_model.py). Gains are a first guess, not measured: the order of a real full-size humanoid's walking gains, raised for this mass; damping about kp / 40.
# Each entry: joint without its side -> (kp N m/rad, kd N m s/rad), at the joint, after any pair or chain.
# speed_ratio: a crank's ratio changes along its stroke, and the trainer's motor has one. Its torque is the weakest point's (KNEE_RATIO). Its speed is the bent end's, about 11 rad/s, where the
# fast swing happens: the slowest point's 6.3 rad/s held the whole stroke back. So the straight end, slower still in a real crank, is optimistic here.
RL = dict(root_z=0.95, imu=[0.0, 0.0, 0.95], speed_ratio=dict(knee=KNEE_RATIO_BENT), gains=dict(hip_yaw=(150, 4), hip_roll=(300, 8), hip_pitch=(300, 8), knee=(400, 10), ankle=(100, 3),
                                                       neck_pitch=(200, 5), head_pitch=(300, 8), head_yaw=(60, 2), head_roll=(60, 2)))


if __name__ == '__main__':
    r = build()
    out = os.path.join(HERE, 'robot_big.json')
    json.dump(r, open(out, 'w', encoding='utf-8'), indent=1)
    # what cad/fusion/build.py reads: no printed part here has holes or pockets yet, so none is listed, and every panel is a plain body
    json.dump(dict(parts={}), open(os.path.join(HERE, 'parts_big.json'), 'w', encoding='utf-8'))
    json.dump({}, open(os.path.join(HERE, 'verify_points_big.json'), 'w', encoding='utf-8'))
    print(f"{len(r['links'])} links, {len(r['joints'])} joints -> {out}")
