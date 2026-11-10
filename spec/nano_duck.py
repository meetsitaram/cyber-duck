"""nano: the duck on 9 g hobby servos, driven by PWM. python -I spec/nano_duck.py -> spec/robot_nano.json, spec/parts_nano.json is tools/export_prints.py's. Then MILLI_RECIPE=nano for the tools.

(2026-11-06, the user: "a palm-sized duck on the cheap blue SG90 micro servos (MG90S where a joint needs more torque), driven by PWM"; "horns and brackets for the SG90's case ears and 21T spline,
and a printed bushing opposite the output shaft"; "the Pi Zero 2 W goes in the torso"; "PWM servos can't be daisy-chained".)

Not spec/robot.py with an SG90 in it: that layout's numbers are the XL330's and STS3215's, with bearings round horns that an SG90 hasn't got. The same joint names, links and shapes, so tools/checks.py,
assembly.py, buildable.py and printable.py run on it unchanged, and the same look: microduck's, a big hood on a neck of bare servos, a small white body, bare servos at the hips and knees, orange shoes.

Every joint is one module (servo() below), as the 9 g servo robots do it:
  - a CRADLE in the link that carries the case: a floor under the case's bottom, one side wall, and a block under each ear. The servo slides in from the open side, its lead along a slot in the block.
  - an EAR CLAMP: a printed frame on the 2 ears, its 2 screws (the servo's own, from its bag) through it and the ears into the blocks. A screw's head on a servo's ear clamps nothing.
  - a HUB on the 21T spline, printed in the bracket of the other link, held by the servo's own horn screw into the output shaft. That is the horn: no bought horn.
  - on the 8 leg joints that carry the duck, the bracket is a U: from the hub round the case's near end (its BAR, outside everything the case's link has at any angle of the joint) to a
    REAR ARM, screwed to the bar, with a printed BUSHING on a printed STUB of the cradle's floor, on the joint's axis: the output shaft no longer carries the bending alone.
    The rear arm is its own part, as the U can't go round the case in one piece. The neck, head pitch and beak, lightly loaded, have the hub alone.
13 joints, not microduck's 15: no head yaw or roll. The head's 12 mm of SG90 across a 44 mm head, and 13 GPIO, not 15, from the Pi.

Smallest it can be: the SG90's case sets it. Each U's bar has to pass 13 mm from its axis (the ear's tip is 12.2), the foot's sole under the ankle so, and the ankle case's 22 mm
above it: the hip is 74 mm up, where microduck's proportions would put it on a 22 cm duck. tools/nano_sweep.py measures the joints' torque as the free lengths grow.
NANO_STRETCH (default 1.0) stretches the free lengths (shin, thigh, neck, body, head and feet) for that sweep only.
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
from robot import ALU, AMBER, BLACK, CELL, DARK, GREY, LENS, ORANGE, PCB, STEEL, WHITE, YELLOW, cut, cyl, fast, loft, plate

ACT = {k: v for k, v in json.load(open(os.path.join(HERE, 'actuators.json'), encoding='utf-8')).items() if not k.startswith('_')}
ST = float(os.environ.get('NANO_STRETCH', '1.0'))
MM = 0.001
SG_BLUE, MG_BLACK = '#2F6FD6', '#24262B'
STUB_C = '#D8D2C2'

# ---------------------------------------------------------------- the servo, in its own frame (mm): u along the case from the shaft to the far end, a from the bottom toward the spline,
# w the third. spec/actuators.json SG90: what's measured on the maker's drawing and what's estimated
S = ACT['SG90']
SX = S['shaft_x'] / MM                             # 5.6: the shaft off the case's middle
CL, CW, CH = (v / MM for v in S['box'])            # 22.7 x 12.2 x 22.5
U0, U1 = SX - CL / 2, SX + CL / 2                  # -5.75 .. 16.95
EU0, EU1 = SX - S['ears']['span'] / MM / 2, SX + S['ears']['span'] / MM / 2     # -10.55 .. 21.75
EA0 = S['ears']['under'] / MM                      # 17.0
EA1 = EA0 + S['ears']['t'] / MM                    # 19.5
HOLE = (SX - S['ears']['hole_u'] / MM, SX + S['ears']['hole_u'] / MM)           # -8.8, 20.0
TR, TA = S['tower'][0] / MM, CH + S['tower'][1] / MM                            # 5.7, 27.0
SPL_R, SPL_A = S['spline']['d'] / MM / 2, TA + S['spline']['h'] / MM           # 2.4, 30.3
HUB_R, HUB_A0, HUB_A1 = 4.0, TA + 0.2, SPL_A + 1.2                             # the printed hub: 27.2 .. 31.5, its face the joint's origin
ARM_A0 = 28.0                                      # the hub arm's plate: 28 .. 31.5, 1 mm over the tower
WALL, GAP = 1.5, 0.15                              # a printed wall; the pocket's clearance each side
RING_T = 2.0                                       # the ear clamp
EB = 3.5                                           # the ear blocks' half width: a post under each ear's screw, not the ear's whole 12.2 mm
FLOOR = 2.0
REAR_A = (-6.0, -3.5)                              # the rear arm's plate
BUSH = dict(r_in=2.15, r_out=3.6, a=(-6.0, -2.6), flange=(5.0, -7.5))
STUB = dict(r=2.0, a=(-6.4, -1.9))
LEAD = dict(out=S['lead']['out'] / MM, w=S['lead']['w'] / MM / 2, a=tuple(v / MM for v in S['lead']['a']))
HEAD_R = 1.9                                       # an M2 pan head, as tools/export_prints.py cuts its counterbore
BAR = dict(r0=13.2, r1=17.2)                       # a near-end bar: 1 mm outside the ear's tip, wherever the joint turns (13.2 > 12.2 to the tip's corner)


def unit(s):
    return tuple((-1.0 if s[0] == '-' else 1.0) * (1.0 if 'xyz'[k] == s[1] else 0.0) for k in range(3))


def crossv(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


class Frame:
    """A servo in place: hub (mm, world), the joint's axis 'x' / 'y' / 'z' and the horn's direction s (+1 / -1) along it, long: the case's u in world ('+x' ...)."""

    def __init__(self, hub, axis, s, long):
        self.axis, self.s, self.long = axis, s, long
        self.A = tuple(s * v for v in unit('+' + axis))
        self.U = unit(long)
        self.W = crossv(self.A, self.U)
        self.O = tuple(hub[k] - HUB_A1 * self.A[k] for k in range(3))

    def p(self, u, w, a):
        return [(self.O[k] + u * self.U[k] + w * self.W[k] + a * self.A[k]) * MM for k in range(3)]

    def box(self, name, ur, wr, ar, color, role='struct', **kw):
        p, q = self.p(ur[0], wr[0], ar[0]), self.p(ur[1], wr[1], ar[1])
        return T.box(name, [min(x, y) for x, y in zip(p, q)], [max(x, y) for x, y in zip(p, q)], color, role, **kw)

    def lohi(self, ur, wr, ar):
        p, q = self.p(ur[0], wr[0], ar[0]), self.p(ur[1], wr[1], ar[1])
        return [min(x, y) for x, y in zip(p, q)], [max(x, y) for x, y in zip(p, q)]

    def cyl(self, name, r, ar, color, role='struct', u=0.0, w=0.0):
        c = self.p(u, w, (ar[0] + ar[1]) / 2)
        return cyl(name, c, self.axis, r * MM, abs(ar[1] - ar[0]) * MM, color, role)

    def plate(self, name, ar, circles, color, role='struct', **kw):
        """A plate square to the axis, a in ar, the hull of circles (u, w, r)."""
        k = 'xyz'.index(self.axis)
        d0, d1 = {'x': (1, 2), 'y': (0, 2), 'z': (0, 1)}[self.axis]
        z0, z1 = sorted(self.p(0, 0, a)[k] for a in ar)
        cs = [(self.p(u, w, 0)[d0], self.p(u, w, 0)[d1], r * MM) for u, w, r in circles]
        return plate(name, z0, z1 - z0, cs, color, role, axis=self.axis, **kw)


def tag(gs, part, **kw):
    for g in gs:
        g['part'] = part
        g.update(kw)
    return gs


def screw(name, f, u, w, a, d, L, part_hint=''):
    """A bought M2 screw: its head on the face at a (frame f), going in along d (+1: toward +a, -1: toward -a), L mm long."""
    a_head = (a - 1.5, a) if d > 0 else (a, a + 1.5)
    g = f.cyl(name, HEAD_R, a_head, STEEL, 'decal', u=u, w=w)
    return fast(g, 'M2', L * MM, f.p(u, w, a), [d * v for v in f.A])


JOINTS, PARTS, LINKS = [], {}, {}
FR = {}                                            # the servos' frames, by name, for the load sections
STEPS = {}
GLUED = set()


def add(link_name, geoms):
    LINKS.setdefault(link_name, dict(name=link_name, budget=0.0, geoms=[], note=''))['geoms'] += geoms


def part(name, kind):
    PARTS.setdefault(name, dict(kind=kind, geoms=[]))


def servo(jn, parent, child, model, f, lim, case_part, horn_part, open_w=1, rear=True, bar=None, case_in='parent', ring_side=None, horn_col=DARK):
    """One joint, as the module above. bar: (side '-u' / '+w' / '-w', r0, r1, t0, t1): radial and tangential range, or None for the hub alone.
    open_w: the side of the cradle with no wall, which the servo slides in from (+1 / -1 along w). Returns the steps of its assembly."""
    n = jn.replace('_', ' ')
    case_link, horn_link = (child, parent) if case_in == 'child' else (parent, child)
    col = SG_BLUE if model == 'SG90' else MG_BLACK
    wall_w = (-open_w * (CW / 2 + GAP), -open_w * (CW / 2 + GAP + WALL))
    bw = bar[0][1] if bar else 'u'
    bs = (-1 if bar[0][0] == '-' else 1) if bar else -1
    # the case, as boxes: body, ears, gear tower, the lead's strain relief at the near end's foot
    boxes = [f.lohi((U0, U1), (-CW / 2, CW / 2), (0, CH)), f.lohi((EU0, EU1), (-CW / 2, CW / 2), (EA0, EA1)), f.lohi((U0, TR), (-TR, TR), (CH - 0.3, TA)),
             f.lohi((U0 - LEAD['out'], U0 + 0.3), (-LEAD['w'], LEAD['w']), LEAD['a'])]
    case = f.box(f'{jn} actuator (buy) {model}', (U0, U1), (-CW / 2, CW / 2), (0, CH), col, 'actuator', fillet=0.0002)
    case = dict(case, kind='stepbox', boxes=boxes, exposed=True)
    flange = dict(f.cyl(f'{jn} flange', SPL_R, (TA - 3.0, SPL_A), '#E6E2D6'), material='electronics', mass=0.0003)
    part(f'{n} servo', 'B')
    PARTS[f'{n} servo']['geoms'] += [case['name'], flange['name']]
    # the cradle
    wide = lambda side: (CW / 2 + GAP + WALL) if (bw != 'w' or bs != side) else CW / 2     # (a w-side bar passes close: nothing past the case's width on its side)
    # (the floor and the blocks 0.05 mm into the case: drawn touching, the checks took the servo as a part of its own; the print's pocket has its clearance)
    cr = [f.box(f'{n} cradle floor', (U0 - GAP - 0.1, U1 + GAP + 0.1), (-wide(-1), wide(1)), (-FLOOR, 0.05), DARK),
          f.box(f'{n} cradle wall', (U0 - GAP, U1 + GAP), tuple(sorted(wall_w)), (-FLOOR, EA0), DARK),
          f.box(f'{n} cradle near block', (EU0, U0 - GAP), (-EB, EB), (-FLOOR, EA0 + 0.05), DARK),
          f.box(f'{n} cradle far block', (U1 + GAP, EU1), (-EB, EB), (-FLOOR, EA0 + 0.05), DARK)]
    # the wall joins the blocks only through the floor: a corner post each end makes it one solid however the boxes round
    cr += [f.box(f'{n} cradle corner {k}', ur, tuple(sorted((-open_w * (EB - 0.5), -open_w * (CW / 2 + GAP + WALL)))), (-FLOOR, EA0 + 0.05), DARK)
           for k, ur in enumerate(((U0 - GAP - 1.6, U0 - GAP), (U1 + GAP, U1 + GAP + 1.6)))]
    lo = -LEAD['w'] - 0.3
    cr.append(cut(f.box(f'{n} lead slot', (U0 - LEAD['out'] - 0.3, U0), (lo, open_w * (CW / 2 + 1)) if open_w > 0 else (-(CW / 2 + 1), -lo), (LEAD['a'][0] - 0.3, LEAD['a'][1] + 0.3), DARK),
                  f'{n} cradle near block'))
    # the ear clamp: 2 bars on the ears and a strip down the wall's side
    rs = -open_w if ring_side is None else ring_side
    strip_w = tuple(sorted((rs * (CW / 2 + GAP), rs * (CW / 2 + GAP + 1.35))))
    ring = [f.box(f'{n} ear clamp near', (EU0 - 0.45, U0 - GAP), tuple(sorted((-rs * CW / 2, rs * (CW / 2 + GAP + 1.35)))), (EA1, EA1 + RING_T), DARK),
            f.box(f'{n} ear clamp far', (U1 + GAP, EU1 + 0.45), tuple(sorted((-rs * CW / 2, rs * (CW / 2 + GAP + 1.35)))), (EA1, EA1 + RING_T), DARK),
            f.box(f'{n} ear clamp strip', (U0 - GAP - 0.5, U1 + GAP + 0.5), strip_w, (EA1, EA1 + RING_T), DARK)]
    # (the near bar's outer corner on the strip's side, cut back so a near-end bar clears it at every angle)
    ring.append(cut(f.box(f'{n} ear clamp corner', (EU0 - 1.5, -7.5), tuple(sorted((rs * CW / 2, rs * (CW / 2 + 3)))), (EA1 - 1, EA1 + RING_T + 1), DARK), f'{n} ear clamp near'))
    scr = [screw(f'{n} cradle screw near', f, HOLE[0], 0, EA1 + RING_T, -1, 8), screw(f'{n} cradle screw far', f, HOLE[1], 0, EA1 + RING_T, -1, 8)]
    add(case_link, tag(cr, case_part) + tag(ring, f'{n} ear clamp') + [case] + scr)
    part(f'{n} ear clamp', 'P')
    PARTS[f'{n} ear clamp']['geoms'] += [g['name'] for g in ring if g['role'] != 'cut']
    PARTS.setdefault(case_part, dict(kind='P', geoms=[]))['geoms'] += [g['name'] for g in cr if g['role'] != 'cut']
    # the horn side: hub on the spline, its arm, and the U
    hub = [f.cyl(f'{n} hub', HUB_R, (HUB_A0, HUB_A1), horn_col),
           cut(f.cyl(f'{n} spline bore', SPL_R + 0.05, (TA - 0.1, SPL_A + 0.15), DARK), f'{n} hub')]
    centre = screw(f'{n} horn screw', f, 0, 0, HUB_A1, -1, 5)
    hg = list(hub)
    corners = []
    if bar:
        side, r0, r1, t0, t1 = bar[:5]
        if side[1] == 'u':
            ur, wr = (-r1, -r0) if side[0] == '-' else (r0, r1), (t0, t1)
        else:
            ur, wr = (t0, t1), (-r1, -r0) if side[0] == '-' else (r0, r1)
        a_lo = REAR_A[1] if rear else ARM_A0
        hg.append(f.box(f'{n} bar', ur, wr, (a_lo, HUB_A1), horn_col))
        corners = [(u, w, 0.6) for u in ur for w in wr]
        arm_c = bar[5] if len(bar) > 5 else corners
        hg.append(f.plate(f'{n} hub arm', (ARM_A0, HUB_A1), [(0, 0, HUB_R)] + arm_c, horn_col))
    add(horn_link, tag(hg, horn_part) + [centre, flange])
    PARTS.setdefault(horn_part, dict(kind='P', geoms=[]))['geoms'] += [g['name'] for g in hg if g['role'] != 'cut']
    steps = dict(servo=(f'{n} servo', [-open_w * v for v in f.W]), ring=(f'{n} ear clamp', [-v for v in f.A]), ear_screws=rf'^{n} cradle screw ',
                 hub=[-v for v in f.A], horn_screw=rf'^{n} horn screw$')
    if bar and rear:
        side, r0, r1, t0, t1 = bar[:5]
        rc = (r0 + r1) / 2
        mid = 0.0 if t0 < -2 and t1 > 2 else (t0 + t1) / 2
        pts = [(-rc if side == '-u' else rc, mid - 1.8), (-rc if side == '-u' else rc, mid + 1.8)] if side[1] == 'u' else \
              [(mid - 1.8, -rc if side[0] == '-' else rc), (mid + 1.8, -rc if side[0] == '-' else rc)]
        ra = [f.plate(f'{n} rear arm', REAR_A, [(0, 0, 5.2)] + (bar[5] if len(bar) > 5 else corners), horn_col),
              cut(f.cyl(f'{n} rear arm bore', BUSH['r_out'], (REAR_A[0] - 0.1, REAR_A[1] + 0.1), DARK), f'{n} rear arm')]
        bu = [dict(f.cyl(f'{n} rear bearing bushing', BUSH['r_out'], BUSH['a'], STUB_C), on_joint=jn),
              f.cyl(f'{n} bushing lip', BUSH['flange'][0], (BUSH['flange'][1], REAR_A[0]), STUB_C),
              cut(f.cyl(f'{n} bushing bore', BUSH['r_in'], (BUSH['flange'][1] - 0.1, BUSH['a'][1] + 0.1), DARK), f'{n} rear bearing bushing', f'{n} bushing lip')]
        stub = dict(f.cyl(f'{n} rear bearing stub', STUB['r'], STUB['a'], DARK), on_joint=jn)
        rsc = [screw(f'{n} rear screw {k}', f, u, w, REAR_A[0], +1, 8) for k, (u, w) in enumerate(pts)]
        add(horn_link, tag(ra, f'{n} rear arm') + tag(bu, f'{n} bushing') + rsc)
        add(case_link, tag([stub], case_part))
        PARTS[case_part]['geoms'].append(stub['name'])
        for p_, gs in ((f'{n} rear arm', ra), (f'{n} bushing', bu)):
            part(p_, 'P')
            PARTS[p_]['geoms'] += [g['name'] for g in gs if g['role'] != 'cut']
        GLUED.add(f'{n} bushing')
        steps.update(bushing=(f'{n} bushing', list(f.A)), rear_arm=(f'{n} rear arm', list(f.A)), rear_screws=rf'^{n} rear screw ')
    act_c = f.p(0, 0, CH / 2)
    JOINTS.append(dict(name=jn, parent=parent, child=child, origin=f.p(0, 0, HUB_A1), axis=f.axis, lower=lim[0], upper=lim[1], act=model, act_c=act_c, out=f.s, long=f.long,
                       exposed=True, remote=False, case_in=case_in, stack=(HUB_A1 - CH) * MM, bearing=[], sides=[], faces=[], fork=bool(bar and rear), post=False,
                       capacity=dict(model=model, count=1, ratio=1.0), rear_bushing=bool(bar and rear)))
    return steps


# ---------------------------------------------------------------- the layout (mm). The left leg; the right is its mirror
YB = 10.0                                          # the pitch servos' case bottoms, y: the leg's 38 mm slab from 3.2 to 41.5
XK = -30.0                                         # knee and ankle, behind the hip: the thigh 62 deg back (microduck's 58)
ZA = 18.7                                          # ankle: 13.2 to the sole's top (the near ear's tip at 12.2) + 4 of sole + 1.5 of TPU
LS = 39.45 + 30 * (ST - 1)                         # shin: the knee's bar under the knee, the ankle case's 21.75 to its ears' tips above the ankle
ZK = ZA + LS
ZP = ZK + 15.65 + 15 * (ST - 1)                    # the thigh: the knee case's top level with the hip case's (6.1 over the axis)
XP = 0.0
ZR = ZP + 28.5                                     # hip roll: its bar (under the roll case) is the hip's bar's top, 14.5 over the hip, clear of the thigh turning -0.7 .. 0.5
YR, XR = 13.05, 28.0                               # the roll case's near ears 2.5 mm from the middle; the roll's hub 28 ahead, so its bar passes over the thigh's front
ZYH = ZR + 7.8                                     # the hip yaw's hub: its horn screw's head 0.2 mm over the roll case, in a counterbore through the roll cradle's top wall
XY, YY = 4.0, 16.0
RANGE = dict(hip_yaw=(-0.4, 0.4), hip_roll=(-0.35, 0.35), hip_pitch=(-0.7, 0.5), knee=(-1.2, 0.2), ankle=(-0.5, 1.45), neck_pitch=(-0.2, 1.6), head_pitch=(-1.3, 0.8), beak=(0.0, 0.45))
MASS_BUDGET = 0.38                                  # what it comes to (tools/mass_model.py), declared: the servos alone are 135 g
LEG = dict(hip_yaw='SG90', hip_roll='MG90S_5V', hip_pitch='SG90', knee='MG90S_5V', ankle='SG90')   # (MG90S_5V: nano's model of the MG90S, at 5 V; the hobby-servo duck's MG90S is at 6 V)


def leg():
    """The left leg's 5 joints; returns the assembly sections."""
    st = {}
    hub_y = YB + HUB_A1
    # ankle: case in the shin, u up; the foot is a U whose bar is the sole
    fa = Frame((XK, hub_y, ZA), 'y', +1, '+z')
    sole_t = (-14.0 - 4 * (ST - 1), 40.0 + 6 * (ST - 1))
    st['ankle'] = servo('L_ankle_pitch', 'L_knee_link', 'L_foot', LEG['ankle'], fa, RANGE['ankle'], 'L shin', 'L foot', open_w=+1, horn_col=ORANGE,
                        bar=('-u', BAR['r0'], BAR['r1'], sole_t[0], sole_t[1], [(-BAR['r1'], w, 0.6) for w in (-12, 12)] + [(-BAR['r0'], w, 0.6) for w in (-12, 12)]))
    # knee: case in the thigh, u up; the shin is a U whose bar is under the knee, wide behind for the ankle cradle's wall to reach it
    fk = Frame((XK, hub_y, ZK), 'y', +1, '+z')
    st['knee'] = servo('L_knee', 'L_hip_pitch_link', 'L_knee_link', LEG['knee'], fk, RANGE['knee'], 'L thigh', 'L shin', open_w=-1,
                       bar=('-u', BAR['r0'], BAR['r1'], -7.75, 4.0))
    # hip pitch: case in the thigh (u back toward the knee), the roll bracket is the U, its bar in front
    fp = Frame((XP, hub_y, ZP), 'y', +1, '-x')
    st['hip'] = servo('L_hip_pitch', 'L_hip_roll_link', 'L_hip_pitch_link', LEG['hip_pitch'], fp, RANGE['hip_pitch'], 'L thigh', 'L roll bracket', open_w=+1,
                      bar=('-u', BAR['r0'], BAR['r1'], -4.0, 4.0), case_in='child')
    # hip roll: case in the yaw link, flat (u out, w up); its bar under the case, which is the hip's bar's top
    fr = Frame((XR, YR, ZR), 'x', +1, '+y')
    st['roll'] = servo('L_hip_roll', 'L_hip_yaw_link', 'L_hip_roll_link', LEG['hip_roll'], fr, RANGE['hip_roll'], 'L yaw link', 'L roll bracket', open_w=-1,
                       bar=('-w', 9.5 + 1.5, 13.5 + 1.5, -4.0, 4.0))
    # hip yaw: case in the trunk, hub down, u forward; the yaw link is a U, its bar behind
    fy = Frame((XY, YY, ZYH), 'z', -1, '+x')
    st['yaw'] = servo('L_hip_yaw', 'trunk', 'L_hip_yaw_link', LEG['hip_yaw'], fy, RANGE['hip_yaw'], 'chassis', 'L yaw link', open_w=-1,
                      bar=('-u', BAR['r0'], BAR['r1'] + 2.0, -4.0, 4.0))
    FR.update(fa=fa, fk=fk, fp=fp, fr=fr, fy=fy)
    return dict(fa=fa, fk=fk, fp=fp, fr=fr, fy=fy, steps=st)


def leg_structure(F):
    """What joins each link's cradle to its U."""
    fa, fk, fp, fr, fy = F['fa'], F['fk'], F['fp'], F['fr'], F['fy']
    mm = lambda *v: [x * MM for x in v]
    out = {}
    # shin: the ankle cradle's wall (behind the ankle) up to the knee's bar
    ya, yb = YB - FLOOR, YB + EA0
    zt = ZK - BAR['r0'] + 0.3
    out['L_knee_link'] = tag([T.box('L shin spine', mm(XK - 10.75, ya, ZA + 12.0), mm(XK - 6.25, yb, zt), DARK)], 'L shin')
    # thigh: the knee cradle's front wall to the hip cradle's far block
    out['L_hip_pitch_link'] = tag([T.box('L thigh web', mm(XK + CW / 2 + GAP, ya, ZP - 6.1), mm(XP - U1 - GAP + 0.3, yb, min(ZP + 1.5, ZK + U1)), DARK)], 'L thigh')
    # roll bracket: a post from the hip's bar up to the roll's bar
    # yaw link: the hub arm sits on the roll cradle's top wall through a seat, with the yaw horn screw's head in a counterbore through both
    hy = (XY + U0 - 0.5, YY - 6.0, ZR + CW / 2 + GAP + WALL - 0.2), (XY + U1 - 6.0, YY + 6.0, ZYH + 0.3)
    out['L_hip_yaw_link'] = tag([T.box('L yaw seat', mm(*hy[0]), mm(*hy[1]), DARK)], 'L yaw link')
    out['L_hip_roll_link'] = tag([T.box('L roll post', mm(XP + BAR['r0'], YR - 6.0, ZP + 3.5), mm(XP + BAR['r0'] + 8.8, YR + 6.0, ZR - 11.0 + 0.3), DARK)], 'L roll bracket')
    # slots for a 2.5 mm cable tie, where the leads of the joints below are tied down between their service loops (docs/nano/WIRING.md)
    out['L_knee_link'].append(cut(T.box('L shin tie slot', mm(XK - 11.0, YB + 6.0, ZK - 24.0), mm(XK - 6.0, YB + 9.0, ZK - 22.5), DARK), 'L shin spine'))
    out['L_hip_pitch_link'].append(cut(T.box('L thigh tie slot', mm(-21.5, YB + 6.0, ZP - 6.5), mm(-19.5, YB + 9.0, ZP + 2.0), DARK), 'L thigh web'))
    out['L_hip_roll_link'].append(cut(T.box('L roll post tie slot', mm(XP + BAR['r0'] - 0.2, YR - 1.5, ZP + 8.0), mm(XP + BAR['r0'] + 9.0, YR + 1.5, ZP + 9.5), DARK), 'L roll post'))
    # foot: the sole plate is the ankle's bar; a TPU sole under it
    so = dict(T.box('L sole', mm(XK - 14.0 - 4 * (ST - 1), YB + REAR_A[1], 0.0), mm(XK + 40.0 + 6 * (ST - 1), YB + HUB_A1, ZA - BAR['r1'] + 0.1), YELLOW, material='tpu'))
    so['contact'] = dict(c=list(so['c']), size=list(so['size']))
    out['L_foot'] = tag([so], 'L sole')
    return out


DZ = ZYH - 108.55                                  # the body's heights were drawn at the smallest duck's: they follow the yaw servos up
XN, ZN = 42.0, 132.0 + DZ + 10 * (ST - 1)               # neck pitch: in front of the body, its servo bare as microduck's
YPL = -2.15                                        # the neck plate, y: the neck's hub on one face, the head pitch's on the other
XH, ZH = XN + 6.0, ZN + 38.0 + 8 * (ST - 1)        # head pitch: the plate leans forward, so the jaw clears the neck servo in the floor pick
DX = XH - 42.0                                     # the head's lengths were drawn with the head pitch at x = 42
XB, ZB, YBH = 70.0 + DX, ZH - 9.5, 7.0                  # beak: the jaw on its hub
HS = 1.0 + 0.5 * (ST - 1)                          # the hood's scale
PX0 = -39.5                                        # the Pi Zero 2 W, on edge between the yaw cases, from x = PX0


def msec(secs):
    return [(z * MM, cx * MM, cy * MM, a * MM, b * MM, n) for z, cx, cy, a, b, n in secs]


def bx(name, lo, hi, color, prt, **kw):
    g = T.box(name, [v * MM for v in lo], [v * MM for v in hi], color, **kw)
    g['part'] = prt
    return g


def upper():
    """Trunk (chassis, electronics, body shell), the neck, the head."""
    # the neck: case in the trunk, far end down; a plate carries its hub and the head pitch's
    fn = Frame((XN, YPL + 4.3, ZN), 'y', +1, '-z')
    st = dict(neck=servo('neck_pitch', 'trunk', 'neck_link', 'SG90', fn, RANGE['neck_pitch'], 'chassis', 'neck plate', open_w=-1, rear=False))
    fh = Frame((XH, YPL, ZH), 'y', -1, '+x')
    st['head'] = servo('head_pitch', 'neck_link', 'head', 'SG90', fh, RANGE['head_pitch'], 'head frame', 'neck plate', open_w=-1, rear=False, case_in='child')
    fb = Frame((XB, YBH, ZB), 'y', +1, '-x')
    st['beak'] = servo('beak', 'head', 'beak_link', 'SG90', fb, RANGE['beak'], 'head frame', 'jaw', open_w=-1, rear=False, horn_col=ORANGE)
    zf = ZYH + 31.5                                # the yaw cradles' floor, under the case bottoms (which face up)
    zs = zf + 3.45                                 # the spine over the Pi, between the yaw rear arms (they turn no nearer the middle than 4.9 mm)
    tr = [
        # the 2 yaw cradles join in front of the Pi, at the neck's bracket, and over it by a spine to the battery tray
        bx('L chassis bridge', (XY + U1 - 0.1, 2.6, 133.0 + DZ), (XY + U1 + 7.5, YY + 5.0, zs + 1.5), DARK, 'chassis'),
        bx('chassis spine', (-44.0, -4.4, zs), (XY + U1 + 7.5, 4.4, zs + 1.5), DARK, 'chassis'),
        bx('Pi plate', (PX0, 2.5, 135.0 + DZ), (PX0 + 65.0, 4.0, zs + 0.2), DARK, 'chassis'),          # a rail the Pi is taped to, along its top edge
        bx('battery tray', (-61.5, -23.0, 116.0 + DZ), (-42.5, 23.0, 117.5 + DZ), DARK, 'chassis'),
        bx('battery tray back', (-61.5, -23.0, 116.0 + DZ), (-60.2, 23.0, 131.2 + DZ), DARK, 'chassis'),
        bx('battery tray front', (-44.0, -23.0, 116.0 + DZ), (-42.5, 23.0, zs + 0.2), DARK, 'chassis'),
        bx('regulator shelf', (-61.5, -23.0, 130.0 + DZ), (-44.0, 23.0, 131.2 + DZ), DARK, 'chassis'),
        bx('neck bracket', (XY + EU1 + 0.65, -(YY + EB), 110.3 + DZ), (XY + EU1 + 3.25, -(YY - EB), zf + FLOOR), DARK, 'chassis'),          # (outside the yaw servo's ears and clamp, then down past them)
        bx('neck bracket low', (XY + EU1 + 0.65, -(YY + EB), 110.3 + DZ), (XN + EB, -15.2, 112.8 + DZ), DARK, 'chassis'),
        # what it carries: chosen in docs/nano/WIRING.md. Masses from the makers' pages where they give one
        bx('Pi Zero 2 W', (PX0, -2.4, 113.3 + DZ), (PX0 + 65.0, 2.55, 143.3 + DZ), PCB, 'Pi Zero 2 W', material='electronics', mass=0.011),
        bx('battery 2S 300 mAh', (-60.0, -22.5, 117.5 + DZ), (-44.0, 22.5, 129.5 + DZ), CELL, 'battery', material='electronics', mass=0.016),
        bx('L 5 V regulator', (-61.3, 0.8, 131.2 + DZ), (-43.5, 21.1, 140.0 + DZ), PCB, 'L 5 V regulator', material='electronics', mass=0.003),
        bx('servo header board', (-61.0, -17.0, 140.0 + DZ), (-45.0, 17.0, 147.0 + DZ), PCB, 'servo header board', material='electronics', mass=0.004),
        bx('IMU', (-41.0, -8.9, zs + 1.5), (-15.0, 8.9, zs + 6.1), PCB, 'IMU', material='electronics', mass=0.0018),          # Adafruit 3886, MPU-6050: 26 x 17.8 x 4.6 mm, 1.8 g
    ]
    # the body: a white pillow round the yaw cases' swept bars and the electronics, open below, the neck servo bare in front of it
    zb0, zb1 = 113.0 + DZ, zs + 6.1 + 3.0
    cxb, ab = -16.5, 50.5
    body = loft('body', msec([(zb0, cxb, 0, ab, 30.5, 5.0), (zb0 + 22, cxb, 0, ab, 30.5, 4.5), (zb1 - 5, cxb, 0, ab - 1, 29.5, 3.5), (zb1, cxb, 0, ab - 7, 25.0, 2.6)]),
                WHITE, 'shell', open=('bottom',))
    body['part'] = 'body shell'
    body['print_wall'] = 0.0012
    bsec = [sec for sec in body['sections']]
    back = lambda y, z: 2 * cxb - T.front_x([(q['z'], q['cx'], q['cy'], q['a'], q['b'], q['n']) for q in bsec], y * MM, z * MM) / MM
    zbs = 124.0 + DZ
    bosses, bscr = [], []
    for k, y in enumerate((-10.0, 10.0)):
        xo = back(y, zbs)
        bosses.append(bx(f'body boss {k}', (xo + 2.2, y - 2.5, zbs - 2.5), (-61.4, y + 2.5, zbs + 2.5), DARK, 'chassis'))
        g = cyl(f'body screw {k}', [(xo - 0.6) * MM, y * MM, zbs * MM], 'x', HEAD_R * MM, 1.5 * MM, STEEL, 'decal')
        bscr.append(fast(g, 'M2', 0.006, [xo * MM, y * MM, zbs * MM], [1.0, 0.0, 0.0]))
    tr += bosses
    add('trunk', tr + [body] + bscr)
    neck = [dict(plate('neck plate', YPL * MM, 4.3 * MM, [(XN * MM, ZN * MM, 4.6 * MM), (XH * MM, ZH * MM, 4.6 * MM)], DARK, 'struct'), part='neck plate', exposed=True)]
    add('neck_link', neck)
    # the head: a plate over both cradles, the hood on it
    hd = [bx('head plate', (XH + U0 - 1.0, -12.0, ZH + CW / 2 + GAP + WALL - 0.05), (XB + 3.0, YPL + 31.5 + FLOOR, ZH + CW / 2 + GAP + WALL + 1.75), DARK, 'head frame'),
          bx('head post', (XB - 4.0, -12.0, ZB + CW / 2 + GAP + WALL - 0.3), (XB, -5.0, ZH + CW / 2 + GAP + WALL + 0.25), DARK, 'head frame')]
    secs = msec([(ZH - 3.0, 66.0 + DX, 0, 42.0 * HS, 37.0 * HS, 4.0), (ZH + 14, 66.0 + DX, 0, 44.0 * HS, 38.0 * HS, 3.0), (ZH + 28, 64.0 + DX, 0, 38.0 * HS, 31.0 * HS, 2.4), (ZH + 34, 62.0 + DX, 0, 26.0 * HS, 20.0 * HS, 2.2)])
    hood = loft('head', secs, WHITE, 'shell', open=('bottom',), clip=T.above(((XH - 20.0) * MM, ZH * MM), ((XH - 8.0) * MM, (ZH - 6.0) * MM)))
    hood['part'] = 'hood'
    hood['print_wall'] = 0.0012
    notch = cut(bx('hood notch', (XH + U0 - 6.0, YPL + 31.5 - 19.5, ZH - 5.0), (XH + U0 + 0.5, YPL + 31.5 + FLOOR + 3.0, ZH + EB + 0.6), DARK, 'hood'), 'head')
    xf = T.front_x(secs, 0.0, (ZH + 16) * MM) / MM
    eye = [dict(cyl('eye ring', [(xf - 1.0) * MM, 0, (ZH + 16) * MM], 'x', 9.0 * MM, 4.0 * MM, AMBER, 'decal'), part='hood'),
           dict(cyl('eye lens', [(xf + 0.4) * MM, 0, (ZH + 16) * MM], 'x', 5.5 * MM, 4.0 * MM, LENS, 'decal'), part='hood')]
    zp0 = ZH + CW / 2 + GAP + WALL + 1.75
    zpu = ZH + CW / 2 + GAP + WALL - 0.05          # the head plate's underside
    posts, hscr = [], []
    for k, (x, y) in enumerate(((50.0 + DX, 0.0), (62.0 + DX, 2.0))):
        posts.append(dict(cyl(f'hood post {k}', [x * MM, y * MM, (zp0 + ZH + 33.0) / 2 * MM], 'z', 3.0 * MM, (ZH + 33.0 - zp0) * MM, WHITE, 'struct'), part='hood'))
        g = cyl(f'hood screw {k}', [x * MM, y * MM, (zpu - 0.75) * MM], 'z', HEAD_R * MM, 1.5 * MM, STEEL, 'decal')
        hscr.append(fast(g, 'M2', 0.006, [x * MM, y * MM, zpu * MM], [0.0, 0.0, 1.0]))
    bill = [bx('upper bill', (88.0 + DX, -22.0, ZB - 3.6), (114.0 + DX, 22.0, ZH - 2.0), ORANGE, 'upper bill', on_cover=True),
            cut(bx('upper bill hollow', (87.0 + DX, -20.4, ZB - 2.0), (112.4 + DX, 20.4, ZH - 3.6), ORANGE, 'upper bill'), 'upper bill')]
    add('head', hd + [hood, notch] + eye + posts + bill + hscr)
    jaw = [dict(plate('jaw arm', (YBH - 3.5) * MM, 3.5 * MM, [(XB * MM, ZB * MM, 4.6 * MM), ((88 + DX) * MM, (ZB - 6.5) * MM, 2.5 * MM)], ORANGE, 'struct'), part='jaw'),
           bx('jaw', (86.0 + DX, -17.0, ZB - 7.5), (110.0 + DX, 17.0, ZB - 4.0), ORANGE, 'jaw')]
    add('beak_link', jaw)
    FR.update(fn=fn, fh=fh, fb=fb)
    for gs in (tr, neck, hd, bill[:1], jaw):
        for g in gs:
            PARTS.setdefault(g['part'], dict(kind='B' if g.get('material') == 'electronics' else 'P', geoms=[]))['geoms'].append(g['name'])
    for p_, gs in (('body shell', [body]), ('hood', [hood] + eye + posts)):
        PARTS.setdefault(p_, dict(kind='P', geoms=[]))['geoms'] += [g['name'] for g in gs]
    return st


def build():
    F = leg()
    for ln, gs in leg_structure(F).items():
        add(ln, gs)
        for g in [g for g in gs if g['role'] != 'cut']:
            PARTS.setdefault(g['part'], dict(kind='P', geoms=[]))['geoms'].append(g['name'])
    GLUED.update({'L sole', 'upper bill'})
    F['steps'].update(upper())
    STEPS.update(F['steps'])
    for ln, l in LINKS.items():
        for g in l['geoms']:
            # bare, as microduck's: the legs, neck and head's servos and brackets. In the body, what isn't inside the shell is the neck's cradle
            if g['role'] in ('struct', 'actuator') and (ln != 'trunk' or g['name'].startswith(('neck pitch', 'neck bracket', 'neck_pitch'))):
                g['exposed'] = True
    return F


def order(st):
    """The build, for tools/assembly.py: sections of steps (caption, [(part or @section, the way it travels in)], [screw patterns]). The right leg is the left's mirror."""
    Z, NZ = [0.0, 0.0, 1.0], [0.0, 0.0, -1.0]

    def mir(v, side):
        return None if v is None else ([v[0], -v[1], v[2]] if side == 'R' else list(v))

    def S_(side, name):
        return rn(name) if side == 'R' else name

    def P(side, pat):
        return pat.replace('^L ', '^R ') if side == 'R' else pat

    def fit(side, k, what, travel=None):
        """servo in, its ear clamp and screws. travel: how far the clamp's last stretch is straight down onto the ears, where a hand brings it in from the side"""
        j = st[k]
        w = mir(j['ring'][1], side) + ([travel] if travel else [])
        return [(f'{what} servo in from its open side, its lead out along the slot', [(S_(side, j['servo'][0]), mir(j['servo'][1], side))], []),
                ('Its ear clamp on the ears, the 2 screws from its bag', [(S_(side, j['ring'][0]), w)], [P(side, j['ear_screws'])])]

    def rear(side, k):
        j = st[k]
        return [('The rear arm over the stub, the bushing pressed in through it from outside; 2 screws into the bar', [(S_(side, j['rear_arm'][0]), mir(j['rear_arm'][1], side)), (S_(side, j['bushing'][0]), mir(j['bushing'][1], side))],
                 [P(side, j['rear_screws'])])]
    out = []
    for side in ('L', 'R'):
        nm = 'left' if side == 'L' else 'right'
        out += [dict(key=f'thigh_{side}', title=f'The {nm} thigh', on=[], steps=[('The thigh', [(S_(side, 'L thigh'), None)], [])] + fit(side, 'hip', 'Hip pitch') + fit(side, 'knee', 'Knee')),
                dict(key=f'shin_{side}', title=f'The {nm} shin', on=[], steps=[('The shin', [(S_(side, 'L shin'), None)], [])] + fit(side, 'ankle', 'Ankle')),
                dict(key=f'leg_{side}', title=f'The {nm} leg', on=[f'thigh_{side}'], steps=[
                    ('The shin onto the knee spline, servo at mid-range; the horn screw', [(f'@shin_{side}', mir(st['knee']['hub'], side))], [P(side, st['knee']['horn_screw'])]),
                    *rear(side, 'knee'),
                    ('The foot onto the ankle spline; the horn screw', [(S_(side, 'L foot'), mir(st['ankle']['hub'], side))], [P(side, st['ankle']['horn_screw'])]),
                    *rear(side, 'ankle'),
                    ('The TPU sole, glued under the foot', [(S_(side, 'L sole'), Z)], []),
                    ('The roll bracket onto the hip pitch spline; the horn screw', [(S_(side, 'L roll bracket'), mir(st['hip']['hub'], side))], [P(side, st['hip']['horn_screw'])]),
                    *rear(side, 'hip')])]
    body = [('The chassis', [('chassis', None)], [])]
    for side in ('L', 'R'):
        body += fit(side, 'yaw', f"{'Left' if side == 'L' else 'Right'} hip yaw")
    body += fit('L', 'neck', 'Neck')
    body += [('The Pi up between the yaw servos, taped to its rail along its top edge, its 13 signal wires and power soldered first', [('Pi Zero 2 W', Z)], []),
             ('The battery into its tray from the side, a strap round both', [('battery', [0.0, -1.0, 0.0])], []),
             ('The 5 V regulators onto the shelf from the sides, foam tape', [('L 5 V regulator', [0.0, -1.0, 0.0]), ('R 5 V regulator', [0.0, 1.0, 0.0])], []),
             ('The servo header board on them, foam tape', [('servo header board', NZ)], []),
             ('The IMU on the spine, foam tape', [('IMU', NZ)], [])]
    hips = []
    for side in ('L', 'R'):
        hips += [('The yaw link up onto the yaw spline, servo at mid-range; the horn screw from below', [(S_(side, 'L yaw link'), mir(st['yaw']['hub'], side))], [P(side, st['yaw']['horn_screw'])]),
                 *rear(side, 'yaw'), *fit(side, 'roll', 'Hip roll')]
    legs = []
    for side in ('L', 'R'):
        legs += [('The leg onto the roll spline, servo at mid-range; the horn screw', [(f'@leg_{side}', mir(st['roll']['hub'], side))], [P(side, st['roll']['horn_screw'])]), *rear(side, 'roll')]
    head = [('The head frame', [('head frame', None)], [])] + fit('L', 'head', 'Head pitch', 10.0) + fit('L', 'beak', 'Beak', 8.0) + [
        ('The jaw onto the beak spline, shut; the horn screw', [('jaw', st['beak']['hub'])], [st['beak']['horn_screw']])]
    out += [dict(key='body', title='The chassis, its servos and electronics', on=[], steps=body),
            dict(key='hips', title='The hip yaw links and roll servos', on=['body'], steps=hips),
            dict(key='legs', title='The legs onto the hips', on=['hips'], steps=legs),
            dict(key='shell', title='The body shell', on=['legs'], steps=[('The body shell down over the chassis, 2 screws from behind', [('body shell', NZ)], [r'^body screw '])]),
            dict(key='headbench', title='The head', on=[], steps=head),
            dict(key='neckhead', title='Neck and head', on=['shell'], steps=[
                ('The neck plate onto the neck spline, servo at mid-range; the horn screw', [('neck plate', st['neck']['hub'])], [st['neck']['horn_screw']]),
                ('The head onto the head pitch spline, from the side; the horn screw', [('@headbench', [-v for v in st['head']['hub']])], [st['head']['horn_screw']]),
                ('The hood down onto the head plate, 2 screws up into its posts', [('hood', NZ)], [r'^hood screw ']),
                ('The upper bill, glued to the hood', [('upper bill', [-1.0, 0.0, 0.0])], [])])]
    return out


def load_sections():
    """tools/buildable.py's LOAD: planes through what carries the duck, in a 3 g landing on one foot (left leg)."""
    X, Y, Z = [1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]
    return [('shin spine', ['L shin'], [(XK - 8.5) * MM, (YB + 7.5) * MM, ((ZA + 21.75 + ZK - BAR['r1']) / 2) * MM], Z),
            ('shin, the knee bar', ['L shin'], [(XK + 2.0) * MM, (YB + 14.0) * MM, (ZK - 15.2) * MM], X),
            ('foot, the hub arm', ['L foot'], [XK * MM, (YB + 29.75) * MM, (ZA - 8.0) * MM], Z),
            ('thigh web', ['L thigh'], [(XK + 10.0) * MM, (YB + 7.5) * MM, (ZP - 2.0) * MM], X),
            ('roll bracket, the hip bar', ['L roll bracket'], [(XP + 15.2) * MM, (YB + 24.0) * MM, ZP * MM], Y),
            ('roll bracket, the post', ['L roll bracket'], [(XP + 15.2) * MM, YR * MM, (ZP + 10.0) * MM], Z),
            ('yaw link, the bar', ['L yaw link'], [(XY - 15.2) * MM, YY * MM, (ZYH + 15.0) * MM], Z)]


def shell_mass(g):
    """PLA at the wall it prints with (3 lines of a 0.4 mm nozzle), not the 2 mm the collision checks give every shell: kg."""
    sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
    import geom as G
    import manifold3d as mf
    import numpy as np
    to = lambda t: mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
    wall = to(G.mesh_of(g)) - to(G.mesh_of(G.inner_of(g, g['print_wall'])))
    return round(float(wall.volume()) * T.MATERIALS['pla']['density'], 5)


def mirror(g):
    m = T.mirror_geom(g)
    m['name'] = rn(m['name'])
    if 'targets' in m:
        m['targets'] = [rn(t) for t in m['targets']]
    if m['kind'] == 'stepbox':
        m['boxes'] = [[[lo[0], -hi[1], lo[2]], [hi[0], -lo[1], hi[2]]] for lo, hi in m['boxes']]
    if 'part' in m:
        m['part'] = rn(m['part'])
    return m


def rn(t):
    return ('R ' + t[2:]) if t.startswith('L ') else ('R_' + t[2:]) if t.startswith('L_') else t


def finish():
    links = {}
    for ln, l in LINKS.items():
        links[ln] = copy.deepcopy(l)
        if ln.startswith('L_'):
            links[rn(ln)] = dict(l, name=rn(ln), geoms=[mirror(g) for g in l['geoms']])
    for ln, l in list(links.items()):
        if ln in ('trunk',) or not ln.startswith(('L_', 'R_')):
            ms = [mirror(g) for g in l['geoms'] if g['name'].startswith(('L ', 'L_'))]
            l['geoms'] = l['geoms'] + ms
    joints = []
    for j in JOINTS:
        joints.append(copy.deepcopy(j))
        if j['name'].startswith('L_'):
            m = copy.deepcopy(j)
            m['name'], m['parent'], m['child'] = rn(j['name']), rn(j['parent']), rn(j['child'])
            m['origin'][1], m['act_c'][1] = -m['origin'][1], -m['act_c'][1]
            if m['axis'] in ('x', 'z'):
                m['lower'], m['upper'] = -j['upper'], -j['lower']
            if j['axis'] == 'y':
                m['out'] = -j['out']
            if m['long'][1] == 'y':
                m['long'] = ('+' if m['long'][0] == '-' else '-') + 'y'
            joints.append(m)
    parts = {}
    for p, v in PARTS.items():
        parts[p] = v
        if p.startswith('L '):
            parts[rn(p)] = dict(kind=v['kind'], geoms=[rn(x) for x in v['geoms']])
    for p, v in list(parts.items()):
        if not p.startswith(('L ', 'R ')):
            v['geoms'] = v['geoms'] + [rn(x) for x in v['geoms'] if x.startswith(('L ', 'L_'))]
    for l in links.values():
        for g in l['geoms']:
            if g['role'] == 'cut':
                continue
            g.setdefault('material', 'actuator' if g['role'] == 'actuator' else 'tpu' if g['color'] == YELLOW else 'pla')
            if 'fast' in g:
                g['material'], g['mass'] = 'steel', T.screw_mass(g['fast'])
    for l in links.values():
        for g in l['geoms']:
            if g['role'] == 'shell' and g.get('print_wall'):
                g['mass'] = shell_mass(g)
    glued = sorted(GLUED | {rn(x) for x in GLUED})
    asm = order(STEPS)
    for sec in asm:
        sec['steps'] = [[c, [[p_, w] for p_, w in items], pats] for c, items, pats in sec['steps']]
    return dict(recipe='nano', recipe_limits=dict(mass_budget_kg=MASS_BUDGET, target_tol=0.35, cannot=['head tucked in'], brief=[]), frame='metres; X forward, Y left, Z up; floor z=0; zero pose standing',
                actuators=ACT, bearings={}, materials=T.MATERIALS, links=links, joints=joints, root='trunk',
                parts=[[p, v['kind'], v['geoms']] for p, v in parts.items()], glued=glued, splined=['jaw'], assembly=asm, load_sections=load_sections())


if __name__ == '__main__':
    build()
    r = finish()
    out = os.path.join(HERE, 'robot_nano.json')
    json.dump(r, open(out, 'w', encoding='utf-8'), indent=1)
    print(f"{len(r['links'])} links, {len(r['joints'])} joints -> {out}")
