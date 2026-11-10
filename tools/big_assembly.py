"""How the big duck goes together: every part and screw in an order a person can follow, and the checks on it. tools/assembly.py does this for the small ducks, on printed parts and their
servo cradles; this one reads the big duck's own data: each screw says what it clamps and what it threads into (spec/big_duck.py `bolt`), and `held` says how the parts that aren't screwed are joined.

  MILLI_RECIPE=big python -I tools/big_assembly.py            the checks: reports/big/assembly.md, the steps: docs/big/ASSEMBLY.md. Exit 1 on any FAIL
  MILLI_RECIPE=big python -I tools/big_assembly.py --bad CASE  break something on purpose and show it caught (join order way bite)

At the zero pose, the whole robot in one frame. Fit, not strength: nothing here says a joint holds.
  JOINED  every two parts of one link (or one moving part of the four-bar) that touch share a screw, or `held` says how they're joined. Printed panels: WARN, as their fixings aren't designed
  APART   no two parts of one link share space (drawn into each other, as the old 0.5 mm overlaps were)
  ONE     each link's frame is one piece through those joints
  HOLD    each part that isn't bought has 2 screws or more, or a `held` joint, so it can't turn about one
  PATH    each part, or a subassembly built on the bench, slides in along its way for 40 mm (less where a step says so) without passing through anything already fitted
  SCREW   each screw goes in along its axis, head and all, through nothing already fitted
  KEY     a hex key reaches each head: 15 mm straight out from it clear of everything fitted by then (a short bit in a low ratchet; the long arm's swing isn't checked)
  BITE    each screw tapped into a part bites 1.25 x its diameter or more, its hole is where it is, and is as long as it is
  EDGE    metal all round each hole out to 0.9 x the screw's diameter from its axis, in each part it passes
Screws into RobStride's own threads aren't measured here (the actuators have no holes in the spec): their engagement is listed from the spec's numbers.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402

DIA = {'M3': 0.003, 'M4': 0.004, 'M5': 0.005}
CLEAR = {'M3': 0.0017, 'M4': 0.00225, 'M5': 0.00275}
TAP = {'M3': 0.00125, 'M4': 0.00165, 'M5': 0.0021}
ROOT = {'M3': 0.0011, 'M4': 0.0015, 'M5': 0.0019}  # a little under the drawn core, so a screw in its own hole reads clear
KEY, KEY_R = 0.015, {'M3': 0.0013, 'M4': 0.0017, 'M5': 0.0023}
BITE, EDGE = 1.25, 0.9
EPS = 0.0001                                       # parts this close touch
TINY = 1e-9                                        # m3: 1 mm3
TRAVEL, STEP = 0.040, 0.004
RS_THREAD = {'RS04 mount': (0.004, 0.006), 'RS04 output': (0.003, 0.0035), 'RS03 mount': (0.005, 0.011), 'RS03 front': (0.008, 0.011), 'RS03 output': (0.004, 0.009)}
AX = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}


def way(s):
    return tuple((-1.0 if s[0] == '-' else 1.0) * v for v in AX[s[1]])


# ---------------------------------------------------------------- the order: left and middle; the right mirrors the left
def S(parts, w, travel=TRAVEL, path=True, note=''):
    return dict(parts=[parts] if isinstance(parts, str) else list(parts), way=w, travel=travel, path=path, note=note)


def B(name, steps, w, travel=TRAVEL, note=''):
    return dict(bench=name, steps=steps, way=w, travel=travel, path=True, note=note)


def leg(s):
    k = lambda t: f'{s} {t}'
    a = lambda t: f'{s}_{t}'
    clip = lambda n: f'{n} clip (buy) DIN 6799 8'
    return [
        S(k('hip yaw inner retainer'), '+z', note='loose, up into the housing'),
        S(k('hip yaw bearing'), '+z', note='61824, into the housing to its shoulder; the inner retainer rests on it'),
        S(k('hip yaw retainer'), '+z', note='8 countersunk M4 into the housing\'s flange'),
        S(k('hip yaw flange'), '+z', note='through the bearing onto the output\'s pins; 8 M4 up into the inner retainer, 9 M5 into the output'),
        S(k('hip yaw beam'), '+z', note='8 M5 into the flange'),
        # hip roll: the front cheek and stator, the yoke on its output, then the back stator and cheek together onto the yoke's pins
        S(k('hip roll cheek F'), '+z'), S(k('hip roll cleat F'), '-x', note='3 M5 up into the beam, 2 through the cheek'),
        S(a('hip_roll actuator F (buy) RS04'), '+x', note='rear step through the cheek; 10 M4 from outside'),
        S(k('hip roll output F'), '+x'),
        S(k('hip roll yoke'), '+x', note='onto the front output\'s pins; 9 M5 through it from inside the hollow'),
        S(k('hip roll output B'), '+x'),
        S(a('hip_roll actuator B (buy) RS04'), '+x', note='onto the back plate\'s pins'),
        S(k('hip roll cheek B'), '+x', note='over the back stator\'s rear step; 10 M4, then 9 M5 into the output from inside the yoke'),
        S(k('hip roll cleat B'), '+x'),
        # hip pitch, inner: the upright and its stator on the bench, then onto the yoke's ring
        B(k('hip pitch inner upright'), [S(k('hip pitch upright in'), '-y'), S(a('hip_pitch actuator I (buy) RS04'), '-y', note='10 M4 from the ring side')], '-y',
          note='onto the yoke\'s ring; 5 M5 through the upright into it'),
        S(k('hip pitch output I'), '-y', note='onto the output\'s pins; 9 M5, low heads'),
        # the thigh, on the bench: plates and spacers, and each knee drive on its rear plate with its crank and rod
        B(k('thigh'), [
            S(k('thigh plate in'), '+y'), S(k('thigh spacer hip'), '-y'), S(k('thigh spacer mid'), '-y'), S(k('thigh plate out'), '-y', note='8 countersunk M5 into the spacers'),
            B(k('knee drive I'), [S(k('thigh rear plate in'), '+y'), S(a('knee actuator I (buy) RS04'), '-y', note='10 M4'),
                                  S(k('knee crank I'), '-y', note='9 M5 into the output'), S(k('knee crank pin I'), '-y', note='pressed in'),
                                  S([k('knee rod I'), k('knee rod I bush A'), k('knee rod I bush B')], '-y', travel=0.008, note='on the pin'),
                                  S(clip(k('knee crank pin I')), '-z', path=False, note='E-clip, from the side'),
                                  S(k('thigh tie 0i'), '-y'), S(k('thigh tie 1i'), '-y', note='an M5 into each from the rear plate')], '+y', note='the ties onto the thigh plate; an M5 into each from inside the thigh'),
            B(k('knee drive O'), [S(k('thigh rear plate out'), '-y'), S(a('knee actuator O (buy) RS04'), '+y', note='10 M4'),
                                  S(k('knee crank O'), '+y', note='9 M5 into the output'), S(k('knee crank pin O'), '+y', note='pressed in'),
                                  S([k('knee rod O'), k('knee rod O bush A'), k('knee rod O bush B')], '+y', travel=0.008, note='on the pin'),
                                  S(clip(k('knee crank pin O')), '-z', path=False, note='E-clip, from the side'),
                                  S(k('thigh tie 0o'), '+y'), S(k('thigh tie 1o'), '+y', note='an M5 into each from the rear plate')], '-y', note='the ties onto the thigh plate; an M5 into each from inside the thigh'),
        ], '-y', note='onto the inner output; 6 M5 from inside the thigh'),
        B(k('hip pitch outer'), [S(k('hip pitch upright out'), '-y'), S(a('hip_pitch actuator O (buy) RS04'), '+y', note='10 M4'),
                                 S(k('hip pitch output O'), '+y', note='9 M5, low heads')], '-y', note='onto the thigh\'s outer plate; 6 M5 from inside the thigh'),
        S(k('hip pitch bridge'), '-z', note='4 M4 through the uprights into its ends'),
        # the shin, on the bench: the ankle's stack, the cap, the tube, the top with the knee's hubs
        B(k('shin'), [
            B(k('ankle stack'), [S(k('ankle mount plate'), '+y'), S(a('ankle_pitch actuator (buy) RS04'), '-y', note='10 M4'),
                                 S(k('ankle spacer'), '+y'), S(k('ankle plate in'), '+y', note='5 M4 through it and the ring into the shoulder plate'),
                                 S(k('ankle stub'), '+y', note='3 M4 in counterbores')], '-z', note='2 M4 from inside the cap'),
            S(k('shin cap'), '+z', travel=0.0, path=False, note='the stack hangs from it'),
            S(k('shin cleat front'), '-z'), S(k('shin cleat back'), '-z', note='2 M4 up through the cap into each'),
            S(k('shin tube'), '-z', note='2 M4 through each end wall into its cleat'),
            B(k('knee top'), [S(k('shin top'), '-z'), S(k('shin spacer'), '-z', note='2 M5 up through the top'),
                              S(k('shin hub in'), '+y'), S(k('shin hub out'), '-y', note='2 countersunk M4 through the first hub and the spacer into the second')], '-z',
              note='into the tube; 8 M4 through its walls'),
        ], '+z', note='up between the thigh plates'),
        S(k('knee bearing'), '+y', note='6908, pressed into the thigh plate'), S(k('knee far bearing'), '-y'),
        S(k('knee shaft'), '+y', note='through both bearings and both hubs, keyed'),
        B(k('knee lever I'), [S(k('knee lever I'), '+y'), S(k('knee lever pin I'), '+y', note='pressed in')], '+y', travel=0.010, note='onto the shaft\'s key, its pin through the rod\'s far bush; circlip'),
        S(clip(k('knee lever pin I')), '-z', path=False, note='E-clip'),
        B(k('knee lever O'), [S(k('knee lever O'), '-y'), S(k('knee lever pin O'), '-y', note='pressed in')], '-y', travel=0.010, note='onto the shaft\'s key, its pin through the rod\'s far bush; circlip'),
        S(clip(k('knee lever pin O')), '-z', path=False, note='E-clip'),
        # the ankle and foot
        S(k('ankle bearing'), '+y', note='6206, onto the stub'),
        S(k('ankle output'), '-y', note='onto the output\'s pins; 9 M5'),
        S(k('ankle bearing housing'), '+y', note='onto the bearing'),
        B(k('foot'), [S(k('foot plate'), '-z'), S(k('ankle ear in'), '-z'), S(k('ankle ear out'), '-z', note='3 M4 up through the plate into each'),
                      S(k('shoe rear'), '-z'), S(k('shoe front'), '-z', note='2 M4 up through the plate into inserts in each'),
                      S(k('sole rear'), '+z'), S(k('sole front'), '+z', note='bonded')], '+z',
          note='the ears up beside the output plate and the housing: 6 M5 through the outer ear, 3 through the inner'),
        # the outer thigh covers
        *[S(k(f'thigh cover post out {t}'), '-y') for t in ('hip', 'rear', 'knee')],
        *[S(k(f'thigh cover out {t}'), '-y', note='an M4 through it and its post into the rear plate') for t in ('hip', 'rear', 'knee')],
    ]


def yaw_top(s):
    """Before anything else goes on the deck: the screws round the yaw housing are under the neck tie's and the computer's."""
    return [S(f'{s} hip yaw housing', '+z', note='bolted to the deck from above, 10 M5'),
            S(f'{s}_hip_yaw actuator (buy) RS04', '+z', note='up into the housing, its rear step through the deck; 10 M4 from above')]


ORDER = [
    S('deck plate', '-z'),
    *yaw_top('L'), *yaw_top('R'),
    *[S(f'body post {i}', '-z', note='an M5 up through the deck') for i in range(4)],
    S('neck tie', '-z', note='2 M5 up through the deck'),
    S('neck plate L', '-z'), S('neck plate R', '-z', note='2 M5 through each into the tie'),
    S('neck cleat L', '-z'), S('neck cleat R', '-z', note='2 M5 up through the deck into each, 1 through it into its plate'),
    S('neck_pitch actuator L (buy) RS04', '+y', note='10 M4 from outside'), S('neck_pitch actuator R (buy) RS04', '-y'),
    S('computer standoff 0', '-z'), S('computer standoff 1', '-z', note='2 M3 up through the deck into each'),
    S('computer and power boards', '-z'), S('battery 13S', '-z'), S('battery BMS, fuse and case, allowance', '-z', note='strapped'),
    S('upper plate', '-z', note='an M5 down into each post'),
    *leg('L'), 'RIGHT LEG',
    # the neck, on the two neck outputs
    S('neck output L', '+y'), S('neck output R', '-y'),
    S('neck spine', '-z', note='9 M5 through each boss from the window in the spine'),
    B('head pitch drive', [S('head pitch stator plate L', '+y'), S('head_pitch actuator L (buy) RS04', '+y', note='10 M4'),
                           S('head pitch drive sprocket L (buy) 13T #40', '+y', note='9 M5')], '-y', note='beside the spine'),
    S('head pitch stator tie L', '-z', note='3 M5 x 80 through the plate and the tie into the spine'),
    S('head pitch L bearing', '-y'), S('head pitch R bearing', '+y', note='6908s, pressed into the spine'),
    S('head pitch sprocket L (buy) 25T #40', '+x'), S('head pitch yoke', '-z'), S('head pitch yoke R', '-z'),
    S('head pitch shaft', '-y', note='through the yokes, the sprocket and both bearings, keyed'),
    S(['head pitch chain L (buy) #40 a', 'head pitch chain L (buy) #40 b'], '-z', path=False, note='the chain, wrapped on'),
    S('head pitch bridge', '-z', note='4 M4 down into the yokes'),
    S('head_yaw actuator (buy) RS03', '-z', note='4 M4 from under the bridge'),
    *[S(f'head yaw front post {i}', '-z') for i in range(4)],
    S('head yaw front ring', '-z', note='8 countersunk M4 into the RS03\'s front holes, 4 into the posts; an M5 up through the bridge into each post'),
    S('head yaw output', '-z', note='6 M4 into the output'), S('head yaw plate', '-z', note='6 M4 into the output plate'),
    S('head roll cradle', '-z', note='3 M4 up through the plate'),
    S('head_roll actuator (buy) RS03', '-x', note='4 M4 from behind the cradle'),
    *[S(f'head roll front post {i}', '-x') for i in range(4)],
    S('head roll front ring', '-x', note='8 countersunk M4 into the RS03, 4 into the posts; an M5 through the cradle into each post'),
    S('head roll output', '-x', note='6 M4 into the output'),
    B('head frame', [S('head main frame', '-x'), S('head rail L', '+x'), S('head rail R', '+x', note='through the main frame\'s notches; 2 M4 into it'),
                     S('head rear frame', '+x'), S('head front frame', '-x', note='2 M4 into each rail\'s end'),
                     S('head beak lug L', '-y'), S('head beak lug R', '+y', note='2 countersunk M4 into each rail'),
                     S('head beak mount', '-z', note='2 M4 through each rail'), S('beak actuator (buy) RS00', '-z', note='placeholder'),
                     S('head boards and camera', '-z', note='placeholder')], '-x', note='onto the roll output; 6 M4'),
    B('beak', [S('beak tray frame', '-z'), S('beak hinge L', '-y'), S('beak hinge R', '+y', note='2 M4 into the tray\'s edge')], '+z', note='its hinge pins: not designed'),
]


def mirror_step(st):
    m = lambda t: ('R ' + t[2:]) if t.startswith('L ') else ('R_' + t[2:]) if t.startswith('L_') else t
    w = st['way']
    w = (('-' if w[0] == '+' else '+') + 'y') if w[1] == 'y' else w
    if 'bench' in st:
        return dict(st, bench=m(st['bench']), steps=[mirror_step(x) for x in st['steps']], way=w)
    return dict(st, parts=[m(p) for p in st['parts']], way=w)


def full_order():
    out = []
    for st in ORDER:
        if st == 'RIGHT LEG':
            out += [mirror_step(x) for x in leg('L')]
        else:
            out.append(st)
    return out


# ---------------------------------------------------------------- the parts and screws
def cylm(c, d, r, h):
    """A cylinder from c, h along the axis-aligned d."""
    ax = 'xyz'[int(np.argmax(np.abs(d)))]
    cc = [c[i] + d[i] * h / 2 for i in range(3)]
    return C.to_manifold(G.mesh_of(dict(kind='cyl', c=cc, axis=ax, r=r, h=abs(h))))


class Parts:
    def __init__(self, spec, S_):
        self.spec, self.S = spec, S_
        self.geoms, self.part, self.dom, self.kind, self.screws = {}, {}, {}, {}, []
        held = spec.get('held', [])
        prefix = {}
        for ln, l in spec['links'].items():
            for i, g in enumerate(l['geoms']):
                if g['role'] == 'actuator':
                    prefix[g['name'].split(' (buy)')[0]] = g['name']
        for ln, l in spec['links'].items():
            for i, g in enumerate(l['geoms']):
                n = g['name']
                if g['role'] == 'cut' or g.get('rotor') or (g.get('stand_in') and n.endswith(' seat')) or n.endswith(' shank'):
                    continue
                if 'fast' in g and 'size' in g['fast']:
                    sh = next(j for j, x in enumerate(l['geoms']) if x['name'] == n + ' shank')
                    self.screws.append(dict(g['fast'], name=n, link=ln, hidx=(ln, i), sidx=(ln, sh)))
                    continue
                p = g.get('unit') or g.get('part') or n
                for suf in (' rear step', ' cable sleeve'):
                    if n.endswith(suf) and n[:-len(suf)] in prefix:
                        p = prefix[n[:-len(suf)]]
                self.part[n] = p
                self.spacer = getattr(self, 'spacer', {})
                self.spacer[p] = self.spacer.get(p) or g.get('spacer', False)
                self.geoms.setdefault(p, []).append((ln, i))
                self.dom.setdefault(p, set()).add(g.get('group') or ln)
                bought = '(buy)' in p or bool(g.get('unit'))
                panel = g.get('material') in ('petg', 'tpu')
                self.kind[p] = 'bought' if bought else 'panel' if panel else 'box' if g.get('material') in ('cells', 'electronics') else 'frame'
                if g.get('real'):
                    self.hub = getattr(self, 'hub', {})
                    self.hub[tuple(round(v, 5) for v in g['real']['hub'])] = p
        # a panel is a shell: its wall, as other parts may sit in its hollow
        self.solid = {p: C.union([S_.wall[k] if k in S_.wall else S_.geom[k] for k in ks]) for p, ks in self.geoms.items()}
        self.box = {p: C.bbox(m) for p, m in self.solid.items()}
        for s in self.screws:
            s['parts'] = [self.name(t) for t in s.get('clamp', [])]
            s['tap_part'] = self.name(s['tap']) if s.get('tap') else None
            s['act'] = self.hub.get(tuple(round(v, 5) for v in s['tap_hub'])) if s.get('tap_hub') else None
            s['needs'] = set(s['parts'] + [x for x in (s['tap_part'], s['act']) if x])
            d, p, L = s['d'], s['p'], s['L']
            rh, hh = s['head']
            back = [-v for v in d]
            if s.get('csk'):
                head, top = cylm(p, d, rh, hh), p
            else:
                head, top = cylm(p, back, rh, hh), [p[i] + back[i] * hh for i in range(3)]
            s['solid'] = head + cylm(p, d, ROOT[s['size']], L)
            s['top'] = top
            s['box'] = C.bbox(s['solid'])
        self.held = {frozenset((self.name(h['a']), self.name(h['b']))): h['how'] for h in held if self.name(h['a']) and self.name(h['b'])}
        self.bad_names = [h for h in held if not (self.name(h['a']) and self.name(h['b']))]

    def name(self, t):
        return self.part.get(t, t if t in self.geoms else None)


def near(a, b, pad):
    (alo, ahi), (blo, bhi) = a, b
    return not (np.any(alo - pad > bhi) or np.any(blo - pad > ahi))


def moved(m, w, t):
    return m.translate([-w[i] * t for i in range(3)])


# ---------------------------------------------------------------- the checks
def check_joins(P, rep):
    joined = set()
    for s in P.screws:
        ps = sorted(s['needs'])
        joined |= {frozenset((a, b)) for a in ps for b in ps if a != b}
    doms = {}
    for p, ds in P.dom.items():
        for d in ds:
            doms.setdefault(d, []).append(p)
    unjoined, apart, links = [], [], {}
    for d, ps in doms.items():
        graph = {p: set() for p in ps}
        for x in range(len(ps)):
            for y in range(x + 1, len(ps)):
                a, b = ps[x], ps[y]
                if not near(P.box[a], P.box[b], EPS):
                    continue
                v = C.overlap(P.solid[a], P.solid[b])
                touch = v > 1e-12 or any(C.overlap(P.solid[a], P.solid[b].translate([e * EPS * (k == i) for k in range(3)])) > 1e-12 for i in range(3) for e in (1, -1))
                if not touch:
                    continue
                pair = frozenset((a, b))
                if v > TINY and pair not in P.held:
                    apart.append((d, a, b, v))
                if not (pair in joined or pair in P.held):
                    unjoined.append((d, a, b, 'panel' in (P.kind[a], P.kind[b])))
        for pair in joined | set(P.held):            # joined by screws or held, touching or not (a sprocket on its output's face)
            a, b = tuple(pair)
            if a in graph and b in graph:
                graph[a].add(b)
                graph[b].add(a)
        links[d] = graph
    for d, a, b, v in apart:
        pan = 'panel' in (P.kind[a], P.kind[b])
        rep.add('APART', f'{a} / {b}', False, f'{d}: drawn into each other, {v * 1e6:.2f} cm3' + (' (a printed panel: not relieved for what it meets)' if pan else ''), warn=pan)
    for d, a, b, pan in unjoined:
        rep.add('JOINED', f'{a} / {b}', False, f'{d}: they touch, and no screw or held joint joins them' + (' (a printed panel: its fixings aren\'t designed)' if pan else ''), warn=pan)
    for d, graph in links.items():
        frame = [p for p in graph if P.kind[p] != 'panel']
        if not frame:
            continue
        seen, todo = set(), [frame[0]]
        while todo:
            x = todo.pop()
            if x in seen:
                continue
            seen.add(x)
            todo += [y for y in graph[x] if P.kind[y] != 'panel']
        loose = [p for p in frame if p not in seen]
        # a moving part of the four-bar is one piece with its pin, joined to its neighbours by joints: a part alone in its group is fine
        rep.add('ONE', d, not loose, f'not joined to the rest: {", ".join(loose[:6])}' if loose else f'{len(frame)} parts')
    for h in P.bad_names:
        rep.add('JOINED', f"held: {h['a']} / {h['b']}", False, 'names a part that isn\'t in the spec')
    n = {}
    for s in P.screws:
        for p in s['needs']:
            n[p] = n.get(p, 0) + 1
    heldp = {p for k in P.held for p in k}
    for p, k in P.kind.items():
        if k in ('frame', 'panel') and n.get(p, 0) < 2 and p not in heldp and not P.spacer.get(p):
            rep.add('HOLD', p, False, f'{n.get(p, 0)} screw(s) and nothing else holds it: it turns about one', warn=(k == 'panel'))
    return joined


def check_screws(P, rep):
    """BITE and EDGE, from the shapes. Into an actuator: listed, not measured."""
    rs = {}
    for s in P.screws:
        d, p, L, sz = s['d'], s['p'], s['L'], s['size']
        D = DIA[sz]
        if s['act']:
            k = ('RS03' if 'RS03' in s['act'] else 'RS04') + (' front' if 'front ring' in s['name'] else ' output' if s['act'] not in s['parts'] and 'mount screw' not in s['name'] else ' mount')
            rs[k] = rs.get(k, 0) + 1
        shank_root = cylm(p, d, ROOT[sz], L)
        for q in s['parts'] + ([s['tap_part']] if s['tap_part'] else []):
            v = C.overlap(shank_root, P.solid[q])
            if v > 2e-10:
                rep.add('BITE', s['name'], False, f'its shank runs into {q}: {v * 1e9:.0f} mm3 (no hole there, or too short a one)')
        for q in s['parts']:
            ring = cylm(p, d, CLEAR[sz] + 0.0006, L) - cylm(p, d, CLEAR[sz] + 0.0002, L)
            area = np.pi * ((CLEAR[sz] + 0.0006) ** 2 - (CLEAR[sz] + 0.0002) ** 2)
            ln = C.overlap(ring, P.solid[q]) / area
            if ln < 0.0003:
                rep.add('BITE', s['name'], False, f'doesn\'t pass through {q}, which it clamps')
                continue
            ed = cylm(p, d, EDGE * D, L) - cylm(p, d, CLEAR[sz] + 0.0002, L)
            want = np.pi * ((EDGE * D) ** 2 - (CLEAR[sz] + 0.0002) ** 2) * ln
            f = C.overlap(ed, P.solid[q]) / want
            if f < 0.97:
                rep.add('EDGE', s['name'], False, f'{q}: metal round its hole only {f:.0%} of the way out to {EDGE * D * 1000:.1f} mm')
        q = s['tap_part']
        if q:
            ring = cylm(p, d, TAP[sz] + 0.0006, L) - cylm(p, d, TAP[sz] + 0.0002, L)
            area = np.pi * ((TAP[sz] + 0.0006) ** 2 - (TAP[sz] + 0.0002) ** 2)
            bite = C.overlap(ring, P.solid[q]) / area
            s['bite'] = bite
            rep.add('BITE', s['name'], bite >= BITE * D - 0.0003, f'{bite * 1000:.1f} mm into {q}, {bite / D:.2f} x its diameter' + ('' if bite >= BITE * D - 0.0003 else f', under {BITE}'))
            ed = cylm(p, d, EDGE * D, L) - cylm(p, d, TAP[sz] + 0.0002, L)
            want = np.pi * ((EDGE * D) ** 2 - (TAP[sz] + 0.0002) ** 2) * bite
            f = C.overlap(ed, P.solid[q]) / max(want, 1e-12)
            if f < 0.97:
                rep.add('EDGE', s['name'], False, f'{q}: metal round its thread only {f:.0%} of the way out to {EDGE * D * 1000:.1f} mm')
        elif not s['act']:
            rep.add('BITE', s['name'], False, 'threads into nothing')
    for k, n in sorted(rs.items()):
        e, depth = RS_THREAD[k]
        rep.add('BITE', f'{k} screws, into RobStride\'s threads', True, f'{n}: {e * 1000:.1f} mm of a {depth * 1000:.1f} mm thread, from the spec (not measured)')


class Fitter:
    def __init__(self, P, rep):
        self.P, self.rep = P, rep
        self.done = set()
        self.log = []

    def solids(self, placed):
        return placed

    def clear(self, m, mbox, placed, skip=()):
        """The largest overlap of m with what's placed, and with what."""
        worst, who = 0.0, None
        for n, s, b in placed:
            if n in skip or not near(mbox, b, 0.0):
                continue
            v = C.overlap(m, s)
            if v > worst:
                worst, who = v, n
        return worst, who

    def fit_screws(self, placed, names, step):
        P = self.P
        for s in P.screws:
            if s['name'] in self.done or not s['needs'] <= names:
                continue
            self.done.add(s['name'])
            d = s['d']
            skip = {s['act']} if s['act'] else set()
            # in along its axis: the whole screw, from a little more than its length out
            for t in np.arange(0.002, s['L'] + 0.0015, 0.002):      # out till its tip leaves its hole
                m = moved(s['solid'], d, t)
                v, who = self.clear(m, C.bbox(m), placed, skip)
                if v > 2e-10:
                    self.rep.add('SCREW', s['name'], False, f'step {step}: going in, {t * 1000:.0f} mm out it passes through {who}')
                    break
            key = cylm(s['top'], [-x for x in d], KEY_R[s['size']], KEY)
            v, who = self.clear(key, C.bbox(key), placed)
            self.rep.add('KEY', s['name'], v <= 2e-10, f'step {step}' + ('' if v <= 2e-10 else f': {who} is in the key\'s way'))
            placed.append((s['name'], s['solid'], s['box']))
            names.add(s['name'])

    def run(self, steps, placed, names, label=''):
        for k, st in enumerate(steps):
            tag = f'{label}{k + 1}'
            w = way(st['way'])
            if 'bench' in st:
                sub, subn = [], set()
                self.log.append((tag, st['bench'], st['way'], st['note'], True))
                self.run(st['steps'], sub, subn, tag + '.')
                ms = [s for _, s, _ in sub]
                parts = [n for n, _, _ in sub]
                title = st['bench']
            else:
                for p in st['parts']:
                    if p not in self.P.solid:
                        self.rep.add('ALL', p, False, f'step {tag}: not a part in the spec')
                parts = [p for p in st['parts'] if p in self.P.solid]
                ms = [self.P.solid[p] for p in parts]
                title = ', '.join(parts)
                self.log.append((tag, title, st['way'], st['note'], False))
                for p in parts:
                    if p in self.fitted:
                        self.rep.add('ALL', p, False, f'step {tag}: fitted twice')
                    self.fitted.add(p)
                sub = [(p, self.P.solid[p], self.P.box[p]) for p in parts]
            if st['path'] and st['travel'] > 0 and ms:
                M = C.union(ms)
                bad = None
                for t in np.arange(STEP, st['travel'] + 1e-9, STEP):
                    m = moved(M, w, t)
                    v, who = self.clear(m, C.bbox(m), placed)
                    if v > TINY:
                        bad = (t, who, v)
                        break
                self.rep.add('PATH', f'{tag} {title}', bad is None, '' if bad is None else f'along {st["way"]}: {bad[0] * 1000:.0f} mm out it passes through {bad[1]}, {bad[2] * 1e6:.2f} cm3')
            placed += sub
            names |= {n for n, _, _ in sub}
            self.fit_screws(placed, names, tag)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bad')
    a = ap.parse_args()
    t0 = time.time()
    spec = G.load()
    order = full_order()
    if a.bad == 'join':                            # a cheek with its screws gone: it touches the beam's cleat and nothing joins them
        for l in spec['links'].values():
            for g in l['geoms']:
                if g.get('fast', {}).get('clamp') == ['L hip roll cheek F']:
                    g['fast']['clamp'] = []
    if a.bad == 'order':                           # the beam before the flange: the flange can't come up past it
        i = next(i for i, s in enumerate(order) if s.get('parts') == ['L hip yaw beam'])
        j = next(i for i, s in enumerate(order) if s.get('parts') == ['L hip yaw flange'])
        order[i], order[j] = order[j], order[i]
    if a.bad == 'way':                             # the hip yaw actuator dropped in from above, through the deck
        i = next(i for i, s in enumerate(order) if s.get('parts') == ['L_hip_yaw actuator (buy) RS04'])
        order[i] = dict(order[i], way='-z')
    if a.bad == 'bite':                            # the ties' screws 6 mm shorter
        for l in spec['links'].values():
            for g in l['geoms']:
                if g.get('fast', {}).get('tap') == 'L thigh tie 0i':
                    g['fast']['L'] -= 0.006
    S_ = C.Solids(spec)
    rep = C.Report()
    P = Parts(spec, S_)
    check_joins(P, rep)
    check_screws(P, rep)
    F = Fitter(P, rep)
    F.fitted = set()
    placed, names = [], set()
    F.run(order, placed, names)
    for p, k in P.kind.items():
        if p not in F.fitted and k != 'panel':
            rep.add('ALL', p, False, 'not in the order')
    left = [s['name'] for s in P.screws if s['name'] not in F.done]
    for n in left:
        rep.add('ALL', n, False, 'never fitted: its parts never all came together')
    panels = sorted(p for p, k in P.kind.items() if k == 'panel' and p not in F.fitted)
    rep.add('ALL', 'printed panels', True, f'{len(panels)} panels and covers aren\'t in the order: their fixings aren\'t designed', warn=True)
    fails = rep.failing()
    if a.bad:
        grp = {'join': 'JOINED', 'order': 'PATH', 'way': 'PATH', 'bite': 'BITE'}[a.bad]
        caught = [r for r in fails if r['group'] == grp]
        print(f"--bad {a.bad}: {'CAUGHT' if caught else 'NOT CAUGHT'} ({len(caught)} failing rows)")
        for r in caught[:4]:
            print('  ', r['group'], r['item'], r['detail'][:150])
        return 0 if caught else 1
    rep.write(G.out_path('reports', 'assembly.md'), f"spec: `spec/robot_big.json`, {time.strftime('%Y-%m-%d %H:%M')}, {time.time() - t0:.0f} s")
    write_steps(F, P)
    for r in rep.rows:
        if r['status'] != 'PASS':
            print(f"{r['status']} {r['group']:6} {r['item'][:70]}: {r['detail'][:170]}")
    n = {g: sum(1 for r in rep.rows if r['group'] == g) for g in ('JOINED', 'APART', 'ONE', 'HOLD', 'PATH', 'SCREW', 'KEY', 'BITE', 'EDGE')}
    print(f"{len(P.geoms)} parts, {len(P.screws)} screws, {len(F.log)} steps; rows {n}")
    print(f"{len(rep.rows)} checks, {len(fails)} failing -> {G.out_path('reports', 'assembly.md')}")
    return 1 if fails else 0


def write_steps(F, P):
    sizes = {}
    for s in P.screws:
        k = f"{s['size']} x {round(s['L'] * 1000)}" + (' countersunk' if s.get('csk') else '')
        sizes[k] = sizes.get(k, 0) + 1
    out = ['# Big duck: how it goes together', '',
           '**NOT SAFE TO BUILD.** This order is checked for fit: each part has a way in and each screw a straight path for its key (`tools/big_assembly.py`, `reports/big/assembly.md`). Nothing here says a joint is strong enough.', '',
           'Generated by `tools/big_assembly.py` from its order and the spec: don\'t edit by hand. The right leg is the left\'s mirror, in the same order. A **bench** step builds a subassembly on the bench, which then goes on as one piece. '
           'Each step says the way the part moves as it goes on, in the robot\'s frame (X forward, Y left, Z up). Screws go in as soon as everything they join is there.', '',
           'Printed panels and covers (body, hood, face, bill, shin, beak) aren\'t in it: their fixings aren\'t designed.', '']
    for tag, title, w, note, bench in F.log:
        depth = tag.count('.')
        out.append(f"{'  ' * depth}- **{tag}** {'bench: ' if bench else ''}{title}, along {w}{': ' + note if note else ''}")
    out += ['', '## Screws', '', '| Size (ISO 4762 unless countersunk, ISO 10642) | Count |', '|---|---|']
    out += [f'| {k} | {v} |' for k, v in sorted(sizes.items())]
    out += ['', f'{len(P.screws)} in all, and the joints that aren\'t screws: `HELD` in `spec/big_duck.py`.']
    p = os.path.join(G.ROOT, 'docs', 'big', 'ASSEMBLY.md')
    open(p, 'w', encoding='utf-8').write('\n'.join(out) + '\n')


if __name__ == '__main__':
    sys.exit(main())
