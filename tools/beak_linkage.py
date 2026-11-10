"""The beak's drive: solves its four bars over the jaw's whole range, and sweeps the crank and rod against everything they pass.
The spec draws them at the jaw's shut pose only, as one link, so the other checks never see them move.

  python -I tools/beak_linkage.py             the checks; report in reports/beak_linkage.md, exit 1 on any FAIL
  python -I tools/beak_linkage.py --bad CASE  break it on purpose and show the check FAIL (rod hood lock)

  DRIVE   every servo drives its joint: a horn on the child, or for a remote servo, a crank and a rod
  SOLVE   the rod's length closes the loop at every jaw angle, and the crank stays inside the servo's range
  ANGLE   the rod never lies within 30 degrees of the crank or the lever: near that, the servo pushes on a dead centre
  CLEAR   crank and rod, at every jaw angle, clear the rest of the head, and what turns inside it, at every head yaw and roll
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402

SERVO_RANGE = math.radians(150)
MIN_ANGLE = math.radians(30)
STEPS = 9


def rot_y(p, c, q):
    """p (x, z) turned by q about +y through c: +q takes +z toward +x."""
    x, z = p[0] - c[0], p[1] - c[1]
    return (c[0] + x * math.cos(q) + z * math.sin(q), c[1] - x * math.sin(q) + z * math.cos(q))


def solve(crank_c, r, pin0, jaw_c, jaw_pin0, q):
    """The crank pin's place for jaw angle q: on a circle round the servo's shaft, one rod length from the jaw's pin."""
    L = math.dist(pin0, jaw_pin0)
    jp = rot_y(jaw_pin0, jaw_c, q)
    d = math.dist(crank_c, jp)
    if d > r + L or d < abs(r - L):
        return None, jp
    a = (r * r - L * L + d * d) / (2 * d)
    h = math.sqrt(max(r * r - a * a, 0))
    ex, ez = (jp[0] - crank_c[0]) / d, (jp[1] - crank_c[1]) / d
    m = (crank_c[0] + a * ex, crank_c[1] + a * ez)
    cands = [(m[0] + h * ez, m[1] - h * ex), (m[0] - h * ez, m[1] + h * ex)]
    return min(cands, key=lambda c: math.dist(c, pin0)), jp


def angle_at(a, b, c):
    """The angle at b between ba and bc, folded to 0..90 degrees."""
    u, v = np.subtract(a, b), np.subtract(c, b)
    t = abs(np.dot(u, v)) / (np.linalg.norm(u) * np.linalg.norm(v))
    return math.acos(min(t, 1.0))


def moved(m, M):
    return C.posed(m, M)


def turn(o, axis, q):
    """A 4x4 turning by q about the axis through o."""
    k = G.AXIS[axis]
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    R_ = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    T = np.eye(4)
    T[:3, :3] = R_
    T[:3, 3] = np.array(o) - R_ @ np.array(o)
    return T


def check(spec, rep):
    S = C.Solids(spec)
    J = {j['name']: j for j in spec['joints']}
    for j in spec['joints']:
        names = [g['name'] for l in spec['links'].values() for g in l['geoms']]
        if j['remote']:
            ok = 'beak crank' in names and 'beak rod' in names
            rep.add('DRIVE', j['name'], ok, 'a crank and a rod' if ok else 'a remote servo that drives nothing')
        else:
            ok = f"{j['name']} flange" in names
            rep.add('DRIVE', j['name'], ok, '' if ok else 'no horn on the child')
    if not any(j['remote'] for j in spec['joints']):
        return                                     # every servo on its joint's horn: nothing to solve (the jaw's crank and rod are gone)
    H = spec['links']['head']['geoms']
    g = {x['name']: x for x in H}
    beak_g = {x['name']: x for x in spec['links']['beak_link']['geoms']}
    b = spec['beak']
    cc, pin0, jaw_pin0 = tuple(b['crank_c']), tuple(b['pin']), tuple(b['jaw_pin'])
    jb = J['beak']
    jc = (jb['origin'][0], jb['origin'][2])
    r = math.dist(cc, pin0)
    th0 = math.atan2(pin0[0] - cc[0], -(pin0[1] - cc[1]))
    ids = {x['name']: i for i, x in enumerate(H)}
    qs = np.linspace(jb['lower'], jb['upper'], STEPS)
    moving = [n for n in ('beak crank', 'beak crank pin', 'beak rod')]
    others = {}
    for ln, l in spec['links'].items():
        ms = [S.wall.get((ln, i), S.geom[(ln, i)]) for i, x in enumerate(l['geoms']) if x['role'] in ('struct', 'actuator', 'shell') and x['name'] not in moving
              and not (ln == 'head' and x['name'] == 'beak flange')]
        others[ln] = C.union(ms)
    th_all = []
    worst = {}
    for q in qs:
        p, jp = solve(cc, r, pin0, jc, jaw_pin0, q)
        rep.add('SOLVE', f'jaw {q:+.2f} rad', p is not None, 'the rod cannot reach' if p is None else '')
        if p is None:
            continue
        th = math.atan2(p[0] - cc[0], -(p[1] - cc[1]))
        th_all.append(th)
        ang_crank, ang_lever = angle_at(cc, p, jp), angle_at(jc, jp, p)
        rep.add('ANGLE', f'jaw {q:+.2f} rad', min(ang_crank, ang_lever) >= MIN_ANGLE,
                f'{math.degrees(ang_crank):.0f} deg at the crank, {math.degrees(ang_lever):.0f} at the lever')
        # the crank turns about its shaft; the rod: its own end points
        dth = th - th0                                # + toward the front (x) at the bottom of the crank
        crank_T = turn((cc[0], 0, cc[1]), 'y', -dth)
        # the rod's motion in its plane: from its shut line to the new one
        a0 = math.atan2(jaw_pin0[1] - pin0[1], jaw_pin0[0] - pin0[0])
        a1 = math.atan2(jp[1] - p[1], jp[0] - p[0])
        rod_T = np.eye(4)
        R_ = turn((pin0[0], 0, pin0[1]), 'y', -(a1 - a0))
        rod_T = R_.copy()
        rod_T[0, 3] += p[0] - pin0[0]
        rod_T[2, 3] += p[1] - pin0[1]
        placed = {n: C.posed(S.geom[('head', ids[n])], crank_T if 'crank' in n else rod_T) for n in moving}
        beak_T = turn(jb['origin'], 'y', q)
        tests = [('head', others['head'], None), ('beak_link', others['beak_link'], beak_T)]
        for n, m in placed.items():
            for ln, o, T in tests:
                v = C.overlap(m, C.posed(o, T) if T is not None else o)
                worst[(n, ln)] = max(worst.get((n, ln), 0), v)
            for rq in (J['head_roll']['lower'], 0.0, J['head_roll']['upper']):
                R0 = turn(J['head_roll']['origin'], 'x', rq)
                v = C.overlap(m, C.posed(others['head_yaw_link'], R0))
                worst[(n, 'head_yaw_link')] = max(worst.get((n, 'head_yaw_link'), 0), v)
                for yq in np.linspace(J['head_yaw']['lower'], J['head_yaw']['upper'], 21):
                    Y0 = R0 @ turn(J['head_yaw']['origin'], 'z', yq)
                    v = C.overlap(m, C.posed(others['head_pitch_link'], Y0))
                    k = (n, 'head_pitch_link') if abs(rq) < 1e-9 or abs(yq) < 1e-9 else (n, 'combined')
                    worst[k] = max(worst.get(k, 0), v)
    for (n, ln), v in worst.items():
        if ln == 'combined':                       # head roll past 0.2 rad with head yaw past 0.5 rad: the rest of the head meets the yaw servo there too. A limit for what commands it
            rep.add('CLEAR', f'{n} / yaw servo, head rolled and yawed together', v <= 0.01e-6, f'{v * 1e6:.3f} cm3: keep |roll| under 0.1 rad when |yaw| is over 0.5 rad', warn=v > 0.01e-6)
        else:
            rep.add('CLEAR', f'{n} / {ln}', v <= 0.01e-6, f'{v * 1e6:.3f} cm3 at the worst pose')
    if th_all:
        travel = max(th_all) - min(th_all)
        rep.add('SOLVE', 'servo travel', travel <= SERVO_RANGE, f'{math.degrees(travel):.0f} deg of crank for {math.degrees(jb["upper"] - jb["lower"]):.0f} deg of jaw')
        print(f'crank radius {r * 1000:.1f} mm, travel {math.degrees(travel):.0f} deg; rod {math.dist(pin0, jaw_pin0) * 1000:.1f} mm; lever {math.dist(jc, jaw_pin0) * 1000:.1f} mm')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bad')
    a = ap.parse_args()
    spec = G.load()
    head = {x['name']: x for x in spec['links']['head']['geoms']}
    if a.bad == 'rod':                             # no rod: the servo drives nothing
        spec['links']['head']['geoms'] = [x for x in spec['links']['head']['geoms'] if x['name'] != 'beak rod'] + [dict(head['beak rod'], name='gone rod')]
        rep = C.Report()
        names = [g['name'] for g in spec['links']['head']['geoms']]
        print(f"--bad rod: {'CAUGHT' if 'beak rod' not in names else 'NOT CAUGHT'}")
        return 0
    if a.bad == 'hood':                            # the rod's plane out through the hood
        head['beak rod']['y0'] += 0.012
    if a.bad == 'lock':                            # the jaw opened far past the range: the rod cannot reach, or lies on dead centre
        next(j for j in spec['joints'] if j['name'] == 'beak')['upper'] = 2.6
    rep = C.Report()
    check(spec, rep)
    fails = rep.failing()
    if a.bad:
        print(f"--bad {a.bad}: {'CAUGHT' if fails else 'NOT CAUGHT'} ({len(fails)} failing)")
        for r in fails[:3]:
            print('  ', r['group'], r['item'], r['detail'])
        return 0 if fails else 1
    rep.write(G.out_path('reports', 'beak_linkage.md'), 'From tools/beak_linkage.py')
    for r in fails:
        print(r['group'], r['item'], r['detail'])
    print(f'{len(rep.rows)} linkage checks, {len(fails)} failing')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
