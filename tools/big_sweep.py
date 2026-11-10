"""What tools/checks.py leaves out on the big duck: its thigh covers are 'decal', so no sweep tests them. Sweeps them through the hip pitch's range against everything above that joint.

  MILLI_RECIPE=big python -I tools/big_sweep.py

Zero pose for every other joint. It doesn't test hip pitch and hip roll together, or the rods, whose real motion is the four-bar's and isn't in the model.
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402
import mass_model as MM  # noqa: E402

C = MM.C
TOUCH = 2e-8                                       # m3: 0.02 cm3


def rotated(solid, origin, ang):
    """A solid turned ang about a y axis through origin."""
    return solid.translate([-v for v in origin]).rotate([0.0, math.degrees(ang), 0.0]).translate(origin)


def main():
    s = G.load()
    S = C.Solids(s)
    j = next(x for x in s['joints'] if x['name'] == 'L_hip_pitch')
    above = ('trunk', 'L_hip_yaw_link', 'L_hip_roll_link')
    fixed = [(ln, g['name'], S.geom[(ln, i)]) for ln in above for i, g in enumerate(s['links'][ln]['geoms']) if S.geom.get((ln, i)) is not None and g['role'] != 'cut']
    bad = {}
    n = 0
    for i, g in enumerate(s['links']['L_hip_pitch_link']['geoms']):
        if 'thigh cover' not in g['name'] or S.geom.get(('L_hip_pitch_link', i)) is None:
            continue
        n += 1
        m = S.geom[('L_hip_pitch_link', i)]
        for a in np.linspace(j['lower'], j['upper'], 21):
            r = rotated(m, j['origin'], float(a))
            for ln, name, f in fixed:
                v = C.overlap(r, f)
                if v > TOUCH:
                    k = (g['name'], name)
                    if k not in bad or v > bad[k][0]:
                        bad[k] = (v, float(a))
    for (a, b), (v, ang) in sorted(bad.items(), key=lambda kv: -kv[1][0]):
        print(f'{a} | {b} | {v * 1e6:.1f} cm3 at {ang:+.2f} rad')
    print(f'{n} covers swept over {j["lower"]:+.2f} to {j["upper"]:+.2f} rad, {len(bad)} pairs clash')
    four_bar(s, S)
    return json.dumps(sorted(k[0] for k in bad))


def four_bar(s, S):
    """The left knee's cranks and rods through their real motion, at the spec's 13 sampled knee angles, against the thigh, the shin and each other. A rod's eye round its lever's pin only clears if the angles are right, so this tests the four-bar closes too."""
    thigh, knee = 'L_hip_pitch_link', 'L_knee_link'
    J = {j['name']: j for j in s['mech']['joints']}
    kj = next(x for x in s['joints'] if x['name'] == 'L_knee')
    solid = lambda ln, pick: [(g['name'], S.geom[(ln, i)]) for i, g in enumerate(s['links'][ln]['geoms']) if S.geom.get((ln, i)) is not None and pick(g)]
    fixed = solid(thigh, lambda g: not g.get('group') and 'thigh cover' not in g['name'] and not g.get('rotor'))
    shin = solid(knee, lambda g: not g.get('rotor'))
    parts = {}
    for t in 'IO':
        parts[f'crank {t}'] = solid(thigh, lambda g, t=t: g.get('group') == f'L_knee_crank_{t}' and not g.get('rotor'))
        parts[f'rod {t}'] = solid(thigh, lambda g, t=t: g.get('group') == f'L_knee_rod_{t}')
    pin0 = {t: J[f'L_knee_rod_{t}']['origin'] for t in 'IO'}
    axis = {t: J[f'L_knee_crank_{t}']['origin'] for t in 'IO'}
    bad = {}
    for sm in s['mech']['samples']:
        posed = {}
        for t in 'IO':
            posed[f'crank {t}'] = [(n, rotated(m, axis[t], sm['crank'])) for n, m in parts[f'crank {t}']]
            a, c = pin0[t], axis[t]
            ang = sm['crank']
            dx, dz = a[0] - c[0], a[2] - c[2]
            # where the crank's pin has gone (a turn about +y: x' = x cos + z sin, z' = -x sin + z cos)
            nx, nz = dx * math.cos(ang) + dz * math.sin(ang), -dx * math.sin(ang) + dz * math.cos(ang)
            shift = [c[0] + nx - a[0], 0.0, c[2] + nz - a[2]]
            posed[f'rod {t}'] = [(n, rotated(m, a, sm['rod_abs']).translate(shift)) for n, m in parts[f'rod {t}']]
        shin_now = [(n, rotated(m, kj['origin'], sm['knee'])) for n, m in shin]
        names = list(posed)
        for x, k1 in enumerate(names):
            for n1, m1 in posed[k1]:
                for n2, m2 in fixed + shin_now:
                    if '(buy)' in n2 and ('seat' in n1 or 'shank' in n1):     # in the spec, an actuator has no holes, and a seat is a contact
                        continue
                    v = C.overlap(m1, m2)
                    if v > TOUCH:
                        kk = (n1, n2)
                        if kk not in bad or v > bad[kk][0]:
                            bad[kk] = (v, sm['knee'])
                for k2 in names[x + 1:]:
                    for n2, m2 in posed[k2]:
                        v = C.overlap(m1, m2)
                        if v > TOUCH:
                            kk = (n1, n2)
                            if kk not in bad or v > bad[kk][0]:
                                bad[kk] = (v, sm['knee'])
    for (a, b), (v, q) in sorted(bad.items(), key=lambda kv: -kv[1][0])[:30]:
        print(f'{a} | {b} | {v * 1e6:.2f} cm3 at knee {q:+.2f}')
    print(f"four-bar: {len(s['mech']['samples'])} knee angles from {s['mech']['samples'][0]['knee']:+.2f} to {s['mech']['samples'][-1]['knee']:+.2f}, cranks and rods against the thigh, shin and each other: {len(bad)} pairs clash")


if __name__ == '__main__':
    main()
