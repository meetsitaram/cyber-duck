"""Cyber Duck: solves the getting-up keyframes so it would not tip, moving slowly: at each lean the legs, neck and
head that keep the centre of mass inside what touches the floor (the bill and the feet mid-way, then the feet), with
the largest margin, inside the joint limits. Prints a GET_UP list for tools/cyber_duck.py.

  MILLI_RECIPE=dynamixel python tools/cyber_duck_getup_solve.py

Statics only (quasi-static, slow): no dynamics, torques or friction yet. Plain python (scipy).
"""
import math
import os
import pickle
import sys

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import memcap  # noqa: E402
import checks as C  # noqa: E402
import cyber_duck as CD  # noqa: E402
import cyber_duck_getup_check as GC  # noqa: E402
import geom as G  # noqa: E402

HEADS = {'head', 'jaw'}
PREV = {'0.95': [-1.496, 1.249, -0.147, 1.159, 0.137], '0.50': [-1.552, 0.810, 0.231, 0.334, 1.175],
        '0.36': [-1.570, 0.940, 0.415, -0.380, 0.600], '0.11': [-0.947, 0.564, 0.429, -0.590, -0.048]}   # last run's good frames
VARS = ['hip', 'knee', 'ankle', 'neck_pitch', 'head_pitch']


def setup():
    with open(os.path.join(CD.OUT, 'painted.pkl'), 'rb') as fh:
        data = pickle.load(fh)
    md = CD.Model()
    CD.attach_flaps(md, data['hinges'], data.get('jaw'))
    parts = [(ln, v[::3]) for ln, v, f, c, r in data['extra']]              # every third vertex: enough for contact
    parts += [(ln, np.asarray(m.to_mesh().vert_properties[:, :3])[::3]) for ln, m in md.keep.items() if m is not None]
    MS = C.Mass(md.spec, md.S)
    import manifold3d as mf
    shell_m = []
    for ln, v, f, c, r in data['extra']:
        m_ = mf.Manifold(mf.Mesh(vert_properties=v.astype(np.float32), tri_verts=f.astype(np.uint32)))
        if m_.volume() > 0:
            shell_m.append((ln, m_.volume() * GC.PLA, v.mean(axis=0)))
    lim = {j['name']: (j['lower'], j['upper']) for j in md.spec['joints']}
    return md, parts, MS, shell_m, lim


def evaluate(env, lean, x, need_bill, need_feet=True):
    md, parts, MS, shell_m, lim = env
    hip, knee, ankle, neck, head = x
    q = {f'{s}_{k}': v for s in 'LR' for k, v in (('hip_pitch', hip), ('knee', knee), ('ankle_pitch', ankle))}
    q.update(neck_pitch=neck, head_pitch=head)
    M = md.R.fk(q, base=CD.Ry(lean))
    allp = [(ln, G.apply(M[ln], v)) for ln, v in parts]
    low = min(p[:, 2].min() for _, p in allp)
    pts = np.vstack([p[p[:, 2] - low < 0.004][:, :2] for _, p in allp])
    zb = min(p[:, 2].min() for ln, p in allp if ln in HEADS) - low
    zf = min(p[:, 2].min() for ln, p in allp if ln in ('L_foot', 'R_foot')) - low
    zl = min(p[:, 2].min() for ln, p in allp if ln.startswith(('L_', 'R_'))) - low     # any part of a leg
    mass = [(MS.m[ln], G.apply(M[ln], np.asarray(MS.com[ln])[None])[0]) for ln in MS.m] + \
           [(kg, G.apply(M[ln], c_[None])[0]) for ln, kg, c_ in shell_m]
    com = sum(m_ * p_ for m_, p_ in mass) / sum(m_ for m_, _ in mass)
    mg = GC.margin(GC.hull(pts), com[:2])
    over = sum(max(0, lo - v, v - hi) for v, (lo, hi) in zip(
        [hip, knee, ankle, neck, head], [lim['L_hip_pitch'], lim['L_knee'], lim['L_ankle_pitch'], lim['neck_pitch'], lim['head_pitch']]))
    cost = -mg + (0.05 * zf if need_feet else 0.0) + 20 * over      # the feet down (once they carry it), inside the limits
    if need_bill:
        cost += 1.0 * zb                                  # the bill down too (strong: else it just lifts it)
    if not need_feet:
        cost += 1.0 * zl                                  # off the chest: the legs still on the floor
    return cost, mg, zb, zf, q


def main():
    memcap.guard(cap_gb=6, need_free_gb=16)
    env = setup()
    # anchors (a sparse solve that balanced: margins +71, +54, +1, +64, +22 mm), then frames between each pair, each
    # started from the in-between of its anchors and kept near it (a warm-started chain got stuck with the bill lifted)
    anchors = [(1.45, False, [-0.40, 0.20, 0.00, 0.00, -0.60]),
               (1.10, True, [-1.553, 1.570, -0.123, 0.653, 0.273]),
               (0.80, True, [-1.555, 0.927, -0.171, 1.549, 0.001]),
               (0.50, False, [-1.552, 0.809, 0.227, 0.334, 1.175]),
               (0.22, False, [-1.566, 1.076, 0.605, -1.050, 0.026]),
               (0.00, False, [-0.299, 0.052, 0.254, -0.107, -0.122])]
    # early, off the chest: the prop is the bill and the legs still lying on the floor (the feet plant later); the bill
    # stays down to lean 0.5
    flat = lambda lean: [min(0.95, math.pi / 2 - lean - 0.4), 0.0, 0.0, 0.5, 0.2]
    plan = [(1.45, True, np.array(flat(1.45)), False, False), (1.33, True, np.array(flat(1.33)), False, False),
            (1.22, True, 0.5 * np.array(flat(1.22)) + 0.5 * np.array(anchors[1][2]), False, False),
            (1.10, True, np.array(anchors[1][2]), True, True), (0.95, True, None, False, True),
            (0.80, True, np.array(anchors[2][2]), True, True), (0.65, True, None, False, True),
            (0.50, True, np.array(anchors[3][2]), False, True), (0.36, False, None, False, True),
            (0.22, False, np.array(anchors[4][2]), True, True), (0.11, False, None, False, True),
            (0.00, False, np.array(anchors[5][2]), True, True)]
    for i, p_ in enumerate(plan):                    # the in-betweens start halfway between their neighbours
        if p_[2] is None:
            plan[i] = (p_[0], p_[1], 0.5 * (plan[i - 1][2] + plan[i + 1][2]), p_[3], p_[4])
    out = []
    for i, (lean, bill, xi, is_anchor, feet) in enumerate(plan):
        u = 0.30 + 0.70 * (1.45 - lean) / 1.45
        if is_anchor or (os.environ.get('CYBER_ONLY') and f'{lean:.2f}' not in os.environ['CYBER_ONLY'].split(',')):
            x = xi if is_anchor else np.array(PREV.get(f'{lean:.2f}', xi))
        else:
            f = lambda xx: evaluate(env, lean, xx, bill, feet)[0] + 0.03 * np.abs(xx - xi).sum()
            best = None
            for start in (xi, xi + np.array([0.0, 0.15, 0.0, 0.2, 0.15]), xi + np.array([-0.15, 0.2, -0.1, 0.4, 0.3])):
                r = minimize(f, start, method='Nelder-Mead', options=dict(maxiter=500, xatol=1e-3, fatol=1e-5))
                if best is None or r.fun < best.fun:
                    best = r
            x = best.x
        cost, mg, zb, zf, q = evaluate(env, lean, x, bill, feet)
        if is_anchor and mg < 0.005 and bill:            # an anchor that balances badly with the bill required: re-solve it
            f = lambda xx: evaluate(env, lean, xx, bill, feet)[0] + 0.03 * np.abs(xx - xi).sum()
            x = minimize(f, xi, method='Nelder-Mead', options=dict(maxiter=500, xatol=1e-3, fatol=1e-5)).x
            cost, mg, zb, zf, q = evaluate(env, lean, x, bill, feet)
        print(f'u {u:.3f} lean {lean:.2f}{" (anchor)" if is_anchor else ""}: margin {1000 * mg:+.1f} mm, bill {1000 * zb:.1f} mm, '
              f'feet {1000 * zf:.1f} mm off; ' + ', '.join(f'{n} {v:+.2f}' for n, v in zip(VARS, x)), flush=True)
        out.append((u, lean, x))
    print('\nGET_UP solved:')
    for u, lean, x in out:
        print(f"    ({u:.2f}, {lean:.3f}, dict(_LEGS({x[0]:+.3f}, {x[1]:+.3f}, {x[2]:+.3f}), neck_pitch={x[3]:+.3f}, head_pitch={x[4]:+.3f})),")


if __name__ == '__main__':
    main()
