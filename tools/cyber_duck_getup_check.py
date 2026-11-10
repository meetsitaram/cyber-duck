"""Cyber Duck: what touches the floor at each step of getting up, with the shells (from cyber_duck.py's painted.pkl),
and which joints leave their limits. Kinematic: lowered onto the lowest point. Static balance: the centre of mass (the mass model's links plus the shells
in PLA) over the support polygon of what touches the floor; a negative margin means it would tip (moving slowly).

  MILLI_RECIPE=dynamixel python tools/cyber_duck_getup_check.py
"""
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import cyber_duck as CD  # noqa: E402
import geom as G  # noqa: E402
import manifold3d as mf  # noqa: E402

PLA = 1240.0                      # kg/m3


def hull(p):
    p = sorted(map(tuple, p))
    if len(p) < 3:
        return p
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for q in p:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], q) <= 0:
            lo.pop()
        lo.append(q)
    for q in reversed(p):
        while len(up) >= 2 and cross(up[-2], up[-1], q) <= 0:
            up.pop()
        up.append(q)
    return lo[:-1] + up[:-1]


def margin(poly, c):
    """Signed distance of c inside the convex polygon (negative: outside)."""
    if len(poly) < 3:
        return -1.0
    d = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        e = np.subtract(b, a)
        n = np.array([-e[1], e[0]]) / np.linalg.norm(e)
        d.append(float(n @ (np.asarray(c) - a)))
    return min(d)


def main():
    with open(os.path.join(CD.OUT, 'painted.pkl'), 'rb') as fh:
        data = pickle.load(fh)
    md = CD.Model()
    CD.attach_flaps(md, data['hinges'], data.get('jaw'))
    parts = [(ln, v) for ln, v, f, c, r in data['extra']]
    parts += [(ln, np.asarray(m.to_mesh().vert_properties[:, :3])) for ln, m in md.keep.items() if m is not None]
    lim = {j['name']: (j['lower'], j['upper']) for j in md.spec['joints']}
    MS = C.Mass(md.spec, md.S)
    shell_m = []                                       # (link, kg, centroid in its zero-pose frame)
    for ln, v, f, c, r in data['extra']:
        m_ = mf.Manifold(mf.Mesh(vert_properties=v.astype(np.float32), tri_verts=f.astype(np.uint32)))
        if m_.volume() > 0:
            shell_m.append((ln, m_.volume() * PLA, v.mean(axis=0)))
    print(f"mass: links {sum(MS.m.values()) * 1000:.0f} g + shells {sum(x[1] for x in shell_m) * 1000:.0f} g")
    for k in range(0, 41):
        u = k / 40
        base, q = CD.transform_pose(1 - u)
        M = md.R.fk(q, base=base)
        z = {}
        for ln, v in parts:
            z[ln] = min(z.get(ln, 9), G.apply(M[ln], v)[:, 2].min())
        low = min(z.values())
        touch = sorted({ln.replace('_link', '') for ln, zz in z.items() if zz - low < 0.004})
        pts = np.vstack([G.apply(M[ln], v) for ln, v in parts])
        pts = pts[pts[:, 2] - low < 0.004][:, :2]
        mass = [(MS.m[ln], G.apply(M[ln], np.asarray(MS.com[ln])[None])[0]) for ln in MS.m] +                [(kg, G.apply(M[ln], c_[None])[0]) for ln, kg, c_ in shell_m]
        com = sum(m_ * p_ for m_, p_ in mass) / sum(m_ for m_, _ in mass)
        hp = hull(pts)
        mg = margin(hp, com[:2])
        ahead = com[0] > np.mean([p_[0] for p_ in hp]) if hp else True
        out = [f'{n} {q[n]:+.2f}' for n in q if n in lim and not lim[n][0] - 1e-6 <= q[n] <= lim[n][1] + 1e-6]
        print(f'u {u:.2f}: on the floor: {", ".join(touch):38s} CoM {"inside" if mg >= 0 else "OUTSIDE"} the support by {1000 * mg:+5.1f} mm' + ('' if mg >= 0 else (' (tips forward)' if ahead else ' (tips BACK)'))
              + (f'   OUT OF LIMITS: {", ".join(out)}' if out else ''))


if __name__ == '__main__':
    main()
