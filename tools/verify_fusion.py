"""Does the Fusion model match what exports to the printer, point by point? Volumes hide a wrong shape.

  python -I tools/verify_fusion.py write        writes spec/verify_points.json: random points in each printed part's box, in or out of its STL solid
  (in Fusion)  build.verify_points()            returns the same points' containment in the Fusion bodies, to reports/fusion_points.json
  python -I tools/verify_fusion.py compare      the share of points that disagree, per part; report in reports/fusion_verify.md

Fusion's lofts are fitted splines and the offline ones polygons, so points near a curved skin can differ: 3 % is allowed.
"""
import json
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly as A  # noqa: E402
import checks as C  # noqa: E402
import export_prints as E  # noqa: E402
import geom as G  # noqa: E402

N = 1500
LIMIT = 0.03
OUT = G.parts_path('verify_points')
BACK = G.parts_path('verify_points').replace('verify_points', 'fusion_points').replace('spec', 'reports')


def write():
    spec = G.load()
    B = A.Build(spec, C.Solids(spec))
    rng = np.random.default_rng(1)
    out = {}
    for part, p in B.parts.items():
        if p['kind'] != 'P':
            continue
        m = E.with_holes(B, part).to_mesh()
        tm = trimesh.Trimesh(np.asarray(m.vert_properties[:, :3], float), np.asarray(m.tri_verts, int))
        pts = rng.uniform(tm.bounds[0], tm.bounds[1], (N, 3))
        out[part] = dict(pts=pts.round(6).tolist(), inside=tm.contains(pts).astype(int).tolist())
    json.dump(out, open(OUT, 'w', encoding='utf-8'))
    print(f'{len(out)} parts, {N} points each -> {OUT}')


def compare():
    a, b = json.load(open(OUT, encoding='utf-8')), json.load(open(BACK, encoding='utf-8'))
    rep = C.Report()
    for part, v in a.items():
        if part not in b:
            rep.add('POINTS', part, False, 'no answer from Fusion')
            continue
        x, y = np.array(v['inside']), np.array(b[part])
        bad = float((x != y).mean())
        solid = max(int(x.sum()), 1)
        # (a part under 1 cm3: the offline solid rounds its edges under 0.5 mm, and Fusion leaves them square, a few percent of a thin lid)
        limit = 0.10 if abs(int(x.sum())) > 0.5 * len(x) else LIMIT
        rep.add('POINTS', part, bad <= limit, f'{bad * 100:.1f} % of {len(x)} points disagree; solid in {x.sum()} offline, {y.sum()} in Fusion')
    rep.write(G.out_path('reports', 'fusion_verify.md'), 'From tools/verify_fusion.py: the Fusion bodies against the exported STL solids.')
    for r in sorted(rep.rows, key=lambda r: r['status'])[:12]:
        print(r['status'], r['item'], r['detail'])
    print(f"{len(rep.rows)} parts, {len(rep.failing())} failing")
    return 1 if rep.failing() else 0


if __name__ == '__main__':
    if sys.argv[1] == 'write':
        write()
    else:
        sys.exit(compare())
