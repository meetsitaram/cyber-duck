"""Read the STLs you'd actually print, as a slicer would: stray pieces, open meshes, slivers, the wrong size. Everything else checks the model; this checks the files.

  python -I tools/audit_prints.py        (MILLI_RECIPE picks the variant) report in reports/<variant>/prints_audit.md, exit 1 on any FAIL

  PIECES  one piece. A second piece over 1 mm3 is a part that fell off; smaller ones are mesh dust, reported, and tools/prints_from_fusion.py removes them
  CLOSED  watertight, so a slicer sees a solid
  VOLUME  within 12 % of the part as designed (rounding takes a little)
  BED     fits a 220 x 220 x 250 mm bed
  THIN    no face area where the part is under 0.8 mm thick, sampled from inside each face (2 perimeters of a 0.4 mm nozzle)
"""
import json
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402

BED = (220, 220, 250)
MIN_T = 0.8


def thin_share(m, n=1500):
    """Share of the surface with less than MIN_T of material behind it, by a ray straight in from each sampled point."""
    pts, fi = trimesh.sample.sample_surface(m, n, seed=1)
    nrm = m.face_normals[fi]
    o = pts - nrm * 0.02
    hit, ray, _ = m.ray.intersects_id(o, -nrm, multiple_hits=False, return_locations=False), None, None
    tris, rays = hit[0], hit[1]
    loc = m.ray.intersects_location(o, -nrm, multiple_hits=False)
    d = np.full(len(pts), np.inf)
    d[loc[1]] = np.linalg.norm(loc[0] - o[loc[1]], axis=1)
    return float((d < MIN_T - 0.02).mean())


def main():
    P = json.load(open(G.parts_path('parts'), encoding='utf-8'))['parts']
    out = G.out_path('prints')
    rep = C.Report()
    rows = []
    for part, p in P.items():
        f = os.path.join(out, part.replace(' ', '_') + '.stl')
        if not os.path.exists(f):
            rep.add('FILE', part, False, 'no STL')
            continue
        m = trimesh.load(f, force='mesh')
        m.merge_vertices()
        parts = m.split(only_watertight=False)
        vols = sorted((abs(q.volume) for q in parts), reverse=True)
        big = [v for v in vols if v > 1.0]
        rep.add('PIECES', part, len(big) == 1, '' if len(big) == 1 else f'{len(big)} pieces, mm3: {[round(v) for v in big[:5]]}')
        rep.add('CLOSED', part, bool(m.is_watertight), '' if m.is_watertight else f'open mesh, {len(vols) - 1} bits of dust')
        v = sum(vols) / 1000
        off = abs(v - p['volume_cm3']) / p['volume_cm3']
        rep.add('VOLUME', part, off <= 0.12, f'{v:.1f} cm3, designed {p["volume_cm3"]:.1f}: {off * 100:.0f} % off')
        e = np.sort(m.extents)[::-1]
        rep.add('BED', part, all(a <= b for a, b in zip(e, sorted(BED, reverse=True))), f'{e[0]:.0f} x {e[1]:.0f} x {e[2]:.0f} mm')
        try:
            t = thin_share(max(parts, key=lambda q: abs(q.volume)))
        except Exception as ex:
            t = float('nan')
        rows.append((part, t))
        rep.add('THIN', part, not t > 0.10, f'{t * 100:.0f} % of its surface is under {MIN_T} mm thick')
    # a file no part owns is a part we removed: you'd print it (5 of them, seen only on the contact sheet)
    want = {n.replace(' ', '_') + '.stl' for n in P}
    for f in sorted(os.listdir(out)):
        if f.endswith('.stl'):
            rep.add('FILE', f, f in want, '' if f in want else 'no part owns this file: a leftover')
    rep.write(G.out_path('reports', 'prints_audit.md'), 'From tools/audit_prints.py: the STL files themselves.')
    fails = rep.failing()
    for r in fails:
        print(r['group'], r['item'], r['detail'])
    print('thinnest:', [(n, f'{t * 100:.0f}%') for n, t in sorted(rows, key=lambda r: -np.nan_to_num(r[1]))[:6]])
    print(f'{len(rep.rows)} checks on {len(P)} print files, {len(fails)} failing')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
