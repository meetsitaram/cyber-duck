"""Can each printed part really be made and carry its load? tools/checks.py asks whether parts fit; this asks if they last.

  python -I tools/buildable.py             the checks; report in reports/buildable.md, exit 1 on any FAIL
  python -I tools/buildable.py --bad CASE  break the design on purpose and show the check FAIL (thin weak split)

  ONE    each printed part is one piece
  WALL   no printed feature under MIN_WALL (a nozzle's 3 lines), except FLOORS, thin on purpose and backed by a servo's face
  FIT    each part fits a 220 mm print bed
  LOAD   stress through the sections that carry the robot, in a 3 g landing on one foot, against PLA across its layers

LOAD is a hand calculation by beam theory, not finite elements: a section's stress is F/A + M/Z for the worst foot contact,
the printed section alone (a bolted servo case helps, and is left out). It finds weak sections; it does not predict failure.
Test prints decide. Assumed: 25 MPa for PLA pulled across its layers, a safety factor of 1.7, so 15 MPa allowed.
"""
import argparse
import copy
import os
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly as A  # noqa: E402
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import mass_model as MM  # noqa: E402

MIN_WALL = 0.001                                   # 1 mm
VOX = 0.00025
FLOORS = ('seat floor', 'seat wall', 'head yaw cradle horn face')   # backed by a servo's flat face, in compression
BED = 0.22
SHOCK = 3.0
ALLOW = 15e6
Y, Z, X = (0, 1, 0), (0, 0, 1), (1, 0, 0)


def sections(spec):
    """(what, part names, point on the plane, plane's normal, load (kg lifted), contact points)."""
    J = {j['name']: j for j in spec['joints']}
    L = spec['links']
    g = {x['name']: x for l in L.values() for x in l['geoms']}
    o = lambda n: np.array(J[n]['origin'])
    k, h, an = o('L_knee'), o('L_hip_pitch'), o('L_ankle_pitch')
    r_ = o('L_hip_roll')
    if 'load_sections' in spec:                    # a spec with its own parts says where its load goes: spec/nano_duck.py
        return k, h, an, r_, [(w, ps, tuple(pt), tuple(n)) for w, ps, pt, n in spec['load_sections']]
    return k, h, an, r_, [
        ('shin, under the knee horn', ['L shin'], (k[0], 0, k[2] - 0.0115), Z),
        ('shin, at the cap', ['L shin'], (k[0], 0, k[2] - 0.019), Z),
        ('thigh, hip to knee', ['L thigh frame', 'L hip pitch lid', 'L knee lid', 'L thigh cover'], ((h + k) / 2).tolist(), tuple((h - k) / np.linalg.norm(h - k))),
        ('roll bracket, side arm', ['L roll bracket'], (0.013, 0, h[2]), X),
        ('roll bracket, front plate', ['L roll bracket'], (r_[0], r_[1] + 0.012, h[2]), Y),
        ('yaw bracket', ['L yaw bracket'], (0, 0, r_[2] + 0.015), Z),
    ]


def second_moments(polys, d0, d1):
    """Area, centroid and second moments of shapely polygons (holes subtracted), about the centroid; furthest distances."""
    A_, cx, cy, Ixx, Iyy = 0.0, 0.0, 0.0, 0.0, 0.0
    pts = []

    def ring(c, sign):
        nonlocal A_, cx, cy, Ixx, Iyy
        c = np.array(c)[:, :2]
        x, y = c[:, 0], c[:, 1]
        x1, y1 = np.roll(x, -1), np.roll(y, -1)
        w = x * y1 - x1 * y
        s0 = np.sign(w.sum()) or 1
        A_ += sign * s0 * w.sum() / 2
        cx += sign * s0 * ((x + x1) * w).sum() / 6
        cy += sign * s0 * ((y + y1) * w).sum() / 6
        Ixx += sign * s0 * ((y * y + y * y1 + y1 * y1) * w).sum() / 12
        Iyy += sign * s0 * ((x * x + x * x1 + x1 * x1) * w).sum() / 12
    for p in polys:
        ring(p.exterior.coords, 1)
        pts.append(np.array(p.exterior.coords)[:, :2])
        for i in p.interiors:
            ring(i.coords, -1)
    cx, cy = cx / A_, cy / A_
    P = np.vstack(pts)
    return A_, cx, cy, Ixx - A_ * cy * cy, Iyy - A_ * cx * cx, np.abs(P[:, 1] - cy).max(), np.abs(P[:, 0] - cx).max()


def check_load(spec, B, rep, S):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    inert = MM.link_inertials(spec, S)
    total = sum(v[0] for v in inert.values())
    foot = inert['L_foot'][0]
    F = (total - foot) * 9.81 * SHOCK
    k, h, an, r_, secs = sections(spec)
    so = next(g for g in spec['links']['L_foot']['geoms'] if 'contact' in g)['contact']
    c, sz = np.array(so['c']), np.array(so['size']) / 2
    contacts = [c + [a * sz[0], b * sz[1], 0] for a in (-1, 0, 1) for b in (-1, 0, 1)]
    rows = []
    for what, parts, pt, n in secs:
        ax = int(np.argmax(np.abs(n)))
        n = np.asarray(n, float)
        m = C.union([B.solid(p) for p in parts])
        mesh = m.to_mesh()
        import trimesh
        tm = trimesh.Trimesh(np.asarray(mesh.vert_properties[:, :3], float), np.asarray(mesh.tri_verts, int))
        sec = tm.section(plane_origin=np.asarray(pt, float), plane_normal=n)
        if sec is None:
            rep.add('LOAD', what, False, 'the plane misses the part')
            continue
        # plane axes
        t0 = np.array([0, 1.0, 0]) if abs(n[1]) < 0.9 else np.array([1.0, 0, 0])
        e0 = np.cross(n, t0)
        e0 /= np.linalg.norm(e0)
        e1 = np.cross(n, e0)
        rings = [np.column_stack([(q - pt) @ e0, (q - pt) @ e1]) for q in sec.discrete if len(q) > 3]
        rings = sorted((Polygon(r_2).buffer(0) for r_2 in rings), key=lambda a: -a.area)
        poly = None
        for r2 in rings:                           # nested rings are holes
            poly = r2 if poly is None else poly.symmetric_difference(r2)
        polys = list(poly.geoms) if hasattr(poly, 'geoms') else [poly]
        polys = [p for p in polys if p.area > 1e-8]
        a_, cx, cy, Ie0, Ie1, c1, c0 = second_moments(polys, 0, 1)
        cen = np.asarray(pt, float) + cx * e0 + cy * e1
        worst = 0.0
        for ct in contacts:
            f = np.array([0, 0, F])
            M = np.cross(ct - cen, f)
            Mu, Mv = M @ e0, M @ e1
            fn = abs(f @ n)
            sig = fn / a_ + abs(Mu) * c1 / Ie0 + abs(Mv) * c0 / Ie1
            worst = max(worst, sig)
        rep.add('LOAD', what, worst <= ALLOW, f'{worst / 1e6:.1f} MPa (allowed {ALLOW / 1e6:.0f}): {F:.0f} N, section {a_ * 1e6:.0f} mm2')
        rows.append((what, worst))
    return rows


def voxels(B, part):
    gs = [B.idx[n][2] for n in B.parts[part]['geoms']]
    keep = [g for g in gs if not any(k in g['name'] for k in FLOORS)]
    if not keep:
        return None, None
    lo, hi = np.full(3, 9.0), np.full(3, -9.0)
    for g in keep:
        b = G.mesh_of(g).bounds
        lo, hi = np.minimum(lo, b[0]), np.maximum(hi, b[1])
    lo, hi = lo - 2 * VOX, hi + 2 * VOX
    n = np.ceil((hi - lo) / VOX).astype(int)
    grid = np.zeros(n, bool)
    for g in keep:
        b = G.mesh_of(g).bounds
        i0, i1 = np.maximum(((b[0] - lo) / VOX).astype(int), 0), np.minimum(((b[1] - lo) / VOX).astype(int) + 2, n)
        if np.any(i1 <= i0):
            continue
        xs = [lo[k] + VOX * (np.arange(i0[k], i1[k]) + 0.5) for k in range(3)]
        X_, Y_, Z_ = np.meshgrid(*xs, indexing='ij')
        P = np.column_stack([X_.ravel(), Y_.ravel(), Z_.ravel()])
        hit = B.inside(g, P).reshape(X_.shape)
        grid[i0[0]:i1[0], i0[1]:i1[1], i0[2]:i1[2]] |= hit
    return grid, lo


def check_walls(B, rep):
    r = int(round(MIN_WALL / 2 / VOX))
    z, y, x = np.ogrid[-r:r + 1, -r:r + 1, -r:r + 1]
    ball = x * x + y * y + z * z <= r * r + 0.5
    for part, p in B.parts.items():
        if p['kind'] != 'P' or part.startswith('R ') or any(B.idx[n][2]['role'] == 'shell' for n in p['geoms']):
            continue
        grid, lo = voxels(B, part)
        if grid is None:
            continue
        _, n_ = ndimage.label(grid)
        rep.add('ONE', part, n_ == 1, f'{n_} separate pieces' if n_ != 1 else '')
        opened = ndimage.binary_opening(grid, ball)
        thin = grid & ~opened
        # the opening also trims every shape's edges and corners; those trimmings are under 2 voxels across, a thin wall is not
        kept = ndimage.binary_opening(thin, np.ones((2, 2, 2), bool))
        lab, c = ndimage.label(kept)
        size = ndimage.sum(kept, lab, range(1, c + 1)) if c else [0]
        worst = float(np.max(size)) * VOX ** 3 * 1e9
        # (steps between overlapping primitives, up to 10 mm3, read as thin here: the exported part needs a look regardless)
        rep.add('WALL', part, worst < 12.0, f'a feature under {MIN_WALL * 1000:.0f} mm, {worst:.1f} mm3' if worst >= 12.0 else '')
        dims = np.array(grid.shape) * VOX
        rep.add('FIT', part, dims.max() <= BED, f'{dims[0] * 1000:.0f} x {dims[1] * 1000:.0f} x {dims[2] * 1000:.0f} mm')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bad')
    a = ap.parse_args()
    spec = G.load()
    if a.bad:
        L = spec['links']
        if a.bad == 'weak':                        # the shin back to a 3 mm plate
            for ln in ('L_knee_link',):
                L[ln]['geoms'] = [g for g in L[ln]['geoms'] if not any(k in g['name'] for k in ('shin rib', 'shin gusset', 'shin collar'))]
            for n, k_, gs in A.PARTS:
                pass
            A.PARTS = [(n, k_, [q for q in gs if not any(z in q for z in ('L shin rib', 'L shin gusset', 'L shin collar'))]) for n, k_, gs in A.PARTS]
        elif a.bad == 'thin':                      # a 0.5 mm fin on a bracket
            L['L_hip_roll_link']['geoms'].append(dict(kind='rbox', name='fin', c=[0.02, 0.0345, 0.118], size=[0.012, 0.0005, 0.012],
                                                      fillet=0.0001, color='#000000', role='struct', material='pla'))
            A.PARTS = [(n, k_, gs + ['fin'] if n == 'L roll bracket' else gs) for n, k_, gs in A.PARTS]
        elif a.bad == 'split':                     # a piece of a part floating away
            for g in L['L_knee_link']['geoms']:
                if g['name'] == 'L shin cap':
                    g['c'][0] += 0.06
    S = C.Solids(spec)
    B = A.Build(spec, S)
    rep = C.Report()
    check_load(spec, B, rep, S)
    check_walls(B, rep)
    fails = rep.failing()
    if a.bad:
        print(f"--bad {a.bad}: {'CAUGHT' if fails else 'NOT CAUGHT'} ({len(fails)} failing)")
        for r in fails[:4]:
            print('  ', r['group'], r['item'], r['detail'])
        return 0 if fails else 1
    rep.write(G.out_path('reports', 'buildable.md'), 'From tools/buildable.py. LOAD is a beam estimate, not FEA: test prints decide.')
    for r in rep.rows:
        if r['group'] == 'LOAD':
            print(r['status'], r['item'], r['detail'])
    for r in fails:
        if r['group'] != 'LOAD':
            print(r['status'], r['group'], r['item'], r['detail'])
    print(f'{len(rep.rows)} buildable checks, {len(fails)} failing')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
