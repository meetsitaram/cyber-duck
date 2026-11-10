"""Can each exported part really be printed, and how? tools/buildable.py asks about walls and load; this looks at the STL a slicer gets.

  python -I tools/printable.py             the checks; report in reports/printable.md, exit 1 on any FAIL
  python -I tools/printable.py --bad CASE  break a part on purpose and show the check FAIL (big void hole mate heads skin)

  ORIENT  the square orientation with the least overhang to support (faces past 45 degrees that aren't on the bed)
  PIECES  it exports as one piece, with no sealed void a slicer would fill or trap supports in
  BED     it fits a 220 mm bed that way, and enough of it touches the bed to stay down
  HOLES   every pilot and clearance hole is at least 1.5 mm across: smaller doesn't print round. No hole is sliced open by its edge
  SUPPORT what needs support is named: a warning, not a failure, but it decides how a part is sliced
  MATE    two exported parts of a link never share space, nor a part and a bought part: what a printer makes has to go together. Horn seats excepted (drawn 0.5 mm into their plates)
  HEADS   no screw's head stands inside a servo or a board, once fitted
  SKIN    a cover has holes only for its own screws: nothing else drills through what shows

A part's overhangs are found from its faces, not by slicing, so bridges over a short gap count too: they print well, and this overstates them.
"""
import argparse
import itertools
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

BED = 220.0                                        # mm
MIN_CONTACT = 40.0                                 # mm2 on the bed, or it comes loose
MIN_HOLE = 0.0015                                  # m
OVERHANG = np.cos(np.radians(45))
SUPPORT_WARN = 250.0                               # mm2


def orientations():
    """The 6 ways of laying a part on a face of its bounding box (turns about the vertical don't matter)."""
    out = []
    for ax, sg in itertools.product(range(3), (1, -1)):
        d = np.zeros(3)
        d[ax] = sg
        T = trimesh.geometry.align_vectors(d, [0, 0, -1])     # this direction points down
        out.append((('x', 'y', 'z')[ax], sg, T))
    return out


def score(tm, T):
    m = tm.copy()
    m.apply_transform(T)
    z0 = m.bounds[0][2]
    n, a = m.face_normals, m.area_faces
    low = m.triangles[:, :, 2].max(axis=1) < z0 + 0.3
    over = (n[:, 2] < -OVERHANG) & ~low
    contact = a[(n[:, 2] < -0.99) & low].sum()
    return dict(overhang=float(a[over].sum()), contact=float(contact), height=float(m.extents[2]), extents=m.extents.copy(), T=T)


def best_orientation(tm):
    """The least overhang; a low print breaks a tie; a part that wouldn't stay down loses."""
    opts = [score(tm, T) for _, _, T in orientations()]
    fine = [o for o in opts if o['contact'] >= MIN_CONTACT] or opts
    return min(fine, key=lambda o: (round(o['overhang'] / 50), o['height']))


def check(rep):
    import assembly as A
    import checks as C
    import export_prints as E
    import geom as G
    spec = G.load()
    S = C.Solids(spec)
    B = A.Build(spec, S)
    solids = {}
    for part, p in B.parts.items():
        if p['kind'] != 'P':
            continue
        m = E.with_holes(B, part)
        solids[part] = m
        check_part(B, part, m, E, rep)
    check_mate(B, solids, rep)


LIMIT_MATE = 0.003e-6                              # 3 mm3 in a part: rounding where two curved skins pass


def check_mate(B, solids, rep):
    import checks as C
    links = {n: {B.idx[g][0] for g in p['geoms']} for n, p in B.parts.items()}
    bad = 0
    names = list(solids)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if not links[a] & links[b]:
                continue
            v = C.overlap(solids[a], solids[b])
            if v > LIMIT_MATE:
                bad += 1
                rep.add('MATE', f'{a} / {b}', False, f'{v * 1e9:.0f} mm3 in the same place')
        for name, p in B.parts.items():
            if p['kind'] != 'B' or not links[a] & links[name] or name.endswith((' horn', 'bearing', 'bearing inner')):
                continue
            for g in p['geoms']:
                ln, k, gg = B.idx[g]
                if gg['role'] == 'cut' or g.endswith(('flange', 'rear boss', 'rear horn')) or 'race' in g:
                    continue
                v = C.overlap(solids[a], B.S.geom[(ln, k)])
                if v > LIMIT_MATE:
                    bad += 1
                    rep.add('MATE', f'{a} / {g}', False, f'{v * 1e9:.0f} mm3 in the same place')
    rep.add('MATE', 'parts that share a link', True, f'{len(names)} parts: {bad} clashes')
    import export_prints as E
    hard = [(n, ln, S_) for n, (ln, i, g) in B.idx.items() if (S_ := B.S.geom.get((ln, i))) is not None and (g['role'] == 'actuator' or n in ('battery 2S 850 mAh', 'servo bus board', 'Waveshare bus board', 'U2D2', 'IMU', 'camera') or n.endswith('5 V regulator'))]
    for n, (ln, g) in B.screws.items():
        f = g['fast']
        sz = f['size'].split()[0]
        tool = E.cylinder(np.array(f['p']), np.array(f['d']), E.HEAD_R[sz], *((0.00005, 0.0012) if f['size'].endswith('flat') else (-0.0015, -0.00005)))   # (a flat head's p is its top: the head is below it, in its seat)
        hit = [h for h, l2, m in hard if l2 == ln and C.overlap(tool, m) > 1e-10]
        rep.add('HEADS', n, not hit, f'its head stands inside {hit}' if hit else '')


def check_part(B, part, m, E, rep):
    mesh = m.to_mesh()
    tm = trimesh.Trimesh(np.asarray(mesh.vert_properties[:, :3], float) * 1000, np.asarray(mesh.tri_verts, int))
    b = best_orientation(tm)
    rep.add('ORIENT', part, True, f"{b['overhang']:.0f} mm2 overhang, {b['contact']:.0f} mm2 on the bed, {b['height']:.0f} mm tall")
    ps = m.decompose()
    n_, voids = len([q for q in ps if q.volume() > 1e-9]), [q for q in ps if q.volume() < -1e-10]
    rep.add('PIECES', part, n_ == 1 and not voids, f'{n_} pieces, {len(voids)} sealed voids' if n_ != 1 or voids else '')
    e = b['extents']
    rep.add('BED', part, e.max() <= BED and b['contact'] >= MIN_CONTACT, f'{e[0]:.0f} x {e[1]:.0f} x {e[2]:.0f} mm, {b["contact"]:.0f} mm2 on the bed')
    if any(B.idx[n][2]['role'] == 'shell' for n in B.parts[part]['geoms']):
        names = sorted({c[5] for c in E.hole_list(B, part)})
        own = [n for n in names if not any(k in n for k in ('cradle', 'fork', 'boss', 'crank', 'Pi '))]
        rep.add('SKIN', part, len(own) == len(names) and len(names) <= 6, f'{len(names)} screws drill it: {names[:6]}')
    bad = [f"{c[5]} {2 * c[2] * 1000:.1f} mm" for c in E.hole_list(B, part) if 2 * c[2] < MIN_HOLE]
    rep.add('HOLES', part, not bad, ', '.join(bad[:3]))
    if b['overhang'] > SUPPORT_WARN:
        rep.add('SUPPORT', part, False, f"{b['overhang']:.0f} mm2 needs support or a bridge: check it in the slicer", warn=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bad')
    a = ap.parse_args()
    import checks as C
    import geom as G
    rep = C.Report()
    if a.bad:
        import assembly as A
        import export_prints as E
        spec = G.load()
        B = A.Build(spec, C.Solids(spec))
        m = E.with_holes(B, 'L shin')
        if a.bad == 'void':                        # a sealed cavity inside the shin's cap
            ln, i, g = B.idx['L shin rib']
            c = np.array(g['c'])
            m = m - C.to_manifold(trimesh.creation.box(extents=(0.003, 0.002, 0.003), transform=trimesh.transformations.translation_matrix(c)))
        if a.bad == 'hole':                        # a pilot hole too small to print
            real = E.hole_list
            E.hole_list = lambda B_, p_: [h[:2] + (0.0005,) + h[3:] for h in real(B_, p_)] or [(np.zeros(3), np.array([0, 0, 1.0]), 0.0005, 0, 0.001, 'test')]
        if a.bad == 'big':                         # too big for the bed
            m = m.scale([4.0, 4.0, 4.0])
        if a.bad == 'skin':                        # a shell taken as solid again
            G_in = G.inner_of
            G.inner_of = lambda g, t: dict(g, sections=[dict(s_, a=1e-4, b=1e-4) for s_ in g.get('sections', [])]) if g['kind'] == 'loft' else G_in(g, t)
            check_part(B, 'body shell', E.with_holes(B, 'body shell'), E, rep)
        if a.bad == 'heads':                       # the head yaw horn screws as they were, under the roll servo
            for g in spec['links']['head_yaw_link']['geoms']:
                if g['name'].startswith('head yaw fork screw') and 'fast' in g:
                    g['fast']['p'][2] += 0.0016
            check_mate(B, {'L shin': m}, rep)
        if a.bad == 'mate':                        # the sole as drawn, 1 mm into its shoe, its recess not cut
            E.pockets = lambda *a_: []
            check_mate(B, {p: E.with_holes(B, p) for p in ('L shoe', 'L sole')}, rep)
        check_part(B, 'L shin', m, E, rep)
        fails = rep.failing()
        print(f"--bad {a.bad}: {'CAUGHT' if fails else 'NOT CAUGHT'} ({len(fails)} failing)")
        for r in fails[:3]:
            print('  ', r['group'], r['item'], r['detail'])
        return 0 if fails else 1
    check(rep)
    rep.write(G.out_path('reports', 'printable.md'), 'From tools/printable.py. Overhang from faces, not a slicer: none of this has been printed.')
    for r in rep.rows:
        if r['status'] != 'PASS':
            print(r['status'], r['group'], r['item'], r['detail'])
    print(f"{len(rep.rows)} printable checks, {len(rep.failing())} failing")
    return 1 if rep.failing() else 0


if __name__ == '__main__':
    sys.exit(main())
