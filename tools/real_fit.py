"""Fusion's interference check, offline: the maker's real servo against the exported printed parts, at the zero pose. Needs no Fusion, so it can run first, and while Fusion is down.

  MILLI_RECIPE=hd1910 python -I tools/real_fit.py        reports/hd1910/real_fit.md

Needs %TEMP%/figrefs/feetech-hd1910/solids.npz: the STEP file's solids, tessellated (Feetech's frame, mm; s0 middle, s4 top and s5 bottom of the case, s2 and s3 sockets, s8-s11 case screws, s7 horn, s12 its screw, s6 rear horn, s13 its screw).
So far only for the HD-1910. Fit, not strength. A clash listed here either has a reason beside it or is a flaw.
"""
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly as A  # noqa: E402
import checks as C  # noqa: E402
import export_prints as E  # noqa: E402
import geom as G  # noqa: E402

D = os.path.join(os.environ['TEMP'], 'figrefs', 'feetech-hd1910')
O, UX, AZ = np.array([0, -7.5, -8.0]), np.array([0, -1.0, 0]), np.array([0, 0, 1.0])
CASE, HORN, IDLER = ('s0', 's4', 's5', 's8', 's9', 's10', 's11'), ('s7', 's12'), ('s6', 's13')
SOCK = ('s2', 's3')
LIMIT = 0.5          # mm3


def matrix(j, A_):
    """The file's frame, in mm, to the servo's place, in m. As cad/fusion/build.py _servo_matrix."""
    lx, ax = 'xyz'.index(j['long'][1]), 'xyz'.index(j['axis'])
    U, Ax = np.zeros(3), np.zeros(3)
    U[lx] = 1.0 if j['long'][0] == '+' else -1.0
    Ax[ax] = float(j['out'])
    c = np.array(j['act_c']) + U * A_['shaft_x']
    F0 = np.column_stack([UX, np.cross(AZ, UX), AZ])
    F1 = np.column_stack([U, np.cross(Ax, U), Ax])
    Rm = F1 @ F0.T
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = c * 1000 - Rm @ O
    return T


def pieces():
    z = np.load(os.path.join(D, 'solids.npz'))
    out = {k[:-2]: trimesh.Trimesh(z[k], z[k[:-2] + '_f'], process=True) for k in z.files if k.endswith('_v')}
    for m in out.values():
        m.fix_normals()                   # (the tessellation's faces come with either winding)
    return out


def main():
    spec = G.load()
    S = C.Solids(spec)
    B = A.Build(spec, S)
    P = pieces()
    parts = {n: E.with_holes(B, n) for n, p in B.parts.items() if p['kind'] == 'P'}
    rows, placed, bad = [], {}, set()
    for j in spec['joints']:
        A_ = spec['actuators'][j['act']]
        n = j['name'].replace('_', ' ')
        T = matrix(j, A_)
        use = list(CASE) + list(HORN) + list(SOCK)
        names = {g['name'] for l in spec['links'].values() for g in l['geoms']}
        if f'{n} rear horn' in names or 'ankle' in n:
            use += list(IDLER)
        for k in use:
            m = P[k].copy()
            if k in SOCK:                     # (not watertight in the file: its box)
                m = trimesh.creation.box(extents=m.extents, transform=trimesh.transformations.translation_matrix(m.bounds.mean(0)))
            m.apply_transform(T)
            placed.setdefault(n, []).append((k, m))
            try:
                tool = C.to_manifold(m)
            except ValueError:
                # (a tessellated solid with a pinched vertex: its pieces, or at worst its hull, which errs toward a clash)
                bad.add(k)
                tool = C.to_manifold(m.convex_hull)
            tool = tool.scale((0.001, 0.001, 0.001))     # (in mm until here: at metres, a tessellation's close points merged and it stopped being a solid)
            for pn, pm in parts.items():
                v = C.overlap(pm, tool) * 1e9
                if v > LIMIT:
                    rows.append((v, n, k, pn))
    rows.sort(reverse=True)
    NAME = dict(s0='case middle', s4='case top', s5='case bottom (and its rear tube)', s2='socket', s3='socket', s7='horn', s12='horn screw', s6='rear horn', s13='rear horn screw', s8='case screw', s9='case screw', s10='case screw', s11='case screw')
    L = ['# The real HD-1910 against the printed parts, as exported', '', 'From `tools/real_fit.py`, at the zero pose, over %.1f mm3. A case is checked against every part, its own pockets cut.' % LIMIT, '',
         f'{len(rows)} clashes.', '', '| mm3 | Servo | Its piece | Printed part |', '|---|---|---|---|']
    for v, n, k, pn in rows:
        L.append(f'| {v:.1f} | {n} | {NAME[k]} | {pn} |')
    os.makedirs(G.out_path('reports'), exist_ok=True)
    open(G.out_path('reports', 'real_fit.md'), 'w', encoding='utf-8').write('\n'.join(L) + '\n')
    print('hulls used for', sorted(bad))
    print(f'{len(rows)} clashes -> reports/{G.RECIPE}/real_fit.md')
    for v, n, k, pn in rows[:60]:
        print(f'{v:8.1f} {n:18s} {NAME[k]:28s} {pn}')


if __name__ == '__main__':
    main()
