"""Close-ups with no Fusion: the printed parts as exported, the maker's real HD-1910 in place, the other bought parts as the spec draws them. At the zero pose; no fillets, as those are Fusion's.

  MILLI_RECIPE=hd1910 python tools/look_real.py name az el dist x y z [--hide regex] [--only regex]    one picture: %TEMP%/look_hd1910/name.png
  MILLI_RECIPE=hd1910 python tools/look_real.py --set                                                  the standard set, into docs/images/fusion/hd1910/offline/
  add --fresh to rebuild the meshes after the spec changed

Say which pictures are these: offline shapes, not CAD.
"""
import argparse
import os
import re
import sys
import tempfile

import mujoco
import numpy as np
import trimesh
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly as A  # noqa: E402
import checks as C  # noqa: E402
import export_prints as E  # noqa: E402
import geom as G  # noqa: E402
import real_fit as RF  # noqa: E402

CACHE = os.path.join(os.environ['TEMP'], f'look_{G.RECIPE}')
COVERS = r'hood|thigh cover|body shell|face|eye|jaw|mouth roof'
SET = {  # name: az, el, dist, x, y, z, hide
    '34': (215, 15, 0.62, 0.0, 0, 0.145, None), 'back': (35, 15, 0.62, 0.0, 0, 0.145, None), 'side': (270, 5, 0.62, 0.0, 0, 0.145, None), 'under': (40, -35, 0.62, 0.0, 0, 0.145, None), 'top': (20, 70, 0.62, 0.0, 0, 0.145, None),
    'inside_34': (215, 15, 0.62, 0.0, 0, 0.145, COVERS), 'inside_back': (35, 15, 0.62, 0.0, 0, 0.145, COVERS),
    'leg_outer': (90, 10, 0.20, -0.02, 0.05, 0.075, None), 'leg_open_outer': (90, 10, 0.20, -0.02, 0.05, 0.075, COVERS), 'leg_open_rear': (180, 10, 0.20, -0.02, 0.05, 0.075, COVERS), 'leg_open_inner': (-60, 10, 0.20, -0.02, 0.05, 0.075, COVERS + '|^R '),
    'hip': (220, -15, 0.16, 0.0, 0.03, 0.125, COVERS), 'hip_rear': (200, -10, 0.16, 0.0, 0.03, 0.125, COVERS),
    'ankle': (120, 10, 0.12, -0.04, 0.04, 0.03, COVERS), 'ankle_rear': (200, 25, 0.12, -0.04, 0.04, 0.03, COVERS),
    'neck': (240, 10, 0.17, 0.02, 0, 0.17, COVERS), 'neck_rear': (230, 10, 0.17, 0.02, 0, 0.17, COVERS),
    'head_open': (215, 20, 0.22, 0.05, 0, 0.25, COVERS), 'head_open_rear': (35, 25, 0.22, 0.05, 0, 0.25, COVERS), 'head_open_top': (70, 65, 0.2, 0.05, 0, 0.25, COVERS),
    'body_open': (215, 30, 0.18, -0.02, 0, 0.135, COVERS),
}


def rgba(c, a=1.0):
    return ' '.join(f'{int(c[i:i + 2], 16) / 255:.3f}' for i in (1, 3, 5)) + f' {a}'


def meshes(fresh=False):
    """{name: (file, colour)}, in metres, world frame. Cached: the holes take half a minute."""
    index = os.path.join(CACHE, 'index.txt')
    if os.path.exists(index) and not fresh:
        return [ln.rstrip('\n').split('\t') for ln in open(index, encoding='utf-8')]
    os.makedirs(CACHE, exist_ok=True)
    spec = G.load()
    S = C.Solids(spec)
    B = A.Build(spec, S)
    out = []

    def add(name, m, col):
        f = os.path.join(CACHE, re.sub(r'\W+', '_', name) + '.stl')
        m.export(f)
        out.append((name, f, col))

    def tm(man):
        mesh = man.to_mesh()
        return trimesh.Trimesh(np.asarray(mesh.vert_properties[:, :3], float), np.asarray(mesh.tri_verts, int))
    for n, p in B.parts.items():
        gs = [B.idx[q] for q in p['geoms']]
        if p['kind'] == 'P':
            add(n, tm(E.with_holes(B, n)), gs[0][2]['color'])
        else:
            for ln, i, g in gs:
                if g['role'] == 'actuator' or g['name'].endswith((' flange', ' rear horn')) or g['name'].endswith('ankle rear boss'):
                    continue
                add(g['name'], tm(S.geom[(ln, i)]), g['color'])
    for nm, (ln, g) in B.screws.items():
        i = B.idx[nm][1]
        add(nm, tm(S.geom[(ln, i)]), '#C9CDD2')
    P = RF.pieces()
    COL = dict(s0='#17181A', s4='#1E1F22', s5='#1E1F22', s2='#EFEBDD', s3='#EFEBDD', s7='#2B2C30', s6='#2B2C30', s12='#0D0E10', s13='#0D0E10', s8='#0D0E10', s9='#0D0E10', s10='#0D0E10', s11='#0D0E10')
    names = {g['name'] for l in spec['links'].values() for g in l['geoms']}
    for j in spec['joints']:
        A_ = spec['actuators'][j['act']]
        n = j['name'].replace('_', ' ')
        T = RF.matrix(j, A_)
        use = list(RF.CASE) + list(RF.HORN) + list(RF.SOCK)
        if f'{n} rear horn' in names or 'ankle' in n:
            use += list(RF.IDLER)
        for k in use:
            m = P[k].copy()
            m.apply_transform(T)
            m.apply_scale(0.001)
            add(f'{n} HD1910 {k}', m, COL[k])
    with open(index, 'w', encoding='utf-8') as f:
        for r in out:
            f.write('\t'.join(r) + '\n')
    return [list(r) for r in out]


def scene(rows, hide=None, only=None):
    A_, Gs = [], []
    for k, (name, f, col) in enumerate(rows):
        if hide and re.search(hide, name):
            continue
        if only and not re.search(only, name):
            continue
        A_.append(f'<mesh name="m{k}" file="{f}"/>')
        Gs.append(f'<geom type="mesh" mesh="m{k}" rgba="{rgba(col)}" contype="0" conaffinity="0"/>')
    return mujoco.MjModel.from_xml_string(f"""<mujoco><compiler meshdir="."/><visual><global offwidth="1400" offheight="1400"/><quality shadowsize="4096"/><headlight ambient="0.45 0.45 0.45" diffuse="0.45 0.45 0.45" specular="0.05 0.05 0.05"/></visual>
<asset>{''.join(A_)}</asset><worldbody><light pos="0.5 0.6 1.2" dir="-0.4 -0.5 -1" diffuse="0.5 0.5 0.5"/><geom type="plane" size="1 1 0.01" pos="0 0 -0.001" rgba="0.82 0.84 0.86 1" contype="0" conaffinity="0"/>{''.join(Gs)}</worldbody></mujoco>""")


def shot(rows, path, az, el, dist, x, y, z, hide=None, only=None, size=1100):
    m = scene(rows, hide, only)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = az, -el, dist
    cam.lookat[:] = (x, y, z)
    r = mujoco.Renderer(m, size, size)
    r.update_scene(d, cam)
    Image.fromarray(r.render()).save(path)
    r.close()
    print(path)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('name', nargs='?')
    ap.add_argument('nums', nargs='*', type=float)
    ap.add_argument('--hide')
    ap.add_argument('--only')
    ap.add_argument('--set', action='store_true')
    ap.add_argument('--fresh', action='store_true')
    a = ap.parse_args()
    rows = meshes(a.fresh)
    if a.set:
        out = os.path.join(G.ROOT, 'docs', 'images', 'fusion', G.RECIPE, 'offline')
        os.makedirs(out, exist_ok=True)
        for n, (az, el, dist, x, y, z, hide) in SET.items():
            shot(rows, os.path.join(out, f'{n}.png'), az, el, dist, x, y, z, hide)
    elif a.name:
        os.makedirs(CACHE, exist_ok=True)
        shot(rows, os.path.join(CACHE, f'{a.name}.png'), *a.nums, a.hide, a.only)
