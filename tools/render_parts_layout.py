"""Every part of one duck, flat on a plain background: docs/<recipe>/parts_layout.png, beside wiring.png.

  MILLI_RECIPE=dynamixel python tools/render_parts_layout.py

Printed parts are Fusion's own bodies, with the spec's shapes only if those are missing. Bought parts are the spec's shapes, so a servo here is its model, not the maker's CAD.
Exits 1 if the picture's counts disagree with the BOM: the picture is drawn from the model, so a difference is a finding.
"""
import os
import re
import sys
import tempfile

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G                                   # noqa: E402
import render_assembly as RA                       # noqa: E402

WIDTH = 0.36                                       # m across the sheet
GAP, SCREW_GAP = 0.006, 0.0022
PIXELS = 2600
BACK = (0.80, 0.80, 0.79)                          # darker than the white parts, or they vanish


def arrays(m):
    if isinstance(m, tuple):
        return np.asarray(m[0], float), np.asarray(m[1], int)
    o = m.to_mesh()
    return np.asarray(o.vert_properties[:, :3], float), np.asarray(o.tri_verts, int)


def lay_flat(pieces):
    """One rigid turn for all of an item's pieces: thinnest way up, longest side across, not mirrored. Returns pieces with the item's corner at the origin."""
    pts = np.vstack([v for v, _, _ in pieces])
    T, ext = trimesh.bounds.oriented_bounds(pts)
    order = np.argsort(-np.asarray(ext))           # long, middle, thin
    R = T[:3, :3][order]
    if np.linalg.det(R) < 0:
        R[2] *= -1
    t = [(v @ R.T, f, c) for v, f, c in pieces]
    a = np.vstack([v for v, _, _ in t])
    lo, hi = a.min(0), a.max(0)
    return [(v - [lo[0], lo[1], hi[2]], f, c) for v, f, c in t], hi - lo


def thread(size):
    """'M2' or 'M2 flat': 2.0"""
    return float(size.split()[0][1:])


def separate(pieces):
    """Pieces of one bought item that don't touch are separate things to handle, like 4 standoffs: only pieces whose boxes meet stay together."""
    groups = []
    for x in pieces:
        lo, hi = x[0].min(0) - 0.0005, x[0].max(0) + 0.0005
        meet = [g for g in groups if (lo <= g[2]).all() and (g[1] <= hi).all()]
        groups = [g for g in groups if not any(g is h for h in meet)]
        groups.append(([x] + [y for g in meet for y in g[0]],
                       np.min([lo] + [g[1] for g in meet], axis=0), np.max([hi] + [g[2] for g in meet], axis=0)))
    return [g[0] for g in groups]


def screw(size, length, colour):
    """A screw on its side, as they lie in a row: the model only holds its head, so its shank is drawn from its size and length."""
    d = thread(size) * 0.001
    shank = trimesh.creation.cylinder(radius=d / 2, height=length, sections=20)
    shank.apply_translation((0, 0, length / 2))
    head = trimesh.creation.cylinder(radius=d * 0.95, height=d * 0.8, sections=24)
    head.apply_translation((0, 0, -d * 0.4))
    m = trimesh.util.concatenate([shank, head])
    return [(np.asarray(m.vertices), np.asarray(m.faces), colour)]


def shelves(items, y, gap, width=WIDTH):
    """Left to right, a row at a time, downwards. Returns the placed pieces and the next free y."""
    x, row, placed = 0.0, 0.0, []
    for pieces, ext in items:
        if x and x + ext[0] > width:
            x, y, row = 0.0, y - row - gap, 0.0
        placed += [(v + [x, y - ext[1], 0.0], f, c) for v, f, c in pieces]
        x += ext[0] + gap
        row = max(row, ext[1])
    return placed, y - row - gap


def bom_counts(path):
    """What the BOM says, for what can be compared with the picture directly."""
    s = open(path, encoding='utf-8').read()
    out = {'printed': int(re.search(r'(\d+) printed parts', s).group(1))}
    for line in s.splitlines():
        cols = [c.strip() for c in line.split('|')]
        if len(cols) < 6 or not cols[1].isdigit() or 'OPTIONAL' in line:
            continue
        spec = cols[4]
        if cols[2].lower().startswith('smart servo'):
            out['servo'] = out.get('servo', 0) + int(cols[5])
        if cols[2].startswith('Screw'):
            listed = re.findall(r'(\d+) x (M[\d.]+) x (\d+)', cols[2] + ' ' + spec)
            for n, size, length in listed:
                out[f'{size} x {length}'] = out.get(f'{size} x {length}', 0) + int(n)
            m = re.match(r'(M[\d.]+) x (\d+) mm', spec)
            if m and cols[5].isdigit() and not listed:     # the same size can be on 2 lines: machine and self-tapping
                out[f'{m.group(1)} x {m.group(2)}'] = out.get(f'{m.group(1)} x {m.group(2)}', 0) + int(cols[5])
    return out


def main():
    recipe = os.environ.get('MILLI_RECIPE', 'hybrid')
    spec = G.load()
    folder = os.path.join(os.environ['TEMP'], 'duck_fusion_stl', recipe)
    sc = RA.Scene.__new__(RA.Scene)
    sc.spec = spec
    sc.B = RA.A.Build(spec)
    B = sc.B
    sc.items = {}
    for n, p in B.parts.items():
        sc.items[n] = [(B.S.geom[B.idx[gn][:2]], B.idx[gn][2]['color']) for gn in p['geoms']]

    printed, bought, spec_shapes = [], [], []
    for n, p in B.parts.items():
        if p['kind'] == 'P':
            real = sc.real_meshes(n, folder)
            if not real:
                spec_shapes.append(n)
            pieces = [(*arrays(m), c) for m, c in (real or sc.items[n])]
        else:
            pieces = [(*arrays(m), c) for m, c in sc.items[n]]
        pieces = [x for x in pieces if len(x[1])]
        if p['kind'] == 'P':
            printed.append(lay_flat(pieces))
        else:
            bought += [lay_flat(g) for g in separate(pieces)]
    area = lambda it: -it[1][0] * it[1][1]
    printed.sort(key=area)
    bought.sort(key=area)

    types = {}
    for n, (ln, g) in B.screws.items():
        f = g['fast']
        types.setdefault((f['size'], round(f['L'] * 1000)), []).append(g['color'])
    placed, y = shelves(printed, 0.0, GAP)
    more, y = shelves(bought, y - GAP, GAP)
    placed += more
    y -= GAP
    for (size, length), cols in sorted(types.items(), key=lambda kv: (thread(kv[0][0]), kv[0][0], -kv[0][1])):
        more, y = shelves([lay_flat(screw(size, length * 0.001, c)) for c in cols], y, SCREW_GAP)
        placed += more

    tmp = tempfile.mkdtemp()
    assets, geoms = [], []
    for k, (v, f, c) in enumerate(placed):
        RA.RP.stl(os.path.join(tmp, f'm{k}.stl'), v, f)
        assets.append(f'<mesh name="m{k}" file="m{k}.stl"/>')
        geoms.append(f'<geom type="mesh" mesh="m{k}" rgba="{" ".join(f"{x:.3f}" for x in RA.RP.rgba(c))}" contype="0" conaffinity="0"/>')
    height = -y
    margin = 0.012
    w_m, h_m = WIDTH + 2 * margin, height + 2 * margin
    W, H = PIXELS, int(PIXELS * h_m / w_m)
    bg = ' '.join(f'{x:.3f}' for x in BACK)
    xml = f"""<mujoco><compiler meshdir="{tmp}"/>
<visual><global offwidth="{W}" offheight="{H}"/><headlight ambient=".34 .34 .34" diffuse=".3 .3 .3" specular="0 0 0"/><quality shadowsize="0"/><map znear="0.001" zfar="50"/></visual>
<asset><material name="bg" rgba="{bg} 1" specular="0" shininess="0"/>{''.join(assets)}</asset>
<worldbody>
<geom name="ground" type="plane" size="5 5 .1" pos="0 0 -0.2" material="bg" contype="0" conaffinity="0"/>
<light directional="true" diffuse=".3 .3 .3" specular=".08 .08 .08" dir="-0.45 0.4 -1" castshadow="false"/>
{''.join(geoms)}
<camera name="top" fovy="{np.degrees(2 * np.arctan(h_m / 2 / 8)):.5f}" pos="{WIDTH / 2:.5f} {-height / 2:.5f} 8" xyaxes="1 0 0 0 1 0"/>
</worldbody></mujoco>"""
    path = os.path.join(tmp, 'scene.xml')
    open(path, 'w').write(xml)
    m = RA.mujoco.MjModel.from_xml_path(path)
    d = RA.mujoco.MjData(m)
    RA.mujoco.mj_forward(m, d)
    r = RA.mujoco.Renderer(m, H, W)
    r.update_scene(d, 'top')
    img = r.render().copy()
    r.enable_segmentation_rendering()              # a flat background, whatever the lights do
    r.update_scene(d, 'top')
    seg = r.render()
    r.close()
    ground = m.geom('ground').id
    img[seg[..., 0] == ground] = (np.array(BACK) * 255).astype(np.uint8)
    img[seg[..., 0] < 0] = (np.array(BACK) * 255).astype(np.uint8)

    from PIL import Image
    out = os.path.join('docs', recipe, 'parts_layout.png')
    Image.fromarray(img).save(out, optimize=True)

    counts = {'printed': len(printed), 'servo': sum(n.endswith(' servo') for n in B.parts)}
    for (s, L), c in types.items():
        k = f'{s.split()[0]} x {L}'
        counts[k] = counts.get(k, 0) + len(c)
    bom_path = os.path.join('docs', recipe, 'BOM.md')
    bom = bom_counts(bom_path)
    bad = [f'{k}: picture {counts.get(k)}, BOM {bom.get(k)}' for k in sorted(set(counts) | set(bom)) if counts.get(k) != bom.get(k)]
    print(f'{out}: {W} x {H}, {len(printed)} printed, {len(bought)} bought items, {len(B.screws)} screws, {len(spec_shapes)} printed parts in spec shapes {spec_shapes}')
    print('compared with the BOM:', {k: counts.get(k) for k in sorted(bom)})
    if bad:
        print('DIFFERENT:', bad)
        sys.exit(1)
    print('0 different')


if __name__ == '__main__':
    main()
