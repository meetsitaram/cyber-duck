"""Prototype of milli's servo mounting, on one STS3215: the holder (the parent link's part), the U-bracket (the child
link's part), both horns and every screw at the measured holes; assembled, turned to its limits, and exploded.

  python tools/proto_mount.py   -> reports/compare/proto_mount.png

The pattern is the LeRobot SO-101's (huggingface.co/docs/lerobot/so101, SO-ARM100's printed parts): the servo drops
into a cradle on the parent link and takes 4 M2 x 6, two through each face at the far pilot holes (spec/actuators.json,
measured on Feetech's STEP; SO-101's slide-over sleeve is left out, for room); the moving link is a fork on the
driven horn (M3 centre screw) and the passive rear horn (no centre screw), 4 M3 x 6 each, its bridge passing round the
case's short end, outside the case's 16 mm reach there. Before fitting: set the servo's ID, plug its 3-pin cable in,
centre the horn. Local frame: the shaft along +z (horn face up), the case's long side along -x from the shaft.
"""
import math
import os
import sys
import tempfile

import mujoco
import numpy as np
import trimesh
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

A = G.load()['actuators']['STS3215_C044']
L, W, H = A['box']
SX, HH = A['shaft_x'], A['horn_h']
T = 0.003                                          # printed plate thickness
COL = dict(case='#1B1C1F', horn='#A8ACB0', holder='#2E86DE', bracket='#E8642A', screw='#F2C94C')


def rbox(c, s, f=0.0008):
    return dict(kind='rbox', c=list(c), size=list(s), fillet=min(f, min(s) / 2 - 1e-4))


def cyl(c, r, h, axis='z'):
    return dict(kind='cyl', c=list(c), axis=axis, r=r, h=h)


def parts():
    """[(name, group, geom)] in the local frame: shaft at the origin, the case's middle at x = -SX."""
    cx = -SX                                         # the case's middle
    z0, z1 = -H / 2, H / 2                           # rear face, horn face
    out = [('STS3215 case', 'case', rbox((cx, 0, 0), (L, W, H), 0.0015)),
           ('driven horn', 'horn', cyl((0, 0, z1 + HH / 2), A['horn_r'], HH)),
           ('rear horn', 'horn', cyl((0, 0, z0 - HH / 2), A['horn_r'], HH))]
    # SO-101's pattern. The cradle (the parent link): a plate on each face over the case's far half, side walls and
    # an end wall with a window for the 3-pin cables; the servo slides in and takes 4 M2 x 6, 2 per face, at the far
    # pilot holes. (SO-101's sleeve, its "motor holder", is left out: it adds 6 mm a side and milli has no room.)
    far = cx - L / 2
    hl = 0.022                                       # the cradle's length along the case
    win = 0.012                                      # cable window in the end wall
    out += [('cradle rear plate', 'holder', rbox((far + hl / 2, 0, z0 - T / 2 - 0.0001), (hl, W + 2 * T, T))),
            ('cradle horn-side plate', 'holder', rbox((far + hl / 2, 0, z1 + T / 2 + 0.0001), (hl, W + 2 * T, T))),
            ('cradle end wall L', 'holder', rbox((far - T / 2, (W / 2 + T + win / 2) / 2, 0), (T, W / 2 + T - win / 2, H + 2 * T))),
            ('cradle end wall R', 'holder', rbox((far - T / 2, -(W / 2 + T + win / 2) / 2, 0), (T, W / 2 + T - win / 2, H + 2 * T))),
            ('cradle side L', 'holder', rbox((far + hl / 2, W / 2 + T / 2 + 0.0001, 0), (hl, T, H))),
            ('cradle side R', 'holder', rbox((far + hl / 2, -W / 2 - T / 2 - 0.0001, 0), (hl, T, H)))]
    # the U-bracket: a plate on each horn, joined by a bridge round the case's short end (x > 0), outside its reach
    reach = math.hypot(L / 2 - SX, W / 2) + 0.0025  # the case's corners on the shaft side
    zt, zb = z1 + HH + T / 2, z0 - HH - T / 2
    out += [('U plate, driven horn', 'bracket', rbox((0.007, 0, zt), (0.026, 0.022, T))),
            ('U plate, rear horn', 'bracket', rbox((0.007, 0, zb), (0.026, 0.022, T))),
            ('U bridge', 'bracket', rbox((reach + T / 2, 0, 0), (T, 0.022, zt - zb + T)))]
    # screws: M3 x 6 into each horn (4 + the driven horn's centre screw), M2 x 6 into the case through the holder
    for zf, sgn, nm in ((zt, 1, 'driven'), (zb, -1, 'rear')):
        for d in A['horn_hole_deg']:
            a = math.radians(d)
            p = (A['horn_hole_r'] * math.cos(a), A['horn_hole_r'] * math.sin(a))
            out.append((f'M3 x 6, {nm} horn', 'screw', cyl((p[0], p[1], zf + sgn * (T / 2 - 0.003 + 0.0012)), 0.0015, 0.006)))
            out.append((f'M3 head', 'screw', cyl((p[0], p[1], zf + sgn * (T / 2 + 0.0012)), 0.0027, 0.0024)))
    out.append(('M3 centre screw, driven horn', 'screw', cyl((0, 0, zt + T / 2 + 0.0012), 0.0027, 0.0024)))
    for (u, v) in A['case_holes_rear'][:2]:           # the two far holes, as SO-101
        out.append(('M2 x 6, rear face', 'screw', cyl((cx + u, v, z0 - T - 0.0011), 0.0019, 0.0022)))
    for (u, v) in A['case_holes_horn'][:2]:          # the two away from the horn
        out.append(('M2 x 6, horn face', 'screw', cyl((cx + u, v, z1 + T + 0.0011), 0.0019, 0.0022)))
    return out


def mesh(g):
    gg = dict(g, name='', role='struct', color='#000000')
    return G.mesh_of(gg)


def render(items, view, title, size=520, dist=0.17, look=(-0.008, 0, 0)):
    with tempfile.TemporaryDirectory() as tmp:
        assets, geoms = [], []
        for k, (n, grp, g, M) in enumerate(items):
            m = mesh(g)
            m.apply_transform(M)
            p = os.path.join(tmp, f'p{k}.stl')
            m.export(p)
            assets.append(f'<mesh name="p{k}" file="p{k}.stl"/>')
            c = RP.rgba(COL[grp])
            geoms.append(f'<geom type="mesh" mesh="p{k}" rgba="{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} 1"/>')
        xml = f"""<mujoco><compiler meshdir="{tmp}"/><visual><global offwidth="1200" offheight="1200"/>
<headlight ambient=".45 .45 .45" diffuse=".45 .45 .45"/></visual><asset>{''.join(assets)}</asset>
<worldbody><light directional="true" dir="-0.4 -0.6 -1" diffuse=".4 .4 .4"/>{''.join(geoms)}</worldbody></mujoco>"""
        open(os.path.join(tmp, 's.xml'), 'w').write(xml)
        m = mujoco.MjModel.from_xml_path(os.path.join(tmp, 's.xml'))
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = look
        cam.distance, cam.azimuth, cam.elevation = dist, view[0], view[1]
        r = mujoco.Renderer(m, size, size)
        r.update_scene(d, cam)
        img = Image.fromarray(r.render())
        r.close()
    ImageDraw.Draw(img).text((8, 6), title, fill=(255, 255, 255))
    return img


def main():
    P = parts()
    I = np.eye(4)

    def turned(q):
        """The bracket and its screws and horns turned by q about the shaft (z)."""
        R = np.eye(4)
        R[:2, :2] = [[math.cos(q), -math.sin(q)], [math.sin(q), math.cos(q)]]
        return [(n, gr, g, R if gr in ('bracket', 'horn') or n.startswith('M3') else I) for n, gr, g in P]

    def exploded():
        """Spread along the slide-in axis in fitting order: cradle, servo (horns on), fork; screws backed out."""
        out = []
        for n, gr, g in P:
            c = g['c']
            if gr == 'holder' or (gr == 'screw' and n.startswith('M2')):
                d = (-0.045, 0, 0.022 * np.sign(c[2]) if gr == 'screw' else 0)
            elif gr == 'horn':
                d = (0, 0, 0.008 * np.sign(c[2]))
            elif gr == 'bracket' or n.startswith('M3'):
                d = (0.095, 0, 0.02 * np.sign(c[2]) if gr == 'screw' else 0)
            else:
                d = (0, 0, 0)
            M = np.eye(4)
            M[:3, 3] = d
            out.append((n, gr, g, M))
        return out
    tiles = [render(turned(0.0), (135, -25), 'assembled'), render(turned(1.2), (135, -25), 'turned +1.2 rad'),
             render(exploded(), (100, -20), 'exploded, fitting order: cradle, servo, fork', dist=0.32, look=(0.02, 0, 0)), render(turned(0.0), (90, -5), 'side')]
    sheet = Image.new('RGB', (1040, 1080), 'white')
    ImageDraw.Draw(sheet)
    for k, t in enumerate(tiles):
        sheet.paste(t, ((k % 2) * 520, (k // 2) * 520))
    leg = ImageDraw.Draw(sheet)
    for i, (n, c) in enumerate(COL.items()):
        leg.rectangle([10 + i * 200, 1052, 24 + i * 200, 1066], fill=c)
        leg.text((30 + i * 200, 1053), {'case': 'STS3215 (bought)', 'horn': 'horns (in its box)', 'holder': 'cradle: parent link',
                                         'bracket': 'U-bracket: child link', 'screw': 'screws'}[n], fill=(0, 0, 0))
    out = os.path.join(G.ROOT, 'reports', 'compare', 'proto_mount.png')
    sheet.save(out)
    print(out)


if __name__ == '__main__':
    main()
