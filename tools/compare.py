"""Side-by-side look comparison: microduck (its own model, rendered locally) beside milli, same view, same scale.

  python tools/compare.py [--duck PATH]   -> reports/compare/compare_<view>.png, compare_views.png

microduck's MJCF (pollen-robotics/microduck_rl, src/mjlab_microduck/robot/microduck/robot_walk.xml) carries CC BY-NC-SA
meshes: it stays outside this repo (default path below, or --duck), and so do these pictures (reports/compare/ is
ignored). Each pair is drawn at one scale: microduck enlarged by milli's height over its own (the proportions are what
milli must match, tools/checks.py 'targets'), with microduck's landmark heights from spec/targets.json drawn across
both at that scale. milli stands with its knees bent as microduck's model does.
"""
import argparse
import json
import math
import os
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

DIR = os.path.join(G.ROOT, 'reports', 'compare')
DUCK = os.path.join(os.environ.get('TEMP', '/tmp'), 'figrefs', 'microduck_rl', 'src', 'mjlab_microduck', 'robot',
                    'microduck', 'robot_walk.xml')
LINES = [('head top', 'height_m'), ('head bottom', 'head_bottom_z'), ('neck pitch', 'neck_pitch_z'),
         ('hip pitch', 'hip_pitch_z')]
W = H = 700


def render_duck(path, view, pic_h, look_z, look_x=0.0):
    """microduck at its zero pose, floor under its soles, orthographic, as render_pose.Scene.render frames milli."""
    spec = mujoco.MjSpec.from_file(path)
    spec.visual.global_.offwidth = spec.visual.global_.offheight = 1600
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    zmin = min(float((m.mesh_vert[m.mesh_vertadr[m.geom_dataid[g]]:m.mesh_vertadr[m.geom_dataid[g]] +
                                  m.mesh_vertnum[m.geom_dataid[g]]] @ d.geom_xmat[g].reshape(3, 3).T)[:, 2].min()
                       + d.geom_xpos[g][2])
               for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH)
    az, el = (math.radians(a) for a in RP.VIEWS[view])
    dv = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [look_x, 0, zmin + look_z]
    cam.distance = 2.0
    cam.azimuth = math.degrees(math.atan2(-dv[1], -dv[0]))
    cam.elevation = -math.degrees(el)
    m.vis.global_.orthographic = 1
    m.vis.global_.fovy = pic_h
    opt = mujoco.MjvOption()
    for k in range(6):
        opt.geomgroup[k] = 1
    r = mujoco.Renderer(m, H, W)
    r.update_scene(d, cam, opt)
    img = r.render().copy()
    r.enable_segmentation_rendering()
    r.update_scene(d, cam, opt)
    seg = r.render()[:, :, 0]
    r.close()
    img[seg < 0] = 255
    return Image.fromarray(img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duck', default=DUCK)
    a = ap.parse_args()
    spec = G.load()
    tg = json.load(open(os.path.join(G.ROOT, 'spec', 'targets.json'), encoding='utf-8'))['model']
    sc = RP.Scene(spec)
    height = C.measure(spec, sc.S)['height_m']
    k = height / tg['height_m']
    pic = 1.15 * height                               # metres shown, top to bottom
    look = 0.5 * height
    q = G.mirror_pose(C.BENT)
    tiles = []
    for view in ('side', '34', 'front'):
        duck = render_duck(a.duck, view, pic / k, look / k)
        milli = sc.render(q, view, W, H, pic, look)
        pair = Image.new('RGB', (2 * W, H + 30), 'white')
        pair.paste(duck, (0, 30))
        pair.paste(milli, (W, 30))
        dr = ImageDraw.Draw(pair)
        dr.text((8, 8), f'microduck (its model, x {k:.2f} to the same height)', fill=(0, 0, 0))
        dr.text((W + 8, 8), f'milli ({height:.3f} m, knees bent as microduck stands)', fill=(0, 0, 0))
        px = H / pic
        for label, key in LINES:
            z = tg[key] * k
            y = 30 + int(H / 2 - (z - look) * px)
            dr.line([(0, y), (2 * W, y)], fill=(40, 120, 220), width=1)
            dr.text((2 * W - 120, y - 12), label, fill=(40, 120, 220))
        out = os.path.join(DIR, f'compare_{view}.png')
        pair.save(out)
        tiles.append(pair)
        print(out, flush=True)
    # head close-ups on a 10 mm grid (milli's millimetres; microduck's scaled by k), 0.22 m tall, centred on the
    # head, so the hood's lines, the face's depth, the eye and the bands can be read off square by square
    hz = {'duck': 0.236, 'milli': None}
    S = sc.S
    hi_ = next(i for i, g in enumerate(spec['links']['head']['geoms']) if g['name'] == 'head')
    hb = C.bbox(S.geom[('head', hi_)])
    hz_m = float((hb[0][2] + hb[1][2]) / 2) - 0.006
    span = 0.22
    heads = []
    for view in ('side', '34', 'front'):
        duck = render_duck(a.duck, view, span / k, hz['duck'], 0.031)       # its head's middle (x -0.030 .. 0.092)
        milli = sc.render(q, view, W, H, span, hz_m, target_x=float((hb[0][0] + hb[1][0]) / 2))
        pair = Image.new('RGB', (2 * W, H + 30), 'white')
        pair.paste(duck, (0, 30))
        pair.paste(milli, (W, 30))
        dr = ImageDraw.Draw(pair)
        dr.text((8, 8), f'microduck head (x {k:.2f}), 10 mm grid', fill=(0, 0, 0))
        dr.text((W + 8, 8), 'milli head, 10 mm grid', fill=(0, 0, 0))
        px = H / span
        for i in range(-8, 9):
            for x0 in (0, W):
                y = 30 + int(H / 2 + i * 0.01 * px)
                dr.line([(x0, y), (x0 + W, y)], fill=(170, 200, 235) if i else (60, 120, 220), width=1)
                xg = x0 + int(W / 2 + i * 0.01 * px)
                dr.line([(xg, 30), (xg, 30 + H)], fill=(170, 200, 235) if i else (60, 120, 220), width=1)
        out = os.path.join(DIR, f'head_{view}.png')
        pair.save(out)
        heads.append(pair)
        print(out, flush=True)
    hs = Image.new('RGB', (2 * W, 3 * (H + 30)), 'white')
    for i, t in enumerate(heads):
        hs.paste(t, (0, i * (H + 30)))
    hs.save(os.path.join(DIR, 'head_views.png'))
    sheet = Image.new('RGB', (2 * W, 3 * (H + 30)), 'white')
    for i, t in enumerate(tiles):
        sheet.paste(t, (0, i * (H + 30)))
    sheet.save(os.path.join(DIR, 'compare_views.png'))


if __name__ == '__main__':
    main()
