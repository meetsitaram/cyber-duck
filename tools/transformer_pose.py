"""Transformer study: the duck sits with its legs stretched forward and folds its neck down, so the head covers the
front half and the body shell the back half (the two halves of a wedge-truck shape, round 1: today's shells).

  MILLI_RECIPE=dynamixel python tools/transformer_pose.py   -> reports/<recipe>/transformer_poses.png

Kinematic: the spec's joints posed by hand, the robot lowered onto whatever is lowest. No physics. Plain python.
"""
import math
import os
import sys
import tempfile

import mujoco
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

VIEWS = dict(RP.VIEWS, top=(0, 89), side_r=(-90, 0), front34=(30, 25))
RP.VIEWS.update(VIEWS)


class SitScene(RP.Scene):
    """The trunk is the base (not the left foot), lowered so the lowest point touches the floor."""

    def model(self, q, tmp, hide=()):
        M = self.R.fk(q)
        low = min(G.apply(M[ln], v)[:, 2].min() for ln, v, f, c, r in self.parts if len(f))
        lift = np.eye(4)
        lift[2, 3] = -low
        orig = self.R.fk
        self.R.fk = lambda qq, base=None: orig(qq, base=lift)
        try:
            return super().model(q, tmp, hide)
        finally:
            self.R.fk = orig


def legs(hip, knee=0.0, ankle=0.0, roll=0.0):
    return {f'{s}_{k}': v for s in 'LR' for k, v in (('hip_pitch', hip), ('knee', knee), ('ankle_pitch', ankle))} | \
        {'L_hip_roll': roll, 'R_hip_roll': -roll}


SIT = legs(-1.57, 0.0, 0.0)
POSES = [
    ('standing (zero pose)', {}, 'side'),
    ('sitting, legs forward (hip -1.57)', SIT, 'side'),
    ('+ neck folded 1.6', SIT | {'neck_pitch': 1.6}, 'side'),
    ('+ neck 1.6, head pitched down 1.0', SIT | {'neck_pitch': 1.6, 'head_pitch': 1.0}, 'side'),
    ('+ neck 1.6, head back -1.4 (face down, crown forward)', SIT | {'neck_pitch': 1.6, 'head_pitch': -1.4}, 'side'),
    ('same, 3/4', SIT | {'neck_pitch': 1.6, 'head_pitch': -1.4}, 'hero'),
    ('same, top', SIT | {'neck_pitch': 1.6, 'head_pitch': -1.4}, 'top'),
    ('same, front', SIT | {'neck_pitch': 1.6, 'head_pitch': -1.4}, 'front'),
    # the sweep's best (tools/transformer_sweep.py): the neck past today's 1.6 limit
    ('sweep best: neck 2.4, head -1.2, ankles -1.5', SIT | legs(-1.57, 0, -1.5) | {'neck_pitch': 2.4, 'head_pitch': -1.2}, 'side'),
    ('same, 3/4', SIT | legs(-1.57, 0, -1.5) | {'neck_pitch': 2.4, 'head_pitch': -1.2}, 'hero'),
    ('neck 2.2, head -0.8, ankles -1.5', SIT | legs(-1.57, 0, -1.5) | {'neck_pitch': 2.2, 'head_pitch': -0.8}, 'side'),
    ('neck 2.0, head -0.4, ankles -1.5', SIT | legs(-1.57, 0, -1.5) | {'neck_pitch': 2.0, 'head_pitch': -0.4}, 'side'),
]


def main():
    sc = SitScene()
    tiles = []
    for title, q, view in POSES:
        im = sc.render(q, view, 640, 480, 0.42, 0.12 if view != 'top' else 0.0, 0.0, (), target_x=0.04)
        ImageDraw.Draw(im).text((8, 6), title, fill=(0, 0, 0))
        tiles.append(im)
        print(title, flush=True)
    cols = 2
    sheet = Image.new('RGB', (cols * 640, math.ceil(len(tiles) / cols) * 480), 'white')
    for k, im in enumerate(tiles):
        sheet.paste(im, ((k % cols) * 640, (k // cols) * 480))
    out = G.out_path('reports', 'transformer_poses.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    sheet.save(out)
    print(out)


if __name__ == '__main__':
    main()
