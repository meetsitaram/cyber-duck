"""A short clip of milli's motions: chatting, looking round, nodding, crouching, the floor pick, tucking the head in.

  python tools/render_motions.py   -> reports/motion/milli_motions.mp4 and .gif

Keyframes (joint angles, the left foot planted) eased with a cosine between them, rendered from a 3/4 view. The poses
are the ones tools/checks.py checks (collisions, torque, balance), so every frame is a pose milli can reach.
"""
import math
import os
import sys

import imageio.v2 as imageio
import numpy as np
from PIL import ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

FPS = 24


def legs(hip=0.0, knee=0.0, ankle=0.0):
    return {f'{s}_{j}': v for s in 'LR' for j, v in (('hip_pitch', hip), ('knee', knee), ('ankle_pitch', ankle))}


STAND = {}
CROUCH = legs(0.38, -0.88, 0.5)
PICK = dict(legs(-0.22, -1.08, 0.7), neck_pitch=0.8, head_pitch=0.78)
# (caption, pose, seconds to reach it, seconds held)
SCRIPT = [
    ('standing', STAND, 0.0, 0.6),
    ('chatting', dict(beak=0.4, head_roll=0.25), 0.35, 0.15),
    ('chatting', dict(beak=0.05, head_roll=0.25), 0.3, 0.1),
    ('chatting', dict(beak=0.4, head_roll=-0.2), 0.35, 0.15),
    ('chatting', dict(beak=0.0), 0.35, 0.3),
    ('looking round', dict(head_yaw=1.2), 0.8, 0.3),
    ('looking round', dict(head_yaw=-1.2, head_roll=0.2), 1.2, 0.3),
    ('looking round', STAND, 0.7, 0.2),
    ('nodding', dict(neck_pitch=0.5, head_pitch=-0.2), 0.5, 0.1),
    ('nodding', dict(neck_pitch=-0.1, head_pitch=0.3), 0.6, 0.1),
    ('nodding', STAND, 0.4, 0.2),
    ('crouching', CROUCH, 0.9, 0.3),
    ('crouching', STAND, 0.8, 0.3),
    ('floor pick', PICK, 1.2, 0.2),
    ('floor pick: beak', dict(PICK, beak=0.3), 0.4, 0.3),
    ('floor pick: beak', PICK, 0.3, 0.2),
    ('floor pick', STAND, 1.2, 0.3),
    ('tucking the head in', dict(neck_pitch=1.55, head_pitch=-1.45), 1.2, 0.6),
    ('standing', STAND, 1.2, 0.8),
]


def blend(a, b, t):
    keys = set(a) | set(b)
    return {k: (1 - t) * a.get(k, 0.0) + t * b.get(k, 0.0) for k in keys}


def frames():
    prev = STAND
    for cap, pose, move, hold in SCRIPT:
        n = max(1, int(round(move * FPS)))
        for i in range(n if move else 0):
            t = 0.5 - 0.5 * math.cos(math.pi * (i + 1) / n)
            yield cap, blend(prev, pose, t)
        for _ in range(int(round(hold * FPS))):
            yield cap, pose
        prev = pose


def main():
    sc = RP.Scene(G.load())
    out_dir = os.path.join(G.ROOT, 'reports', 'motion')
    os.makedirs(out_dir, exist_ok=True)
    imgs = []
    for k, (cap, q) in enumerate(frames()):
        im = sc.render(q, '34', 640, 640, 0.40, 0.165)
        ImageDraw.Draw(im).text((14, 12), f'milli: {cap}', fill=(40, 40, 40))
        imgs.append(np.asarray(im))
        if k % 48 == 0:
            print(k, cap, flush=True)
    mp4 = os.path.join(out_dir, 'milli_motions.mp4')
    imageio.mimsave(mp4, imgs, fps=FPS, quality=8, macro_block_size=16)
    gif = os.path.join(out_dir, 'milli_motions.gif')
    small = [np.asarray(__import__('PIL.Image', fromlist=['Image']).fromarray(f).resize((400, 400))) for f in imgs[::2]]
    imageio.mimsave(gif, small, duration=2 / FPS, loop=0)
    print(mp4, len(imgs), 'frames')


if __name__ == '__main__':
    main()
