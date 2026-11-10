"""Cyber Duck's transformation, truck -> standing duck, as a short MP4 and a contact sheet: lie down on the chest,
tip the legs to the floor, pitch the head back into the windshield, fold the feet into the tail.

  MILLI_RECIPE=dynamixel python tools/cyber_duck.py         (first: the painted shells, reports/<recipe>/cyber_duck/painted.pkl)
  MILLI_RECIPE=dynamixel python tools/cyber_duck_video.py   -> reports/<recipe>/cyber_duck/transform.mp4, transform_sheet.png

Kinematic: the joints and the body's lean follow eased keyframes, lowered each frame onto whatever is lowest. It
shows the moves, not physics: nothing here says the servos can lift it through them. Plain python.
"""
import math
import os
import pickle
import sys

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cyber_duck as CD  # noqa: E402
import render_pose as RP  # noqa: E402

FPS = 20
W, H = 960, 600


ease = CD.ease
pose = CD.transform_pose


def main():
    import memcap
    memcap.guard(cap_gb=6, need_free_gb=16)
    with open(os.path.join(CD.OUT, 'painted.pkl'), 'rb') as fh:
        data = pickle.load(fh)
    md = CD.Model()
    sc = CD.Scene(md, data['extra'])
    CD.attach_flaps(md, data['hinges'], data.get('jaw'))
    sc.R = md.R
    RP.VIEWS.update(front34=(35, 14))
    # the user, 2026-11-10: truck -> standing duck (the reveal)
    seq = [1.0] * 25 + list(np.linspace(1, 0, 80)) + [0.0] * 10
    # at the end the duck chats: the bill (the truck's nose and front wheel) opens and shuts twice on the wheel hub
    chat = [0.35 * math.sin(math.pi * k / 18) ** 2 for k in range(36)] + [0.0] * 14
    seq = [(t, None) for t in seq] + [(0.0, j) for j in chat]
    frames, sheet = [], []
    for k, (t, jaw) in enumerate(seq):
        base, q = pose(t)
        if jaw is not None:
            q['jaw'] = jaw
        sc.base = base
        tz = 0.17 * (1 - ease(t)) + 0.06 * ease(t)
        cx = 0.5 * (data['geo']['x_nose'] + data['geo']['x_tail'])     # the truck's middle
        im = sc.render(q, 'front34', W, H, 0.44, tz, 0.0, (), target_x=ease(t) * cx)
        ImageDraw.Draw(im).text((12, 10), 'Cyber Duck (kinematic: shapes and moves, no physics)', fill=(60, 60, 60))
        frames.append(np.asarray(im))
        if k in (20, 50, 75, 104, 124):
            sheet.append(im.resize((W // 2, H // 2)))
        print(k, len(seq), flush=True)
    out = os.path.join(CD.OUT, 'transform.mp4')
    imageio.mimsave(out, frames, fps=FPS, codec='libx264', quality=8, macro_block_size=8)
    st = Image.new('RGB', (len(sheet) * W // 2, H // 2), 'white')
    for k, im in enumerate(sheet):
        st.paste(im, (k * W // 2, 0))
    st.save(os.path.join(CD.OUT, 'transform_sheet.png'))
    print(out)


if __name__ == '__main__':
    main()
