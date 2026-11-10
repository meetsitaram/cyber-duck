"""Joins the frames from cad/fusion/reveal_frames.py into the 30-second reveal, with a title for each stage. docs/images/assembly/<variant>/reveal_30s.mp4

  python -I tools/reveal_video.py
"""
import glob
import os
import sys

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402

SRC = os.path.join(os.environ['TEMP'], 'duck_reveal')
MOTORS, DUCK = ('15 HD-1910 servos', 'Feetech HD-1910 duck') if G.RECIPE == 'hd1910' else ('15 XL330 servos', 'All-Dynamixel duck')
TITLES = [(0, 150, 'Motors', MOTORS), (150, 355, 'Skeleton', 'printed frames, bearings and screws'),
          (360, 505, 'Electronics', 'Pi Zero 2 W, bus board, battery, IMU, camera'), (510, 700, 'Covers', 'hood, jaw, thigh covers, body shell, soles'),
          (705, 900, DUCK, '')]
if G.RECIPE == 'hybrid':
    TITLES = [(0, 150, 'Motors', '10 STS3215 in the legs, 5 Dynamixels at the neck and head'), (150, 355, 'Skeleton', 'printed frames, bearings and screws'),
              (360, 505, 'Electronics', 'Pi Zero 2 W, 2 bus boards, 2 regulators, battery, IMU, camera'), (510, 700, 'Covers', 'hood, jaw, thigh covers, body shell, soles'),
              (705, 900, 'Hybrid duck', '')]


if G.RECIPE == 'feetech':
    TITLES[0] = (0, 150, 'Motors', '15 STS3215 servos')
    TITLES[-1] = (705, 900, 'All-Feetech duck', '')
HOLD, FADE = 30, 18                                 # the opening: 1 s on the whole duck, 0.6 s fading away


def font(size):
    try:
        return ImageFont.truetype('arialbd.ttf', size)
    except OSError:
        return ImageFont.load_default()


def main():
    files = sorted(glob.glob(os.path.join(SRC, 'f*.png')))
    p = os.path.join(G.ROOT, 'docs', 'images', 'assembly', G.RECIPE, 'reveal_30s.mp4')
    w = imageio.get_writer(p, fps=30, codec='libx264', quality=9, macro_block_size=1, pixelformat='yuv420p')
    big, small = font(76), font(36)

    def titled(im, t, s, al=255):
        ov = Image.new('RGBA', im.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)
        d.text((im.width / 2 - d.textlength(t, font=big) / 2, 1640), t, fill=(24, 28, 30, al), font=big)
        d.text((im.width / 2 - d.textlength(s, font=small) / 2, 1730), s, fill=(90, 96, 100, al), font=small)
        return Image.alpha_composite(im, ov)

    # the opening, which is also the thumbnail: the whole duck from the front, then a fade into the start
    last, first = Image.open(files[-1]).convert('RGBA'), Image.open(files[0]).convert('RGBA')
    _, _, t_end, s_end = TITLES[-1]
    for _ in range(HOLD):
        w.append_data(np.asarray(titled(last, t_end, s_end).convert('RGB')))
    for k in range(FADE):
        a = (k + 1) / FADE
        w.append_data(np.asarray(Image.blend(titled(last, t_end, s_end), first, a * a * (3 - 2 * a)).convert('RGB')))
    for k, f in enumerate(files):
        im = Image.open(f).convert('RGBA')
        for a, b, t, s in TITLES:
            if a <= k < b:
                im = titled(im, t, s, int(255 * min(1.0, (k - a) / 12, (b - k) / 12)))
        w.append_data(np.asarray(im.convert('RGB')))
    w.close()
    print(len(files), 'frames ->', p, os.path.getsize(p) // 1024, 'kB')


if __name__ == '__main__':
    main()
