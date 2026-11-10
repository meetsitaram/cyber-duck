"""Cyber Duck's final video: the poster (a 3 s hold with a slow push-in), a crossfade into the desk clip.

  MILLI_RECIPE=dynamixel python tools/cyber_duck_final.py   -> reports/<recipe>/cyber_duck/cyber_duck_final.mp4
"""
import os
import sys

import imageio.v2 as imageio
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cyber_duck as CD  # noqa: E402

FPS, HOLD, FADE = 20, 3.0, 0.75


def main():
    poster = Image.open(os.path.join(CD.OUT, 'poster.png')).convert('RGB')
    clip = imageio.get_reader(os.path.join(CD.OUT, 'desk_transform.mp4'))
    first = clip.get_data(0)
    h, w = first.shape[:2]
    frames = []
    n_hold, n_fade = int(HOLD * FPS), int(FADE * FPS)
    for k in range(n_hold + n_fade):
        z = 1.0 + 0.04 * k / (n_hold + n_fade)               # a slow push-in
        pw, ph = int(poster.width / z), int(poster.height / z)
        x0, y0 = (poster.width - pw) // 2, (poster.height - ph) // 2
        p = np.asarray(poster.crop((x0, y0, x0 + pw, y0 + ph)).resize((w, h), Image.LANCZOS), np.float32)
        a = max(0.0, (k - n_hold) / n_fade)
        frames.append((p * (1 - a) + first.astype(np.float32) * a).astype(np.uint8))
    for fr in clip:
        frames.append(fr)
    out = os.path.join(CD.OUT, 'cyber_duck_final.mp4')
    imageio.mimsave(out, frames, fps=FPS, codec='libx264', quality=8, macro_block_size=8)
    print(out, f'{len(frames) / FPS:.1f} s')


if __name__ == '__main__':
    main()
