"""Every print file on one contact sheet, as it sits on the bed: the look that finds an odd piece. reports/<variant>/prints_sheet.png

  python -I tools/prints_sheet.py
"""
import glob
import math
import os
import sys

import numpy as np
import trimesh
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402

CELL, COLS = 300, 7


def view(m, size):
    v = m.vertices - m.bounds.mean(axis=0)
    az, el = math.radians(35), math.radians(30)
    R1 = np.array([[math.cos(az), -math.sin(az), 0], [math.sin(az), math.cos(az), 0], [0, 0, 1]])
    R2 = np.array([[1, 0, 0], [0, math.cos(el), -math.sin(el)], [0, math.sin(el), math.cos(el)]])
    p = v @ R1.T @ R2.T
    s = (size - 30) / max(np.ptp(p[:, 0]), np.ptp(p[:, 2]), 1e-6)
    xy = np.c_[size / 2 + p[:, 0] * s, size / 2 - p[:, 2] * s + 8]
    f = m.faces
    n = np.cross(p[f[:, 1]] - p[f[:, 0]], p[f[:, 2]] - p[f[:, 0]])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    light = np.array([0.35, -0.75, 0.55])
    shade = np.clip(np.abs(n @ light), 0, 1)
    order = np.argsort(p[f].mean(axis=1)[:, 1])[::-1]
    im = Image.new('RGB', (size, size), (250, 250, 250))
    d = ImageDraw.Draw(im)
    for i in order:
        c = int(70 + 160 * shade[i])
        d.polygon([tuple(q) for q in xy[f[i]]], fill=(c, c, int(c * 0.96)))
    return im


def main():
    files = sorted(glob.glob(os.path.join(G.out_path('prints'), '*.stl')))
    rows = math.ceil(len(files) / COLS)
    sheet = Image.new('RGB', (COLS * CELL, rows * (CELL + 22)), 'white')
    d = ImageDraw.Draw(sheet)
    for k, f in enumerate(files):
        m = trimesh.load(f, force='mesh')
        im = view(m, CELL)
        x, y = (k % COLS) * CELL, (k // COLS) * (CELL + 22)
        sheet.paste(im, (x, y + 20))
        e = np.sort(m.extents)[::-1]
        d.text((x + 6, y + 4), f"{os.path.basename(f)[:-4]}  {e[0]:.0f}x{e[1]:.0f}x{e[2]:.0f}", fill=(20, 20, 20))
    p = G.out_path('reports', 'prints_sheet.png')
    sheet.save(p)
    print(len(files), 'parts ->', p)


if __name__ == '__main__':
    main()
