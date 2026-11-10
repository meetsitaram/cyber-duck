"""Does any item in the assembly videos carry a piece far from the rest of it? That's a mark riding on the wrong part: it would float in mid-air until its real part arrives.

  python tools/video_floaters.py

A set of separate parts, such as 4 standoffs, shows here on purpose: list those in SETS.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
SETS = ('bus board standoffs',)
import numpy as np
import geom as G
import render_assembly as RA
spec = G.load()
sc = RA.Scene(spec)
bad = 0
for n, ms in sc.items.items():
    if n in SETS:
        continue
    boxes = []
    for m, col in ms:
        mesh = m.to_mesh()
        v = np.asarray(mesh.vert_properties[:, :3], float)
        if len(v):
            boxes.append((v.min(0), v.max(0)))
    if len(boxes) < 2:
        continue
    # grow a connected group from the largest box
    size = [np.prod(b[1] - b[0]) for b in boxes]
    start = int(np.argmax(size))
    seen = {start}
    grew = True
    while grew:
        grew = False
        for i, b in enumerate(boxes):
            if i in seen:
                continue
            for j in seen:
                gap = np.maximum(np.maximum(boxes[j][0] - b[1], b[0] - boxes[j][1]), 0)
                if np.linalg.norm(gap) < 0.003:
                    seen.add(i)
                    grew = True
                    break
    far = [i for i in range(len(boxes)) if i not in seen]
    if far:
        bad += 1
        print(n, 'has', len(far), 'of', len(boxes), 'pieces apart:', [np.round((boxes[i][0] + boxes[i][1]) * 500, 0).tolist() for i in far][:4], 'main at', np.round((boxes[start][0] + boxes[start][1]) * 500, 0).tolist())
print(f'{len(sc.items)} items, {bad} failing')
sys.exit(1 if bad else 0)
