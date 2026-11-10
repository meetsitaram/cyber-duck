"""Frames for the 30-second reveal from the open Fusion model: the motors, then the skeleton, then the electronics, then the covers, on a slow turn. Set FIRST and LAST; one short batch per MCP call.

  python -I tools/fusion_call.py cad/fusion/reveal_frames.py --write      (tools/reveal_video.py joins them, with their titles)

Changes what's visible for the pictures and puts it back; saves nothing.
"""
import math
import os
import time

import adsk.core
import adsk.fusion

OUT = os.path.join(os.environ['TEMP'], 'duck_reveal')
FIRST, LAST = 0, 2
N = 900
W, H = 1080, 1920
FRONT_AZ = 25   # where the turn ends: degrees round from straight ahead (+X)
STAGES = [('motors', 0, 150), ('skeleton', 150, 330), ('electronics', 360, 480), ('covers', 510, 650)]   # frames over which each stage's parts appear, from the ground up
COVERS = ('hood', 'face', 'eye ring', 'eye iris', 'mouth roof', 'jaw', 'jaw soft insert', 'L thigh cover', 'R thigh cover', 'body shell', 'L sole', 'R sole')
ELEC = ('camera', 'IMU', 'head 5 V regulator', 'L 5 V regulator', 'R 5 V regulator', 'U2D2', 'servo bus board', 'L foot switch', 'R foot switch', 'leg bus splice')


def stage_of(body_name, comp_name):
    n = body_name
    if any(k in comp_name for k in (' XL330 ', ' STS3215 ', ' HD1910 ')) or 'actuator' in n:
        return 'motors'
    if comp_name in ('Pi Zero 2 W', 'Waveshare bus board', 'leg bus board') or n.startswith('leg bus board') or n.startswith('battery') or n in ELEC or 'cable plugs' in n or 'standoff' in n:
        return 'electronics'
    if n in COVERS or any(k in n for k in ('shoe bottom screw', 'hood rear screw', 'hinge screw', 'thigh cover screw', 'body screw')):
        return 'covers'
    return 'skeleton'


def run(_context: str):
    t0 = time.time()
    os.makedirs(OUT, exist_ok=True)
    app = adsk.core.Application.get()
    d = adsk.fusion.Design.cast(app.activeProduct)
    root = d.rootComponent
    print('doc', app.activeDocument.name)
    items = []
    for o in root.occurrences:
        for lst, cname in [(o.component.bRepBodies, o.component.name)] + [(c.component.bRepBodies, c.component.name) for c in o.childOccurrences]:
            for i in range(lst.count):
                b = lst.item(i)
                bb = b.boundingBox
                items.append((b, stage_of(b.name, cname), bb.minPoint.z, b.isLightBulbOn))
    counts = {}
    for _, s, _, _ in items:
        counts[s] = counts.get(s, 0) + 1
    print(counts)
    show = {}
    for name, a, b_ in STAGES:
        group = sorted((x for x in items if x[1] == name), key=lambda x: x[2])
        for k, x in enumerate(group):
            show[id(x)] = a + (b_ - a) * k / max(1, len(group) - 1)
    root.isJointsFolderLightBulbOn = False
    root.isSketchFolderLightBulbOn = False
    root.isConstructionFolderLightBulbOn = False
    vp = app.activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedVisualStyle
    state = {}
    try:
        for f in range(FIRST, LAST + 1):
            for x in items:
                want = f >= show[id(x)]
                if state.get(id(x)) != want:
                    x[0].isLightBulbOn = want
                    state[id(x)] = want
            t = f / (N - 1)
            # one full turn that slows to a stop on the front, then holds: the last look is the face and its eye
            turn = min(1.0, f / (N * 0.92))
            az = math.radians(FRONT_AZ - 360 * (1 - turn) ** 1.5)
            el = math.radians(16 + 7 * math.sin(t * math.pi))
            cam = vp.camera
            cam.isSmoothTransition = False
            cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
            # (framed for the all-Dynamixel's 29.5 cm. The all-Feetech is 38 cm, with a 19 cm head: as these numbers, its hood's top and base left the picture)
            big_ = 'feetech' in app.activeDocument.name
            tgt = adsk.core.Point3D.create(3.0, 0, 16.0) if big_ else adsk.core.Point3D.create(0.8, 0, 12.4)
            cam.target = tgt
            cam.eye = adsk.core.Point3D.create(tgt.x + 80 * math.cos(az) * math.cos(el), tgt.y + 80 * math.sin(az) * math.cos(el), tgt.z + 80 * math.sin(el))
            cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
            cam.viewExtents = 35.0 if big_ else 25.5
            vp.camera = cam
            vp.refresh()
            vp.saveAsImageFile(os.path.join(OUT, f'f{f:04d}.png'), W, H)
    finally:
        for x in items:
            x[0].isLightBulbOn = x[3]
    print('frames', FIRST, LAST, round(time.time() - t0, 1), 's')
