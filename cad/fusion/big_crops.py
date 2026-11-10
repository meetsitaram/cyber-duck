"""The big duck's joints, close up, in Fusion's own pictures: docs/images/fusion/big/crops/<joint>_<view>.png. Opens its assembly, hides the skin (in this open copy only: nothing's saved), looks, closes.

  python -I tools/fusion_call.py cad/fusion/big_crops.py --write

CROPS picks which; empty is all. Zero pose only: joints set from the API here moved boxes, but neither the saved pictures nor the clash check followed them (2026-11-05, docs/big/FUSION.md).
"""
import json
import os
import sys
import time

P = r'C:\Users\meets\Projects\duck-droids\cad\fusion'
if P not in sys.path:
    sys.path.insert(0, P)
import adsk.core  # noqa: E402
import adsk.fusion  # noqa: E402

REPO = os.path.dirname(os.path.dirname(P))
CROPS = ()
HIDE = ()
SKIN = ('body panel', 'hood panel', 'bill panel', 'face panel', 'thigh cover', 'shin cover', 'beak panel', 'shoe', 'sole', 'eye ')
# target, half width in m, from the spec's anchors: read below
VIEWS = {'front': (1, 0, 0), 'back': (-1, 0, 0), 'out': (0, 1, 0), 'in': (0, -1, 0), 'high': (0.6, 0.55, 0.6), 'low': (0.6, 0.55, -0.6)}


def hide_skin(root):
    n = 0
    for o in root.allOccurrences:
        c = o.component
        for i in range(c.bRepBodies.count):
            b = c.bRepBodies.item(i)
            if any(k in b.name for k in SKIN + HIDE) and b.isVisible:
                b.isLightBulbOn = False
                n += 1
    return n


def run(_context: str):
    t0 = time.time()
    app = adsk.core.Application.get()
    s = json.load(open(os.path.join(REPO, 'spec', 'robot_big.json'), encoding='utf-8'))
    O = {j['name']: j['origin'] for j in s['joints']}
    hub = lambda n: next(g['real']['hub'] for l in s['links'].values() for g in l['geoms'] if g.get('real') and g['name'].startswith(n))
    crops = {
        'hip_yaw': (O['L_hip_yaw'][:2] + [O['L_hip_yaw'][2] + 0.03], 0.13),
        'hip_roll': (O['L_hip_roll'][:1] and [0.0, O['L_hip_roll'][1], O['L_hip_roll'][2]], 0.17),
        'hip_pitch': ([0.0, 0.27, O['L_hip_pitch'][2]], 0.13),
        'knee_stators': ([hub('L_knee actuator I')[0], 0.27, hub('L_knee actuator I')[2]], 0.13),
        'knee': ([O['L_knee'][0], 0.27, O['L_knee'][2]], 0.13),
        'ankle': ([O['L_ankle_pitch'][0], 0.27, O['L_ankle_pitch'][2]], 0.12),
        'neck': ([O['neck_pitch'][0], 0.0, O['neck_pitch'][2]], 0.14),
        'head_pitch_drive': ([hub('head_pitch actuator L')[0], 0.05, hub('head_pitch actuator L')[2]], 0.12),
        'head_pitch': ([O['head_pitch'][0], 0.0, O['head_pitch'][2]], 0.12),
        'head_yaw': ([O['head_yaw'][0], 0.0, O['head_yaw'][2] - 0.02], 0.12),
        'head_roll': ([O['head_roll'][0] - 0.03, 0.0, O['head_roll'][2]], 0.12),
        'beak': (hub('beak actuator'), 0.10),
        # the right leg: the left's mirror in the spec, with the actuators turned, not mirrored
        'r_hip_roll': ([0.0, O['R_hip_roll'][1], O['R_hip_roll'][2]], 0.17),
        'r_hip_pitch': ([0.0, -0.27, O['R_hip_pitch'][2]], 0.13),
        'r_knee': ([O['R_knee'][0], -0.27, O['R_knee'][2]], 0.13),
        'r_ankle': ([O['R_ankle_pitch'][0], -0.27, O['R_ankle_pitch'][2]], 0.12),
    }
    hub_ = next(app.data.activeHub.dataProjects.item(i) for i in range(app.data.activeHub.dataProjects.count) if app.data.activeHub.dataProjects.item(i).name == 'my-projects')
    top = next(hub_.rootFolder.dataFolders.item(i) for i in range(hub_.rootFolder.dataFolders.count) if hub_.rootFolder.dataFolders.item(i).name == 'duck-droids')
    fd = next(top.dataFolders.item(i) for i in range(top.dataFolders.count) if top.dataFolders.item(i).name == 'big')
    f = next(fd.dataFiles.item(i) for i in range(fd.dataFiles.count) if fd.dataFiles.item(i).name == 'big assembly real')
    doc = app.documents.open(f)
    d = adsk.fusion.Design.cast(app.activeProduct)
    root = d.rootComponent
    root.isJointsFolderLightBulbOn = False
    root.isSketchFolderLightBulbOn = False
    print('hidden', hide_skin(root), round(time.time() - t0), 's')
    out = os.path.join(REPO, 'docs', 'images', 'fusion', 'big', 'crops')
    os.makedirs(out, exist_ok=True)
    vp = app.activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedWithVisibleEdgesOnlyVisualStyle
    n = 0
    for name, (tg, half) in crops.items():
        if CROPS and name not in CROPS:
            continue
        for k, v in VIEWS.items():
            cam = vp.camera
            cam.isSmoothTransition = False
            cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
            cam.target = adsk.core.Point3D.create(*[x * 100 for x in tg])
            cam.eye = adsk.core.Point3D.create(*[(tg[i] + v[i] * 3.0) * 100 for i in range(3)])
            cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
            cam.viewExtents = half * 2 * 100
            vp.camera = cam
            vp.refresh()
            vp.saveAsImageFile(os.path.join(out, f'{name}_{k}.png'), 700, 700)
            n += 1
    print('pictures', n, round(time.time() - t0), 's')
    doc.close(False)
    print('open now', app.documents.count)
