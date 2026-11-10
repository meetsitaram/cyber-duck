"""One picture of each electrical part, from the saved Fusion model, for the wiring diagram: docs/dynamixel/parts/. Opens the model, shows one part at a time, closes it unsaved.

  python -I tools/fusion_call.py cad/fusion/part_pictures.py --write
"""
import os

import adsk.core
import adsk.fusion

REPO = r'C:\Users\meets\Projects\duck-droids'
OUT = os.path.join(REPO, 'docs', 'dynamixel', 'parts')
NAME = 'duck-droids dynamixel'
# picture name -> (does this body belong? (component name, body name)), view direction
TARGETS = {
    'pi': (lambda c, b: c == 'Pi Zero 2 W', (1, -.25, .25)),
    'waveshare': (lambda c, b: c == 'Waveshare bus board' or b.startswith('bus board cable plugs'), (-1, -.25, .3)),
    'servo': (lambda c, b: ' XL330 ' in c and c.startswith('head roll'), (.5, -.75, .55)),
    'regulator': (lambda c, b: b == 'L 5 V regulator', (-1, -.25, .3)),
    'imu': (lambda c, b: b == 'IMU', (-.3, -.25, 1)),
    'camera': (lambda c, b: b == 'camera', (-1, -.25, .3)),
    'switch': (lambda c, b: b == 'L foot switch', (.5, -.6, .6)),
}


def run(_context: str):
    app = adsk.core.Application.get()
    os.makedirs(OUT, exist_ok=True)
    hub = app.data.activeHub
    proj = next(hub.dataProjects.item(i) for i in range(hub.dataProjects.count) if hub.dataProjects.item(i).name == 'my-projects')
    fs = proj.rootFolder.dataFolders
    fd = next(fs.item(i) for i in range(fs.count) if fs.item(i).name == 'duck-droids')
    f = next(fd.dataFiles.item(i) for i in range(fd.dataFiles.count) if fd.dataFiles.item(i).name == NAME)
    doc = app.documents.open(f)
    d = adsk.fusion.Design.cast(app.activeProduct)
    root = d.rootComponent
    root.isJointsFolderLightBulbOn = False
    root.isSketchFolderLightBulbOn = False
    root.isConstructionFolderLightBulbOn = False
    items = []
    for o in root.allOccurrences:
        for b in o.bRepBodies:
            items.append((o.component.name, b))
    for b in root.bRepBodies:
        items.append(('', b))
    print(len(items), 'bodies; components', sorted({c for c, _ in items if 'XL330' in c or 'Pi' in c or 'Wave' in c})[:8])
    vp = app.activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedVisualStyle
    for name, (test, view) in TARGETS.items():
        lo, hi = [9e9] * 3, [-9e9] * 3
        n = 0
        for c, b in items:
            on = test(c, b.name)
            b.isLightBulbOn = on
            if on:
                n += 1
                bb = b.boundingBox
                lo = [min(lo[0], bb.minPoint.x), min(lo[1], bb.minPoint.y), min(lo[2], bb.minPoint.z)]
                hi = [max(hi[0], bb.maxPoint.x), max(hi[1], bb.maxPoint.y), max(hi[2], bb.maxPoint.z)]
        if not n:
            print(name, 'NOTHING')
            continue
        c_ = [(a + b_) / 2 for a, b_ in zip(lo, hi)]
        size = max(b_ - a for a, b_ in zip(lo, hi))
        cam = vp.camera
        cam.isSmoothTransition = False
        cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
        cam.target = adsk.core.Point3D.create(*c_)
        L = sum(v * v for v in view) ** .5
        cam.eye = adsk.core.Point3D.create(*[c_[k] + 40 * view[k] / L for k in range(3)])
        cam.upVector = adsk.core.Vector3D.create(0, 1, 0) if abs(view[2]) > .9 else adsk.core.Vector3D.create(0, 0, 1)
        cam.viewExtents = size * 1.5
        vp.camera = cam
        vp.refresh()
        vp.saveAsImageFile(os.path.join(OUT, f'{name}_raw.png'), 1000, 1000)
        print(name, n, 'bodies', round(size * 10, 1), 'mm')
    doc.close(False)
    print('open now', [x.name for x in app.documents])
