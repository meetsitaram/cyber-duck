"""Close-ups from a saved one-document model, and its STEP: docs/images/fusion/<recipe>/close_*.png, cad/duck-droids-<recipe>.step. Opens the model from the hub, closes it unsaved.

  python -I tools/fusion_call.py cad/fusion/close_ups.py --write --recipe hd1910
"""
import os
import re
import traceback

import adsk.core
import adsk.fusion

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) if '__file__' in globals() else r'C:\Users\meets\Projects\duck-droids'
RECIPE = 'hybrid'
COVERS = r'hood|thigh cover|body shell'
# name: eye, target, width (cm), what to hide
VIEWS = {'leg_open': ((-2, 28, 9), (-2, 5, 8), 17, COVERS), 'leg_rear': ((-28, 6, 8), (-2, 5, 8), 17, COVERS), 'hip_under': ((14, 12, -1), (0, 3, 11), 14, ''), 'ankle': ((16, 18, 9), (-3.8, 3, 3), 10, ''),
         'neck': ((20, 12, 24), (2, 0, 18), 14, COVERS), 'neck_rear': ((-20, -12, 24), (2, 0, 18), 14, COVERS), 'head_open': ((-14, -20, 40), (5, 0, 25), 20, 'hood'), 'head_rear': ((-24, 3, 36), (5, 0, 25), 20, 'hood'),
         'regulator': ((8, -26, 27), (8, -4, 25), 12, 'hood'), 'body_open': ((-20, -12, 23), (-2, 0, 13), 15, COVERS)}


def bodies(root):
    out = [root.bRepBodies.item(i) for i in range(root.bRepBodies.count)]
    for o in [root.allOccurrences.item(i) for i in range(root.allOccurrences.count)]:
        out += [o.bRepBodies.item(i) for i in range(o.bRepBodies.count)]
    return out


def run(_context=None):
    app = adsk.core.Application.get()
    hub = app.data.activeHub
    proj = next(hub.dataProjects.item(i) for i in range(hub.dataProjects.count) if hub.dataProjects.item(i).name == 'my-projects')
    fs = proj.rootFolder.dataFolders
    fd = next(fs.item(i) for i in range(fs.count) if fs.item(i).name == 'duck-droids')
    df = next(fd.dataFiles.item(i) for i in range(fd.dataFiles.count) if fd.dataFiles.item(i).name == f'duck-droids {RECIPE}')
    doc = app.documents.open(df, True)
    try:
        d = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        p = os.path.join(REPO, 'cad', f'duck-droids-{RECIPE}.step')
        em = d.exportManager
        print('step', em.execute(em.createSTEPExportOptions(p)), os.path.getsize(p))
        all_ = bodies(d.rootComponent)
        print('bodies', len(all_))
        out = os.path.join(REPO, 'docs', 'images', 'fusion', RECIPE)
        vp = app.activeViewport
        for name, (eye, tgt, width, hide) in VIEWS.items():
            for b in all_:
                b.isLightBulbOn = not (hide and re.search(hide, b.name))
            cam = vp.camera
            cam.isSmoothTransition = False
            cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
            cam.eye = adsk.core.Point3D.create(*eye)
            cam.target = adsk.core.Point3D.create(*tgt)
            cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
            cam.isFitView = False
            cam.viewExtents = width
            vp.camera = cam
            vp.refresh()
            adsk.doEvents()
            vp.saveAsImageFile(os.path.join(out, f'close_{name}.png'), 1200, 1200)
        print('views', len(VIEWS))
    except Exception:
        print('ERR', traceback.format_exc()[-600:])
    finally:
        doc.close(False)
    print('open', app.documents.count)


run()
