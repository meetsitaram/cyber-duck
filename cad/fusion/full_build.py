"""One Fusion MCP script: build a variant whole, check it, save it in the hub, export it, close it. Set RECIPE.

  python -I tools/fusion_call.py cad/fusion/full_build.py --recipe dynamixel    or paste it into fusion_mcp_execute

First: MILLI_RECIPE=<recipe> python -I tools/export_prints.py, and tools/verify_fusion.py write. Afterwards: tools/verify_fusion.py compare.
Takes minutes: give the call 10. Saved at my-projects / duck-droids as "duck-droids <recipe>", replacing an older save of that name.
"""
import importlib
import json
import os
import sys
import time

P = r'C:\Users\meets\Projects\duck-droids\cad\fusion'
if P not in sys.path:
    sys.path.insert(0, P)
import figlib, build  # noqa: E402,E401
importlib.reload(figlib)
importlib.reload(build)
import adsk.core  # noqa: E402
import adsk.fusion  # noqa: E402

RECIPE = 'hybrid'
KEEP_OPEN = False                                         # leave the finished model open, to look at
VIEWS = {'34': (60, 40, 30), 'back': (-60, 35, 30), 'side': (0, 80, 16), 'under': (40, 30, -25), 'top': (20, 10, 90)}


def interference(d):
    occs = adsk.core.ObjectCollection.create()
    for o in d.rootComponent.occurrences:
        occs.add(o)
    i = d.createInterferenceInput(occs)
    i.areCoincidentFacesIncluded = False
    r = d.analyzeInterference(i)
    if r is None:                                         # (Fusion can return nothing on a model with a broken body)
        print('INTERFERENCE: no result')
        return [], []
    named = lambda e: getattr(e, 'name', '?')
    out = sorted(((named(r.item(k).entityOne), named(r.item(k).entityTwo), round(r.item(k).interferenceBody.volume * 1000, 2)) for k in range(r.count)), key=lambda q: -q[2])
    # what is drawn so on purpose: horn seats, heads in their counterbores, a race on its horn
    # a real servo's own pieces overlap in its maker's model; a plug is drawn into its real socket
    ok = ('flange', 'rear horn', 'rear boss', 'screw', 'inner race', 'cable plugs')
    same = lambda a, b: any(m in a and m in b and a.split(m)[0] == b.split(m)[0] for m in (' XL330 ', ' STS3215 ', ' HD1910 '))
    return out, [q for q in out if not any(k in q[0] + q[1] for k in ok) and not same(q[0], q[1]) and q[2] > 1.0]


def run(_context: str):
    t = time.time()
    app = adsk.core.Application.get()
    name = f'duck-droids {RECIPE}'
    for i in range(app.documents.count - 1, -1, -1):      # one document at a time: anything saved can go
        x = app.documents.item(i)
        # (this variant's old model too, even marked modified: it's generated, and left open its file can't be replaced
        # without an "Unsaved changes" dialog, which stops Fusion answering anything: 2026-11-04, a reveal's show and hide marked it)
        if (x.isSaved and not x.isModified) or x.name.split(' v')[0] == name:
            x.close(False)
    spec_name = 'robot.json' if RECIPE == 'hybrid' else f'robot_{RECIPE}.json'
    build.SPEC = spec_name
    print(build.new_document())
    for n in build.spec()['links']:
        print(build.link(n))
    print('bodies', round(time.time() - t), 's;', build.parametric(), build.joints())
    st = build.status()
    print(json.dumps(st))
    v = build.verify()
    print('verify', v['parts'], 'bad', json.dumps(v['bad']))
    print(build.verify_points())
    d = build.design()
    root = d.rootComponent
    S = {j['name']: j for j in build.spec()['joints']}
    refused = []
    for i in range(root.asBuiltJoints.count):
        jt = root.asBuiltJoints.item(i)
        m = adsk.fusion.RevoluteJointMotion.cast(jt.jointMotion)
        for lim in (S[jt.name]['lower'], S[jt.name]['upper']):
            m.rotationValue = lim * 0.98
            if abs(m.rotationValue - lim * 0.98) > 0.02:
                refused.append(jt.name)
        m.rotationValue = 0.0
    print('joints refused', refused)
    clash, odd = interference(d)
    json.dump(clash, open(os.path.join(build.REPO, 'reports', build._variant_file('fusion_interference')), 'w', encoding='utf-8'), indent=0)
    print('clashes', len(clash), 'to explain', odd[:12])
    print('look', json.dumps(build.look_check()))
    print(build.export_print_stls(os.path.join(os.environ['TEMP'], 'duck_fusion_stl', RECIPE)))
    vp = app.activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedVisualStyle
    root.isJointsFolderLightBulbOn = False
    root.isSketchFolderLightBulbOn = False
    for o in root.occurrences:
        o.component.isSketchFolderLightBulbOn = False
        o.component.isConstructionFolderLightBulbOn = False
    folder = os.path.join(build.REPO, 'docs', 'images', 'fusion', RECIPE)
    os.makedirs(folder, exist_ok=True)
    for k, eye in VIEWS.items():
        cam = vp.camera
        cam.isSmoothTransition = False
        cam.cameraType = adsk.core.CameraTypes.PerspectiveCameraType
        cam.target = adsk.core.Point3D.create(0, 0, 16)
        cam.eye = adsk.core.Point3D.create(*eye)
        cam.upVector = adsk.core.Vector3D.create(0, 1, 0) if k == 'top' else adsk.core.Vector3D.create(0, 0, 1)
        vp.camera = cam
        vp.fit()
        vp.refresh()
        p = os.path.join(folder, f'{k}.png')
        if os.path.exists(p):
            os.remove(p)
        vp.saveAsImageFile(p, 1400, 1400)
    if st['problems'] or v['bad'] and False:
        print('NOT SAVED: problems')
        doc = app.activeDocument
        doc.close(False)
        return
    hub = app.data.activeHub
    proj = next(hub.dataProjects.item(i) for i in range(hub.dataProjects.count) if hub.dataProjects.item(i).name == 'my-projects')
    fs = proj.rootFolder.dataFolders
    fd = next((fs.item(i) for i in range(fs.count) if fs.item(i).name == 'duck-droids'), None) or fs.add('duck-droids')
    for i in range(fd.dataFiles.count - 1, -1, -1):
        if fd.dataFiles.item(i).name == name:             # the older save of this variant
            fd.dataFiles.item(i).deleteMe()
    doc = app.activeDocument
    print('saved', doc.saveAs(name, fd, 'Generated from spec/robot.py: github.com/meetsitaram/duck-droids', ''), doc.name)
    em = d.exportManager
    stem = os.path.join(build.REPO, 'cad', name.replace(' ', '-'))
    em.execute(em.createFusionArchiveExportOptions(stem + '.f3d'))
    print('f3d', os.path.getsize(stem + '.f3d'), '; now run cad/fusion/export_step.py for the STEP')
    if not KEEP_OPEN:
        doc.close(False)
    print('open now', [app.documents.item(i).name for i in range(app.documents.count)], round(time.time() - t), 's')
