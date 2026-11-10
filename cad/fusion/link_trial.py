"""The small trial before splitting a build into linked documents: 2 links as their own documents, linked into an assembly, their joint, change one, refresh. Times each step.

  python -I tools/fusion_call.py cad/fusion/link_trial.py --write --recipe dynamixel

Saves in my-projects / duck-droids / link trial, deletes what it saved last time, and leaves nothing open.
"""
import importlib
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

RECIPE = 'dynamixel'
FOLDER = 'link trial'


def wait(fn, seconds=180, what=''):
    """Saves upload in the background: try until it works."""
    t = time.time()
    last = None
    while time.time() - t < seconds:
        adsk.doEvents()
        try:
            r = fn()
            if r:
                return r
        except Exception as e:      # noqa: BLE001
            last = e
        time.sleep(1.0)
    raise RuntimeError(f'timed out: {what}: {last}')


def run(_context: str):
    app = adsk.core.Application.get()
    tm = {}
    t0 = time.time()

    def lap(k):
        nonlocal t0
        tm[k] = round(time.time() - t0, 1)
        print(k, tm[k], 's')
        t0 = time.time()

    for i in range(app.documents.count - 1, -1, -1):
        app.documents.item(i).close(False)
    build.SPEC = f'robot_{RECIPE}.json'
    s = build.spec()
    j = next(x for x in s['joints'] if 'ankle' in x['name'] and x['name'].startswith('L_'))
    parent, child = j['parent'], j['child']
    print('joint', j['name'], parent, '->', child)
    hub = app.data.activeHub
    proj = next(hub.dataProjects.item(i) for i in range(hub.dataProjects.count) if hub.dataProjects.item(i).name == 'my-projects')
    fs = proj.rootFolder.dataFolders
    top = next((fs.item(i) for i in range(fs.count) if fs.item(i).name == 'duck-droids'), None) or fs.add('duck-droids')
    subs = top.dataFolders
    fd = next((subs.item(i) for i in range(subs.count) if subs.item(i).name == FOLDER), None) or subs.add(FOLDER)
    for i in range(fd.dataFiles.count - 1, -1, -1):
        fd.dataFiles.item(i).deleteMe()
    lap('folder')
    files = {}
    docs = {}
    for name in (parent, child):
        print(build.new_document())
        print(build.link(name))
        lap(f'build {name}')
        doc = app.activeDocument
        print('saved', doc.saveAs(f'trial {name}', fd, 'trial', ''))
        files[name] = wait(lambda: doc.dataFile, what='dataFile')
        docs[name] = doc
        lap(f'save {name}')
    # the assembly
    asm = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    d = adsk.fusion.Design.cast(app.activeProduct)
    d.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = d.rootComponent
    occ = {}
    for name in (parent, child):
        occ[name] = wait(lambda n=name: root.occurrences.addByInsert(files[n], adsk.core.Matrix3D.create(), True), what=f'insert {name}')
        print('inserted', name, 'referenced', occ[name].isReferencedComponent, 'bodies in it', occ[name].childOccurrences.count)
    lap('insert 2')
    occ[parent].isGrounded = True
    sk = root.sketches.add(root.xYConstructionPlane)
    sp = sk.sketchPoints.add(sk.modelToSketchSpace(figlib.P(j['origin'])))
    geo = adsk.fusion.JointGeometry.createByPoint(sp)
    ji = root.asBuiltJoints.createInput(occ[child], occ[parent], geo)
    dirs = {'x': adsk.fusion.JointDirections.XAxisJointDirection, 'y': adsk.fusion.JointDirections.YAxisJointDirection, 'z': adsk.fusion.JointDirections.ZAxisJointDirection}
    ji.setAsRevoluteJointMotion(dirs[j['axis']])
    jt = root.asBuiltJoints.add(ji)
    m = adsk.fusion.RevoluteJointMotion.cast(jt.jointMotion)
    m.rotationValue = 0.3
    print('joint moves', round(m.rotationValue, 3))
    m.rotationValue = 0.0
    lap('joint')
    print('saved assembly', asm.saveAs('trial assembly', fd, 'trial', ''))
    wait(lambda: asm.dataFile, what='assembly file')
    lap('save assembly')
    # change one, as a real rebuild would: a new version of the child's document
    cdoc = docs[child]
    cdoc.activate()
    cd = adsk.fusion.Design.cast(app.activeProduct)
    tmp = cd.rootComponent.occurrences.item(0).component
    before = tmp.bRepBodies.count
    tb = adsk.fusion.TemporaryBRepManager.get()
    box = tb.createBox(adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(0, 0, 60), adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 1, 0), 1, 1, 1))
    if cd.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        bf = tmp.features.baseFeatures.add()
        bf.startEdit()
        tmp.bRepBodies.add(box, bf)
        bf.finishEdit()
    else:
        tmp.bRepBodies.add(box)
    print('saved version 2', cdoc.save('trial change'))
    lap('change and save')
    asm.activate()
    d = adsk.fusion.Design.cast(app.activeProduct)
    n = 0

    def latest():
        nonlocal n
        adsk.doEvents()
        for k in range(asm.documentReferences.count):
            r = asm.documentReferences.item(k)
            if r.isOutOfDate:
                r.getLatestVersion()
                n += 1
        return n > 0
    wait(latest, what='refresh')
    lap('refresh')
    o = next(d.rootComponent.occurrences.item(i) for i in range(d.rootComponent.occurrences.count) if child in d.rootComponent.occurrences.item(i).component.name or 'trial' in d.rootComponent.occurrences.item(i).name)
    print('refreshed', n, 'reference; child bodies were', before)
    print('joints still', d.rootComponent.asBuiltJoints.count, 'problems', [d.timeline.item(i).name for i in range(d.timeline.count) if d.timeline.item(i).healthState != adsk.fusion.FeatureHealthStates.HealthyFeatureHealthState])
    asm.save('refreshed')
    print('archive', end=' ')
    try:
        em = d.exportManager
        out = os.path.join(os.environ['TEMP'], 'link_trial')
        ok = em.execute(em.createFusionArchiveExportOptions(out + '.f3z'))
        print(ok, [f for f in os.listdir(os.environ['TEMP']) if f.startswith('link_trial')])
    except Exception as e:          # noqa: BLE001
        print('failed', e)
    lap('save and archive')
    for i in range(app.documents.count - 1, -1, -1):
        app.documents.item(i).close(False)
    print('TIMES', tm)
