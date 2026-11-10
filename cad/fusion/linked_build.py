"""Build a variant as linked documents: one Fusion document per link, in my-projects / duck-droids / <recipe>, and one assembly that references them. A link whose inputs haven't changed isn't rebuilt.

  python -I tools/fusion_call.py cad/fusion/linked_build.py --write --recipe dynamixel

Timed on the all-Dynamixel duck, 2026-11-05: the one-document build takes 11 min. This: 9 min the first time, 31 s with nothing changed, about 2 min for one changed link, joints, clashes and pictures included.
A link's hash covers its spec, its printed parts, its joints and the build code, so a change to build.py rebuilds everything. Each link is verified, and its print STLs exported, when it's built.
A changed link becomes a new version of its document, and the assembly refreshes, so its joints stay. Deleting a link's document would break them.

A Fusion archive of the assembly is only references (89 KB, no geometry), so the repo's .f3d still comes from cad/fusion/full_build.py, the one-document build, which the 30-second reveal uses too. Use this one to iterate, and that one at milestones.
"""
import hashlib
import importlib
import json
import math
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
VIEWS = {'34': (60, 40, 30), 'back': (-60, 35, 30), 'side': (0, 80, 16), 'under': (40, 30, -25), 'top': (20, 10, 90)}
BIG_ONLY = None                                   # the big duck: only these links, for a trial
BIG_SECONDS = 900                                 # the big duck: stop starting links after this, so a turn at Fusion stays short. Built links stay built
BIG_ASSEMBLY = True                               # the big duck: go on to the assembly once every link is built
BIG_CLASH = True
BIG_REV = 'real-1'                                 # the big duck's own build code here isn't in a link's hash: change this to rebuild them all
OK_CLASH = ('flange', 'rear horn', 'rear boss', 'screw', 'inner race', 'cable plugs', ' plug', 'XT30', 'camera ribbon', 'microSD')     # (the battery's plugs rest 0.2 mm into the pack's side: what mounts them, offline, as they hang on its leads)


def wait(fn, seconds=240, what=''):
    """Saves upload in the background: try until it works."""
    t, last = time.time(), None
    while time.time() - t < seconds:
        adsk.doEvents()
        try:
            r = fn()
            if r:
                return r
        except Exception as e:      # noqa: BLE001
            last = e
        time.sleep(0.5)
    raise RuntimeError(f'timed out: {what}: {last}')


def folder(app, name):
    hub = app.data.activeHub
    proj = next(hub.dataProjects.item(i) for i in range(hub.dataProjects.count) if hub.dataProjects.item(i).name == 'my-projects')
    fs = proj.rootFolder.dataFolders
    top = next((fs.item(i) for i in range(fs.count) if fs.item(i).name == 'duck-droids'), None) or fs.add('duck-droids')
    subs = top.dataFolders
    return top, next((subs.item(i) for i in range(subs.count) if subs.item(i).name == name), None) or subs.add(name)


def files(fd):
    return {fd.dataFiles.item(i).name: fd.dataFiles.item(i) for i in range(fd.dataFiles.count)}


def link_hash(s, parts, name, code):
    data = dict(link=s['links'][name], parts={k: v for k, v in parts.items() if v['link'] == name}, joints=[j for j in s['joints'] if name in (j['parent'], j['child'])])
    return hashlib.sha256((json.dumps(data, sort_keys=True) + code).encode()).hexdigest()


def verify_link(name, parts, occ):
    """As build.verify, for this link's printed bodies."""
    bad = []
    c = occ.component
    for pn, p in parts.items():
        if p['link'] != name:
            continue
        b = c.bRepBodies.itemByName(pn)
        if b is None:
            bad.append(f'{pn}: no body')
            continue
        v = b.physicalProperties.volume
        e = abs(v - p['volume_cm3']) / p['volume_cm3']
        if e > 0.04 or not b.isSolid:
            bad.append(f"{pn}: Fusion {v:.2f} cm3, STL {p['volume_cm3']:.2f} ({e * 100:.1f} % off), solid {b.isSolid}")
    return bad


def points_link(name, parts, occ, data):
    IN = adsk.fusion.PointContainment.PointInsidePointContainment
    out = {}
    for pn, v in data.items():
        if parts[pn]['link'] != name:
            continue
        body = occ.component.bRepBodies.itemByName(pn)
        if body is not None:
            out[pn] = [1 if body.pointContainment(figlib.P(q)) == IN else 0 for q in v['pts']]
    return out


def stls_link(name, parts, occ, out):
    os.makedirs(out, exist_ok=True)
    em = build.design().exportManager
    n = 0
    for pn, p in parts.items():
        if p['link'] != name:
            continue
        b = occ.component.bRepBodies.itemByName(pn)
        if b is None:
            continue
        opt = em.createSTLExportOptions(b, os.path.join(out, pn.replace(' ', '_') + '.stl'))
        opt.isBinaryFormat = True
        opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        opt.unitType = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        em.execute(opt)
        n += 1
    return n


def _occ(name):
    """A link rebuilt in its own document can come back as `name 1`: only its bodies' names matter here."""
    occs = build.occurrences()
    return occs.get(name) or next(o for n, o in occs.items() if n.startswith(name + ' '))


def big_pictures(app, name):
    """Each left and middle link on its own, from where its parts show. The right legs are the left's mirror, by the spec's code."""
    if name.startswith('R_'):
        return
    folder_ = os.path.join(build.REPO, 'docs', 'images', 'fusion', 'big', 'links')
    os.makedirs(folder_, exist_ok=True)
    vp = app.activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedVisualStyle
    for k, eye in {'a': (60, 50, 30), 'b': (-60, -50, 30), 'c': (50, -40, -35)}.items():
        cam = vp.camera
        cam.isSmoothTransition = False
        cam.cameraType = adsk.core.CameraTypes.PerspectiveCameraType
        cam.target = adsk.core.Point3D.create(0, 0, 0)
        cam.eye = adsk.core.Point3D.create(*eye)
        cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
        vp.camera = cam
        vp.fit()
        vp.refresh()
        vp.saveAsImageFile(os.path.join(folder_, f'{name}_{k}.png'), 1000, 1000)


def big_spec():
    """The big duck's rods, chain and thigh covers are 'decal' to keep them out of the offline sweep checks, and build.link skips decals with no screw. Here they're real: mark them, in this script only, so build.py and its hash stay as they are for every other duck."""
    plain = build.spec
    cache = {}

    def spec():
        if cache:                                 # (read once: 3000 geoms, and build.link asks per link)
            return json.loads(cache['s'])
        s = plain()
        # the knee's cranks and rods: drawn in the thigh's link at knee 0, each its own moving part here (spec['mech'])
        for ln in list(s['links']):
            l = s['links'][ln]
            moved = [g for g in l['geoms'] if g.get('group')]
            l['geoms'] = [g for g in l['geoms'] if not g.get('group')]
            for g in moved:
                s['links'].setdefault(g['group'], dict(name=g['group'], budget=0.0, geoms=[], note='a moving part of the knee four-bar'))['geoms'].append(g)
        for j in s.get('mech', dict(joints=()))['joints']:
            s['joints'].append(dict(j, act='', act_c=j['origin'], out=1, long='-x', remote=True))
        for l in s['links'].values():
            for g in l['geoms']:
                if g['role'] == 'decal' and 'fast' not in g:
                    g['fast'] = dict(fusion_only=True)
        cache['s'] = json.dumps(s)
        return json.loads(cache['s'])
    build.spec = spec
    # what stands in for RobStride's own shapes in the spec isn't built here: big_real places the real thing
    build.real_skip = lambda s_: {g['name'] for l in s_['links'].values() for g in l['geoms'] if g.get('stand_in')}


# RobStride's own models: local only, %TEMP%/figrefs/robstride (tools/fetch_servo_cad.py). Each is one solid with its rotor in it, so the rotor's cut off here (hub r and z, cm: file frame) and goes to the link it drives. See spec/big_duck.py's RS for the measurements.
RS_FILES = {'RS04': dict(path='robstride/RS04-20261030.stp', hub=2.55, zout=1, cable=75.0, cutter=(2.8, 2.45, 3.0), keep_all=False),
            'RS03': dict(path='robstride/RS03-20261030.stp', hub=0.0, zout=-1, cable=0.0, cutter=(3.52, -0.1, 0.22), keep_all=False),
            'RS00': dict(path='robstride/RS00-20261030.stp', hub=-0.6, zout=-1, cable=0.0, cutter=None, keep_all=True)}
_RS_TEMP = {}


def _rs_bodies(model):
    """{'case': [temp bodies], 'rotor': [...]}, in the file's frame, read once a run."""
    if model in _RS_TEMP:
        return _RS_TEMP[model]
    R = RS_FILES[model]
    app = adsk.core.Application.get()
    im = app.importManager
    t0 = time.time()
    src = im.importToTarget2(im.createSTEPImportOptions(os.path.join(build.FIGREFS, R['path'])), build.design().rootComponent).item(0)
    comp = src.component
    bs = [comp.bRepBodies.item(i) for i in range(comp.bRepBodies.count)]
    for i in range(comp.allOccurrences.count):
        o = comp.allOccurrences.item(i)
        bs += [o.bRepBodies.item(k) for k in range(o.bRepBodies.count)]
    bs.sort(key=lambda b: -b.volume)
    tbm = adsk.fusion.TemporaryBRepManager.get()
    res = dict(case=[tbm.copy(b) for b in (bs if R['keep_all'] else bs[:1])], rotor=[])     # (the RS04's second solid, 2 cm3, lies wholly inside the case)
    src.deleteMe()
    if R['cutter']:
        r, z0, z1 = R['cutter']
        whole = res['case'][0]
        rotor = tbm.copy(whole)
        cy = lambda: tbm.createCylinderOrCone(adsk.core.Point3D.create(0, 0, z0), r, adsk.core.Point3D.create(0, 0, z1), r)
        tbm.booleanOperation(whole, cy(), adsk.fusion.BooleanTypes.DifferenceBooleanType)
        tbm.booleanOperation(rotor, cy(), adsk.fusion.BooleanTypes.IntersectionBooleanType)
        res['rotor'] = [rotor]
    print(model, 'read and split', round(time.time() - t0, 1), 's, case', round(res['case'][0].volume, 1), 'cm3, rotor', round(res['rotor'][0].volume, 2) if res['rotor'] else None)
    _RS_TEMP[model] = res
    return res


def big_real(name, s, comp):
    """RobStride's model at each actuator of this link: the case, or the rotor. Rotation only, never a mirror: the right side's is the left's turned 180 deg about its cable. Returns how many."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    d = build.design()
    V = lambda v: adsk.core.Vector3D.create(*[float(x) for x in v])
    cr = lambda a, b: (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    n = 0
    for g in s['links'][name]['geoms']:
        for key, piece, col, fin in (('real', 'case', '#17181A', 'matte'), ('rotor', 'rotor', '#A8ACB0', 'metal')):
            r = g.get(key)
            if not r or not _rs_bodies(r['model'])[piece]:
                continue
            R = RS_FILES[r['model']]
            a = math.radians(R['cable'])
            f_g, f_o = (math.cos(a), math.sin(a), 0.0), (0.0, 0.0, float(R['zout']))
            M = adsk.core.Matrix3D.create()
            M.setToAlignCoordinateSystems(adsk.core.Point3D.create(0, 0, R['hub']), V(f_g), V(cr(f_o, f_g)), V(f_o),
                                          adsk.core.Point3D.create(*[v * 100 for v in r['hub']]), V(r['g']), V(cr(r['out'], r['g'])), V(r['out']))
            c = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
            label = g['name'].replace(' (buy)', '')
            c.name = f'{label} {piece}'
            for k, tb in enumerate(_rs_bodies(r['model'])[piece]):
                t = tbm.copy(tb)
                tbm.transform(t, M)
                b = c.bRepBodies.add(t)
                b.name = f"{label} real {piece}" + (f' {k}' if k else '')
                b.appearance = build.F.appearance(d, col, fin)
            n += 1
    return n


def big_hollow(name, s, occ, wall=0.003):
    """build.link makes a lofted panel solid, and its offline wall comes from its printed part, which the big duck hasn't got. One inner loft for each set of panels, cut from them all: (panels, failures, seconds)."""
    t, comp = time.time(), occ.component
    b = figlib.Builder(comp)
    groups = {}
    for g in s['links'][name]['geoms']:
        if g['role'] == 'shell' and g['kind'] == 'loft' and comp.bRepBodies.itemByName(g['name']):
            groups.setdefault(json.dumps(g['sections'], sort_keys=True), []).append(g)
    done = fail = 0
    for gs in groups.values():
        g0 = gs[0]
        secs = [dict(x, a=x['a'] - wall, b=x['b'] - wall) for x in g0['sections']]
        op = g0.get('open') or ()
        secs[0]['z'] += -0.001 if 'bottom' in op else wall
        secs[-1]['z'] += 0.001 if 'top' in op else -wall
        tool = b.loft(dict(name=f"{g0['name']} inner", sections=secs, along=g0.get('along')))
        for g in gs:
            ci = b.f.combineFeatures.createInput(comp.bRepBodies.itemByName(g['name']), figlib.coll([tool]))
            ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
            ci.isKeepToolBodies = True
            try:
                b.f.combineFeatures.add(ci)
                done += 1
            except RuntimeError:
                fail += 1
        tool.deleteMe()
    return done, fail, round(time.time() - t, 1)


def close_all(app):
    """Closing an assembly can close its linked documents too, and a link can refuse to close while its assembly is open. So assemblies first, and this always ends.
    (2026-11-05: `while count: item(last).close()` spun forever on a link that refused, on Fusion's main thread. It froze Fusion for 5 hours, and looked like a big model's fault)"""
    for _ in range(300):
        n = app.documents.count
        if not n:
            return
        def refs(x):
            try:
                return x.documentReferences.count
            except RuntimeError:                  # a link opened by its assembly: Fusion won't say
                return -1
        for doc in sorted((app.documents.item(i) for i in range(n)), key=lambda x: -refs(x)):
            try:
                doc.close(False)
            except RuntimeError:
                pass
            adsk.doEvents()
            if app.documents.count < n:
                break
        else:
            print('close_all: these would not close:', [app.documents.item(i).name for i in range(n)])
            return


def make_assembly(app, s, have, fd, aname):
    """The first time only: after this, changed links arrive as new versions, and the joints stay."""
    t0 = time.time()
    asm = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    d = adsk.fusion.Design.cast(app.activeProduct)
    d.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = d.rootComponent
    occ = {}
    for name in s['links']:
        f = have[f'{RECIPE} {name}']
        occ[name] = wait(lambda ff=f: root.occurrences.addByInsert(ff, adsk.core.Matrix3D.create(), True), seconds=600, what=f'insert {name}')      # (a link just saved can refuse for over 240 s while the hub takes it, then insert fine: seen on 2 ducks on 2026-11-05)
    print('inserted', len(occ), round(time.time() - t0), 's')
    occ[s['root']].isGrounded = True
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'joint points'
    sk.isVisible = False
    dirs = {'x': adsk.fusion.JointDirections.XAxisJointDirection, 'y': adsk.fusion.JointDirections.YAxisJointDirection, 'z': adsk.fusion.JointDirections.ZAxisJointDirection}
    for j in s['joints']:
        sp = sk.sketchPoints.add(sk.modelToSketchSpace(figlib.P(j['origin'])))
        ji = root.asBuiltJoints.createInput(occ[j['child']], occ[j['parent']], adsk.fusion.JointGeometry.createByPoint(sp))
        ji.setAsRevoluteJointMotion(dirs[j['axis']])
        jt = root.asBuiltJoints.add(ji)
        jt.name = j['name']
        m = adsk.fusion.RevoluteJointMotion.cast(jt.jointMotion)
        m.rotationLimits.isMinimumValueEnabled = True
        m.rotationLimits.minimumValue = j['lower']
        m.rotationLimits.isMaximumValueEnabled = True
        m.rotationLimits.maximumValue = j['upper']
    print('saved', asm.saveAs(aname, fd, 'Linked: generated from spec/robot.py, github.com/meetsitaram/duck-droids', ''))
    wait(lambda: asm.dataFile, what='assembly file')
    return asm, d


def check_assembly(app, s, d):
    root = d.rootComponent
    refused = []
    for j in s['joints']:
        jt = root.asBuiltJoints.itemByName(j['name'])
        if jt is None:
            refused.append(j['name'] + ': gone')
            continue
        m = adsk.fusion.RevoluteJointMotion.cast(jt.jointMotion)
        for lim in (j['lower'], j['upper']):
            m.rotationValue = lim * 0.98
            if abs(m.rotationValue - lim * 0.98) > 0.02:
                refused.append(j['name'])
        m.rotationValue = 0.0
    print('joints', root.asBuiltJoints.count, 'refused', refused)
    if RECIPE == 'big' and not BIG_CLASH:
        print('clashes: not run')
        return
    occs = adsk.core.ObjectCollection.create()
    for o in root.occurrences:
        occs.add(o)
    ii = d.createInterferenceInput(occs)
    ii.areCoincidentFacesIncluded = False
    res = d.analyzeInterference(ii)
    clash, boxes = [], []
    for k in range(res.count if res else 0):
        r = res.item(k)
        bb_ = r.interferenceBody.boundingBox              # where it is, in mm: a clash can be measured, not only named
        boxes.append((getattr(r.entityOne, 'name', '?'), getattr(r.entityTwo, 'name', '?'), round(r.interferenceBody.volume * 1000, 2), [round(v * 10, 2) for v in (bb_.minPoint.x, bb_.minPoint.y, bb_.minPoint.z, bb_.maxPoint.x, bb_.maxPoint.y, bb_.maxPoint.z)]))
        clash.append((getattr(r.entityOne, 'name', '?'), getattr(r.entityTwo, 'name', '?'), round(r.interferenceBody.volume * 1000, 2)))
    clash.sort(key=lambda c: -c[2])
    same = lambda a, b: any(m in a and m in b and a.split(m)[0] == b.split(m)[0] for m in (' XL330 ', ' STS3215 ', ' HD1910 '))
    json.dump(clash, open(os.path.join(build.REPO, 'reports', build._variant_file('fusion_interference')), 'w', encoding='utf-8'), indent=0)
    json.dump(sorted(boxes, key=lambda c: -c[2]), open(os.path.join(build.REPO, 'reports', build._variant_file('fusion_clash_boxes')), 'w', encoding='utf-8'), indent=0)
    print('clashes', len(clash), 'to explain', [c for c in clash if not any(k in c[0] + c[1] for k in OK_CLASH) and not same(c[0], c[1]) and c[2] > 1.0][:12])
    vp = app.activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedVisualStyle
    root.isJointsFolderLightBulbOn = False
    root.isSketchFolderLightBulbOn = False
    pics = os.path.join(build.REPO, 'docs', 'images', 'fusion', RECIPE)
    os.makedirs(pics, exist_ok=True)
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
        vp.saveAsImageFile(os.path.join(pics, f'{k}.png'), 1400, 1400)


def run(_context: str):
    t00 = time.time()
    app = adsk.core.Application.get()
    close_all(app)
    build.SPEC = 'robot.json' if RECIPE == 'hybrid' else f'robot_{RECIPE}.json'
    if RECIPE == 'big':
        big_spec()
    s = build.spec()
    parts = build.parts_spec()['parts']
    code = ''.join(open(os.path.join(P, f), encoding='utf-8').read() for f in ('build.py', 'figlib.py')) + (BIG_REV if RECIPE == 'big' else '')
    mpath = os.path.join(build.REPO, 'reports', build._variant_file('fusion_links'))
    man = json.load(open(mpath, encoding='utf-8')) if os.path.exists(mpath) else {}
    pts_path = os.path.join(build.REPO, 'reports', build._variant_file('fusion_points'))
    points = json.load(open(pts_path, encoding='utf-8')) if os.path.exists(pts_path) else {}
    vdata = json.load(open(os.path.join(build.REPO, 'spec', build._variant_file('verify_points')), encoding='utf-8'))
    stl = os.path.join(os.environ['TEMP'], 'duck_fusion_stl', RECIPE)
    top, fd = folder(app, RECIPE)
    aname = f'{RECIPE} assembly real' if RECIPE == 'big' else f'{RECIPE} assembly'     # (the big duck's first assembly, with no cranks or rods as links, stays in the hub unused: joints can't be added to it by refreshing, and deleting a file is the user's)
    have = files(fd)
    rebuilt, kept, bad = [], [], []
    t0 = time.time()
    for name in s['links']:
        if RECIPE == 'big' and ((BIG_ONLY and name not in BIG_ONLY) or time.time() - t00 > BIG_SECONDS):
            continue
        h = link_hash(s, parts, name, code)
        doc_name = f'{RECIPE} {name}'
        prints_ok = all(os.path.exists(os.path.join(stl, pn.replace(' ', '_') + '.stl')) for pn, p in parts.items() if p['link'] == name)
        if man.get(name, {}).get('hash') == h and doc_name in have and prints_ok:
            kept.append(name)
            bad += man[name].get('bad', [])
            continue
        t = time.time()
        if doc_name in have:
            doc = app.documents.open(have[doc_name])          # a new version, so the assembly's reference and joints stay
        else:
            build.new_document()
            doc = app.activeDocument
        print(build.link(name))
        occ = _occ(name)
        if RECIPE == 'big':
            r_ = big_hollow(name, s, occ)
            print(name, 'hollowed, failed, s', r_, 'real', big_real(name, s, occ.component))
            if '_crank_' not in name and '_rod_' not in name:
                big_pictures(app, name)
        vb = verify_link(name, parts, occ)
        points.update(points_link(name, parts, occ, vdata))
        stls_link(name, parts, occ, stl)
        if doc_name in have:
            doc.save(f'{name}: rebuilt from spec/robot.py')
        else:
            doc.saveAs(doc_name, fd, f'{name}: generated from spec/robot.py', '')
            wait(lambda d_=doc: d_.dataFile, what=f'{name} file')
        doc.close(False)
        bad += vb
        man[name] = dict(hash=h, bad=vb, seconds=round(time.time() - t, 1))
        rebuilt.append(name)
        if RECIPE == 'big':                       # so a turn that runs out loses nothing
            json.dump(man, open(mpath, 'w', encoding='utf-8'), indent=1)
    json.dump(man, open(mpath, 'w', encoding='utf-8'), indent=1)
    json.dump(points, open(pts_path, 'w', encoding='utf-8'))
    print(f'links: {len(rebuilt)} rebuilt {rebuilt}, {len(kept)} unchanged, {round(time.time() - t0)} s')
    print('verify', len(parts), 'bad', json.dumps(bad))
    t0 = time.time()
    # (2026-11-05: a run that failed after saving its links still left them in the manifest. The next run said "nothing changed" over a stale clash report. So links stay owed until the assembly has taken them)
    owed_path = os.path.join(os.environ['TEMP'], f'duck_fusion_owed_{RECIPE}.json')
    owed = sorted(set(json.load(open(owed_path)) if os.path.exists(owed_path) else []) | set(rebuilt))
    json.dump(owed, open(owed_path, 'w'))
    have = wait(lambda: files(fd), what='the hub file list')      # just after a save it can raise status.isOk() && files
    if RECIPE == 'big' and (not BIG_ASSEMBLY or any(f'{RECIPE} {n}' not in have or n not in man for n in s['links'])):
        print('not yet: links left', [n for n in s['links'] if f'{RECIPE} {n}' not in have or n not in man], 'seconds so far', round(time.time() - t00))
        close_all(app)
        return
    if RECIPE == 'big' and aname in have:         # its links are built in earlier turns, so the assembly refreshes them all here
        rebuilt = rebuilt or list(s['links'])
    if aname not in have:
        asm, d = make_assembly(app, s, have, fd, aname)
        check_assembly(app, s, d)
        asm.save('joints checked')
    elif owed:
        asm = app.documents.open(have[aname])
        d = adsk.fusion.Design.cast(app.activeProduct)
        n = set()

        def latest():
            adsk.doEvents()
            for k in range(asm.documentReferences.count):
                r = asm.documentReferences.item(k)
                if r.isOutOfDate:
                    n.add(r.dataFile.name)
                    r.getLatestVersion()
            if RECIPE == 'big':                   # its links come from earlier turns too, and a reference shows as out of date only after a while: wait for them all (one count stopped at 9 of 16, and left the old thigh in)
                return all(asm.documentReferences.item(k).dataFile.versionNumber == asm.documentReferences.item(k).dataFile.latestVersionNumber for k in range(asm.documentReferences.count))
            return len(n) >= len(rebuilt)
        wait(latest, seconds=180, what=f'references, {sorted(n)} of {rebuilt}')
        print('refreshed', sorted(n), round(time.time() - t0), 's')
        check_assembly(app, s, d)
        asm.save(f'refreshed: {len(owed)} links')
    else:
        print('nothing changed: the assembly stands')
    os.remove(owed_path)
    close_all(app)
    print('assembly', round(time.time() - t0), 's, total', round(time.time() - t00), 's; open', app.documents.count)
