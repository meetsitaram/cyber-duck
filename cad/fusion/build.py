"""Build milli in Fusion from spec/robot.json and spec/parts.json (the scripts are the source of truth: never hand-edit
the document; change spec/robot.py, rerun the checks and tools/export_prints.py, rebuild).

One short job per MCP call (long calls time out and freeze Fusion; CDX-3R lessons 78-79):
  build.new_document()          a new Z-up design document with no timeline: a full model in about 80 features' time, not 1,800
  build.link('trunk')           build one link as its own component, at the zero pose: one body per printed part, with its holes
                                and pockets, ready to print; each bought part its own body; every screw
  build.parametric()            after the last link: turn on the timeline, which joints need
  build.joints()                as-built revolute joints at the spec origins and limits
  build.verify()                every printed body against the volume tools/export_prints.py exported
  build.status()                components, body counts, unhealthy features, duplicates
  build.pose({'L_elbow': -1})   set joint angles (rad); missing joints go to 0
  build.show(skins=False)       hide the skins to see the internals (and back)
  build.export_stl(folder)      one STL per link at the zero pose, for the URDF and the mesh checks

Frame: Fusion world = spec world (Z up, X forward, Y left); the API works in cm, the spec in m (figlib converts).
"""
import json
import os
import time

import adsk.core
import adsk.fusion

import figlib as F

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DOC = 'duck-droids'
GLOSS, ALU, LENS = '#08090B', '#9EA3A8', '#0A1420'
METAL = (ALU, '#A8ACB0', '#C9CDD2')                      # bearings and horns, the spec's aluminium, its steel screws


def app():
    return adsk.core.Application.get()


def design():
    return adsk.fusion.Design.cast(app().activeProduct)


SPEC = 'robot.json'


def use(name):
    """Which spec to build: build.use('robot_big.json'), once after importing this."""
    global SPEC
    SPEC = name
    return json.load(open(os.path.join(REPO, 'spec', SPEC), encoding='utf-8'))['root']


def spec():
    return json.load(open(os.path.join(REPO, 'spec', SPEC), encoding='utf-8'))


def finish(g):
    return 'gloss' if g['color'] in (GLOSS, LENS) else 'metal' if g['color'] in METAL else 'matte'


# ---------------------------------------------------------------- the real servos
# The makers' own CAD, not boxes (2026-11-04, the user: "why can't we have these servo cad models in our duck model?"). It's Fusion's only: the offline checks keep their simple
# shapes, and Fusion's interference check then tests them against the printed parts. Fetched by tools/fetch_servo_cad.py, kept out of the repo (see THIRD_PARTY_NOTICES.md).
# Each file's own frame, in mm: o, the case's middle at the shaft's axis level; ux, toward the far end; az, toward the horn. Colours from ROBOTIS's product photos.
FIGREFS = os.path.join(os.environ.get('TEMP', ''), 'figrefs')
BLACK_CASE, BLACK_HORN, BLACK_SCREW, CREAM, WINDOW, LABEL = '#17181A', '#141517', '#0D0E10', '#EFEBDD', '#BDC1C2', '#8E9194'
XL_FILE, XL_O = 'koch-v1-1/Robotis_DYNAMIXEL_XL330_M288_T.STEP', (11.65, 221.06, 33.39)
HD_FILE = 'feetech-hd1910/HD-1910-C001-20260902.stp'
REAL = {
    'XL330': dict(
        name='XL330', o=XL_O, ux=(1, 0, 0), az=(0, 0, -1),
        case=(XL_FILE, {'Body12': ('case middle', BLACK_CASE, 'matte'), 'Body11': ('case front', BLACK_CASE, 'matte'), 'Body10': ('case rear', BLACK_CASE, 'matte'),
                        'Body4': ('case screw 0', BLACK_SCREW, 'gloss'), 'Body5': ('case screw 1', BLACK_SCREW, 'gloss'), 'Body6': ('case screw 2', BLACK_SCREW, 'gloss'), 'Body7': ('case screw 3', BLACK_SCREW, 'gloss'),
                        'Body8': ('LED window', WINDOW, 'gloss'), 'Body1': ('socket 0', CREAM, 'matte'), 'Body2': ('socket 1', CREAM, 'matte')}),
        horn=(XL_FILE, {'Body9': ('horn', BLACK_HORN, 'matte'), 'Body3': ('centre screw', BLACK_SCREW, 'gloss')}),
        idler=('koch-v1-1/Robotis_FPX330_H101.STEP', {'Body3': ('idler', BLACK_HORN, 'matte'), 'Body2': ('idler bush', BLACK_SCREW, 'gloss'), 'Body1': ('idler screw', BLACK_SCREW, 'gloss')}),
        label=(17.45, 23.85, 214.46, 227.66, 21.84, 21.90)),      # x, y, z: where ROBOTIS's label is on the case's face (no text: a plain plate)
    'STS3215': dict(
        name='STS3215', o=(0, 0, -0.35), ux=(-1, 0, 0), az=(0, 0, 1), shaft=(12.5, 0), horn_cut=(14.45, 21.0, 10.3),
        case=('SO-ARM100/STEP/SO100/STS3215_03a.step', {'Body1': ('case', BLACK_CASE, 'matte')}),
        horn=('SO-ARM100/STEP/SO100/STS3215_03a.step', {'Body1': ('horn', ALU, 'metal')}),
        idler=('SO-ARM100/STEP/SO100/Passive_Horn_01.step', {'Body1': ('rear horn', ALU, 'metal')}), idler_frame=((12.5, 0, -14.9), (1, 0, 0), (0, 0, -1))),      # (the STEP's axis is at its own origin, and _servo_matrix's origin is the case's middle: the shaft is 12.5 mm along. The disc stood 12.5 mm off the shaft, in Fusion only. Found by a clash with an ankle's plugs)
    # Feetech's own file, an assembly: its solids sit in nested components with repeated names, so they're told apart by where they are (HD_PIECES). In the file, mm: shaft on z at x = y = 0, the far end toward -y, horn up.
    # The rear tube the rear horn turns on is part of its bottom case, and stays where the horn comes off. Its output gear is inside the case, and left out
    'HD1910': dict(
        name='HD1910', o=(0, -7.5, -8.0), ux=(0, -1, 0), az=(0, 0, 1), by_place=True,
        case=(HD_FILE, {'case middle': ('case middle', BLACK_CASE, 'matte'), 'case top': ('case top', BLACK_CASE, 'matte'), 'case bottom': ('case bottom', BLACK_CASE, 'matte'),
                        'case screw 0': ('case screw 0', BLACK_SCREW, 'gloss'), 'case screw 1': ('case screw 1', BLACK_SCREW, 'gloss'), 'case screw 2': ('case screw 2', BLACK_SCREW, 'gloss'), 'case screw 3': ('case screw 3', BLACK_SCREW, 'gloss'),
                        'socket 0': ('socket 0', CREAM, 'matte'), 'socket 1': ('socket 1', CREAM, 'matte')}),
        horn=(HD_FILE, {'horn': ('horn', BLACK_HORN, 'matte'), 'centre screw': ('centre screw', BLACK_SCREW, 'gloss')}),
        idler=(HD_FILE, {'idler': ('idler', BLACK_HORN, 'matte'), 'idler screw': ('idler screw', BLACK_SCREW, 'gloss')})),
}


def hd_pieces(comp):
    """The HD-1910 file's solids by name, from where each one lies (cm in Fusion). See REAL['HD1910']."""
    found, screws, sockets = {}, [], []
    occs = [comp.allOccurrences.item(i) for i in range(comp.allOccurrences.count)]
    for b in [comp.bRepBodies.item(i) for i in range(comp.bRepBodies.count)] + [o.bRepBodies.item(i) for o in occs for i in range(o.bRepBodies.count)]:
        lo, hi = b.boundingBox.minPoint, b.boundingBox.maxPoint
        dx, dz, zc = (hi.x - lo.x) * 10, (hi.z - lo.z) * 10, (hi.z + lo.z) * 5
        if dz > 20:
            screws.append((round(lo.x, 3), round(lo.y, 3), b))
        elif dx < 4:
            sockets.append((round(lo.x, 3), b))
        elif dx > 19:
            found['case top' if zc > 0 else 'case bottom' if dz < 10 else 'case middle'] = b
        elif dx < 6:
            found['centre screw' if zc > -8 else 'idler screw'] = b
        elif dz > 12:
            found['gear'] = b
        else:
            found['horn' if zc > -8 else 'idler'] = b
    for k, s in enumerate(sorted(screws, key=lambda t: t[:2])):
        found[f'case screw {k}'] = s[2]
    for k, s in enumerate(sorted(sockets, key=lambda t: t[0])):
        found[f'socket {k}'] = s[1]
    return found


SOFT_R = 0.1            # cm: the finished part's edges. Its pockets and holes are cut afterwards, so whatever a servo or a bearing fits against stays crisp (the user)
SOFT_SECONDS = 12.0     # for one part: what can't be rounded in that time stays sharp, and is counted


SOFT_SKIP = ('body shell', 'hood', ' lid', 'sheet', 'spacer', 'inner ear', 'crank', 'rod', 'eye ', 'sole', 'soft', 'mouth roof', 'face')   # small parts, and what a bearing or a servo fits in


SEAT_WORDS = ('seat boss', 'boss shoulder', 'ankle stub', 'ear hub', 'rear hub boss', 'rear hub shoulder')   # what a bearing's ring presses onto or sits against


def seats_of(link_geoms, names):
    """(centre, axis, radius, half height), in cm, of every bearing seat among these geoms: soften() rounds nothing there."""
    out = []
    for g in link_geoms:
        if g['name'] in names and g.get('kind') == 'cyl' and any(w in g['name'] for w in SEAT_WORDS):
            a = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}[g['axis']]
            out.append(([v * 100 for v in g['c']], a, g['r'] * 100, g['h'] * 50))
    return out


def soften(body, comp, r=SOFT_R, seats=()):
    """Round a joined part's outside edges, and its straight inside corners (where a print cracks), but not a round inside corner: that's a boss on a plate, and a bearing's race sits there.
    seats: (centre, axis, radius, half height) of bearing seats; no edge on a seat or its shoulder is rounded (a rounded seat let the ring rock: 2026-11-10, the user).
    Fusion refuses a set if any one edge fails, so it halves the set until the rest go through. A result that lost over 3 % is thrown away: on a thin part a fillet can eat the part
    (an inner ear went from 2.4 cm3 to nothing, and the build called it a success). Returns (body, edges rounded, left sharp)."""
    t0, skip, chunk = time.time(), set(), 10 ** 6
    done = 0
    tbm = adsk.fusion.TemporaryBRepManager.get()
    keep, v0, name, look = tbm.copy(body), body.volume, body.name, body.appearance
    IN = adsk.fusion.PointContainment.PointInsidePointContainment

    def key(e):
        p = e.pointOnEdge
        return (round(p.x, 3), round(p.y, 3), round(p.z, 3))

    def on_seat(p):
        for c, a, rr, hh in seats:
            d = (p.x - c[0], p.y - c[1], p.z - c[2])
            al = d[0] * a[0] + d[1] * a[1] + d[2] * a[2]
            rad = sum((d[i] - al * a[i]) ** 2 for i in range(3)) ** 0.5
            if abs(al) <= hh + 0.06 and rad <= rr + 0.06:
                return True
        return False

    def wanted(e):
        p = e.pointOnEdge
        if on_seat(p):
            return False
        ok0, n0 = e.faces.item(0).evaluator.getNormalAtPoint(p)
        ok1, n1 = e.faces.item(1).evaluator.getNormalAtPoint(p)
        if abs(n0.dotProduct(n1)) >= 0.95:             # two faces that already run smoothly into each other
            return False
        q = adsk.core.Point3D.create(p.x + (n0.x + n1.x) * 0.02, p.y + (n0.y + n1.y) * 0.02, p.z + (n0.z + n1.z) * 0.02)
        convex = body.pointContainment(q) != IN
        return convex or e.geometry.curveType == adsk.core.Curve3DTypes.Line3DCurveType

    def corners():
        return [e for e in (body.edges.item(i) for i in range(body.edges.count)) if e.length >= 0.2 and e.faces.count == 2 and key(e) not in skip and wanted(e)]

    while time.time() - t0 < SOFT_SECONDS:
        es = corners()
        if not es:
            break
        batch = es[:chunk]
        first = key(batch[0])                          # (a failed fillet can leave these edges unreadable)
        fi = comp.features.filletFeatures.createInput()
        fi.edgeSetInputs.addConstantRadiusEdgeSet(F.coll(batch), adsk.core.ValueInput.createByReal(r), True)
        n0 = body.faces.count
        try:
            comp.features.filletFeatures.add(fi)
            if body.isValid and body.faces.count > n0:
                done += len(batch)
                continue
        except RuntimeError:
            pass
        if len(batch) == 1:
            skip.add(first)
            chunk = 16
        else:
            chunk = max(1, len(batch) // 2)
    if not body.isValid or body.volume < v0 * 0.97:
        if body.isValid:
            body.deleteMe()
        body = comp.bRepBodies.add(keep)
        body.name = name
        body.appearance = look
        return body, 0, -1
    return body, done, len(skip)


COVER_R = (0.7, 0.5)    # cm: a big visible shell's outer rim and the inside of it, a 2 mm wall all round the curve. 1 mm doesn't show, and the user wanted it curved. 7 is what clears the thigh plate's corner, 6 mm under the face


def _rim(body, comp, side, r):
    """Fillet the edges in a shell's outermost plane (y, side +1 or -1)."""
    bb = body.boundingBox
    y = bb.maxPoint.y if side > 0 else bb.minPoint.y
    es = [e for e in (body.edges.item(i) for i in range(body.edges.count)) if abs(e.boundingBox.maxPoint.y - y) < 1e-4 and abs(e.boundingBox.minPoint.y - y) < 1e-4]
    if not es:
        return 0
    fi = comp.features.filletFeatures.createInput()
    fi.edgeSetInputs.addConstantRadiusEdgeSet(F.coll(es), adsk.core.ValueInput.createByReal(r), True)
    comp.features.filletFeatures.add(fi)
    return len(es)


def edge_count(body):
    """(sharp, smooth): edges of 3 mm or longer. What a look check reads: a part with no smooth edges has no fillets."""
    sharp = smooth = 0
    for i in range(body.edges.count):
        e = body.edges.item(i)
        if e.length < 0.3 or e.faces.count != 2:
            continue
        ok0, n0 = e.faces.item(0).evaluator.getNormalAtPoint(e.pointOnEdge)
        ok1, n1 = e.faces.item(1).evaluator.getNormalAtPoint(e.pointOnEdge)
        if abs(n0.dotProduct(n1)) < 0.9:
            sharp += 1
        else:
            smooth += 1
    return sharp, smooth


def look_check():
    """SOFT: is every part you see rounded? It failed for 2 builds with nothing to say so: reports/<variant>/fusion_look. Fails a part other than SOFT_SKIP with no smooth edges at all."""
    occ = occurrences()
    rows, bad, steps = {}, [], {}
    for pn, p in parts_spec()['parts'].items():
        b = occ[p['link']].component.bRepBodies.itemByName(pn)
        if b is None:
            continue
        sh, sm = edge_count(b)
        rows[pn] = dict(sharp=sh, smooth=sm)
        # a tiny flat face is what two shapes that don't quite meet leave: a step, a notch (the user, 2026-11-04). It flags where to look; a picture decides
        tiny = 0
        for i in range(b.faces.count):
            f = b.faces.item(i)
            try:
                if f.area < 0.03 and f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                    tiny += 1
            except RuntimeError:                         # Fusion can't always measure a sliver: that's worth flagging too
                tiny += 1
        if tiny:
            steps[pn] = tiny
        if sm == 0 and not any(k in pn for k in SOFT_SKIP):
            bad.append(pn)
    return dict(sharp=sum(r['sharp'] for r in rows.values()), smooth=sum(r['smooth'] for r in rows.values()), not_rounded=bad, tiny_faces=dict(sorted(steps.items(), key=lambda kv: -kv[1])[:14]))


def real_model(act):
    return 'XL330' if act.startswith(('XL330', 'XC330')) else 'STS3215' if act.startswith('STS3215') else 'HD1910' if act.startswith('HD1910') else None


def real_skip(s):
    """The spec's stand-ins for a real servo: not built where there's the maker's model."""
    out = {'Pi Zero 2 W', 'Waveshare bus board', 'leg bus board'}
    for j in s['joints']:
        if not real_model(j['act']):
            continue
        n = j['name'].replace('_', ' ')
        out |= {f"{j['name']} actuator (buy) {j['act']}", f"{j['name']} flange", f'{n} rear horn'}
        if 'ankle' in n:
            out.add(f'{n[0]} ankle rear boss')
    return out


def _bodies(c, out=None):
    out = {} if out is None else out
    for i in range(c.bRepBodies.count):
        out.setdefault(c.bRepBodies.item(i).name, c.bRepBodies.item(i))
    return out


_TEMP = {}


def _temp_bodies(model, piece):
    """One piece's solids from its STEP file, as temporary bodies in the file's frame: [(body, name, colour, finish)]. Read once."""
    key = (model, piece)
    if key in _TEMP:
        return _TEMP[key]
    R = REAL[model]
    path, table = R[piece]
    path = os.path.join(FIGREFS, path)
    if not os.path.exists(path):
        raise RuntimeError(f'{path} is missing: python -I tools/fetch_servo_cad.py')
    im = app().importManager
    src = im.importToTarget2(im.createSTEPImportOptions(path), design().rootComponent).item(0)
    have = hd_pieces(src.component) if R.get('by_place') else _bodies(src.component)
    tbm = adsk.fusion.TemporaryBRepManager.get()
    out = []
    for key_, (nm, col, fin) in table.items():
        tb = tbm.copy(have[key_])
        if model == 'STS3215' and piece in ('case', 'horn'):     # one solid in the file, and the horn turns with the other link
            z0, z1, r = R['horn_cut']
            sx, sy = R['shaft']
            cy = tbm.createCylinderOrCone(adsk.core.Point3D.create(sx / 10, sy / 10, z0 / 10), r / 10, adsk.core.Point3D.create(sx / 10, sy / 10, z1 / 10), r / 10)
            tbm.booleanOperation(tb, cy, adsk.fusion.BooleanTypes.DifferenceBooleanType if piece == 'case' else adsk.fusion.BooleanTypes.IntersectionBooleanType)
            if piece == 'case':                                  # the file's servo has a passive horn and its screw on the rear too: only the ankles here have one, added as a piece of its own
                cy = tbm.createCylinderOrCone(adsk.core.Point3D.create(sx / 10, sy / 10, -2.1), r / 10, adsk.core.Point3D.create(sx / 10, sy / 10, -1.445), r / 10)
                tbm.booleanOperation(tb, cy, adsk.fusion.BooleanTypes.DifferenceBooleanType)
        out.append((tb, nm, col, fin))
    if piece == 'case' and 'label' in R:
        x0, x1, y0, y1, z0, z1 = R['label']
        box = adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create((x0 + x1) / 20, (y0 + y1) / 20, (z0 + z1) / 20),
                                                     adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 1, 0), (x1 - x0) / 10, (y1 - y0) / 10, (z1 - z0) / 10)
        out.append((tbm.createBox(box), 'label', LABEL, 'matte'))
    src.deleteMe()
    _TEMP[key] = out
    return out


def _place(comp, model, piece, M, joint):
    """A copy of one piece, in its place, as its own component. (Not instances of one component: Fusion composed each copy's transform with the last one's, in a nested occurrence, and put them all in the wrong place.)"""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    c = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
    c.name = f"{joint} {REAL[model]['name']} {piece}"
    d = design()
    for tb, nm, col, fin in _temp_bodies(model, piece):
        t = tbm.copy(tb)
        tbm.transform(t, M)
        b = c.bRepBodies.add(t)
        b.name = f"{joint} {REAL[model]['name']} {nm}"
        b.appearance = F.appearance(d, col, fin)


def _servo_matrix(j, model, frame=None):
    """From the file's frame to the servo's place, at the zero pose."""
    A = spec_actuators()[j['act']]
    lx, ax = 'xyz'.index(j['long'][1]), 'xyz'.index(j['axis'])
    U = [0.0, 0.0, 0.0]
    U[lx] = 1.0 if j['long'][0] == '+' else -1.0
    Ax = [0.0, 0.0, 0.0]
    Ax[ax] = float(j['out'])
    c = [j['act_c'][k] + U[k] * A['shaft_x'] for k in range(3)]
    R = REAL[model]
    o, ux, az = frame or (R['o'], R['ux'], R['az'])
    cross = lambda a, b: (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    V3 = lambda v: adsk.core.Vector3D.create(*[float(x) for x in v])
    M = adsk.core.Matrix3D.create()
    M.setToAlignCoordinateSystems(adsk.core.Point3D.create(*[v / 10 for v in o]), V3(ux), V3(cross(az, ux)), V3(az),
                                  F.P(c), V3(U), V3(cross(Ax, U)), V3(Ax))
    return M


def spec_actuators():
    return spec()['actuators']


def _details(comp, title, body, at):
    """A board as what you connect to: parts in their colours, from the makers' drawings and photographs; nothing that only decorates. at(u, v, w): the board's own mm, w up from its parts' side, to world cm.
    body: (name, colour, finish, [(u0, u1, v0, v1, w0, w1) or (u, v, r, w0, w1)...]). One name, one body."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    d = design()
    c = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
    c.name = title
    for nm, col, fin, shapes in body:
        whole = None
        for sh in shapes:
            if len(sh) == 6:
                p0, p1 = at(sh[0], sh[2], sh[4]), at(sh[1], sh[3], sh[5])
                lo, hi = [min(a, b) for a, b in zip(p0, p1)], [max(a, b) for a, b in zip(p0, p1)]
                t = tbm.createBox(adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(*[(a + b) / 2 for a, b in zip(lo, hi)]),
                                                                         adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 1, 0), hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]))
            else:
                t = tbm.createCylinderOrCone(adsk.core.Point3D.create(*at(sh[0], sh[1], sh[3])), sh[2] / 10, adsk.core.Point3D.create(*at(sh[0], sh[1], sh[4])), sh[2] / 10)
            if whole is None:
                whole = t
            else:
                tbm.booleanOperation(whole, t, adsk.fusion.BooleanTypes.UnionBooleanType)
        b = c.bRepBodies.add(whole)
        b.name = f'{title} {nm}'
        b.appearance = F.appearance(d, col, fin)


SILVER, GOLD, GREEN_PCB, BLACK_PCB = '#C4C7CA', '#D8B95A', '#1F9C4E', '#141517'


def real_boards(name, comp, s):
    """The Pi and the bus board in their real looks, in place of the spec's boxes. Pi Zero 2 W: Raspberry Pi's mechanical drawing, 65 x 30 mm, holes 58 x 23; only what connects. The box's +x face is its parts' side."""
    gs = {g['name']: g for g in s['links'][name]['geoms']}
    if 'Pi Zero 2 W' in gs:
        g = gs['Pi Zero 2 W']
        x0, y0, z0 = (g['c'][k] - g['size'][k] / 2 for k in range(3))
        # its ports up, where there's room for a plug under the hood (they were down, in the base plate: the real parts' interference check found it)
        at = lambda u, v, w: ((x0 * 1000 + 1.4 + w) / 10, (y0 * 1000 + 65 - u) / 10, (z0 * 1000 + 30 - v) / 10)   # (65 - u: it was mirrored, with its microSD at -y. Parts toward +x and ports up puts the microSD at +y, as spec/robot.py has it)
        pads = [(8.37 + 2.54 * k, 30 - 3.5 + o, 0.75, 0, 0.05) for k in range(20) for o in (-1.27, 1.27)]
        rings = [(u, v, 2.9, 0, 0.05) for u in (3.5, 61.5) for v in (3.5, 26.5)]
        _details(comp, 'Pi Zero 2 W', [
            ('board', GREEN_PCB, 'gloss', [(0, 65, 0, 30, -1.4, 0)]),
            ('mounting rings', GOLD, 'metal', rings),
            ('GPIO pads', GOLD, 'metal', pads),
            ('processor', '#2A2C2E', 'matte', [(25.5, 39.5, 8, 22, 0, 1.0)]),
            ('microSD socket', SILVER, 'metal', [(1, 14, 9.5, 24, 0, 1.4)]),
            ('mini HDMI', SILVER, 'metal', [(6.8, 18, -0.5, 6.8, 0, 3.2)]),
            ('USB port', SILVER, 'metal', [(37.4, 45.4, -1, 4.6, 0, 2.6)]),
            ('power port', SILVER, 'metal', [(50, 58, -1, 4.6, 0, 2.6)]),
            ('camera socket', '#1B1C1F', 'matte', [(60, 64.6, 6.5, 23.5, 0, 1.2)]),
            ('camera socket latch', '#E9E6DA', 'matte', [(59.2, 60, 6.5, 23.5, 0, 1.2)]),
        ], at)
    if 'Waveshare bus board' in gs:
        g = gs['Waveshare bus board']
        back = (g['c'][0] + g['size'][0] / 2) * 1000
        ytop, ztop = (g['c'][1] + g['size'][1] / 2) * 1000, (g['c'][2] + g['size'][2] / 2) * 1000
        at = lambda u, v, w: ((back - 1.6 - w) / 10, (ytop - v) / 10, (ztop - u) / 10)     # the drawing's own: u along the 42, v up the 33 from the jack's edge, w off the board
        _details(comp, 'Waveshare bus board', [
            ('board', BLACK_PCB, 'gloss', [(0, 42, 0, 33, -1.6, 0)]),
            ('mounting rings', SILVER, 'metal', [(u, v, 2.3, 0, 0.05) for u in (2.5, 39.5) for v in (2.5, 30.5)]),
            ('servo sockets', CREAM, 'matte', [(5.2, 15.9, 28, 33, 0, 6.0), (17.5, 28.3, 28, 33, 0, 6.0)]),
            ('UART pins', '#17181A', 'matte', [(29.6, 37.0, 30.5, 33, 0, 8.5)]),
            ('DC jack', '#17181A', 'matte', [(4.9, 14.5, -1.1, 14, 0, 11)]),
            ('DC terminals', '#3E9B5F', 'matte', [(17.3, 25.3, -1, 6.3, 0, 10)]),
            ('USB-C', SILVER, 'metal', [(28, 35.1, -1.3, 7.4, 0, 3.3)]),
            ('jumpers', '#F2C230', 'matte', [(33.6, 39.3, 14.2, 22, 0, 6)]),
        ], at)
    if 'leg bus board' in gs:                      # the hybrid's second one, flat on its plate: the 42 across, the jack's edge down
        g = gs['leg bus board']
        back, zlow = (g['c'][0] + g['size'][0] / 2) * 1000, (g['c'][2] + g['size'][2] / 2) * 1000 - 33
        at = lambda u, v, w: ((back - 1.6 - w) / 10, (21 - u) / 10, (zlow + v) / 10)
        _details(comp, 'leg bus board', [
            ('board', BLACK_PCB, 'gloss', [(0, 42, 0, 33, -1.6, 0)]),
            ('mounting rings', SILVER, 'metal', [(u, v, 2.3, 0, 0.05) for u in (2.5, 39.5) for v in (2.5, 30.5)]),
            ('servo sockets', CREAM, 'matte', [(5.2, 15.9, 28, 33, 0, 6.0), (17.5, 28.3, 28, 33, 0, 6.0)]),
            ('UART pins', '#17181A', 'matte', [(29.6, 37.0, 30.5, 33, 0, 8.5)]),
            ('DC jack', '#17181A', 'matte', [(4.9, 14.5, -1.1, 14, 0, 11)]),
            ('DC terminals', '#3E9B5F', 'matte', [(17.3, 25.3, -1, 6.3, 0, 10)]),
            ('USB-C', SILVER, 'metal', [(28, 35.1, -1.3, 7.4, 0, 3.3)]),
            ('jumpers', '#F2C230', 'matte', [(33.6, 39.3, 14.2, 22, 0, 6)]),
        ], at)
    return ('Pi Zero 2 W' in gs) + ('Waveshare bus board' in gs) + ('leg bus board' in gs)


def real_servos(name, comp, s):
    """Put into this link the real pieces it carries: a case, a horn (it turns with the link it's screwed to), a rear idler."""
    n_ = 0
    names = {g['name'] for g in s['links'][name]['geoms']}
    for j in s['joints']:
        model = real_model(j['act'])
        if not model:
            continue
        case_link, horn_link = (j['child'], j['parent']) if j.get('case_in') == 'child' else (j['parent'], j['child'])
        n = j['name'].replace('_', ' ')
        if j['remote']:                                  # its horn turns the crank, in the link that carries the crank
            horn_link = next(ln for ln, l in s['links'].items() if any(g['name'] == 'beak crank' for g in l['geoms']))
        if case_link == name:
            _place(comp, model, 'case', _servo_matrix(j, model), n)
            n_ += 1
        if horn_link == name:
            _place(comp, model, 'horn', _servo_matrix(j, model), n)
            n_ += 1
        if f'{n} rear horn' in names or ('ankle' in n and f'{n[0]} ankle rear boss' in names):
            _place(comp, model, 'idler', _servo_matrix(j, model, REAL[model].get('idler_frame')), n)
            n_ += 1
    return n_


def occurrences():
    return {o.component.name: o for o in design().rootComponent.occurrences}


def new_document():
    doc = app().documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    d = design()
    d.designType = adsk.fusion.DesignTypes.DirectDesignType   # no timeline while the bodies go in (parametric() turns it on: joints need one)
    return f'new document, orientation {app().preferences.generalPreferences.defaultModelingOrientation} (1 = Z up)'


def parametric():
    d = design()
    d.designType = adsk.fusion.DesignTypes.ParametricDesignType
    return f'parametric, timeline {d.timeline.count}'


def _variant_file(stem):
    """robot.json -> parts.json; robot_dynamixel.json -> parts_dynamixel.json."""
    return SPEC.replace('robot', stem, 1)


def parts_spec():
    return json.load(open(os.path.join(REPO, 'spec', _variant_file('parts')), encoding='utf-8'))


def drawn_only(g, real):
    """A decal that is only a picture of a thing: nothing to buy or print, so not built. Screws, printed parts and what the spec marks in_cad (the battery's plugs) are built."""
    return g['role'] == 'decal' and 'fast' not in g and g['name'] not in real and not g.get('in_cad')


def link(name):
    """The link's component: its geoms, then each printed part joined, hollowed, holed and pocketed."""
    t = time.time()
    s = spec()
    d = design()
    root = d.rootComponent
    P_ = parts_spec()
    ps = {n: p for n, p in P_['parts'].items() if p['link'] == name}
    in_part = {g: n for n, p in ps.items() for g in p['geoms']}
    for o in list(root.occurrences):                     # rebuild: remove the old copy first (no "name (1)")
        if o.component.name == name or o.component.name.startswith(name + ' ('):
            if o.isGrounded:
                o.isGrounded = False                     # (a grounded occurrence refuses to go)
            o.deleteMe()
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    b = F.Builder(comp)
    geoms = s['links'][name]['geoms']
    made = {}
    real = {n for p in P_['parts'].values() for n in p['geoms']}
    closer = {n: v for p in ps.values() for n, v in p.get('dense', {}).items()}
    stand_in = real_skip(s)
    for g in geoms:
        if g['role'] == 'cut' or g['name'] in stand_in:
            continue
        if drawn_only(g, real):
            continue                                     # a picture of a thing, not a thing: nothing to buy or print
        body = b.build(dict(g, sections=closer[g['name']]) if g['name'] in closer else g)
        body.name = g['name']
        body.appearance = F.appearance(d, g['color'], finish(g))
        made[g['name']] = body
    for g in geoms:                                      # cuts: windows through the bodies built above
        if g['role'] == 'cut':
            b.cut(b.build(g), [comp.bRepBodies.itemByName(n) for n in g['targets'] if comp.bRepBodies.itemByName(n)], g['name'])
    problems = []
    for pn, p in ps.items():                             # what you print: hollow first, as the next step cuts by these shapes
        for gn, inner in p.get('inner', {}).items():
            bd = comp.bRepBodies.itemByName(gn)
            tool = b.build(dict(inner, name=f'{gn} hollow'))
            if 'thigh cover' in gn:                      # the big soft rim, on the outside and the inside together
                side = 1 if gn.startswith('L') else -1
                try:
                    _rim(bd, comp, side, COVER_R[0])
                    _rim(tool, comp, side, COVER_R[1])
                except RuntimeError as e:
                    problems.append(f'{gn} rim: {e}')
            b.cut_from(bd, tool)
    for pn, p in ps.items():                             # where a part gives way to its neighbour's exact shape
        for k in p['pockets']:
            if 'exact' not in k:
                continue
            tool = comp.bRepBodies.itemByName(k['exact'])
            for n in p['geoms']:
                ci = b.f.combineFeatures.createInput(comp.bRepBodies.itemByName(n), F.coll([tool]))
                ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
                ci.isKeepToolBodies = True
                try:
                    b.f.combineFeatures.add(ci)
                except RuntimeError:                     # no overlap with this piece of the part
                    pass
    soft = [0, 0, []]
    for pn, p in ps.items():                             # one body, with its screw holes and pockets
        # (a cut that splits a body leaves "name (1)", "name (2)": they're the part too. Joining by exact name left the hood's front band loose, and only the user's eye found it)
        bodies = [x for n in p['geoms'] for x in (comp.bRepBodies.item(i) for i in range(comp.bRepBodies.count)) if x.name == n or x.name.startswith(n + ' (')]
        whole = b.join(bodies, pn)
        whole.name = pn
        if not any(k in pn for k in SOFT_SKIP):
            whole, nr_, ns_ = soften(whole, comp, seats=seats_of(s['links'][name]['geoms'], set(p['geoms'])))
            soft[0] += nr_
            if ns_ < 0:
                soft[2].append(pn)
            else:
                soft[1] += ns_
        for k in p['pockets']:
            if 'exact' in k:
                continue
            k = dict(k, name=f"{pn} pocket")
            n_ = comp.bRepBodies.count
            try:
                b.cut_from(whole, b.build(k))
            except RuntimeError:
                # (a pocket whose face lies exactly on the part's: nudge it, and leave no stray tool behind)
                while comp.bRepBodies.count > n_:
                    comp.bRepBodies.item(comp.bRepBodies.count - 1).deleteMe()
                k2 = json.loads(json.dumps(k))
                if 'size' in k2:
                    k2['size'] = [v + 0.00004 for v in k2['size']]
                elif 'r' in k2:
                    k2['r'] += 0.00002
                try:
                    b.cut_from(whole, b.build(k2))
                except RuntimeError as e2:
                    while comp.bRepBodies.count > n_:
                        comp.bRepBodies.item(comp.bRepBodies.count - 1).deleteMe()
                    problems.append(f'{pn} pocket {k.get("kind")}: {e2}')
        for h in p['holes']:
            try:
                b.cut_from(whole, b.bore(tuple(h['p']), tuple(h['d']), h['r'], h['a'], h['b'], f'{pn} hole'))
            except RuntimeError as e:
                problems.append(f"{pn} hole {h['screw']}: {e}")
        if pn == 'hood':                                 # one body, in two colours: the orange band at its rim
            zcut = next(g['c'][2] - g['size'][2] / 2 for g in geoms if g['name'] == 'above the bill') * 100
            orange = F.appearance(d, next(g['color'] for g in geoms if g['name'] == 'upper bill'), 'matte')
            for i in range(whole.faces.count):
                if whole.faces.item(i).centroid.z < zcut:
                    whole.faces.item(i).appearance = orange
    nreal = real_servos(name, comp, s) + real_boards(name, comp, s)
    if name == s['root']:
        occ.isGrounded = True
    else:
        occ.isGroundToParent = False                     # new occurrences come in grounded (CDX-3R lesson 24)
    return f'{name}: {comp.bRepBodies.count} bodies, {len(ps)} printed parts, {nreal} real servo pieces, {soft[0]} edges rounded, {soft[1]} left sharp, not rounded {soft[2]}, {time.time() - t:.1f} s' + (f'; PROBLEMS {problems[:6]}' if problems else '')


def parts(geoms):
    """The geoms that are bodies ('cut' geoms are removed from others and leave none)."""
    return [g for g in geoms if g['role'] != 'cut']


def links(names):
    return [link(n) for n in names]


def _joint_sketch(root):
    for i in range(root.sketches.count):
        if root.sketches.item(i).name == 'joint points':
            return root.sketches.item(i)
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'joint points'
    sk.isVisible = False
    return sk


def joints(only=None):
    t = time.time()
    s = spec()
    root = design().rootComponent
    occ = occurrences()
    sk = _joint_sketch(root)
    dirs = {'x': adsk.fusion.JointDirections.XAxisJointDirection,
            'y': adsk.fusion.JointDirections.YAxisJointDirection,
            'z': adsk.fusion.JointDirections.ZAxisJointDirection}
    out = []
    for j in s['joints']:
        if only and j['name'] not in only:
            continue
        old = root.asBuiltJoints.itemByName(j['name'])
        if old:
            old.deleteMe()
        sp = sk.sketchPoints.add(sk.modelToSketchSpace(F.P(j['origin'])))
        geo = adsk.fusion.JointGeometry.createByPoint(sp)
        ji = root.asBuiltJoints.createInput(occ[j['child']], occ[j['parent']], geo)
        ji.setAsRevoluteJointMotion(dirs[j['axis']])
        jt = root.asBuiltJoints.add(ji)
        jt.name = j['name']
        lim = adsk.fusion.RevoluteJointMotion.cast(jt.jointMotion).rotationLimits
        lim.isMinimumValueEnabled = True
        lim.minimumValue = j['lower']
        lim.isMaximumValueEnabled = True
        lim.maximumValue = j['upper']
        out.append(j['name'])
    return f'{len(out)} joints, {time.time() - t:.1f} s'


def pose(q):
    root = design().rootComponent
    for i in range(root.asBuiltJoints.count):
        jt = root.asBuiltJoints.item(i)
        adsk.fusion.RevoluteJointMotion.cast(jt.jointMotion).rotationValue = float(q.get(jt.name, 0.0))
    return f'posed {len(q)} joints'


def show(skins=True, internals=True):
    s = spec()
    role = {g['name']: 'part' if g.get('in_cad') else g['role'] for l in s['links'].values() for g in l['geoms']}     # (the battery's plugs are inside: shown with the internals, not the skins)
    exposed = {g['name'] for l in s['links'].values() for g in l['geoms'] if g.get('exposed')}
    for j in s['joints']:
        if j['act'] and j['exposed']:
            exposed |= {f"{j['name']} flange"} if j['exposed'] == 'flange' else                 {f"{j['name']} actuator (buy) {j['act']}", f"{j['name']} flange"}
    n = 0
    for o in design().rootComponent.occurrences:
        for b in o.component.bRepBodies:
            b.isLightBulbOn = skins if role.get(b.name) in ('shell', 'decal') else (internals or b.name in exposed)
            n += 1
    return f'{n} bodies: skins {"on" if skins else "off"}, internals {"on" if internals else "off"}'


def verify():
    """Each printed body against the STL's volume: a pocket or hole that didn't cut, or a piece that didn't join, shows here."""
    P_ = parts_spec()
    occ = occurrences()
    out, bad = [], []
    for pn, p in P_['parts'].items():
        o = occ.get(p['link'])
        bodies = [o.component.bRepBodies.item(i) for i in range(o.component.bRepBodies.count)] if o else []
        b = next((x for x in bodies if x.name == pn), None)
        if b is None:
            bad.append(f'{pn}: no body')
            continue
        v = b.physicalProperties.volume
        e = abs(v - p['volume_cm3']) / p['volume_cm3']
        pieces = sum(1 for x in bodies if x.name.startswith(pn + ' ') and False)
        out.append((pn, round(v, 2), round(p['volume_cm3'], 2), round(e * 100, 1), b.isSolid))
        if e > 0.04 or not b.isSolid:
            bad.append(f"{pn}: Fusion {v:.2f} cm3, STL {p['volume_cm3']:.2f} ({e * 100:.1f} % off), solid {b.isSolid}")
    return dict(parts=len(out), worst=sorted(out, key=lambda r: -r[3])[:8], bad=bad)


def verify_points():
    """tools/verify_fusion.py: is each of its points inside the Fusion body? Written to reports/fusion_points.json."""
    t = time.time()
    data = json.load(open(os.path.join(REPO, 'spec', _variant_file('verify_points')), encoding='utf-8'))
    P_ = parts_spec()
    occ = occurrences()
    IN = adsk.fusion.PointContainment.PointInsidePointContainment
    out = {}
    for pn, v in data.items():
        c = occ[P_['parts'][pn]['link']].component
        body = next((c.bRepBodies.item(i) for i in range(c.bRepBodies.count) if c.bRepBodies.item(i).name == pn), None)
        if body is None:
            continue
        out[pn] = [1 if body.pointContainment(F.P(q)) == IN else 0 for q in v['pts']]
    json.dump(out, open(os.path.join(REPO, 'reports', _variant_file('fusion_points')), 'w', encoding='utf-8'))
    return f'{len(out)} parts, {time.time() - t:.1f} s'


def status():
    s = spec()
    d = design()
    occ = occurrences()
    rows, problems = [], []
    P_ = parts_spec()
    for name, l in s['links'].items():
        o = occ.get(name)
        if not o:
            problems.append(f'missing {name}')
            continue
        nb = o.component.bRepBodies.count
        real = {n for p in P_['parts'].values() for n in p['geoms']}
        stand_in = real_skip(s)
        nreal = [g for g in parts(l['geoms']) if g['name'] not in stand_in and not drawn_only(g, real)]
        want = len(nreal) - sum(len(p['geoms']) - 1 for p in P_['parts'].values() if p['link'] == name)
        if nb != want:
            problems.append(f'{name}: {nb} bodies, spec has {want}')
        rows.append(f'{name}:{nb}')
    extra = [n for n in occ if n not in s['links']]
    if extra:
        problems.append(f'extra components {extra}')
    count = 0
    nj = 0
    if d.designType == adsk.fusion.DesignTypes.ParametricDesignType:    # no timeline, and no joints, until parametric()
        tl = d.timeline
        count = tl.count
        for i in range(tl.count):
            it = tl.item(i)
            if not it.isGroup and it.healthState != adsk.fusion.FeatureHealthStates.HealthyFeatureHealthState:
                problems.append(f'unhealthy: {it.name} {it.errorOrWarningMessage}')
        nj = d.rootComponent.asBuiltJoints.count
    return {'links': len(rows), 'joints': nj, 'timeline': count, 'problems': problems[:30]}


EXTENT_PER_CM = 1.0       # orthographic viewExtents per cm of visible height (calibrated by tools/compare.py)
VIEWS = {
    # name: (azimuth deg from the robot's front, towards its left; elevation deg), as the reference photos
    # '34' at 60 deg: the reveal frame's silhouette widths fit best there (rms 3 cm, 2026-10-31); at 40 deg every
    # width was 10 cm too wide and the legs read heavy
    'front': (0, 0), 'side': (90, 0), 'back': (180, 0), '34': (60, 8), 'hero': (40, 8),
}


def render(path, view='side', w=800, h=600, height_cm=190.0, target_z_cm=86.0):
    """Orthographic picture of the whole robot from one of VIEWS, saved as PNG (for tools/compare.py)."""
    import math
    az, el = (math.radians(v) for v in VIEWS[view])
    vp = app().activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedVisualStyle   # no edge lines: they made the model look busy
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    cam.isSmoothTransition = False
    t = adsk.core.Point3D.create(0, 0, target_z_cm)
    d = (math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el))
    cam.target = t
    cam.eye = adsk.core.Point3D.create(600 * d[0], 600 * d[1], target_z_cm + 600 * d[2])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.viewExtents = height_cm * EXTENT_PER_CM
    vp.camera = cam
    vp.refresh()
    root = design().rootComponent
    root.isJointsFolderLightBulbOn = False
    root.isSketchFolderLightBulbOn = False
    root.isConstructionFolderLightBulbOn = False
    if os.path.exists(path):              # so 'written' means this save wrote it (an old file fooled the check)
        os.remove(path)
    ok = vp.saveAsImageFile(path, w, h)
    written = os.path.exists(path)
    src = os.path.join(os.path.dirname(path), 'replica_source.txt')   # tools/render_pose.py's label for compare.py
    if written and os.path.basename(path).startswith('replica_') and os.path.exists(src):
        os.remove(src)
    return f'{view}: {path} ({ok}, written: {written})'


def render_posed(q, shots):
    """render() for each (path, view, w, h, height_cm, target_z_cm) in shots, in pose q. While a moved joint's
    position is uncaptured, saveAsImageFile returns True and writes nothing (2026-10-31): capture it (a snapshot),
    render, then delete the snapshot and go back to the zero pose."""
    d = design()
    pose(q)
    snap = d.snapshots.add()
    out = [render(*s) for s in shots]
    snap.deleteMe()
    pose({})
    return out + [f'pending position after: {d.snapshots.hasPendingSnapshot}']


def export_print_stls(folder):
    """Each printed body as it is here, untransformed: tools/prints_from_fusion.py turns them to print and writes prints/."""
    os.makedirs(folder, exist_ok=True)
    em = design().exportManager
    occ = occurrences()
    n = 0
    for pn, p in parts_spec()['parts'].items():
        b = occ[p['link']].component.bRepBodies.itemByName(pn)
        if b is None:
            continue
        opt = em.createSTLExportOptions(b, os.path.join(folder, pn.replace(' ', '_') + '.stl'))
        opt.isBinaryFormat = True
        opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        opt.unitType = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        em.execute(opt)
        n += 1
    return f'{n} print STLs in {folder}'


def export_stl(folder):
    t = time.time()
    os.makedirs(folder, exist_ok=True)
    em = design().exportManager
    out = []
    for name, o in occurrences().items():
        path = os.path.join(folder, f'{name}.stl')
        opt = em.createSTLExportOptions(o, path)
        opt.isBinaryFormat = True
        opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
        em.execute(opt)
        out.append(name)
    return f'{len(out)} STLs in {folder}, {time.time() - t:.1f} s'
