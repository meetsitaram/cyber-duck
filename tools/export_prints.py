"""One STL per printed part, with its screw holes, for slicing: prints/<part>.stl and prints/README.md.

  python -I tools/export_prints.py                every printed part (the left leg's; the right is mirrored in the slicer)
  python -I tools/export_prints.py --test knee    only the one-joint test print

Every screw in the spec cuts its holes: clearance through the part it clamps, a counterbore for its head where it is sunk,
and a pilot in any printed part it threads into (M2 1.6 mm, M3 2.5 mm). Check one print before trusting the fits: printers
differ, so bores may need 0.1-0.2 mm of adjustment. Sizes are as designed, in millimetres.
"""
import argparse
import os
import sys

import math

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import json  # noqa: E402
import assembly as A  # noqa: E402
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import printable as PR  # noqa: E402

CLEAR = {'M2': 0.00115, 'M2.5': 0.00145, 'M3': 0.0017}
PILOT = {'M2': 0.0008, 'M2.5': 0.00105, 'M3': 0.00125}
SERVO_GAP = 0.00015                                # a printed pocket round a bought part, each side: 0.3 mm on a width. The model draws them touching, and FDM prints bores small
BEARING_GAP = 0.00005                              # on a bearing seat's radius: 0.1 mm on the diameter, a firm press in PLA. Print one joint first
HEAD_R, HEAD_H = {'M2': 0.0019, 'M2.5': 0.00225, 'M3': 0.0027}, 0.002   # a pan head, and a turn of tolerance
MATE_GAP = 0.0001                                  # where one printed part sits in another
FUSION = G.parts_path()
TEST = {'knee': ['L thigh frame', 'L knee lid', 'L shin']}


def cylinder(p, d, r, a, b):
    """A cylinder from a to b along d from p."""
    T = np.eye(4)
    T[:3, :3] = G._frame_to(np.asarray(d, float))
    T[:3, 3] = np.asarray(p) + np.asarray(d) * (a + b) / 2
    m = trimesh.creation.cylinder(radius=r, height=b - a, sections=32)
    m.apply_transform(T)
    return C.to_manifold(m)


def solid_part(B, part):
    # (a cover prints hollow: its wall)
    def body(n):
        ln, i, g = B.idx[n]
        thin = g['kind'] == 'plate' and g['t'] < 0.0045   # thinner than two walls: solid
        return B.S.geom[(ln, i)] if thin else B.S.wall.get((ln, i), B.S.geom[(ln, i)])
    return C.union([body(n) for n in B.parts[part]['geoms']])


def grown(g, c):
    """A bought part's solid, c bigger each side."""
    g = dict(g)
    if g['kind'] == 'stepbox':
        g['boxes'] = [[[v - c for v in lo], [v + c for v in hi]] for lo, hi in g['boxes']]
    elif g['kind'] == 'rbox':
        g['size'] = [v + 2 * c for v in g['size']]
    elif g['kind'] == 'cyl':
        g['r'] += c
    return g


def shell_round(g, c):
    """What a shell leaves for a lid in it: the lid's plate, whole, with a gap (its bore is the lid's business, not the shell's)."""
    g = {k: v for k, v in g.items() if k != 'targets'}
    if g['kind'] == 'plate':
        cx, cy = (sum(p[k] for p in g['pts']) / len(g['pts']) for k in (0, 1))
        g['pts'] = [[x + c * (x - cx) / math.hypot(x - cx, y - cy), y + c * (y - cy) / math.hypot(x - cx, y - cy)] for x, y in g['pts']]
        return g
    return grown(g, c)


def rank(B, part):
    """Where two printed parts are drawn in the same place, the lower rank gives way. (The spec overlaps them so one solid joins them; a printer can't. 2026-11-03:
    a sole 1 mm into its shoe, an eye ring through its face, lids in their trays' end walls: all invisible, as every check took a link as one solid.)"""
    gs = [B.idx[n][2] for n in B.parts[part]['geoms']]
    if part == 'eye iris':
        return 3.5
    if part == 'eye ring':
        return 2.5
    if part == 'face':
        return 2.2
    if any(g.get('material') == 'tpu' for g in gs):
        return 1
    if ' lid' in part or 'sheet' in part:
        # (a lid with a bearing seat ranks over a shell: the cover ate a slot through the seat's wall, the race showing through it, and only the user's eye found it, 2026-11-04)
        return 3.2 if any(' seat' in g['name'] for g in gs) else 2
    if any(g['role'] == 'shell' for g in gs):
        return 3
    return 5


def pockets(B, part, m):
    """The room this part must leave round its neighbours: the geom dicts of what to cut."""
    links = {B.idx[n][0] for n in B.parts[part]['geoms']}
    out = []
    mine = rank(B, part)
    for name, p in B.parts.items():
        if name == part:
            continue
        for n in p['geoms']:
            ln, i, g = B.idx[n]
            if ln not in links or g['role'] == 'cut':
                continue
            if p['kind'] == 'P':
                if rank(B, name) <= mine:
                    continue
                if mine == 3 and rank(B, name) == 3.2:
                    t = shell_round(g, MATE_GAP)
                    tool = C.to_manifold(G.mesh_of(t))
                    if C.overlap(m, tool) > 1e-12:
                        out.append((t, tool))
                    continue
                # a plain shape: with a gap. One with cuts, or a shell's wall: exactly that shape, or the cuts would be lost
                if g['kind'] in ('rbox', 'cyl') and n not in B.cuts and (ln, i) not in B.S.wall:
                    t = grown(g, MATE_GAP)
                else:
                    tool = B.S.wall.get((ln, i), B.S.geom[(ln, i)])
                    if C.overlap(m, tool) > 1e-12:
                        out.append((dict(exact=n, name=n), tool))
                    continue
            elif g['role'] == 'actuator' or n in ('Pi Zero 2 W', 'U2D2', 'Waveshare bus board', 'IMU', 'battery 2S 850 mAh', 'servo bus board', 'camera', 'PWM bonnet', 'Pi 5 V converter') or n.endswith(('5 V regulator', '6 V BEC')) or n.endswith('foot switch'):
                t = grown(g, SERVO_GAP)
            elif n.endswith('outer race'):
                t = dict(g, r=g['r'] + BEARING_GAP)
            elif n.endswith(('flange', 'rear horn')) and g['kind'] == 'cyl':
                # drawn 0.5 mm into its plate, so the solids join; more than that is a real clash (a horn through the shoe's upper)
                tool = C.to_manifold(G.mesh_of(g))
                if C.overlap(m, tool) <= 3.14159 * g['r'] ** 2 * 0.00065:
                    continue
                t = grown(g, SERVO_GAP)
            else:
                continue
            tool = C.to_manifold(G.mesh_of(t))
            if C.overlap(m, tool) > 1e-12:
                out.append((t, tool))
    return out


def dense(g):
    """A squarish shell's sections every 2 mm, on the offline solid's own curve: Fusion's smooth loft through a few sections bows away from it, by enough to thicken a 2 mm wall."""
    z0, z1, f = G._loft_params(g)
    zs = sorted(set(np.linspace(z0, z1, max(int((z1 - z0) / 0.002), 3)).tolist()))
    return [dict(z=z, **{k: float(f[k](z)) for k in ('cx', 'cy', 'a', 'b', 'n')}) for z in zs]


def hole_list(B, part):
    """(p, d, r, a, b, screw): every screw's cuts on this part."""
    out = []
    names = set(B.parts[part]['geoms'])
    for n, (ln, g) in B.screws.items():
        f = g['fast']
        p, d = np.array(f['p']), np.array(f['d'])
        sz = f['size'].split()[0]
        # (a pivot's screw goes through parts of 2 links: the jaw had no holes for its own hinge)
        here = lambda q: B.idx[q][0] == ln or f.get('pivot')
        under = [q for q in names if here(q) and B.inside(B.idx[q][2], (p + d * 0.0002)[None]).any()]
        ts = np.linspace(0, f['L'], 25)
        on_axis = [q for q in names if here(q) and B.inside(B.idx[q][2], p + np.outer(ts, d)).any()]
        if not on_axis:
            if rank(B, part) == 1 and any(B.idx[q][0] == ln for q in names):   # a head that presses into a soft part: a pocket
                tool = cylinder(p, d, HEAD_R[sz] + 0.0002, -HEAD_H, 0.0)
                if C.overlap(solid_part(B, part), tool) > 1e-11:
                    out.append((p, d, HEAD_R[sz] + 0.0002, -HEAD_H, 0.0, n))
            continue
        if under or ' spacer' in part:             # it clamps this part: a clearance hole, its head's counterbore above
            out.append((p, d, f.get('clear', CLEAR[sz]), -0.0005, f['L'], n))
            out.append((p, d, {'M3': 0.0029, 'M2.5': 0.0024}.get(sz, 0.0021), -0.03, 0.0, n))
            if f.get('recess'):                    # a shallow seat in the plate itself, for a flat head
                out.append((p, d, f['recess'][0], -0.0005, f['recess'][1], n))
        else:
            out.append((p, d, PILOT[sz], -0.0005, f['L'] + 0.0005, n))
    return out


def with_holes(B, part):
    m = solid_part(B, part)
    for _, tool in pockets(B, part, m):
        m = m - tool
    for p, d, r, a, b, _ in hole_list(B, part):
        m = m - cylinder(p, d, r, a, b)
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--test')
    a = ap.parse_args()
    spec = G.load()
    S = C.Solids(spec)
    B = A.Build(spec, S)
    out = G.out_path('prints')
    os.makedirs(out, exist_ok=True)
    want = TEST[a.test] if a.test else [n for n, p in B.parts.items() if p['kind'] == 'P']
    fusion = {}
    rows = []
    orient = {}
    for part in want:
        m = with_holes(B, part)
        mesh = m.to_mesh()
        tm = trimesh.Trimesh(np.asarray(mesh.vert_properties[:, :3], float) * 1000, np.asarray(mesh.tri_verts, int))
        best = PR.best_orientation(tm)
        if G.RECIPE == 'dynamixel' and part.endswith((' shoe', ' sole')):     # on their flat faces, the shoe's underside and the sole's top (2026-11-09, the user)
            ax = ('z', -1) if part.endswith(' shoe') else ('z', 1)
            best = PR.score(tm, next(T for x, sg, T in PR.orientations() if (x, sg) == ax))
        if part == 'head frame':
            # (least overhang area stood it 141 mm tall on 330 mm2, its supports trapped inside. With its band on the bed: 60 mm, 3 x the first layer, and under half the support landing on the part)
            best = dict(best, T=trimesh.transformations.rotation_matrix(np.radians(90), [0, 1, 0]) @ np.asarray(best['T'], float))
        tm.apply_transform(best['T'])
        tm.apply_translation(-tm.bounds[0])
        name = part.replace(' ', '_') + '.stl'
        for _try in range(6):                      # (Windows refuses a write now and then, OSError 22, a scanner holding the file: try again)
            try:
                tm.export(os.path.join(out, name))
                break
            except OSError:
                if _try == 5:
                    raise
                __import__('time').sleep(1.0)
        e = tm.extents
        gs = B.parts[part]['geoms']
        solid = solid_part(B, part)
        fusion[part] = dict(link=B.idx[gs[0]][0], geoms=gs, volume_cm3=abs(m.volume()) * 1e6, print_T=np.asarray(best['T'], float).tolist(),
                            material=B.idx[gs[0]][2].get('material'), color=B.idx[gs[0]][2]['color'],
                            holes=[dict(p=p.tolist(), d=d.tolist(), r=r, a=a_, b=b_, screw=sc) for p, d, r, a_, b_, sc in hole_list(B, part)],
                            pockets=[t for t, _ in pockets(B, part, solid)],
                            inner={n: G.inner_of(B.idx[n][2], C.WALL) for n in gs if (ln_i := B.idx[n]) and (ln_i[0], ln_i[1]) in B.S.wall and not (B.idx[n][2]['kind'] == 'plate' and B.idx[n][2]['t'] < 0.0045)})
        for n in gs:
            g_ = B.idx[n][2]
            if g_['kind'] == 'loft' and g_['role'] == 'shell' and max(q['n'] for q in g_['sections']) >= 5:
                fusion[part].setdefault('dense', {})[n] = dense(g_)
                if n in fusion[part]['inner']:
                    fusion[part]['inner'][n]['sections'] = dense(fusion[part]['inner'][n])
        orient[part] = best
        n_ = len([p for p in m.decompose() if p.volume() > 1e-9])   # (a negative one is a closed void)
        rows.append((part, name, e, abs(m.volume()) * 1e6, n_))
        print(f'{name:32s} {e[0]:5.0f} x {e[1]:5.0f} x {e[2]:5.0f} mm  {abs(m.volume()) * 1e6:6.1f} cm3  {n_} piece(s)')
    if not a.test:
        json.dump(dict(parts=fusion, bought=[n for n, p in B.parts.items() if p['kind'] == 'B'], groups={n: p['geoms'] for n, p in B.parts.items() if p['kind'] == 'B'}), open(FUSION, 'w', encoding='utf-8'), indent=0)
        L = ['# Printed parts', '', 'From `python -I tools/export_prints.py`, in millimetres: every part, left and right, already turned the way it should print, its long overhangs down',
             '(tools/printable.py chooses). Pockets are cut 0.3 mm over each bought part and bearing seats 0.1 mm over. None has been test printed: print one joint first.', '',
             '| Part | File | Size on the bed (mm) | Volume (cm3) | Material | Overhang to support (mm2) |', '|---|---|---|---|---|---|']
        for part, name, e, v, n_ in rows:
            mat = 'TPU 95A' if fusion[part]['material'] == 'tpu' else 'PLA'
            L.append(f"| {part} | {name} | {e[0]:.0f} x {e[1]:.0f} x {e[2]:.0f} | {v:.1f} | {mat} | {orient[part]['overhang']:.0f}{' (none)' if orient[part]['overhang'] < 1 else ''} |")
        L += ['', 'The head frame prints in orange, in one filament: only its rim shows below the hood. The hood is modelled in two colours, a white top and an orange 5 mm band at its rim: one STL, so print it white and paint the band, or split the colour in a multi-material slicer.',
              'A part with an overhang figure needs support or a bridge there; tools/printable.py counts faces, not a slicer, so it overstates short bridges. Check each in your slicer.', '']
        servo_, bear_, screws_ = ('MG90S', 'printed bushing', 'its own ear and horn screws and 2 M2 x 8') if G.RECIPE == 'nano' else ('XL330-M288', '6703ZZ', 'its M2 screws') if G.RECIPE == 'dynamixel' else ('MG996R, its 25T aluminium disc horn and its 4 M3 horn screws', '6704ZZ', '4 M3 x 16 and 2 M3 x 10 self-tapping through its ears') if G.RECIPE == 'hobby' else ('HD-1910-C001', 'MR2216ZZ', 'its M2 screws') if G.RECIPE == 'hd1910' else ('STS3215-C044', '6704ZZ', '6 M2 x 6, 2 M2 x 8 and 4 M3 x 10')
        first = '`L_thigh`, `L_knee_ear_clamp`, `L_shin`, `L_knee_rear_arm` and `L_knee_bushing`' if G.RECIPE == 'nano' else '`L_thigh_frame`, `L_knee_lid` and `L_shin`'
        L += ['', '## Print one joint first', '', f'Before buying the rest, print {first}, with 1 {servo_}, 1 {bear_},',
              f'{screws_}. It tests: does the servo sit in its tray; does the bearing press into the seat and over the horn; do the screws bite;',
              'how much the shin flexes with about 1.3 kg hung from it, off-centre as a foot would load it. Note what fits, and change the spec, not the STL.']
        if G.RECIPE == 'nano':                     # its own notes: no bearings, no orange head frame; a 21T spline in every hub
            L = [x for x in L if not x.startswith(('The head frame prints in orange', '## Print one joint first', 'Before buying the rest', f'{screws_}. It tests', 'how much the shin flexes'))]
            L += ['## Splines and bushings', '',
                  "Each hub is drawn with a plain 4.9 mm bore over the SG90's 21-tooth, 4.8 mm spline (spec/actuators.json): the teeth aren't in the STL. Either print the bore 0.1 mm under the spline and press the shaft in warm,",
                  "so it cuts its own teeth in the PLA, or glue the hub of the servo's own plastic horn into the bore. Test which holds on one joint first. The bushings print upright, 0.15 mm over their stubs: ream them if tight.", '',
                  '## Print one joint first', '', f'Before buying the rest, print {first}, with 1 {servo_} and {screws_}. It tests: does the servo slide into its cradle and its ears onto their blocks',
                  '(the ear holes are read off a drawing that is not to scale); does the hub grip the spline; does the bushing turn freely on its stub; how much the shin flexes with about 350 g hung from the foot. Change the spec, not the STL.']
        open(os.path.join(out, 'README.md'), 'w', encoding='utf-8').write('\n'.join(L) + '\n')
        import variant_banner
        variant_banner.stamp(os.path.join(out, 'README.md'), G.RECIPE, 'prints')


if __name__ == '__main__':
    main()
