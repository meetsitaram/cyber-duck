"""The assembly guide's animations, from tools/assembly.py's order: docs/images/assembly/<section>.gif.

Each part flies in along the path the checks tested, each screw along its own axis, a caption per step; parts fitted in
earlier steps stay put. Plain python (MuJoCo lives in the user site-packages).

  python tools/render_assembly.py             every section
  python tools/render_assembly.py thigh_L     one section
  python tools/render_assembly.py --still     one still per section instead, to check the framing
"""
import math
import os
import re
import sys
import tempfile

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly as A  # noqa: E402
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

OUT = G.out_path('docs/images/assembly')
W, H, CAP = 560, 480, 44
FLY = 0.045                                        # how far out a part starts
SCREW_FLY = 0.018
N_IN, HOLD = 8, 5                                  # frames flying in; frames held after each step
LEFT_ONLY = re.compile(r'_R$')
VIEW = dict(framebench=(215, 20), thigh=(235, 20), shin=(235, 20), shoe=(40, -25), leg=(235, 15), body=(30, -30), legs=(35, 12), column=(50, 15),
            torso=(140, 25), headbench=(50, 25), jawbench=(40, 25), head=(40, 25))


class Scene:
    """One MuJoCo model with a body per item (a part, or a screw), moved by its mocap position, hidden far away."""

    def __init__(self, spec, extra='', visual=None, asset='', real_folder=None):
        """real_folder: Fusion's own bodies, as cad/fusion/build.py export_print_stls wrote them, for the printed parts. The spec's shapes have no fillets: Fusion rounds them, so a picture from the spec shows less than you'd print."""
        self.spec = spec
        self.B = A.Build(spec)
        self.items = {}                            # item -> list of (mesh verts, faces, colour)
        S = self.B.S
        placed = set()
        for n, p in self.B.parts.items():
            for gn in p['geoms']:
                ln, i, g = self.B.idx[gn]
                self.items.setdefault(n, []).append((S.geom[(ln, i)], g['color']))
                placed.add(gn)
        for n, (ln, g) in self.B.screws.items():
            i = self.B.idx[n][1]
            self.items[n] = [(self.head(g), g['color'])]
            placed.add(n)
        # the other marks (eye, face, hubs) ride on the part they sit on
        for gn, (ln, i, g) in self.B.idx.items():
            if gn in placed or g['role'] != 'decal':
                continue
            c = np.array(G.mesh_of(g).bounds).mean(0)
            best = min((n for n, p in self.B.parts.items() if self.B.idx[p['geoms'][0]][0] == ln),
                       key=lambda n: min(np.linalg.norm(np.array(G.mesh_of(self.B.idx[q][2]).bounds).mean(0) - c)
                                         for q in self.B.parts[n]['geoms']))
            self.items[best].append((S.geom[(ln, i)], g['color']))
        self.names = list(self.items)
        tmp = tempfile.mkdtemp()
        assets, bodies = [], []
        self.gcol = {}
        k = 0
        for bi, n in enumerate(self.names):
            gs = ''
            real = self.real_meshes(n, real_folder)
            for m, col in (real if real else self.items[n]):
                if isinstance(m, tuple):
                    v, f = m
                else:
                    mesh = m.to_mesh()
                    v, f = np.asarray(mesh.vert_properties[:, :3], float), np.asarray(mesh.tri_verts, int)
                if len(f) == 0:
                    continue
                RP.stl(os.path.join(tmp, f'm{k}.stl'), v, f)
                assets.append(f'<mesh name="m{k}" file="m{k}.stl"/>')
                gs += f'<geom type="mesh" mesh="m{k}" rgba="{" ".join(f"{c:.3f}" for c in RP.rgba(col))}"/>'
                k += 1
            bodies.append(f'<body name="b{bi}" mocap="true">{gs}</body>')
        visual = visual or """<visual><global offwidth="1600" offheight="1600"/><headlight ambient=".42 .42 .42" diffuse=".45 .45 .45" specular=".1 .1 .1"/>
<quality shadowsize="0"/></visual><worldbody><light directional="true" diffuse=".35 .35 .35" specular=".2 .2 .2" dir="-0.6 -0.3 -1" castshadow="false"/></worldbody>"""
        xml = f"""<mujoco><compiler meshdir="{tmp}"/>
{visual}<asset>{asset}{''.join(assets)}</asset>
<worldbody>
{''.join(bodies)}{extra}</worldbody></mujoco>"""
        path = os.path.join(tmp, 'scene.xml')
        open(path, 'w').write(xml)
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        self.r = mujoco.Renderer(self.m, H, W)
        self.bid = {n: self.m.body(f'b{i}').id for i, n in enumerate(self.names)}

    def real_meshes(self, n, folder):
        """This item's meshes from Fusion's body, plus its marks, or None to use the spec's shapes."""
        if not folder or n not in self.B.parts or self.B.parts[n]['kind'] != 'P':
            return None
        path = os.path.join(folder, n.replace(' ', '_') + '.stl')
        if not os.path.exists(path):
            print('no Fusion body for', n, ': the spec shape it is')
            return None
        import trimesh
        tm = trimesh.load(path)
        tm.apply_scale(0.001)                      # mm to m
        solids = [x for x in self.items[n] if self._not_mark(n, x)]
        marks = [x for x in self.items[n] if not self._not_mark(n, x)]
        if not solids:                             # a part drawn wholly as marks, like the face: Fusion's body is all of it, and already holds the others (the face's sensor pad): drawn again they share its surface and flicker
            marks.sort(key=lambda x: -abs(x[0].volume()))
            solids, marks = marks[:1], []
        return self.colour_faces(n, tm, solids) + marks

    def colour_faces(self, n, tm, solids):
        """Colours as cad/fusion/build.py paints them, so a picture can't disagree with the model: a body in its part's colour, and the hood orange below the bill's cut."""
        vols = {}
        for m, c in solids:
            vols[c] = vols.get(c, 0.0) + abs(m.volume())
        main = max(vols, key=vols.get)
        if n != 'hood':
            return [((tm.vertices, tm.faces), main)]
        geoms = self.spec['links'][self.B.idx['head'][0]]['geoms']
        zcut = next(g['c'][2] - g['size'][2] / 2 for g in geoms if g['name'] == 'above the bill')
        band = next(g['color'] for g in geoms if g['name'] == 'upper bill')
        low = tm.triangles_center[:, 2] < zcut
        return [((tm.vertices, tm.faces[~low]), main), ((tm.vertices, tm.faces[low]), band)]

    def _not_mark(self, n, item):
        S = self.B.S
        for gn in self.B.parts[n]['geoms']:
            ln, i, g = self.B.idx[gn]
            if S.geom[(ln, i)] is item[0]:
                return g['role'] != 'decal'
        return False

    def head(self, g):
        """A screw drawn at its real length, for the fly-in."""
        f = g['fast']
        sh = 0.0015 if f['size'].startswith('M3') else 0.001
        hd = 0.0027 if f['size'].startswith('M3') else 0.0019
        p, d = np.array(f['p']), np.array(f['d'])
        ax = d.tolist()
        s = C.to_manifold(G.mesh_of(dict(kind='cyl', c=(p + d * f['L'] / 2).tolist(), axis=ax, r=sh, h=f['L'])))
        h = C.to_manifold(G.mesh_of(dict(kind='cyl', c=(p - d * 0.00075).tolist(), axis=ax, r=hd, h=0.0015)))
        return s + h

    def frame(self, shown, offsets, view, box, quats=None):
        for n, b in self.bid.items():
            mc = self.m.body_mocapid[b]
            self.d.mocap_pos[mc] = offsets.get(n, (0, 0, 0)) if n in shown else (50, 50, 50)
            self.d.mocap_quat[mc] = quats[n] if quats else (1, 0, 0, 0)
        mujoco.mj_forward(self.m, self.d)
        az, el = (math.radians(a) for a in view)
        dv = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = box[0]
        cam.distance = 1.5
        cam.azimuth = math.degrees(math.atan2(-dv[1], -dv[0]))
        cam.elevation = -math.degrees(el)
        self.m.vis.global_.orthographic = 1
        self.m.vis.global_.fovy = box[1]
        self.r.update_scene(self.d, cam)
        img = self.r.render().copy()
        self.r.enable_depth_rendering()            # (segmentation rendering overflows with far-away mocap bodies)
        self.r.update_scene(self.d, cam)
        dep = self.r.render()
        self.r.disable_depth_rendering()
        img[dep >= dep.max() - 1e-6] = 255
        return img


def bounds(sc, names):
    lo, hi = np.full(3, 9.0), np.full(3, -9.0)
    for n in names:
        for m, _ in sc.items[n]:
            a, b = C.bbox(m)
            lo, hi = np.minimum(lo, a), np.maximum(hi, b)
    return lo, hi


def caption(img, title, cap, k, n):
    im = Image.new('RGB', (W, H + CAP), 'white')
    im.paste(Image.fromarray(img), (0, CAP))
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, W, CAP), fill=(32, 36, 44))
    try:
        f = ImageFont.truetype('arialbd.ttf', 15)
    except OSError:
        f = ImageFont.load_default()
    d.text((10, 13), f'{k} / {n}   {cap}', fill=(255, 255, 255), font=f)
    return im


def render_section(sc, sec, done, still=False):
    have = [p for o in sec['on'] for p in done[o]]
    for cap, items, pats in sec['steps']:
        for item, _ in items:
            have += done[item[1:]] if item.startswith('@') else [item]
        for pat in pats:
            have += [n for n in sc.B.screws if re.search(pat, n)]
    done_now = list(have)
    key = sec['key'].split('_')[0]
    view = VIEW[key]
    if sec['key'].endswith('_R'):                  # the mirror's camera
        view = (-view[0], view[1])
    lo, hi = bounds(sc, done_now)
    c = (lo + hi) / 2
    size = float(np.linalg.norm(hi - lo)) * 1.05 + 2 * FLY * 0.6
    box = (c.tolist(), size)
    if still:
        return [caption(sc.frame(set(done_now), {}, view, box), sec['title'], sec['title'], 1, 1)], done_now
    frames = []
    shown = [p for o in sec['on'] for p in done[o]]
    n = len(sec['steps'])
    for k, (cap, items, pats) in enumerate(sec['steps'], 1):
        moves = []
        for item, way in items:
            group = done[item[1:]] if item.startswith('@') else [item]
            w = np.array(way if way is not None else (0, 0, 1), float)
            moves.append((group, w * FLY, way is not None))
        for pat in pats:
            for s_ in [x for x in sc.B.screws if re.search(pat, x)]:
                moves.append(([s_], np.array(sc.B.screws[s_][1]['fast']['d']) * SCREW_FLY, True))
        new = [p for g, _, _ in moves for p in g]
        for t in np.linspace(1, 0, N_IN):
            e = t * t * (3 - 2 * t)
            off = {}
            for g, v, fly in moves:
                for p in g:
                    if fly:
                        off[p] = tuple(-v * e)
            frames.append(caption(sc.frame(set(shown + new), off, view, box), sec['title'], cap, k, n))
        shown += new
        frames += [frames[-1]] * HOLD
    return frames, done_now


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    still = '--still' in sys.argv
    spec = G.load()
    sc = Scene(spec)
    os.makedirs(OUT, exist_ok=True)
    done = {}
    secs = A.sections_for(A.Build(G.load()))
    sheet = []
    for sec in secs:
        want = not args or sec['key'] in args
        fr, done[sec['key']] = render_section(sc, sec, done, still or not want)
        if not want:
            continue
        if still:
            sheet.append(fr[0])
            continue
        path = os.path.join(OUT, f"{sec['key']}.gif")
        pal = [f.convert('P', palette=Image.ADAPTIVE, colors=96) for f in fr]
        pal[0].save(path, save_all=True, append_images=pal[1:], duration=70, loop=0, optimize=True)
        print(path, f'{len(fr)} frames, {os.path.getsize(path) // 1024} kB')
    if still:
        cols = 4
        rows = (len(sheet) + cols - 1) // cols
        im = Image.new('RGB', (cols * W // 2, rows * (H + CAP) // 2), 'white')
        for i, f in enumerate(sheet):
            im.paste(f.resize((W // 2, (H + CAP) // 2)), ((i % cols) * W // 2, (i // cols) * (H + CAP) // 2))
        p = os.path.join(G.ROOT, 'reports', 'compare', 'assembly_stills.png')
        im.save(p)
        print(p)


if __name__ == '__main__':
    main()
