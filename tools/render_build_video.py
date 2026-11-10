"""One short video of the whole build, for sharing: both legs side by side at once, then the body, then the head, then the
finished robot walking and bending its head. Parts, order and paths are tools/assembly.py's. Each part's name stacks up on
the left as it goes on, and fades; Feetech and Dynamixel servos have a colour each, on the label and on the servo as it goes in.
docs/images/assembly/milli_build.mp4

The walk is kinematic: a gait from the leg's own inverse kinematics, the stance foot planted, every sampled frame tested
for collisions and joint limits. It shows the range of motion. It is not a physics simulation or a trained policy.

  python tools/render_build_video.py          the video
  python tools/render_build_video.py --test   a contact sheet of frames instead, in reports/compare

Plain python (MuJoCo lives in the user site-packages).
"""
import os
import re
import sys

import imageio.v2 as imageio
import mujoco
import numpy as np
from scipy.optimize import least_squares
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly as A  # noqa: E402
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import render_assembly as RA  # noqa: E402

FPS, W, H = 30, 1280, 720
PER_BEAT = 8                                       # frames per build step
STEP_T, STEP_LEN, LIFT, SWAY, N_STEPS = 0.5, 0.036, 0.016, 0.07, 10   # the gait: s, m, m, rad of hip roll, steps
FLY, SCREW_FLY = 0.04, 0.016
VIEW = (35, 14)
FEETECH, DYNAMIXEL = (30, 120, 220), (30, 165, 90)
MAX_LABELS = 20                                    # as many as fit above the caption
LABEL_LIFE, LABEL_FADE = 90, 30                    # frames a label stays, then fades (3 s, 1 s)
TINT_LIFE, TINT_FADE = 40, 25                      # frames a servo keeps its colour, then returns to black
RA.W, RA.H = W, H


def leg_home(s):
    """Where each bench section of a leg is built, from its place on the robot (the leg itself waits in front)."""
    sg = 1 if s == 'L' else -1
    return {f'thigh_{s}': np.zeros(3), f'shin_{s}': np.array([0, -sg * 0.02, -0.05]), f'shoe_{s}': np.array([0, 0, -0.11])}


WAIT = np.array([0.075, 0, 0])                     # the finished legs wait here while the body is built


def timeline(sc):
    """[(label, moves)], a beat each. A move: (items, from offset, to offset). Items appear when first moved."""
    secs = {s['key']: s for s in A.sections()}
    screws = lambda pats: [x for p in pats for x in sc.B.screws if re.search(p, x)]
    built = {}

    def items_of(key):
        out = [p for o in secs[key]['on'] for p in items_of(o)]   # (a section carried in brings what it was built on)
        for cap, its, pats in secs[key]['steps']:
            for it, _ in its:
                out += items_of(it[1:]) if it.startswith('@') else [it]
            out += screws(pats)
        return out

    def beats(key, home):
        res = []
        for cap, its, pats in secs[key]['steps']:
            mv = []
            for it, way in its:
                if it.startswith('@'):
                    mv.append((items_of(it[1:]), built[it[1:]], home))
                else:
                    v = np.array(way if way is not None else (0, 0, 1.0), float) * FLY
                    mv.append(([it], home - v, home))
            for n in screws(pats):
                v = np.array(sc.B.screws[n][1]['fast']['d'], float) * SCREW_FLY
                mv.append(([n], home - v, home))
            if mv:
                res.append(mv)
        built[key] = home
        return res

    tl = []
    for keys in (('thigh', 'thigh'), ('shin', 'shin'), ('shoe', 'shoe'), ('leg', None)):
        L_, R_ = (f'{keys[0]}_L', f'{keys[0]}_R')
        a = beats(L_, leg_home('L').get(L_, np.zeros(3)) + WAIT)
        b = beats(R_, leg_home('R').get(R_, np.zeros(3)) + WAIT)
        for x, y in zip(a, b):
            tl.append(('Two legs, side by side', x + y))
    for mv in beats('body', np.zeros(3)):
        tl.append(('The body', mv))
    tl.append(('Legs onto the body', [(items_of('leg_L') + items_of('leg_R'), WAIT, np.zeros(3))]))
    tl.append(('Legs onto the body', [(screws([r'^[LR] hip roll fork screw']), None, None)]))
    built['legs'] = np.zeros(3)
    up = np.array([0, 0, 0.09])
    for mv in beats('column', up):
        tl.append(('The neck', mv))
    secs['torso']['steps'] = secs['torso']['steps']
    for mv in beats('torso', np.zeros(3)):
        tl.append(('The neck and body shell', mv))
    for mv in beats('headbench', np.array([0, 0, 0.10])):
        tl.append(('The head', mv))
    for mv in beats('head', np.zeros(3)):
        tl.append(('The head', mv))
    return tl


def font(size):
    try:
        return ImageFont.truetype('arialbd.ttf', size)
    except OSError:
        return ImageFont.load_default()


def beat_labels(sc, spec, mv):
    """[(text, colour)] for the parts of one beat: both sides counted once, screws by size."""
    parts, screws = {}, {}
    for items, _, _ in mv:
        if len(items) > 1:                         # a section carried in: already named as it was built
            continue
        for n in items:
            if n in sc.B.screws:
                f = sc.B.screws[n][1]['fast']
                k = f"{f['size']} x {f['L'] * 1000:.0f} screw"
                screws[k] = screws.get(k, 0) + 1
            else:
                short = n[2:] if n[:2] in ('L ', 'R ') else n
                parts[short] = (parts.get(short, (0, n))[0] + 1, n)
    out = []
    for short, (c, n) in parts.items():
        act = [g.split()[-1] for g in sc.B.parts[n]['geoms'] if 'actuator (buy)' in g]
        x = f' x{c}' if c > 1 else ''
        if act:
            where = short.replace(' servo', '')
            if act[0].startswith('STS'):
                out.append((f"Feetech {act[0].replace('_', '-')}: {where}{x}", FEETECH))
            else:
                out.append((f"Dynamixel {act[0].replace('_', '-')}: {where}{x}", DYNAMIXEL))
        elif 'bearing' in short:
            b = spec['bearings']['ankle' if 'ankle' in short else ('STS3215' if 'hip' in short or 'knee' in short else 'XL330')]['part']
            out.append((f"{b} bearing: {short.replace(' bearing', '').replace(' inner', '')}{x}", (50, 54, 62)))
        elif 'horn' in short:                      # (screws and horns would only bury the servos' names)
            continue
        elif sc.B.parts[n]['kind'] == 'P':
            out.append((f'Printed: {short}{x}', (50, 54, 62)))
        else:
            out.append((f'{short}{x}', (50, 54, 62)))
    return out


def label(img, text, stack, now):
    """The caption, the servo legend, and the stack of labels on the left."""
    im = Image.fromarray(img).convert('RGBA')
    ov = Image.new('RGBA', im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    d.text((44, H - 68), text, fill=(30, 34, 42, 255), font=font(32))
    f2 = font(20)
    for k, (t, col) in enumerate((('Feetech STS3215', FEETECH), ('Dynamixel XC330 / XL330', DYNAMIXEL))):
        x, y = W - 300, 26 + 32 * k
        d.ellipse((x, y + 3, x + 18, y + 21), fill=(*col, 255))
        d.text((x + 28, y), t, fill=(*col, 255), font=f2)
    live = [e for e in stack if now - e['born'] < LABEL_LIFE + LABEL_FADE][-MAX_LABELS:]
    stack[:] = live
    for k, e in enumerate(live):
        e['y'] += (26 + 28 * k - e['y']) * 0.25
        age = now - e['born']
        a = 255 if age < LABEL_LIFE else int(255 * (1 - (age - LABEL_LIFE) / LABEL_FADE))
        servo = e['col'] in (FEETECH, DYNAMIXEL)
        if servo:
            d.ellipse((40, e['y'] + 3, 56, e['y'] + 19), fill=(*e['col'], a))
        d.text((64 if servo else 44, e['y']), e['text'], fill=(*e['col'], a), font=f2)
    return np.asarray(Image.alpha_composite(im, ov).convert('RGB'))


def leg_ik(R, side, dx, dz, q0):
    """Hip pitch, knee, ankle putting this foot at (dx, dz) from its zero-pose place, flat."""
    names = [f'{side}_hip_pitch', f'{side}_knee', f'{side}_ankle_pitch']

    def res(x):
        M = R.fk(dict(zip(names, x)))[f'{side}_foot']
        return [M[0, 3] - dx, M[2, 3] - dz, M[0, 2]]
    x = least_squares(res, q0, xtol=1e-10).x
    return dict(zip(names, x)), x


def gait(R):
    """[(q, stance foot, its world pose)] per frame. The body leans on the stance leg through both hip rolls, which lifts
    the other side: there is no ankle roll, as on microduck."""
    n = int(STEP_T * FPS)
    out = []
    x_l, x_r = np.zeros(3), np.zeros(3)
    guess = {'L': np.zeros(3), 'R': np.zeros(3)}
    # which roll sign lifts the right foot while standing on the left
    M = R.fk(dict(L_hip_roll=0.05, R_hip_roll=0.05))
    lift = (np.linalg.inv(M['L_foot']) @ M['R_foot'])[2, 3]
    sg = 1 if lift > 0 else -1
    foot_x = {'L': 0.0, 'R': 0.0}
    for st in range(N_STEPS):
        stance, swing = ('L', 'R') if st % 2 == 0 else ('R', 'L')
        first, last = st == 0, st == N_STEPS - 1
        for k in range(n):
            t = k / n
            a = STEP_LEN / 2
            # the body travels over the stance foot; half steps to start and to stop
            s_from, s_to = (0.0, -a) if first else ((a, 0.0) if last else (a, -a))
            w_from, w_to = (0.0, a) if first else ((-a, 0.0) if last else (-a, a))
            e = t * t * (3 - 2 * t) if first or last else t
            sx = s_from + (s_to - s_from) * e
            wx = w_from + (w_to - w_from) * e
            wz = LIFT * np.sin(np.pi * t) ** 1.2
            q = {}
            d, guess[stance] = leg_ik(R, stance, sx, 0.0, guess[stance])
            q.update(d)
            d, guess[swing] = leg_ik(R, swing, wx, wz, guess[swing])
            q.update(d)
            r = sg * (1 if stance == 'L' else -1) * SWAY * np.sin(np.pi * t)
            q['L_hip_roll'] = q['R_hip_roll'] = r
            q['head_pitch'] = -0.05 * np.sin(np.pi * t)
            place = foot_x[stance]
            out.append((q, stance, place))
        foot_x[swing] = foot_x[stance] + (STEP_LEN / 2 if first else (0.0 if last else STEP_LEN))
    return out


HEAD = [(0.0, {}), (0.35, dict(beak=0.4)), (0.7, {}), (1.05, dict(beak=0.4)), (1.4, {}), (2.1, dict(head_yaw=0.9)), (2.7, dict(head_yaw=-0.9, head_roll=0.1)), (3.3, dict(neck_pitch=0.9, head_pitch=0.3)),
        (4.0, dict(neck_pitch=1.55, head_pitch=-1.45)), (4.7, {}), (5.0, {})]


def head_at(t):
    for (t0, a), (t1, b) in zip(HEAD, HEAD[1:]):
        if t <= t1:
            u = (t - t0) / (t1 - t0)
            u = u * u * (3 - 2 * u)
            return {k: a.get(k, 0.0) + (b.get(k, 0.0) - a.get(k, 0.0)) * u for k in set(a) | set(b)}
    return {}


def motion(R):
    """[(q, stance, foot x, caption)] for the whole moving part."""
    g = [(q, s, x, 'Walking') for q, s, x in gait(R)]
    end = g[-1][2] if g else 0.0
    end_stance = g[-1][1]
    for k in range(int(HEAD[-1][0] * FPS)):
        g.append((head_at(k / FPS), end_stance, end, 'Talking' if k / FPS < 1.4 else 'Looking round, bending its head'))
    return g


def link_poses(R, q, stance, x):
    M = R.fk(q)
    F = np.eye(4)
    F[0, 3] = x
    W = F @ np.linalg.inv(M[f'{stance}_foot'])
    return {ln: W @ m for ln, m in M.items()}


def check_motion(spec, R, S, mo):
    """Joint limits on every frame; collisions between links not next to each other, and the floor, on every 3rd."""
    links = list(spec['links'])
    bad, worst = [], 0.0
    for k, (q, st, x, _) in enumerate(mo):
        for n, v in q.items():
            if not R.joints[n]['lower'] - 1e-6 <= v <= R.joints[n]['upper'] + 1e-6:
                bad.append(f'frame {k}: {n} {v:+.2f} outside its limits')
        if k % 3:
            continue
        P = link_poses(R, q, st, x)
        for a in range(len(links)):
            for b in range(a + 1, len(links)):
                la, lb = links[a], links[b]
                if R.adjacent(la, lb):
                    continue
                v = C.overlap(C.posed(S.hard[la], P[la]), C.posed(S.hard[lb], P[lb]))
                worst = max(worst, v)
                if v > C.TOUCH:
                    bad.append(f'frame {k} ({mo[k][3]}): {la} / {lb} {v * 1e6:.2f} cm3')
        for foot in ('L_foot', 'R_foot'):
            z = C.bbox(C.posed(S.all[foot], P[foot]))[0][2]
            if z < -0.0015:
                bad.append(f'frame {k}: {foot} {z * 1000:.1f} mm through the floor')
    return bad


def quat(Rm):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, Rm.flatten())
    return q


def run(test=False):
    spec = G.load()
    R = G.Robot(spec)
    tiles = ''.join(
        f'<geom type="box" pos="{-2 + 0.1 * i + 0.05:.2f} {-2 + 0.1 * j + 0.05:.2f} -0.001" size="0.05 0.05 0.001" '
        f'rgba="{".95 .95 .95" if (i + j) % 2 else ".88 .88 .88"} 1"/>' for i in range(40) for j in range(40))
    sc = RA.Scene(spec, extra=f'<body name="floor" mocap="true">{tiles}</body>')
    floor_mc = sc.m.body_mocapid[sc.m.body('floor').id]
    mo = motion(R)
    bad = check_motion(spec, R, sc.B.S, mo)
    print(f'motion: {len(mo)} frames, {len(bad)} problems')
    for b_ in bad[:12]:
        print('  ', b_)
    tl = timeline(sc)
    ibox = {n: RA.bounds(sc, [n]) for n in sc.names}
    link_of = {}
    for n in sc.names:
        if n in sc.B.parts:
            link_of[n] = sc.B.idx[sc.B.parts[n]['geoms'][0]][0]
        else:
            link_of[n] = sc.B.screws[n][0]
    off, shown = {}, set()
    cen, size = None, None
    nframes, sheet = 0, []
    d_ = os.path.join(G.ROOT, 'docs', 'images', 'assembly')
    w1 = None
    if not test:
        kw = dict(fps=FPS, codec='libx264', quality=8, macro_block_size=1, pixelformat='yuv420p')
        w1 = imageio.get_writer(os.path.join(d_, 'milli_build.mp4'), **kw)
    total = len(tl) * PER_BEAT + len(mo)
    print(f'{len(tl)} beats, {total / FPS:.1f} s')
    every = max(1, total // 20)

    def target():
        lo, hi = np.full(3, 9.0), np.full(3, -9.0)
        for n in shown:
            o = np.array(off.get(n, (0, 0, 0)))
            lo, hi = np.minimum(lo, ibox[n][0] + o), np.maximum(hi, ibox[n][1] + o)
        return (lo + hi) / 2, max(float(np.linalg.norm(hi - lo)) * 0.95, 0.20)

    stack, tinted = [], {}
    base = sc.m.geom_rgba.copy()

    def tint(now):
        for n, (col, born) in list(tinted.items()):
            b_ = sc.bid[n]
            ga, gn = sc.m.body_geomadr[b_], sc.m.body_geomnum[b_]
            age = now - born
            u = 1.0 if age < TINT_LIFE else max(0.0, 1 - (age - TINT_LIFE) / TINT_FADE)
            sc.m.geom_rgba[ga:ga + gn, :3] = base[ga:ga + gn, :3] * (1 - u) + np.array(col) / 255 * u
            if u == 0:
                del tinted[n]

    def out(img, text):
        nonlocal nframes
        img = label(img, text, stack, nframes)
        if w1:
            w1.append_data(img)
        nframes += 1
        if test and nframes % every == 0:
            sheet.append(Image.fromarray(img).resize((W // 3, H // 3)))

    def shifted(c, sz, az, u):
        """The look-at point moved so the robot sits right of the labels (u: how much)."""
        a_ = np.radians(az)
        right = np.array([-np.sin(a_), np.cos(a_), 0.0])
        return np.asarray(c) - right * sz * (W / H) * 0.13 * u

    def emit(az, text):
        nonlocal cen, size
        c, s_ = target()
        cen = c if cen is None else cen + 0.06 * (c - cen)
        size = s_ if size is None else size + 0.06 * (s_ - size)
        tint(nframes)
        out(sc.frame(shown, off, (az, VIEW[1]), (shifted(cen, size, az, 1.0).tolist(), size)), text)

    sc.d.mocap_pos[floor_mc] = (50, 50, 50)
    for text, mv in tl:
        if text == 'Legs onto the body':           # the feet reach the floor here
            sc.d.mocap_pos[floor_mc] = (0, 0, 0)
        for t_, col in beat_labels(sc, spec, mv):
            stack.append(dict(text=t_, col=col, born=nframes, y=26 + 28 * len(stack)))
        for items, _, _ in mv:
            for n in items:
                if len(items) == 1 and n in sc.B.parts and any('actuator (buy)' in g for g in sc.B.parts[n]['geoms']):
                    tinted[n] = (FEETECH if any('STS3215' in g for g in sc.B.parts[n]['geoms']) else DYNAMIXEL, nframes)
        for k in range(PER_BEAT):
            t = min(1.0, k / max(1, PER_BEAT - 2))
            e = t * t * (3 - 2 * t)
            for items, a, b_ in mv:
                for n in items:
                    shown.add(n)
                    off[n] = (0, 0, 0) if a is None else tuple(a + (b_ - a) * e)
            emit(VIEW[0], text)
    # the finished robot: every part in its place, or the picture is wrong
    astray = [n for n in sc.names if n not in shown or np.linalg.norm(off[n]) > 1e-9]
    if astray:
        raise SystemExit(f'not in place at the end of the build: {astray[:8]}')
    # it moves, straight away; the camera follows
    c0 = np.array(cen)
    s0 = size
    for k, (q, st, x, cap) in enumerate(mo):
        P = link_poses(R, q, st, x)
        pos, qu = {}, {}
        for n in sc.names:
            M = P[link_of[n]]
            pos[n], qu[n] = tuple(M[:3, 3]), quat(M[:3, :3])
        body_x = P['trunk'][0, 3]
        u = min(1.0, k / 20)
        c = c0 + (np.array([body_x, 0, 0.16]) - c0) * u
        s_ = s0 + (0.40 - s0) * u
        tint(nframes)
        out(sc.frame(shown, pos, VIEW, (shifted(c, s_, VIEW[0], max(0.0, 1 - k / 45)).tolist(), s_), qu), cap)
    if w1:
        w1.close()
    print(f'{nframes / FPS:.1f} s')
    if test:
        cols = 5
        rows = (len(sheet) + cols - 1) // cols
        im = Image.new('RGB', (cols * W // 3, rows * H // 3), 'white')
        for i, f in enumerate(sheet):
            im.paste(f, ((i % cols) * W // 3, (i // cols) * H // 3))
        p_ = os.path.join(G.ROOT, 'reports', 'compare', 'build_video_frames.png')
        im.save(p_)
        print(p_)


if __name__ == '__main__':
    run('--test' in sys.argv)
