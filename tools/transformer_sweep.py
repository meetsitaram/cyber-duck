"""Transformer study: which neck / head / ankle angles tuck the duck into the lowest, longest box with nothing
colliding, sitting with its legs stretched forward. Neck angles past today's limit are tried too, flagged.

  MILLI_RECIPE=dynamixel python tools/transformer_sweep.py  -> reports/<recipe>/transformer_sweep.txt

Kinematic, the trunk upright on the floor (lowered onto whatever is lowest). Collisions: checks.py's hard solids
(rigid parts and shell walls), non-adjacent links, as check_poses. Plain python.
"""
import itertools
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402

HEAD = ('head_pitch_link', 'head_yaw_link', 'head', 'beak_link')


def frange(a, b, s):
    return [round(a + k * s, 3) for k in range(int(round((b - a) / s)) + 1)]


def main():
    spec = G.load()
    S, R = C.Solids(spec), G.Robot(spec)
    links = list(spec['links'])
    lim = {j['name']: (j['lower'], j['upper']) for j in spec['joints']}
    rows = []
    grid = itertools.product(frange(1.2, 2.6, 0.2), frange(-1.6, 1.6, 0.2), (-1.5, 0.0, 1.5), (0.0, 0.6))
    for neck, head, ankle, knee in grid:
        q = {f'{s}_{k}': v for s in 'LR' for k, v in (('hip_pitch', -1.57), ('knee', knee), ('ankle_pitch', ankle))}
        q.update(neck_pitch=neck, head_pitch=head)
        M = R.fk(q)
        posed = {ln: C.posed(S.all[ln], M[ln]) for ln in links if S.all[ln] is not None}
        lo = np.min([C.bbox(m)[0] for m in posed.values()], axis=0)
        hi = np.max([C.bbox(m)[1] for m in posed.values()], axis=0)
        H, L, W = hi[2] - lo[2], hi[0] - lo[0], hi[1] - lo[1]
        hlo = np.min([C.bbox(posed[l])[0] for l in HEAD if l in posed], axis=0)
        hhi = np.max([C.bbox(posed[l])[1] for l in HEAD if l in posed], axis=0)
        tlo, thi = C.bbox(posed['trunk'])
        hits = []
        for a, b in itertools.combinations(links, 2):
            if R.adjacent(a, b) or S.hard[a] is None or S.hard[b] is None:
                continue
            v = C.overlap(C.posed(S.hard[a], M[a]), C.posed(S.hard[b], M[b]))
            if v > C.TOUCH:
                hits.append(f'{a}/{b} {v * 1e6:.1f}cm3')
        rows.append(dict(neck=neck, head=head, ankle=ankle, knee=knee, H=H, L=L, W=W, LH=L / H,
                         head_front=hlo[0] - thi[0], head_low=hlo[2] - lo[2], head_top=hhi[2] - lo[2],
                         in_limits=lim['neck_pitch'][0] <= neck <= lim['neck_pitch'][1], hits=hits))
        print(f"neck {neck:+.1f} head {head:+.1f} ankle {ankle:+.1f} knee {knee:.1f}: H {H * 100:.1f} L {L * 100:.1f} cm "
              f"L/H {L / H:.2f} {'clear' if not hits else hits[0]}", flush=True)
    ok = sorted([r for r in rows if not r['hits']], key=lambda r: -r['LH'])
    out = G.out_path('reports', 'transformer_sweep.txt')
    with open(out, 'w', encoding='utf-8') as fh:
        fh.write(f'{len(rows)} poses, {len(ok)} with nothing colliding; best length/height first\n')
        fh.write('neck head ankle knee | H cm  L cm  W cm  L/H | head: front of trunk cm, lowest cm, top cm | neck in limits\n')
        for r in ok[:40]:
            fh.write(f"{r['neck']:+.1f} {r['head']:+.1f} {r['ankle']:+.1f} {r['knee']:.1f} | {r['H'] * 100:5.1f} {r['L'] * 100:5.1f} "
                     f"{r['W'] * 100:5.1f} {r['LH']:.2f} | {r['head_front'] * 100:+5.1f} {r['head_low'] * 100:5.1f} "
                     f"{r['head_top'] * 100:5.1f} | {'yes' if r['in_limits'] else 'NO'}\n")
    print(out)


if __name__ == '__main__':
    main()
