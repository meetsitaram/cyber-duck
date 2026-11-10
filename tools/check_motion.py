"""A recorded motion against the real parts: do they pass through each other? python tools/check_motion.py WALK.npz [every_nth_frame]

The trainer's floor tasks have no self-collision, as hulls on neighbouring links overlap, so a policy can learn a move the real robot can't make. This replays one through
the design's own solids, as tools/render_build_video.py does for the animated walk. WALK.npz: from the trainer's scripts (--traj or --trajdir). Plain python.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import make_rl_model as RL  # noqa: E402


def main():
    Z = np.load(sys.argv[1])
    every = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    q, names = Z['q'], [str(n) for n in Z['names']]
    spec = G.load()
    R = G.Robot(spec)
    S = C.Solids(spec)
    links = list(spec['links'])
    sign = {rl: (sn, s) for sn, (rl, s) in RL.JOINT.items()}
    worst, hits = {}, 0
    frames = range(0, len(q), every)
    for i in frames:
        M = R.fk({sign[jn][0]: sign[jn][1] * float(v) for jn, v in zip(names, q[i, 7:])})
        hit = False
        for a in range(len(links)):
            for b in range(a + 1, len(links)):
                la, lb = links[a], links[b]
                if R.adjacent(la, lb):
                    continue
                v = C.overlap(C.posed(S.hard[la], M[la]), C.posed(S.hard[lb], M[lb]))
                if v > C.TOUCH:
                    hit = True
                    worst[(la, lb)] = max(worst.get((la, lb), 0.0), v)
        hits += hit
    print(f'{os.path.basename(sys.argv[1])}: {hits} of {len(frames)} sampled frames have parts through each other')
    for (la, lb), v in sorted(worst.items(), key=lambda kv: -kv[1])[:10]:
        print(f'  {la} / {lb}: up to {v * 1e6:.1f} cm3')
    return 1 if hits else 0


if __name__ == '__main__':
    sys.exit(main())
