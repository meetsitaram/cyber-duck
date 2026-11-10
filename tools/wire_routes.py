"""Does every wire reach, in every pose? Each lead in spec/wiring_<recipe>.json's `routes`, as points on the links it passes, measured at each end of each joint it crosses, against what's bought.

  MILLI_RECIPE=hybrid python -I tools/wire_routes.py      reports/wire_routes.md (or reports/<recipe>/), exit 1 if a stock lead is too short

A point is a geom's centre by name, or x, y, z in metres on a link at the zero pose. Straight runs between points: bends add length, so `need` adds 15 % and 30 mm. A joint a wire winds round, as the head yaw, is routed through its axis, and its turning adds nothing here.
"""
import itertools
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402


def main():
    spec = G.load()
    R = G.Robot(spec)
    routes = json.load(open(os.path.join(G.ROOT, 'spec', f'wiring_{G.RECIPE}.json'), encoding='utf-8'))['routes']
    geoms = {g['name']: (ln, g) for ln, l in spec['links'].items() for g in l['geoms']}
    parent = {j['child']: j for j in spec['joints']}

    def point(p):
        if isinstance(p, str):
            ln, g = geoms[p]
            return ln, np.array(G.mesh_of(g).bounds).mean(0)
        return p[0], np.array(p[1:], float)

    def chain(ln):
        out = []
        while ln in parent:
            out.append(parent[ln]['name'])
            ln = parent[ln]['parent']
        return out
    rows, bad = [], 0
    for r in routes:
        pts = [point(p) for p in r['via']]
        # the joints between the route's links: on one link's chain to the root, but not on all of them
        chains = [set(chain(ln)) for ln, _ in pts]
        crossed = sorted(set.union(*chains) - set.intersection(*chains))
        lim = {j['name']: (j['lower'], j['upper']) for j in spec['joints']}
        worst, at = 0.0, {}
        for combo in itertools.product(*[(lim[j][0], 0.0, lim[j][1]) for j in crossed]):
            q = dict(zip(crossed, combo))
            M = R.fk(q)
            world = [(M[ln] @ np.append(p, 1))[:3] for ln, p in pts]
            d = sum(np.linalg.norm(a - b) for a, b in zip(world, world[1:]))
            if d > worst:
                worst, at = d, q
        worst += r.get('extra', 0.0)                # what a joint winds on, beyond the straight run
        need = worst * 1.15 + 0.030
        have = r.get('have')
        ok = have is None or need <= have
        bad += not ok
        rows.append((r['name'], r['what'], worst, need, have, ok, ', '.join(c.replace('_', ' ') for c in crossed) or 'none'))
    L = [f'# Wire routes: {G.RECIPE}', '', 'From `tools/wire_routes.py`. Longest straight run through the points in any pose, at each end of each joint crossed; need adds 15 % and 30 mm. Not a build: measure on one.', '',
         '| Wire | What | Longest run, mm | Need, mm | Stock, mm | Joints it crosses |', '|---|---|---|---|---|---|']
    for n, w, d, nd, h, ok, c in rows:
        L.append(f"| {'' if ok else '**TOO SHORT:** '}{n} | {w} | {d * 1000:.0f} | {nd * 1000:.0f} | {'cut to length' if h is None else f'{h * 1000:.0f}'} | {c} |")
    L += ['', f'{len(rows)} wires, {bad} failing', '']
    p = G.out_path('reports', 'wire_routes.md')
    open(p, 'w', encoding='utf-8').write('\n'.join(L))
    print(f'{len(rows)} wires, {bad} failing -> {p}')
    for n, w, d, nd, h, ok, c in rows:
        print(f"{'FAIL' if not ok else 'ok  '} {n}: {d * 1000:.0f} mm, need {nd * 1000:.0f}, stock {h}")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
