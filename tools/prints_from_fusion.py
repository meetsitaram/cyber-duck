"""Replace prints/<variant>/*.stl with what the Fusion model actually is: its fillets and curved covers, which the offline meshes don't have.

  python -I tools/prints_from_fusion.py          (after cad/fusion/full_build.py has written its raw STLs; MILLI_RECIPE picks the variant)

Each body is turned as tools/export_prints.py chose to print it (print_T in spec/parts_<variant>.json). A file is replaced only if it is one piece, watertight
and within 12 % of the offline volume: rounding takes some away, and more than that means the body went wrong. Anything refused keeps its offline STL, and is named.
"""
import json
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402


def main():
    P = json.load(open(G.parts_path('parts'), encoding='utf-8'))['parts']
    out = G.out_path('prints')
    raw = os.path.join(os.environ['TEMP'], 'duck_fusion_stl', G.RECIPE)      # (the hybrid's prints are in prints/hybrid/ like every duck's)
    done, refused = 0, []
    for part, p in P.items():
        f = os.path.join(raw, part.replace(' ', '_') + '.stl')
        if not os.path.exists(f):
            refused.append((part, 'no STL from Fusion'))
            continue
        m = trimesh.load(f, force='mesh')
        m.merge_vertices()
        lumps = m.split(only_watertight=False)
        keep = [q for q in lumps if abs(q.volume) > 1.0]         # mesh dust out: bits under 1 mm3 that Fusion's mesher leaves at a sliver
        if keep:
            m = trimesh.util.concatenate(keep)
            m.merge_vertices()
        v = abs(m.volume) / 1000
        pieces = len(keep)
        off = abs(v - p['volume_cm3']) / p['volume_cm3']
        if not m.is_watertight or pieces != 1 or off > 0.12:
            refused.append((part, f'watertight {m.is_watertight}, {pieces} pieces, {off * 100:.0f} % off'))
            continue
        m.apply_transform(np.array(p['print_T']))
        m.apply_translation(-m.bounds[0])
        for _try in range(6):                      # (Windows refuses a write now and then, OSError 22: try again)
            try:
                m.export(os.path.join(out, part.replace(' ', '_') + '.stl'))
                break
            except OSError:
                if _try == 5:
                    raise
                __import__('time').sleep(1.0)
        done += 1
    for r in refused:
        print('KEPT OFFLINE', r[0], r[1])
    print(f'{done} of {len(P)} prints now from Fusion -> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
