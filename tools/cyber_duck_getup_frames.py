"""Cyber Duck: solves extra getting-up frames at given leans (CYBER_FRAMES="lean:bill:feet,..." with 1/0 flags),
started from tools/cyber_duck.py's GET_UP at that lean; prints GET_UP lines. Statics only, as cyber_duck_getup_solve.

  CYBER_FRAMES="0.875:1:1,0.72:1:1" MILLI_RECIPE=dynamixel python tools/cyber_duck_getup_frames.py
"""
import os
import sys

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import memcap  # noqa: E402
import cyber_duck as CD  # noqa: E402
import cyber_duck_getup_solve as GS  # noqa: E402


def init_at(lean):
    """The GET_UP pose at this lean (interpolated), as [hip, knee, ankle, neck, head]."""
    for (u0, l0, q0), (u1, l1, q1) in zip(CD.GET_UP, CD.GET_UP[1:]):
        if min(l0, l1) <= lean <= max(l0, l1) and l0 != l1:
            w = (lean - l0) / (l1 - l0)
            g = lambda k: (1 - w) * q0.get(k, 0.0) + w * q1.get(k, 0.0)
            return u0 + w * (u1 - u0), np.array([g('L_hip_pitch'), g('L_knee'), g('L_ankle_pitch'), g('neck_pitch'), g('head_pitch')])
    raise ValueError(lean)


def main():
    memcap.guard(cap_gb=6, need_free_gb=16)
    env = GS.setup()
    for spec in os.environ['CYBER_FRAMES'].split(','):
        lean, bill, feet = spec.split(':')
        lean, bill, feet = float(lean), bill == '1', feet == '1'
        u, xi = init_at(lean)
        f = lambda xx: GS.evaluate(env, lean, xx, bill, feet)[0] + 0.03 * np.abs(xx - xi).sum()
        best = None
        for d in ([0, 0, 0, 0, 0], [0, 0.2, 0, 0.2, 0.15], [-0.1, 0.4, 0, 0.4, 0.3], [0, -0.2, 0, -0.2, -0.2]):
            r = minimize(f, xi + np.array(d), method='Nelder-Mead', options=dict(maxiter=600, xatol=1e-3, fatol=1e-5))
            if best is None or r.fun < best.fun:
                best = r
        x = best.x
        cost, mg, zb, zf, q = GS.evaluate(env, lean, x, bill, feet)
        print(f'lean {lean:.3f} (u {u:.3f}): margin {1000 * mg:+.1f} mm, bill {1000 * zb:.1f} mm, feet {1000 * zf:.1f} mm off', flush=True)
        print(f"    ({u:.3f}, {lean:.3f}, dict(_LEGS({x[0]:+.3f}, {x[1]:+.3f}, {x[2]:+.3f}), neck_pitch={x[3]:+.3f}, head_pitch={x[4]:+.3f})),", flush=True)


if __name__ == '__main__':
    main()
