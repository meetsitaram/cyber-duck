"""An optional printed idler for the XL330, in place of ROBOTIS's FPX330-H101. Our own simple shapes, sized to the interface measured from ROBOTIS's model: same outside, same screw pattern.

  python -I tools/idler_print.py      prints/dynamixel/optional/idler_wheel.stl and idler_bushing.stl

Not proven: nothing has been printed, and a plastic wheel on a plastic bushing will wear faster than the bought one. Print a test first.
"""
import os
import sys

import numpy as np
import manifold3d as M3
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402

SEG = 96
# mm. Measured from Robotis_FPX330_H101.STEP
OD, DISC_T = 16.0, 2.5           # the wheel: fits the bearing bore and the plates as the bought one does
BOSS_D, BOSS_T = 6.5, 1.45       # the side toward the servo
RECESS_D, RECESS_T = 7.2, 2.3    # the bushing's flange sits in this
PCD, HOLE_D, HOLE_DEPTH = 12.0, 1.6, 2.9    # 4 M2 self-tappers, at 0 and 90 degrees
FLANGE_D, FLANGE_T = 6.8, 2.2
BUSH_D, BUSH_T = 4.8, 2.2
SCREW_CLEAR, HEAD_D, HEAD_T = 2.8, 5.6, 1.6  # an M2.5 screw into the servo's rear hole
# what a printer needs on top: printed holes come out small
BORE_D = 5.2                     # the wheel turns on the bushing: 0.4 mm over it, as printed holes shrink


def cyl(d, h, z=0.0, x=0.0, y=0.0):
    return M3.Manifold.cylinder(h, d / 2, d / 2, SEG).translate((x, y, z))


def wheel():
    """Lies outer face down: no supports. z = 0 is the face against the plate."""
    body = cyl(OD, DISC_T) + cyl(BOSS_D, BOSS_T, DISC_T)
    body -= cyl(RECESS_D, RECESS_T + 0.001, -0.001)
    body -= cyl(BORE_D, DISC_T + BOSS_T + 1, -0.5)
    for a in (0, 90, 180, 270):
        x, y = PCD / 2 * np.cos(np.radians(a)), PCD / 2 * np.sin(np.radians(a))
        body -= cyl(HOLE_D, HOLE_DEPTH + 0.001, -0.001, x, y)
    return body


def bushing():
    """Lies flange down."""
    body = cyl(FLANGE_D, FLANGE_T - 0.2) + cyl(BUSH_D, BUSH_T + 0.2, FLANGE_T - 0.2)
    body -= cyl(SCREW_CLEAR, FLANGE_T + BUSH_T + 1, -0.5)
    body -= cyl(HEAD_D, HEAD_T + 0.001, -0.001)
    return body


def main():
    out = os.path.join(G.ROOT, 'prints', 'dynamixel', 'optional')
    os.makedirs(out, exist_ok=True)
    for name, m in (('idler_wheel', wheel()), ('idler_bushing', bushing())):
        mesh = m.to_mesh()
        t = trimesh.Trimesh(np.asarray(mesh.vert_properties[:, :3], float), np.asarray(mesh.tri_verts, int))
        t.export(os.path.join(out, f'{name}.stl'))
        print(f'{name}.stl  {t.extents.round(2).tolist()} mm  {t.volume / 1000:.3f} cm3  {t.volume / 1000 * 1.24:.2f} g  watertight {t.is_watertight}  bodies {len(t.split(only_watertight=False))}')
    # edge check: plastic left round each screw hole
    print(f'plastic outside each M2 hole: {(OD - PCD - HOLE_D) / 2:.2f} mm; between hole and recess: {(PCD - HOLE_D - RECESS_D) / 2:.2f} mm')


if __name__ == '__main__':
    main()
