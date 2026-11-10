"""Cyber Duck on an office desk: stainless shells, shadows, a reflective desk top, a wall behind, a few desk props.
Stills of the truck and the duck, and the truck -> duck clip, rendered with MuJoCo (perspective camera, shadows,
planar reflection on the desk top, 8x multisampling).

  MILLI_RECIPE=dynamixel python tools/cyber_duck.py          (first: the painted shells, reports/<recipe>/cyber_duck/painted.pkl)
  MILLI_RECIPE=dynamixel python tools/cyber_duck_render.py   -> reports/<recipe>/cyber_duck/desk_*.png, desk_transform.mp4
  ... --stills                                              stills only

The ending: it stands, chats, and walks; the covers come off as it starts walking (the user, 2026-11-10) and the bare
skeleton walks on. The walk is the repo's kinematic gait (tools/render_build_video.py): all of it is kinematic, the
moves, not physics. MuJoCo has no environment maps: the steel reads from its
highlights and the room's lights, not from mirrored surroundings (that needs a path tracer). Plain python.
"""
import math
import os
import pickle
import sys
import tempfile

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import memcap  # noqa: E402
import cyber_duck as CD  # noqa: E402
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

W, H = 1080, 1920                           # portrait (the user, 2026-11-10), as the reveal videos
FPS = 20

# finishes: stainless (bright, tight highlights), glass (dark, glossy), cladding (matte), lights (glowing)
MATERIALS = {
    CD.STEEL: dict(rgba='0.66 0.645 0.615 1', specular=1.0, shininess=0.92, reflectance=0.0, emission=0.0),   # the photo's stainless: a touch warm
    CD.GLASS: dict(rgba='0.10 0.13 0.16 0.55', specular=1.0, shininess=1.0, reflectance=0.0, emission=0.0),   # tinted, see-through
    CD.BLACK: dict(rgba='0.10 0.10 0.11 1', specular=0.15, shininess=0.2, reflectance=0.0, emission=0.0),
    CD.TYRE: dict(rgba='0.05 0.05 0.06 1', specular=0.05, shininess=0.1, reflectance=0.0, emission=0.0),
    CD.HUB: dict(rgba='0.35 0.37 0.40 1', specular=0.8, shininess=0.8, reflectance=0.0, emission=0.0),
    CD.RIM: dict(rgba='0.16 0.17 0.19 1', specular=0.6, shininess=0.7, reflectance=0.0, emission=0.0),
    CD.LIGHT: dict(rgba='0.95 0.97 1.0 1', specular=0.3, shininess=0.5, reflectance=0.0, emission=1.0),
    CD.RED: dict(rgba='0.85 0.05 0.12 1', specular=0.3, shininess=0.5, reflectance=0.0, emission=0.8),
}


def wood_texture(path, w=1024, h=512, seed=3):
    """A procedural oak top: long grain lines, a little figure, a warm tone."""
    rng = np.random.default_rng(seed)
    y = np.linspace(0, 1, h)[:, None]
    x = np.linspace(0, 1, w)[None, :]
    wobble = 0.003 * np.sin(2 * np.pi * (x * 1.3 + rng.random())) + 0.001 * np.sin(2 * np.pi * x * 7)
    grain = np.sin(2 * np.pi * 140 * (y + wobble) + 0.8 * np.sin(2 * np.pi * 5 * y)) * 0.5 + 0.5
    fine = rng.random((h, w)) * 0.5
    fine = (fine + np.roll(fine, 1, 1) + np.roll(fine, 2, 1) + np.roll(fine, 3, 1)) / 4      # streaks along the grain
    t = 0.35 * grain ** 4 + 0.30 * fine + 0.10
    base, dark = np.array([186, 152, 116]), np.array([132, 100, 70])
    img = (dark[None, None] * t[..., None] + base[None, None] * (1 - t[..., None])).clip(0, 255).astype(np.uint8)
    Image.fromarray(img).save(path)


def room_xml(tmp):
    """The desk, the walls and the props, as MuJoCo XML (assets, bodies). The desk top is z = 0."""
    wood_texture(os.path.join(tmp, 'wood.png'))
    assets = f"""
<texture name="wood" type="2d" file="{os.path.join(tmp, 'wood.png')}"/>
<texture name="sky" type="skybox" builtin="gradient" rgb1="0.86 0.85 0.82" rgb2="0.55 0.56 0.58" width="256" height="256"/>
<material name="desk" texture="wood" texrepeat="1.6 1" specular="0.3" shininess="0.55" reflectance="0.08"/>
<material name="deskedge" texture="wood" texrepeat="4 1" specular="0.2" shininess="0.3"/>
<material name="wall" rgba="0.83 0.81 0.77 1" specular="0.05" shininess="0.1"/>
<material name="wall2" rgba="0.78 0.79 0.78 1" specular="0.05" shininess="0.1"/>
<material name="skirting" rgba="0.95 0.95 0.93 1" specular="0.2" shininess="0.3"/>
<material name="floor" rgba="0.35 0.33 0.31 1" specular="0.1" shininess="0.2"/>
<material name="plastic_dark" rgba="0.08 0.08 0.09 1" specular="0.35" shininess="0.6"/>
<material name="screen" rgba="0.10 0.14 0.22 1" specular="0.9" shininess="0.95" emission="0.25"/>
<material name="keycap" rgba="0.16 0.16 0.17 1" specular="0.35" shininess="0.5"/>
<material name="notebook" rgba="0.10 0.14 0.26 1" specular="0.25" shininess="0.4"/>
<material name="pages" rgba="0.92 0.90 0.84 1" specular="0.05" shininess="0.1"/>
<material name="pen" rgba="0.08 0.08 0.09 1" specular="0.8" shininess="0.9"/>
<material name="mug" rgba="0.93 0.92 0.88 1" specular="0.5" shininess="0.7"/>
<material name="coffee" rgba="0.18 0.10 0.05 1" specular="0.6" shininess="0.8"/>
<material name="pot" rgba="0.75 0.40 0.25 1" specular="0.2" shininess="0.3"/>
<material name="leaf" rgba="0.20 0.45 0.22 1" specular="0.15" shininess="0.3"/>
<material name="paper" rgba="0.97 0.97 0.95 1" specular="0.05" shininess="0.1"/>
"""
    bodies = """
<geom name="desktop" type="plane" size="0.75 0.42 0.01" pos="0 0.05 0" material="desk"/>
<geom type="box" size="0.75 0.42 0.015" pos="0 0.05 -0.0151" material="deskedge"/>
<geom type="box" size="0.03 0.03 0.36" pos="-0.70 0.42 -0.39" material="plastic_dark"/>
<geom type="box" size="0.03 0.03 0.36" pos="0.70 0.42 -0.39" material="plastic_dark"/>
<geom type="plane" size="3 3 0.01" pos="0 0 -0.75" material="floor"/>
<geom type="box" size="2.0 0.01 1.4" pos="0 0.52 0.5" material="wall"/>
<geom type="box" size="2.0 0.012 0.04" pos="0 0.508 -0.71" material="skirting"/>
<geom type="box" size="0.01 2.0 1.4" pos="-1.7 0 0.5" material="wall2"/>
<!-- a monitor, back left -->
<geom type="box" size="0.24 0.012 0.15" pos="-0.33 0.36 0.27" material="plastic_dark"/>
<geom type="box" size="0.228 0.002 0.138" pos="-0.33 0.347 0.27" material="screen"/>
<geom type="box" size="0.02 0.02 0.07" pos="-0.33 0.385 0.07" material="plastic_dark"/>
<geom type="box" size="0.09 0.06 0.006" pos="-0.33 0.38 0.006" material="plastic_dark"/>
<!-- a dark keyboard with keys, front left -->
<geom type="box" size="0.19 0.06 0.009" pos="-0.36 0.10 0.009" material="plastic_dark"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.5250 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4996 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4742 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4488 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4234 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3980 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3726 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3472 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3218 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2964 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2710 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2456 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2202 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.1948 0.0650 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.5250 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4996 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4742 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4488 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4234 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3980 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3726 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3472 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3218 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2964 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2710 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2456 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2202 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.1948 0.0885 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.5250 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4996 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4742 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4488 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4234 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3980 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3726 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3472 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3218 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2964 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2710 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2456 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2202 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.1948 0.1120 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.5250 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4996 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4742 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4488 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.4234 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3980 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3726 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3472 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.3218 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2964 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2710 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2456 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.2202 0.1355 0.022" material="keycap"/>
<geom type="box" size="0.0105 0.0098 0.004" pos="-0.1948 0.1355 0.022" material="keycap"/>
<!-- a mug, right -->
<geom type="cylinder" size="0.042 0.05" pos="0.38 0.22 0.05" material="mug"/>
<geom type="cylinder" size="0.037 0.002" pos="0.38 0.22 0.098" material="coffee"/>
<geom type="capsule" size="0.008 0.025" pos="0.428 0.22 0.05" material="mug"/>
<!-- a small plant, back right -->
<geom type="cylinder" size="0.045 0.045" pos="0.52 0.36 0.045" material="pot"/>
<geom type="ellipsoid" size="0.07 0.06 0.08" pos="0.52 0.36 0.15" material="leaf"/>
<geom type="ellipsoid" size="0.05 0.05 0.06" pos="0.49 0.33 0.20" material="leaf"/>
<!-- a closed notebook and a pen, front right -->
<geom type="box" size="0.074 0.104 0.006" pos="0.30 0.02 0.006" euler="0 0 12" material="notebook"/>
<geom type="box" size="0.071 0.101 0.0045" pos="0.302 0.022 0.006" euler="0 0 12" material="pages"/>
<geom type="box" size="0.074 0.104 0.0008" pos="0.30 0.02 0.0124" euler="0 0 12" material="notebook"/>
<geom type="capsule" size="0.0045 0.065" pos="0.40 0.03 0.0045" euler="90 0 20" material="pen"/>
"""
    return assets, bodies


class DeskScene(CD.Scene):
    """CD.Scene's parts, posed and lowered onto the desk, in the room, lit for shadows and reflections."""

    def xml(self, q, tmp, offset=(0.0, 0.0), exact=None, cover=1.0):
        if exact is not None:                      # the walk: the recorded root, the feet already on the floor
            M = self.R.fk(q, base=exact)
        else:
            M = self.R.fk(q, base=self.base)
            low = min(G.apply(M[ln], v)[:, 2].min() for ln, v, f, c, r in self.parts if len(f))
            lift = np.eye(4)
            lift[:3, 3] = [offset[0], offset[1], -low]
            M = self.R.fk(q, base=lift @ self.base)
        r_assets, r_bodies = room_xml(tmp)
        assets, geoms, mats = [], [], set()
        for k, (ln, v, f, color, role) in enumerate(self.parts):
            if len(f) == 0:
                continue
            RP.stl(os.path.join(tmp, f'p{k}.stl'), G.apply(M[ln], v), f)
            assets.append(f'<mesh name="p{k}" file="p{k}.stl"/>')
            if color in MATERIALS:                  # the covers: fading out as it starts walking
                if cover <= 0.01:
                    continue
                mname = 'm' + color.lstrip('#')
                if mname not in mats:
                    a = MATERIALS[color]
                    rgba = a['rgba'].split()
                    rgba[3] = f'{float(rgba[3]) * cover:.3f}'
                    a = dict(a, rgba=' '.join(rgba))
                    assets.append(f'<material name="{mname}" rgba="{a["rgba"]}" specular="{a["specular"]}" '
                                  f'shininess="{a["shininess"]}" reflectance="{a["reflectance"]}" emission="{a["emission"]}"/>')
                    mats.add(mname)
                geoms.append(f'<geom type="mesh" mesh="p{k}" material="{mname}"/>')
            else:                                   # the duck's own parts (the user: motors black, other joint parts grey)
                if role == 'actuator':
                    mname, spec_ = 'motor', 'rgba="0.04 0.04 0.045 1" specular="0.7" shininess="0.85"'
                elif ln in ('L_foot', 'R_foot') and color.upper() == '#F2B233':   # the sole: black rubber
                    mname, spec_ = 'sole', 'rgba="0.05 0.05 0.055 1" specular="0.1" shininess="0.2"'
                elif ln in ('L_foot', 'R_foot') and color.upper() == '#E8642A':   # the shoe: stainless, as the shells
                    mname, spec_ = 'shoe', 'rgba="0.60 0.62 0.64 1" specular="1.0" shininess="0.92"'
                elif role == 'decal':
                    mname, spec_ = 'screw', 'rgba="0.80 0.82 0.85 1" specular="0.9" shininess="0.9"'
                else:
                    mname, spec_ = 'joint', 'rgba="0.55 0.57 0.60 1" specular="0.55" shininess="0.65"'
                if mname not in mats:
                    assets.append(f'<material name="{mname}" {spec_}/>')
                    mats.add(mname)
                geoms.append(f'<geom type="mesh" mesh="p{k}" material="{mname}"/>')
        return f"""<mujoco><compiler meshdir="{tmp}"/>
<visual><global offwidth="{W}" offheight="{H}" fovy="32"/><quality shadowsize="8192" offsamples="8"/>
<headlight ambient="0.22 0.22 0.23" diffuse="0.0 0.0 0.0" specular="0 0 0"/><map shadowclip="1.2" shadowscale="0.8"/></visual>
<asset>{r_assets}{''.join(assets)}</asset>
<worldbody>
<light name="key" pos="0.55 -0.55 0.95" dir="-0.55 0.5 -0.75" diffuse="0.85 0.82 0.78" specular="0.7 0.7 0.7" castshadow="true" cutoff="55" exponent="6"/>
<light name="fill" pos="-0.7 -0.6 0.6" dir="0.6 0.5 -0.45" diffuse="0.30 0.32 0.36" specular="0.25 0.25 0.3" castshadow="false"/>
<light name="rim" pos="0.1 0.45 0.55" dir="0 -0.6 -0.6" diffuse="0.35 0.35 0.38" specular="0.8 0.8 0.85" castshadow="false"/>
<light name="window" directional="true" dir="0.3 0.2 -1" diffuse="0.18 0.18 0.18" specular="0.1 0.1 0.1" castshadow="false"/>
{r_bodies}
{''.join(geoms)}
</worldbody></mujoco>"""

    def shot(self, q, theta, el, dist, target, offset=(0.0, 0.0), exact=None, cover=1.0):
        """theta: degrees round from the front (0) toward the viewer's side, where the back wall is (90 side, 180 rear)."""
        az = 180.0 - theta
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'scene.xml')
            with open(path, 'w') as fh:
                fh.write(self.xml(q, tmp, offset, exact, cover))
            m = mujoco.MjModel.from_xml_path(path)
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.lookat[:] = target
            cam.distance, cam.azimuth, cam.elevation = dist, az, el
            r = mujoco.Renderer(m, H, W)
            opt = mujoco.MjvOption()
            r.update_scene(d, cam, opt)
            img = r.render().copy()
            r.close()
        return Image.fromarray(img)


def walk_frames(sc, md, base_end, blend=12, fade=14):
    """The repo's kinematic walk (tools/render_build_video.py's gait: leg IK, the body swaying onto the stance leg,
    each frame placed by the stance foot flat on the desk), along +x from where the duck stands; the covers fade over
    its first steps (the user: they come off only as it starts walking); `blend` frames ease standing into it."""
    import render_build_video as RBV
    R0 = md.R.R                                             # the spec's kinematics (the flaps stay tucked)
    g = RBV.gait(R0)
    step = RBV.FPS / FPS                                     # its frames are at RBV.FPS
    stand_q = dict(base_end[1])

    def placed(q, stance, x):
        M = R0.fk(q)
        F = np.eye(4)
        F[0, 3], F[1, 3] = x, -0.02
        return F @ np.linalg.inv(M[f'{stance}_foot'])

    out, n = [], int(len(g) / step)
    for k in range(n + blend):
        if k < blend:                                        # ease from standing into the walk's first frame
            w = CD.ease(k / blend)
            q0, st0, x0 = g[0]
            q = {j: (1 - w) * stand_q.get(j, 0.0) + w * q0.get(j, 0.0) for j in set(stand_q) | set(q0)}
            B, cover, root_x = placed(q, st0, x0), 1.0, 0.0
        else:
            q, st, x = g[min(len(g) - 1, int(round((k - blend) * step)))]
            B = placed(q, st, x)
            cover = 1.0 - CD.ease((k - blend) / fade)
            root_x = B[0, 3]
        f_ = max(0.0, (k - blend) / max(1, n - 1))
        im = sc.shot(q, 20 + 55 * CD.ease(f_), -9 + 3 * math.sin(math.pi * f_), 1.0 - 0.1 * f_,
                     (root_x, -0.02, 0.12), exact=B, cover=cover)
        out.append(np.asarray(im.resize((720, 1280), Image.LANCZOS)))
        print(f'walk frame {k + 1}/{n + blend}', flush=True)
    return out


def main():
    memcap.guard(cap_gb=6, need_free_gb=16)
    with open(os.path.join(CD.OUT, 'painted.pkl'), 'rb') as fh:
        data = pickle.load(fh)
    md = CD.Model()
    CD.attach_flaps(md, data['hinges'], data.get('jaw'))
    sc = DeskScene(md, data['extra'])
    sc.R = md.R
    # the truck sits mid-desk: its measured middle (posed, as drawn) at the desk's centre
    Mt = md.R.fk(CD.TRUCK, base=CD.TRUCK_BASE)
    xs_ = np.concatenate([G.apply(Mt[ln], v)[:, 0] for ln, v, f, c, r in sc.parts if len(f)])
    off_truck = (-0.5 * (xs_.min() + xs_.max()), -0.02)
    # close: portrait frames the duck's height; the truck is seen nose-on at three quarters so it fills the frame
    # portrait is ~18 deg wide: the truck needs ~1 m, the duck ~0.65 m
    stills = [('desk_truck_hero', CD.TRUCK, CD.TRUCK_BASE, 35, -15, 1.15, (0.0, -0.02, 0.05), off_truck),
              ('desk_truck_rear', CD.TRUCK, CD.TRUCK_BASE, 145, -17, 1.30, (0.0, -0.02, 0.05), off_truck),
              ('desk_duck_hero', {'jaw': 0.3}, np.eye(4), 35, -8, 1.00, (0.0, -0.02, 0.14), (0.0, -0.02)),
              ('desk_duck_front', {'jaw': 0.3}, np.eye(4), 8, -5, 1.02, (0.0, -0.02, 0.14), (0.0, -0.02))]
    for name, q, base, az, el, dist, tgt, off in stills:
        sc.base = base
        im = sc.shot(q, az, el, dist, tgt, off)
        out = os.path.join(CD.OUT, f'{name}.png')
        im.save(out)
        print(out, flush=True)
    if '--stills' in sys.argv:
        return
    # the clip: truck -> standing duck (bill-supported), then it chats; the camera eases from the truck's framing to the duck's
    seq = [(1.0, None)] * 25 + [(t, None) for t in np.linspace(1, 0, 90)] + [(0.0, None)] * 10
    seq += [(0.0, 0.35 * math.sin(math.pi * k / 18) ** 2) for k in range(36)] + [(0.0, 0.0)] * 15
    frames = []
    for k, (t, jaw) in enumerate(seq):
        base, q = CD.transform_pose(t)
        if jaw is not None:
            q['jaw'] = jaw
        sc.base = base
        w = CD.ease(t)
        off = (w * off_truck[0], -0.02)
        tgt = (0.0, -0.02, 0.05 * w + 0.14 * (1 - w))
        # the camera circles: from the truck's front three-quarter, round the side as it gets up, to the duck's front
        f_ = k / (len(seq) - 1)
        th = 35 + 110 * math.sin(math.pi * min(f_ / 0.72, 1.0)) ** 2 if f_ < 0.72 else 35 - 20 * CD.ease((f_ - 0.72) / 0.28)
        el = -15 + 7 * math.sin(math.pi * f_)
        dist = 1.30 * w + 1.00 * (1 - w)
        im = sc.shot(q, th, el, dist, tgt, off)
        frames.append(np.asarray(im.resize((720, 1280), Image.LANCZOS)))
        print(f'frame {k + 1}/{len(seq)}', flush=True)
    frames += walk_frames(sc, md, base_end=CD.transform_pose(0.0))
    out = os.path.join(CD.OUT, 'desk_transform.mp4')
    imageio.mimsave(out, frames, fps=FPS, codec='libx264', quality=8, macro_block_size=8)
    print(out)


if __name__ == '__main__':
    main()
