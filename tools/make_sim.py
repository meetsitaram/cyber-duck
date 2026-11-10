"""milli's MuJoCo model from spec/robot.json, and a first test: does it stand, and at what torque?

  python tools/make_sim.py              writes sim/milli.xml and sim/meshes/, runs the stand test, reports to reports/sim_stand.md
  python tools/make_sim.py --view PNG   also saves a picture of the model

Masses and inertias come from tools/mass_model.py, one body per link. Servos are position actuators with their force limited
to the servo's peak torque; armature, damping and friction from spec/actuators.json (STS3215: Open Duck Mini's identification,
scaled by gear ratio for the C044 and C046; XL/XC330: microduck's). Collision: the soles only, 4 corner spheres and a box each,
on a floor. Nothing here is tuned against a real robot yet.
Plain python (MuJoCo lives in the user site-packages).
"""
import argparse
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import mass_model as MM  # noqa: E402
import render_pose as RP  # noqa: E402

OUT = os.path.join(G.ROOT, 'sim')
KP = {'STS': 17.8, 'XL': 6.0, 'XC': 8.0}           # N m/rad: Open Duck Mini's for the STS3215; the Dynamixels' are a first guess
FLOOR = 0.0


def kp_of(act):
    return KP['STS' if act.startswith('STS') else ('XC' if act.startswith('XC') else 'XL')]


def build(spec):
    S = C.Solids(spec)
    inert = MM.link_inertials(spec, S)
    R = G.Robot(spec)
    os.makedirs(os.path.join(OUT, 'meshes'), exist_ok=True)
    assets, bodies = [], {}

    def visual(ln):
        out = []
        for i, g in enumerate(spec['links'][ln]['geoms']):
            m = S.geom.get((ln, i))
            if m is None or g['role'] == 'decal':
                continue
            mesh = m.to_mesh()
            if len(mesh.tri_verts) == 0:
                continue
            o = np.array(spec_origin(ln))
            name = f"{ln}_{i}"
            RP.stl(os.path.join(OUT, 'meshes', name + '.stl'), np.asarray(mesh.vert_properties[:, :3], float) - o, np.asarray(mesh.tri_verts, int))
            assets.append(f'<mesh name="{name}" file="{name}.stl"/>')
            rgba = ' '.join(f'{c:.3f}' for c in RP.rgba(g['color']))
            out.append(f'<geom type="mesh" mesh="{name}" rgba="{rgba}" contype="0" conaffinity="0" group="2" mass="0"/>')
        return out

    def spec_origin(ln):
        j = R.parent_joint.get(ln)
        return j['origin'] if j else [0.0, 0.0, 0.0]

    def body(ln, parent_o):
        o = np.array(spec_origin(ln))
        pos = o - np.array(parent_o)
        m, com, I, _ = inert[ln]
        c = com - o
        full = ' '.join(f'{v:.8g}' for v in (I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]))
        x = [f'<body name="{ln}" pos="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}">',
             f'<inertial pos="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}" mass="{m:.5f}" fullinertia="{full}"/>']
        j = R.parent_joint.get(ln)
        if j:
            a = spec['actuators'][j['act']]
            ax = {'x': '1 0 0', 'y': '0 1 0', 'z': '0 0 1'}[j['axis']]
            x.append(f'<joint name="{j["name"]}" axis="{ax}" range="{j["lower"]:.4f} {j["upper"]:.4f}" armature="{a["armature_kgm2"]:.6g}" '
                     f'damping="{a["damping_nms"]:.5g}" frictionloss="{a["friction_nm"]:.5g}"/>')
        else:
            x.append('<freejoint name="root"/>')
        x += visual(ln)
        for g in spec['links'][ln]['geoms']:
            if 'contact' in g:
                cn = np.array(g['contact']['c']) - o
                sz = np.array(g['contact']['size']) / 2
                x.append(f'<geom name="{ln}_sole" type="box" pos="{cn[0]:.5f} {cn[1]:.5f} {cn[2]:.5f}" size="{sz[0]:.5f} {sz[1]:.5f} {sz[2]:.5f}" '
                         'rgba="1 1 0 0" friction="0.8 0.02 0.01" group="3"/>')
        for ch in R.children.get(ln, []):
            x += body(ch['child'], o)
        x.append('</body>')
        return x

    tree = body(spec['root'], [0.0, 0.0, 0.0])
    # (the root stands where the spec drew it: its frame at the world origin; joint positions are all from there)
    acts = []
    for j in spec['joints']:
        a = spec['actuators'][j['act']]
        kp = kp_of(j['act'])
        acts.append(f'<position name="{j["name"]}" joint="{j["name"]}" kp="{kp}" ctrlrange="{j["lower"]:.4f} {j["upper"]:.4f}" '
                    f'forcerange="{-a["peak_nm"]:.3f} {a["peak_nm"]:.3f}"/>')
    xml = f"""<mujoco model="milli">
<compiler meshdir="meshes" angle="radian" autolimits="true" balanceinertia="false"/>
<option timestep="0.002" integrator="implicitfast"/>
<visual><global offwidth="1280" offheight="720"/><headlight ambient=".42 .42 .42" diffuse=".45 .45 .45"/></visual>
<asset>{''.join(assets)}
<texture type="2d" name="grid" builtin="checker" rgb1=".88 .88 .88" rgb2=".95 .95 .95" width="256" height="256"/>
<material name="grid" texture="grid" texrepeat="20 20"/></asset>
<worldbody><light directional="true" dir="-0.5 -0.3 -1" diffuse=".4 .4 .4"/>
<geom name="floor" type="plane" size="3 3 .1" material="grid" friction="0.8 0.02 0.01"/>
{chr(10).join(tree)}
</worldbody>
<actuator>{chr(10).join(acts)}</actuator>
</mujoco>
"""
    path = os.path.join(OUT, 'milli.xml')
    open(path, 'w', encoding='utf-8').write(xml)
    return path, sum(v[0] for v in inert.values())


def stand(path, spec):
    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    # the zero pose's centre of mass is ahead of the feet: lean back at the ankles until it is over them, as tools/checks.py does
    R = G.Robot(spec)
    q = C.balance(spec, R, C.Mass(spec, C.Solids(spec)), G.mirror_pose({}), 'double', 0)
    root = m.joint('root').qposadr[0]
    for k, v in q.items():
        d.qpos[m.joint(k).qposadr[0]] = v
    W = np.linalg.inv(R.fk(q)['L_foot'])
    d.qpos[root:root + 3] = W[:3, 3] + [0, 0, 0.0005]
    qt = np.zeros(4)
    mujoco.mju_mat2Quat(qt, W[:3, :3].flatten())
    d.qpos[root + 3:root + 7] = qt
    for i in range(m.nu):
        d.ctrl[i] = q.get(m.actuator(i).name, 0.0)
    names = [m.actuator(i).name for i in range(m.nu)]
    rows, tilt, zs = {}, [], []
    peak = np.zeros(m.nu)
    rms = np.zeros(m.nu)
    n = 0
    steps = int(5.0 / m.opt.timestep)
    for k in range(steps):
        mujoco.mj_step(m, d)
        if k > steps // 5:
            peak = np.maximum(peak, np.abs(d.actuator_force))
            rms += d.actuator_force ** 2
            n += 1
            qq = d.qpos[root + 3:root + 7]
            tilt.append(2 * np.arcsin(min(1.0, np.linalg.norm(qq[1:3]))))
            zs.append(d.qpos[root + 2])
    rms = np.sqrt(rms / n)
    return names, peak, rms, max(tilt), zs[-1], m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--view')
    a = ap.parse_args()
    spec = G.load()
    path, total = build(spec)
    names, peak, rms, tilt, z, m = stand(path, spec)
    J = {j['name']: j for j in spec['joints']}
    L = ['# Stand test in MuJoCo', '', f'{total:.2f} kg. 5 s holding its stance, leaned back at the ankles until balanced, servos at their peak-torque limit. Not tuned against a real robot.', '',
         f'Largest tilt {np.degrees(tilt):.1f} deg; root height change {z * 1000:+.1f} mm.', '',
         '| joint | servo | peak N m | RMS N m | rated N m | RMS / rated |', '|---|---|---|---|---|---|']
    for i, n in enumerate(names):
        a_ = spec['actuators'][J[n]['act']]
        L.append(f"| {n} | {J[n]['act']} | {peak[i]:.3f} | {rms[i]:.3f} | {a_['rated_nm']:.2f} | {rms[i] / a_['rated_nm'] * 100:.0f} % |")
    open(os.path.join(G.ROOT, 'reports', 'sim_stand.md'), 'w', encoding='utf-8').write('\n'.join(L) + '\n')
    print('\n'.join(L[2:]))
    print(path, m.nv, 'dof', m.nbody, 'bodies')
    return 0 if tilt < 0.35 else 1


if __name__ == '__main__':
    sys.exit(main())
