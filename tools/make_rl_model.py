"""milli as a robot for the microduck_rl trainer: python tools/make_rl_model.py OUT_DIR [--salvage] [--contact]

--salvage: the servos from a dismantled SO-101 pair (SALVAGE below), to find out if they can walk, before the spec changes.
--contact: every link above the shoes also gets a collision shape, its convex hull, for tasks where the body meets the floor.
Writes OUT_DIR/milli.xml, one mesh per link, and home.json. Follows that trainer's conventions, so its rewards, symmetry and
observations apply: microduck's link, joint, site and sensor names, its joint directions and joint order, the beak locked.

The standing pose is level (the trunk square to the floor, the feet moved forward at the same height until the centre of mass is
over them), as microduck's HOME is. The servos go in home.json, each with its own numbers from spec/actuators.json, for that
trainer's DC motor model (a torque-speed curve and a delay), not its measured BAM model, which fits only the XL330. Stiffness: 0.55 N m/rad for the XL330 (microduck's identification, kp 200); the XC330 scaled by stall torque; the
STS3215 from Open Duck Mini's 17.8 for the 1:345 version, scaled by gear ratio. Unmeasured: expect this to change with a real servo.
Collision: the soles only, as that trainer's walking model. Plain python.
"""
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares, brentq
from scipy.spatial import ConvexHull

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import mass_model as MM  # noqa: E402
import render_pose as RP  # noqa: E402

LINK = {'trunk': 'trunk_base', 'L_hip_yaw_link': 'yaw2roll', 'R_hip_yaw_link': 'bearing_roll', 'L_hip_roll_link': 'hip_l', 'R_hip_roll_link': 'hip_l_2',
        'L_hip_pitch_link': 'upper_leg_left', 'R_hip_pitch_link': 'upper_leg_right', 'L_knee_link': 'leg', 'R_knee_link': 'leg_2',
        'L_foot': 'ankle_left', 'R_foot': 'ankle_right', 'neck_link': 'neck', 'head_pitch_link': 'neck_pitch', 'head_yaw_link': 'yaw_roll_motion',
        'head': 'jaw_soft', 'beak_link': 'jaw'}
# milli's joint -> (the trainer's name, its direction against milli's world axis)
JOINT = {'L_hip_yaw': ('left_hip_yaw', -1), 'R_hip_yaw': ('right_hip_yaw', -1), 'L_hip_roll': ('left_hip_roll', 1), 'R_hip_roll': ('right_hip_roll', 1),
         'L_hip_pitch': ('left_hip_pitch', 1), 'R_hip_pitch': ('right_hip_pitch', -1), 'L_knee': ('left_knee', -1), 'R_knee': ('right_knee', 1),
         'L_ankle_pitch': ('left_ankle', 1), 'R_ankle_pitch': ('right_ankle', -1), 'neck_pitch': ('neck_pitch', -1), 'head_pitch': ('head_pitch', 1),
         'head_yaw': ('head_yaw', 1), 'head_roll': ('head_roll', -1)}
ORDER = ['L_hip_yaw', 'neck_pitch', 'R_hip_yaw']   # the trunk's children, in the trainer's joint order
ROOT_Z = G.load().get('rl', {}).get('root_z', 0.17)   # the root body's height: about the trunk's middle
KP = {'STS3215_C001': 17.8, 'STS3215_C044': 17.8 * 191 / 345, 'STS3215_C046': 17.8 * 147 / 345, 'XL330_M288': 0.55, 'XC330_M288': 0.55 * 0.93 / 0.52,
      # an estimate: the STS3215's gain by gear ratio, at the position gain XGO-Duck's runtime starts from (5 against a Feetech default of 32). Measure it
      'HD1910_C001': 17.8 * 320 / 345 * 5 / 32}
# An SO-101 pair has 7 C001, 2 C044 and 3 C046. The C046s stay at the hip yaws, and the 2 C044s go to the knees, the legs' fastest joints
# in the C044 policy's walk (4.8 rad/s RMS). The same case throughout, so only the servo's numbers change.
SALVAGE = {f'{s}_{j}': 'STS3215_C001' for s in 'LR' for j in ('hip_roll', 'hip_pitch', 'ankle_pitch')}
AXIS = {'x': np.array([1, 0, 0]), 'y': np.array([0, 1, 0]), 'z': np.array([0, 0, 1])}


def level_home(spec, S):
    """Feet forward at the same height, the trunk level, until the centre of mass is over the soles' middle."""
    R = G.Robot(spec)
    MS = C.Mass(spec, S)
    names = lambda s: [f'{s}_hip_pitch', f'{s}_knee', f'{s}_ankle_pitch']

    def ik(s, dx):
        def res(x):
            M = R.fk(dict(zip(names(s), x)))[f'{s}_foot']
            return [M[0, 3] - dx, M[2, 3], M[0, 2]]
        return dict(zip(names(s), least_squares(res, np.zeros(3), xtol=1e-10).x))

    def pose(dx):
        return {**ik('L', dx), **ik('R', dx)}

    def miss(dx):
        q = pose(dx)
        M = R.fk(q)
        com = sum(MS.m[l] * G.apply(M[l], MS.com[l][None])[0] for l in spec['links']) / sum(MS.m.values())
        sole = next(g for g in spec['links']['L_foot']['geoms'] if 'contact' in g)['contact']['c']
        return com[0] - (sole[0] + dx)
    dx = brentq(miss, -0.02 * ROOT_Z / 0.17, 0.07 * ROOT_Z / 0.17, xtol=1e-4)
    return pose(dx), dx


def at_joint(spec, j):
    """A big duck's joint, through its pair and chain: (peak, no-load speed, armature, damping, friction, rated). A joint that has only its actuator's numbers says so by no 'rl' in the spec."""
    a = spec['actuators'][j['act']]
    if 'rl' not in spec:
        return a['peak_nm'], a['noload_rads'], a['armature_kgm2'], a['damping_nms'], a['friction_nm'], a.get('rated_nm')
    n, r = j['capacity']['count'], j['capacity']['ratio']
    slowest = next((v for k, v in spec['rl'].get('speed_ratio', {}).items() if k in j['name']), r)      # a crank's ratio changes: see spec/big_duck.py
    return a['peak_nm'] * n * r, a['noload_rads'] / slowest, a['rotor_kgm2'] * a['gear'] ** 2 * n * r * r, a['damping_nms'] * n * r * r, a['friction_nm'] * n * r, a['rated_nm'] * n * r


def main():
    out = sys.argv[1]
    os.makedirs(os.path.join(out, 'meshes'), exist_ok=True)
    spec = G.load()
    if '--salvage' in sys.argv:
        for j in spec['joints']:
            j['act'] = SALVAGE.get(j['name'], j['act'])
    S = C.Solids(spec)
    R = G.Robot(spec)
    inert = MM.link_inertials(spec, S)
    assets = []

    def origin(ln):
        j = R.parent_joint.get(ln)
        return np.array(j['origin']) if j else np.array([0.0, 0.0, ROOT_Z])

    def body(ln, po):
        o = origin(ln)
        pos = o - po
        m, com, I, _ = inert[ln]
        c = com - o
        full = ' '.join(f'{v:.8g}' for v in (I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]))
        name = LINK[ln]
        x = [f'<body name="{name}" pos="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}">',
             f'<inertial pos="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}" mass="{m:.5f}" fullinertia="{full}"/>']
        j = R.parent_joint.get(ln)
        if j is None:
            x.append('<freejoint name="trunk_base_freejoint"/>')
            sens = np.array(spec['rl']['imu'] if 'rl' in spec else next(g for g in spec['links']['trunk']['geoms'] if g['name'] == 'IMU')['c']) - o
            x.append(f'<site name="imu" pos="{sens[0]:.5f} {sens[1]:.5f} {sens[2]:.5f}"/>')
        elif j['name'] in JOINT:
            nm, sg = JOINT[j['name']]
            _, _, arm, dmp, frc, _ = at_joint(spec, j)
            ax = sg * AXIS[j['axis']]
            lo, hi = sorted((sg * j['lower'], sg * j['upper']))
            x.append(f'<joint name="{nm}" axis="{ax[0]} {ax[1]} {ax[2]}" range="{lo:.4f} {hi:.4f}" armature="{arm:.6g}" '
                     f'damping="{dmp:.5g}" frictionloss="{frc:.5g}"/>')
        mesh = S.all[ln].to_mesh()
        RP.stl(os.path.join(out, 'meshes', name + '.stl'), np.asarray(mesh.vert_properties[:, :3], float) - o, np.asarray(mesh.tri_verts, int))
        assets.append(f'<mesh name="{name}" file="{name}.stl"/>')
        x.append(f'<geom type="mesh" mesh="{name}" rgba=".75 .75 .75 1" contype="0" conaffinity="0" group="2" mass="0"/>')
        if '--contact' in sys.argv and ln not in ('L_foot', 'R_foot'):     # the shoes keep their sole boxes: nothing frictionless below them
            hv = np.asarray(mesh.vert_properties[:, :3], float) - o
            hull = ConvexHull(hv)
            RP.stl(os.path.join(out, 'meshes', name + '_hull.stl'), hv, hull.simplices)
            assets.append(f'<mesh name="{name}_hull" file="{name}_hull.stl" maxhullvert="32"/>')
            x.append(f'<geom name="{name}_collision" type="mesh" mesh="{name}_hull" rgba="1 1 0 0" group="3" mass="0" contype="3" conaffinity="2"/>')
        elif '--contact' in sys.argv:                                      # the shoe's hull meets the robot, never the floor: contype 2 against the floor's 1
            hv = np.asarray(mesh.vert_properties[:, :3], float) - o
            hull = ConvexHull(hv)
            RP.stl(os.path.join(out, 'meshes', name + '_hull.stl'), hv, hull.simplices)
            assets.append(f'<mesh name="{name}_hull" file="{name}_hull.stl" maxhullvert="32"/>')
            side = 'left' if ln == 'L_foot' else 'right'
            x.append(f'<geom name="{side}_foot_hull_collision" type="mesh" mesh="{name}_hull" rgba="1 1 0 0" group="3" mass="0" contype="2" conaffinity="2"/>')
        for g in spec['links'][ln]['geoms']:
            if 'contact' in g:
                side = 'left' if ln == 'L_foot' else 'right'
                cn = np.array(g['contact']['c']) - o
                sz = np.array(g['contact']['size']) / 2
                x.append(f'<geom name="{side}_foot_collision" type="box" pos="{cn[0]:.5f} {cn[1]:.5f} {cn[2]:.5f}" size="{sz[0]:.5f} {sz[1]:.5f} {sz[2]:.5f}" '
                         'rgba="1 1 0 0" group="3" friction="1"' + (' contype="3" conaffinity="2"' if '--contact' in sys.argv else '') + '/>')
                x.append(f'<site name="{side}_foot" pos="{cn[0]:.5f} {cn[1]:.5f} {cn[2] - sz[2]:.5f}"/>')
        kids = R.children.get(ln, [])
        if ln == 'trunk':
            kids = sorted(kids, key=lambda c_: ORDER.index(c_['name']))
        for ch in kids:
            x += body(ch['child'], o)
        x.append('</body>')
        return x

    tree = body(spec['root'], np.zeros(3))
    J = {j['name']: j for j in spec['joints']}
    seq = ['L_hip_yaw', 'L_hip_roll', 'L_hip_pitch', 'L_knee', 'L_ankle_pitch', 'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll',
           'R_hip_yaw', 'R_hip_roll', 'R_hip_pitch', 'R_knee', 'R_ankle_pitch']
    servos = {}
    for n in seq:
        peak, noload, _, _, _, rated = at_joint(spec, J[n])
        if 'rl' in spec:                           # gains by joint, as each joint has its own pair and chain
            kind = JOINT[n][0].replace('left_', '').replace('right_', '')
            kp, kd = spec['rl']['gains'][kind]
            g = servos.setdefault(kind, dict(kp=kp, kd=kd, peak_nm=peak, rated_nm=rated, noload_rads=noload, joints=[]))
        else:
            g = servos.setdefault(J[n]['act'], dict(kp=KP[J[n]['act']], peak_nm=peak, noload_rads=noload, joints=[]))
        g['joints'].append(JOINT[n][0])
    xml = f"""<mujoco model="milli">
<compiler meshdir="meshes" angle="radian" autolimits="true"/>
<option timestep="0.005" integrator="implicitfast"/>
<asset>{''.join(assets)}</asset>
<worldbody>
{chr(10).join(tree)}
</worldbody>
<sensor>
<framequat name="orientation" objtype="site" noise="0.001" objname="imu"/>
<gyro name="angular-velocity" site="imu" noise="0.005"/>
<gyro name="imu_ang_vel" site="imu"/>
<velocimeter name="imu_lin_vel" site="imu"/>
<accelerometer name="imu_accel" site="imu"/>
<subtreeangmom name="root_angmom" body="trunk_base"/>
</sensor>
</mujoco>
"""
    open(os.path.join(out, 'milli.xml'), 'w', encoding='utf-8').write(xml)
    q, dx = level_home(spec, S)
    home = {JOINT[n][0]: float(JOINT[n][1] * v) for n, v in q.items()}
    for n in ('left_hip_yaw', 'right_hip_yaw', 'left_hip_roll', 'right_hip_roll', 'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll'):
        home[n] = 0.0
    json.dump(dict(servos=servos, joint_pos=home, root_z=ROOT_Z + 0.001, feet_forward_m=dx, mass_kg=sum(v[0] for v in inert.values())), open(os.path.join(out, 'home.json'), 'w'), indent=1)
    if '--contact' in sys.argv:
        import mujoco
        # hulls are fatter than the parts: a pair that touches standing, with nothing wrong, can't collide in sim. Which pairs that leaves unguarded is printed: tools/check_motion.py covers them
        m = mujoco.MjModel.from_xml_path(os.path.join(out, 'milli.xml'))
        d = mujoco.MjData(m)
        d.qpos[2] = ROOT_Z + 0.001
        for jn, v in home.items():
            d.qpos[m.joint(jn).qposadr[0]] = v
        mujoco.mj_forward(m, d)
        pairs = sorted({tuple(sorted((m.body(int(m.geom_bodyid[d.contact[i].geom1])).name, m.body(int(m.geom_bodyid[d.contact[i].geom2])).name))) for i in range(d.ncon)})
        # and at the ends of single-joint sweeps, where tools/checks.py finds the parts clear
        pairs = sorted(set(pairs) | {('upper_leg_left', 'yaw2roll'), ('bearing_roll', 'upper_leg_right'), ('bearing_roll', 'leg_2')})
        print('excluded, as their hulls touch where the parts do not:', pairs)
        xml = xml.replace('<sensor>', '<contact>' + ''.join(f'<exclude body1="{a}" body2="{b}"/>' for a, b in pairs) + '</contact>\n<sensor>', 1)
        open(os.path.join(out, 'milli.xml'), 'w', encoding='utf-8').write(xml)
    print(f"mass {sum(v[0] for v in inert.values()):.2f} kg; level stance: feet {dx * 1000:.0f} mm forward; home", {k: round(v, 3) for k, v in home.items() if v})
    return 0


if __name__ == '__main__':
    sys.exit(main())
