"""Why won't a big duck lift a foot? The same numbers on it and on the hybrid, which walks: python tools/leg_limits.py TRAINER_ROBOT_DIR. Plain MuJoCo on the trainer's models.
A: how high a foot lifts inside the soft joint limits (0.9 of the range, as in training), sole level, trunk still.
B: the body leaned over one foot on hip roll alone, no ankle roll: can the centre of mass reach the sole?
C: holding the whole robot on one foot, each joint's torque against its peak (static: real steps need more).
D: the swing a 1.2 s stride asks for: joint speeds, and torque left at that speed on the trainer's motor (falls in a line to 0 at no-load)."""
import json, sys
import mujoco, numpy as np
from scipy.optimize import least_squares

R = sys.argv[1].rstrip('/') + '/'                   # the trainer's src/mjlab_microduck/robot
LEG = ['left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle']
SWING = {'big': 0.48, 'hybrid': 0.48 / (1.797 / 0.321) ** 0.5}


def load(folder):
    m = mujoco.MjModel.from_xml_path(R + folder + '/milli.xml')
    h = json.load(open(R + folder + '/home.json'))
    return m, mujoco.MjData(m), h


def setq(m, d, h, extra=None, root=None):
    d.qpos[:] = 0
    d.qpos[3] = 1
    d.qpos[2] = h['root_z']
    for k, v in {**h['joint_pos'], **(extra or {})}.items():
        d.qpos[m.joint(k).qposadr[0]] = v
    if root is not None:
        d.qpos[:7] = root
    mujoco.mj_forward(m, d)


def run(name, folder, ranges=None):
    m, d, h = load(folder)
    peak = {j: s['peak_nm'] for s in h['servos'].values() for j in s['joints']}
    noload = {j: s['noload_rads'] for s in h['servos'].values() for j in s['joints']}
    rng = {j: tuple(m.jnt_range[m.joint(j).id]) for j in LEG}
    if ranges:
        rng.update(ranges)
    soft = {j: ((a + b) / 2 - 0.45 * (b - a), (a + b) / 2 + 0.45 * (b - a)) for j, (a, b) in rng.items()}
    setq(m, d, h)
    site = m.site('left_foot').id
    foot = m.body('ankle_left').id
    height = d.xpos[m.body('jaw_soft').id][2]
    com0 = d.subtree_com[m.body('trunk_base').id].copy()
    g = m.geom('left_foot_collision').id
    half = m.geom_size[g]
    sole0 = d.geom_xpos[g].copy()
    ank = d.xanchor[m.joint('left_ankle').id].copy()
    mass = m.body_subtreemass[m.body('trunk_base').id]
    scale = 1.797 if name == 'big' else 0.321
    print(f"\n== {name}: {mass:.2f} kg, CoM height {com0[2]:.3f} m ({com0[2] / scale:.2f} of the robot), sole half size x {half[0]:.3f} y {half[1]:.3f}")
    print(f"   left sole centre y {sole0[1]:.3f} ({sole0[1] / scale:.3f} of height); ankle is {sole0[0] - ank[0]:+.3f} m behind the sole's centre; CoM is {com0[0] - ank[0]:+.3f} m ahead of the ankle, home")
    print('   soft limits', {k: (round(a, 2), round(b, 2)) for k, (a, b) in soft.items()}, ' home knee', round(h['joint_pos']['left_knee'], 2))

    # A: foot lift
    names = ['left_hip_pitch', 'left_knee', 'left_ankle']
    lim = np.array([soft[n] for n in names]).T
    x0 = np.clip([h['joint_pos'][n] for n in names], lim[0] + 1e-3, lim[1] - 1e-3)

    def foot_at(x):
        setq(m, d, h, dict(zip(names, x)))
        return d.site_xpos[site].copy(), d.xmat[foot].reshape(3, 3)
    base = foot_at(x0)[0][2]
    best, bx = 0, x0
    for target in np.arange(0.005, 0.9, 0.005) * scale:
        r = least_squares(lambda x: [(foot_at(x)[0][2] - base - target) * 10, foot_at(x)[1][2, 0], foot_at(x)[0][0] - foot_at(x0)[0][0]], bx, bounds=lim, xtol=1e-7)
        if np.abs(r.fun).max() > 0.002:
            break
        best, bx = target, r.x
    print(f"A  foot lift, level, straight up: {best * 100:.1f} cm = {best / scale * 100:.1f} % of height; at {dict(zip(names, np.round(bx, 2)))}")

    # B: lean over one foot. Unknown: root pose and the left leg's joints; the left foot stays where it is, flat
    s0 = d.site_xpos[site].copy()
    setq(m, d, h)
    s0 = d.site_xpos[site].copy()
    R0 = d.xmat[foot].reshape(3, 3).copy()
    best_gap = None
    for rollq in np.linspace(*soft['left_hip_roll'], 13):
        un = ['left_hip_yaw', 'left_hip_pitch', 'left_knee', 'left_ankle']

        def res(x):
            q = np.zeros(4)
            mujoco.mju_euler2Quat(q, x[3:6], 'xyz')
            setq(m, d, h, {**dict(zip(un, x[6:])), 'left_hip_roll': rollq, 'right_hip_roll': -0.0}, np.r_[x[:3], q])
            e = d.xmat[foot].reshape(3, 3) - R0
            t = d.subtree_com[1]
            return np.r_[d.site_xpos[site] - s0, e[:, 0] * 0.2, e[:, 2] * 0.2, (d.xmat[m.body('trunk_base').id].reshape(3, 3)[0, 2]) * 0.1]
        x0_ = np.r_[0, 0, h['root_z'], 0, 0, 0, [h['joint_pos'][n] for n in un]]
        r = least_squares(res, x0_, xtol=1e-7, max_nfev=60)
        res(r.x)
        com = d.subtree_com[m.body('trunk_base').id]
        gap = abs(com[1] - sole0[1]) - half[1]
        if np.abs(r.fun).max() < 0.003 and (best_gap is None or gap < best_gap[0]):
            best_gap = (gap, rollq, com.copy(), r.x.copy())
    gap, rollq, com, xb = best_gap
    print(f"B  leaned on left hip roll {rollq:+.2f}: CoM {abs(com[1] - sole0[1]) * 100:.1f} cm from the sole's centre line, {gap * 100:+.1f} cm past its edge = {gap / scale * 100:+.1f} % of height")

    # C: on one foot, static, knee at its standing angle, trunk level, the centre of mass over a chosen point ahead of the ankle: nothing left free to wander. The load is under the CoM, as it must be
    print('C  one foot, whole weight, joint torque / peak; leg as it stands, CoM moved along the sole:')
    un = ['left_hip_yaw', 'left_hip_pitch', 'left_ankle']
    for fwd in (-0.03, 0.0, 0.03, 0.06, 0.10):
        dx = fwd * scale

        def res(x):
            q = np.zeros(4)
            mujoco.mju_euler2Quat(q, x[3:6], 'xyz')
            setq(m, d, h, {**dict(zip(un, x[6:])), 'left_hip_roll': rollq}, np.r_[x[:3], q])
            e = d.xmat[foot].reshape(3, 3) - R0
            return np.r_[d.site_xpos[site] - s0, e[:, 0] * 0.2, e[:, 2] * 0.2, d.xmat[m.body('trunk_base').id].reshape(3, 3)[0, 2] * 0.1, (d.subtree_com[m.body('trunk_base').id][0] - d.xanchor[m.joint('left_ankle').id][0] - dx), x[6] * 0.05]
        r = least_squares(res, np.r_[xb[:6], 0, h['joint_pos']['left_hip_pitch'], h['joint_pos']['left_ankle']], xtol=1e-8, max_nfev=80)
        res(r.x)
        p = d.subtree_com[m.body('trunk_base').id].copy()
        p[2] = 0
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, jp, jr, p, foot)
        tau = d.qfrc_bias - jp.T @ np.array([0, 0, mass * 9.81])
        tt = {j: float(tau[m.joint(j).dofadr[0]]) for j in LEG}
        print(f"     CoM {dx * 100:5.1f} cm ahead of ankle ({fwd * 100:3.0f} % of height), fit {np.abs(r.fun).max():.4f}: " + '  '.join(f"{j[5:]} {abs(v):5.1f}/{peak[j]:.0f}={abs(v) / peak[j] * 100:3.0f}%" for j, v in tt.items()))
    print('   peak over (weight x height): ' + '  '.join(f"{j[5:]} {peak[j] / (mass * 9.81 * scale):.3f}" for j in LEG))

    # D: swing speed
    T = SWING[name]
    pts = []
    for s in np.linspace(0, 1, 25):
        tgt = min(best, 0.056 * scale) * np.sin(np.pi * s)
        r = least_squares(lambda x: [(foot_at(x)[0][2] - base - tgt) * 10, foot_at(x)[1][2, 0], foot_at(x)[0][0] - foot_at(x0)[0][0] - 0.25 * scale * (s - 0.5)], pts[-1] if pts else x0, bounds=lim, xtol=1e-7)
        pts.append(r.x)
    pts = np.array(pts)
    v = np.gradient(pts, T / 24, axis=0)
    for i, n in enumerate(names):
        w = np.abs(v[:, i]).max()
        print(f"D  {n[5:]:10s} swing peak speed {w:5.2f} rad/s of no-load {noload[n]:5.2f}: {max(0, 1 - w / noload[n]) * 100:3.0f} % of its torque left ({peak[n] * max(0, 1 - w / noload[n]):.1f} of {peak[n]:.1f} N m); range used {pts[:, i].min():+.2f}..{pts[:, i].max():+.2f}")


run('hybrid', 'milli')
run('big', 'milli_big')
