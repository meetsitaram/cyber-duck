"""Play a retargeted clip in MuJoCo: what torque would each joint need, and how long does it stay up with no policy?

  python tools/sim_replay.py CLIP.npz [--video out.mp4]

Two tests, neither a controller:
  inverse  the clip's motion (mj_inverse, no contacts), with the force the root would need carried by the foot or feet near the
           floor instead: the torque each joint needs, against its servo. Where that force pulls on the floor or its centre of
           pressure leaves the sole, a real foot could not do it: counted as "infeasible". No ankle roll, so most single-support
           frames are, as the clip's balance is the human's: a policy has to find its own.
  open     the servos just follow the clip from the first frame, torque-limited, with no balance: how long until it falls.
           Expect a fall: that is what a trained policy is for.
Plain python.
"""
import argparse
import os
import sys

import imageio.v2 as imageio
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402

XML = os.path.join(G.ROOT, 'sim', 'milli.xml')


def set_pose(m, d, names, qi, pos, quat_xyzw):
    root = m.joint('root').qposadr[0]
    d.qpos[root:root + 3] = pos
    d.qpos[root + 3:root + 7] = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
    for n, v in zip(names, qi):
        d.qpos[m.joint(n).qposadr[0]] = v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clip')
    ap.add_argument('--video')
    a = ap.parse_args()
    Z = np.load(a.clip)
    names = [str(n) for n in Z['names']]
    t, q, rp, rq = Z['t'], Z['q'], Z['root_pos'], Z['root_quat_xyzw']
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    spec = G.load()
    J = {j['name']: j for j in spec['joints']}
    dt = t[1] - t[0]
    qpos = np.zeros((len(t), m.nq))
    for i in range(len(t)):
        set_pose(m, d, names, q[i], rp[i], rq[i])
        qpos[i] = d.qpos
    qvel = np.zeros((len(t), m.nv))
    for i in range(len(t) - 1):
        mujoco.mj_differentiatePos(m, qvel[i], dt, qpos[i], qpos[i + 1])
    qvel[-1] = qvel[-2]
    k = np.ones(5) / 5                             # (a light smoothing: differences of a clip are noisy)
    qvel = np.apply_along_axis(lambda v: np.convolve(v, k, mode='same'), 0, qvel)
    qacc = np.gradient(qvel, dt, axis=0)
    qacc = np.apply_along_axis(lambda v: np.convolve(v, k, mode='same'), 0, qacc)
    tau = np.zeros((len(t), m.nu))
    feet = [m.body('L_foot').id, m.body('R_foot').id]
    dof_of = [m.jnt_dofadr[m.actuator_trnid[u, 0]] for u in range(m.nu)]
    Fz = np.zeros(len(t))
    infeasible = 0
    no_contact = 0
    m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    sole = {b_: next(g for g in range(m.ngeom) if m.geom_bodyid[g] == b_ and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX) for b_ in feet}
    for i in range(len(t)):
        d.qpos[:], d.qvel[:], d.qacc[:] = qpos[i], qvel[i], qacc[i]
        mujoco.mj_inverse(m, d)
        zs = {b_: d.geom_xpos[sole[b_]][2] - m.geom_size[sole[b_]][2] for b_ in feet}
        low = min(zs.values())
        on = [b_ for b_ in feet if zs[b_] < low + 0.004]
        Js = []
        for b_ in on:
            mujoco.mj_jacGeom(m, d, jp, jr, sole[b_])
            Js.append(np.vstack([jp, jr]))
        Jc = np.vstack(Js)                         # (6 per foot, nv)
        w = np.linalg.lstsq(Jc[:, :6].T, d.qfrc_inverse[:6], rcond=None)[0]
        res = d.qfrc_inverse - Jc.T @ w
        tau[i] = res[dof_of]
        bad = False
        fz_total = 0.0
        for k_, b_ in enumerate(on):
            f = w[6 * k_:6 * k_ + 3]
            tq = w[6 * k_ + 3:6 * k_ + 6]
            fz_total += f[2]
            if f[2] <= 0:
                bad = True
                continue
            # the centre of pressure, from the sole's centre, in the world's x and y
            cop = np.array([-tq[1], tq[0]]) / f[2]
            ax = d.geom_xmat[sole[b_]].reshape(3, 3)
            if abs(ax[:2, 0] @ cop) > m.geom_size[sole[b_]][0] + 0.009 or abs(ax[:2, 1] @ cop) > m.geom_size[sole[b_]][1] + 0.005:
                bad = True
        Fz[i] = fz_total
        infeasible += bad
    m.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    weight = float(np.sum(m.body_mass)) * 9.81
    print(f'inverse: {len(t)} frames; floor force median {np.median(Fz) / weight * 100:.0f} % of its weight, 99th percentile {np.percentile(Fz, 99) / weight * 100:.0f} %; '
          f'{infeasible / len(t) * 100:.0f} % of frames infeasible for a real foot')
    print(f"{'joint':16s}{'servo':15s}{'peak':>7s}{'RMS':>7s}{'rated':>7s}{'peak lim':>9s}{'q speed':>9s}{'no load':>8s}")
    for u in range(m.nu):
        n = m.actuator(u).name
        A = spec['actuators'][J[n]['act']]
        v = np.percentile(np.abs(qvel[:, m.jnt_dofadr[m.actuator_trnid[u, 0]]]), 99)
        print(f"{n:16s}{J[n]['act']:15s}{np.percentile(np.abs(tau[:, u]), 99):7.2f}{np.sqrt(np.mean(tau[:, u] ** 2)):7.2f}{A['rated_nm']:7.2f}{A['peak_nm']:9.2f}{v:9.1f}{A['noload_rads']:8.1f}")
    # open loop
    mujoco.mj_resetData(m, d)
    d.qpos[:] = qpos[0]
    root = m.joint('root').qposadr[0]
    n_sub = int(round(dt / m.opt.timestep))
    fell = None
    ctrl_of = {m.actuator(u).name: u for u in range(m.nu)}
    frames = []
    r = None
    if a.video:
        r = mujoco.Renderer(m, 480, 720)
        cam = mujoco.MjvCamera()
        cam.azimuth, cam.elevation, cam.distance = 140, -12, 0.75
    for i in range(len(t)):
        for n, v in zip(names, q[i]):
            d.ctrl[ctrl_of[n]] = v
        for _ in range(n_sub):
            mujoco.mj_step(m, d)
        w = d.qpos[root + 3:root + 7]
        up = 1 - 2 * (w[1] ** 2 + w[2] ** 2)
        if fell is None and up < np.cos(0.8):      # (the root's origin is at the spec's world origin: tilt is what tells)
            fell = t[i]
        if r is not None and i % 2 == 0:
            cam.lookat[:] = d.qpos[root:root + 2].tolist() + [0.15]
            r.update_scene(d, cam)
            frames.append(r.render().copy())
    print('open loop:', f'fell at {fell:.2f} s' if fell is not None else f'stayed up for the whole {t[-1]:.1f} s')
    if a.video and frames:
        imageio.mimsave(a.video, frames, fps=25, codec='libx264', quality=7, macro_block_size=1)
        print(a.video)
    return 0


if __name__ == '__main__':
    sys.exit(main())
