"""Human walking onto milli: BONES-SEED clips, already retargeted to a full-size humanoid, as milli's joint motions.

  python tools/retarget_bones.py CLIP.csv [...] [--out reports/motion/retarget] [--seconds 20]

The mapping (the user's, from an earlier project):
  legs       foot paths relative to the hip, scaled to milli's leg, then milli's own leg IK; hip yaw and roll as they are
  arms       averaged onto the head: mean shoulder pitch to head pitch, their difference to head yaw, mean roll to head roll
  torso      forward lean to the neck, going up and down
The root's path and the foot paths are scaled by the leg ratio and STEP, and time by PACE. Clips stay on
this machine: the BONES-SEED licence is the user's to check, and neither they nor retargets go in the repo (reports/motion is
ignored).
Plain python.
"""
import argparse
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402

SRC_FPS, OUT_FPS = 120.0, 50.0
SRC_LEG = (0.30, 0.30)                             # the source humanoid's thigh and shin, m (close enough for foot paths)
HEAD_GAIN, NECK_GAIN = 0.8, 0.8
# A human stride on legs this short, at a dynamically similar pace, wants 19 rad/s at the knee, and the servo's no-load speed is
# 7.9, and half the time the knee is at its limit (first try, 2026-11-03). So steps at this share of the scaled length, and this pace.
STEP, PACE = 0.5, 1.25


def load(path):
    h = open(path).readline().strip().split(',')
    A = np.loadtxt(path, delimiter=',', skiprows=1)
    return {n: A[:, i] for i, n in enumerate(h)}


def leg_ik(R, side, dx, dz, q0):
    names = [f'{side}_hip_pitch', f'{side}_knee', f'{side}_ankle_pitch']

    def res(x):
        M = R.fk(dict(zip(names, x)))[f'{side}_foot']
        return [M[0, 3] - dx, M[2, 3] - dz, M[0, 2]]
    return least_squares(res, q0, xtol=1e-9).x


def retarget(spec, path, seconds=None):
    R = G.Robot(spec)
    D = load(path)
    J = R.joints
    rad = np.radians
    # milli's leg: hip to knee, knee to ankle
    o = lambda n: np.array(J[n]['origin'])
    leg = np.linalg.norm((o('L_hip_pitch') - o('L_knee'))[[0, 2]]) + abs(o('L_knee')[2] - o('L_ankle_pitch')[2])
    s = leg / sum(SRC_LEG) * STEP
    speed = PACE
    T = len(D['Frame']) / SRC_FPS / speed          # seconds at milli's pace
    if seconds:
        T = min(T, seconds)
    t = np.arange(0, T, 1 / OUT_FPS)
    src_i = t * speed * SRC_FPS
    samp = lambda n: np.interp(src_i, np.arange(len(D[n])), D[n])
    N = len(t)
    q = {n: np.zeros(N) for n in J}

    def foot(side):
        h, k = rad(samp(f'{side}_hip_pitch_joint_dof')), rad(samp(f'{side}_knee_joint_dof'))
        L1, L2 = SRC_LEG
        return -L1 * np.sin(h) - L2 * np.sin(h + k), -(L1 * np.cos(h) + L2 * np.cos(h + k)), h, k
    # (milli's zero is its stance: the source's standing foot height is the reference)
    limit = lambda n, v: np.clip(v, J[n]['lower'], J[n]['upper'])
    for side, lw in (('L', 'left'), ('R', 'right')):
        x, z, h, k = foot(lw)
        z0 = np.percentile(z, 5)                   # the foot at its lowest: on the floor
        x0 = x.mean()
        guess = np.zeros(3)
        for i in range(N):
            guess = leg_ik(R, side, s * (x[i] - x0), s * (z[i] - z0), guess)
            q[f'{side}_hip_pitch'][i], q[f'{side}_knee'][i], q[f'{side}_ankle_pitch'][i] = guess
        a = rad(samp(f'{lw}_ankle_pitch_joint_dof'))
        q[f'{side}_ankle_pitch'] += 0.5 * (h + k + a - np.mean(h + k + a))     # (half the foot's own pitch, for heel and toe)
        q[f'{side}_hip_yaw'] = rad(samp(f'{lw}_hip_yaw_joint_dof'))
        q[f'{side}_hip_roll'] = rad(samp(f'{lw}_hip_roll_joint_dof'))
    sp = (rad(samp('left_shoulder_pitch_joint_dof')) + rad(samp('right_shoulder_pitch_joint_dof'))) / 2
    sd = (rad(samp('left_shoulder_pitch_joint_dof')) - rad(samp('right_shoulder_pitch_joint_dof'))) / 2
    sr = (rad(samp('left_shoulder_roll_joint_dof')) + rad(samp('right_shoulder_roll_joint_dof'))) / 2
    q['head_pitch'] = HEAD_GAIN * (sp - sp.mean())
    q['head_yaw'] = HEAD_GAIN * sd
    q['head_roll'] = HEAD_GAIN * sr
    wp = rad(samp('waist_pitch_joint_dof'))
    q['neck_pitch'] = NECK_GAIN * (wp - wp.mean())
    clipped = {}
    for n in q:
        over = (q[n] < J[n]['lower'] - 1e-6) | (q[n] > J[n]['upper'] + 1e-6)
        if over.any():
            clipped[n] = float(over.mean())
        q[n] = limit(n, q[n])
    # the root: its path scaled, its roll and pitch as they are, then lowered so the lower foot is on the floor
    rp = np.column_stack([samp('root_translateX'), samp('root_translateY'), samp('root_translateZ')]) / 100.0 * s
    e = Rot.from_euler('xyz', np.column_stack([samp('root_rotateX'), samp('root_rotateY'), samp('root_rotateZ')]), degrees=True)
    yaw0 = e.as_euler('ZYX')[0, 0]
    rot = Rot.from_euler('Z', -yaw0) * e           # facing +x at the start
    rp = (Rot.from_euler('Z', -yaw0).apply(rp - rp[0]))
    pos = np.zeros((N, 3))
    feet = np.zeros((N, 2))
    for i in range(N):
        qi = {n: q[n][i] for n in q}
        M = R.fk(qi)
        Rw = rot[i].as_matrix()
        lowest = 9.0
        for f, side in enumerate('LR'):
            Mf = M[f'{side}_foot']
            pts = np.array([[spec_x, y, 0.0] for spec_x in SOLE_X(spec) for y in SOLE_Y(spec, side)])
            w = (pts @ Mf[:3, :3].T + Mf[:3, 3]) @ Rw.T
            feet[i, f] = w[:, 2].min()
            lowest = min(lowest, w[:, 2].min())
        pos[i] = [rp[i, 0], rp[i, 1], -lowest]
    return dict(t=t, q=q, root_pos=pos, root_quat_xyzw=rot.as_quat(), scale=s, speed=speed, clipped=clipped,
                foot_z=feet - feet.min(axis=1, keepdims=True), name=os.path.basename(path)[:-4])


def SOLE_X(spec):
    c = next(g for g in spec['links']['L_foot']['geoms'] if 'contact' in g)['contact']
    return c['c'][0] - c['size'][0] / 2, c['c'][0] + c['size'][0] / 2


def SOLE_Y(spec, side):
    g = next(g for g in spec['links'][f'{side}_foot']['geoms'] if 'contact' in g)['contact']
    return g['c'][1] - g['size'][1] / 2, g['c'][1] + g['size'][1] / 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--out', default=os.path.join(G.ROOT, 'reports', 'motion', 'retarget'))
    ap.add_argument('--seconds', type=float, default=None)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    spec = G.load()
    for c in a.clips:
        r = retarget(spec, c, a.seconds)
        names = list(r['q'])
        np.savez(os.path.join(a.out, r['name'] + '.npz'), t=r['t'], names=np.array(names), q=np.column_stack([r['q'][n] for n in names]),
                 root_pos=r['root_pos'], root_quat_xyzw=r['root_quat_xyzw'], foot_z=r['foot_z'])
        dist = np.linalg.norm(r['root_pos'][-1, :2] - r['root_pos'][0, :2])
        print(f"{r['name']}: {r['t'][-1]:.1f} s, scale {r['scale']:.3f}, pace x{r['speed']:.2f}, moved {dist:.2f} m, "
              f"speed {np.linalg.norm(np.diff(r['root_pos'][:, :2], axis=0), axis=1).sum() / r['t'][-1]:.2f} m/s, "
              f"swing foot lift max {r['foot_z'].max() * 1000:.0f} mm, at a limit: { {k: round(v, 3) for k, v in r['clipped'].items()} }")
        qv = np.column_stack([np.gradient(r['q'][n], r['t']) for n in names])
        print('  peak joint speed rad/s:', {n: round(float(np.abs(qv[:, i]).max()), 1) for i, n in enumerate(names) if n.startswith('L_') or n in ('neck_pitch', 'head_pitch')})


if __name__ == '__main__':
    main()
