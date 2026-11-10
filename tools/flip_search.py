"""Can a roll be thrown? python tools/flip_search.py ROBOT_FOLDER OUT.npz [generations] [population]

No policy, and a few minutes instead of hours of training: symmetric keyframes (crouch, drive up and forward with the legs, tuck, recover) searched by CEM in plain
MuJoCo, with self-collision and each servo's DC limits. ROBOT_FOLDER: the trainer's generated model (make_rl_model.py --contact). It can show a move is possible, never that
it is impossible. Then replay the result with tools/check_motion.py, with MILLI_RECIPE set. INIT=earlier.npz starts from an earlier search; WORKERS: processes (default 10):
count them against the machine, beside any training. Plain python.
"""
import json, os, sys, time
from multiprocessing import Pool

import mujoco
import numpy as np

R = sys.argv[1]
GENS = int(sys.argv[3]) if len(sys.argv) > 3 else 60
POP = int(sys.argv[4]) if len(sys.argv) > 4 else 300
DT = 0.002
STAND = json.load(open(R + "/home.json"))["root_z"]
SECONDS = 3.5
SEND = 0.02                                        # the servo command's rate
LAG = 10                                           # 20 ms, the middle of the trainer's delay
N_KEY = 5
_m = {}


def setup():
    x = open(R + "/milli.xml").read().replace("<worldbody>", '<worldbody><geom name="floor" type="plane" size="20 20 .1" friction="0.7"/>', 1)
    p = R + f"/_t{os.getpid()}.xml"
    open(p, "w").write(x)
    M = mujoco.MjModel.from_xml_path(p)
    os.remove(p)
    M.opt.timestep = DT
    home = json.load(open(R + "/home.json"))
    names, kp, peak, speed = [], [], [], []
    for s in home["servos"].values():
        for j in s["joints"]:
            names.append(j); kp.append(s["kp"]); peak.append(s["peak_nm"]); speed.append(s["noload_rads"])
    qa = np.array([M.joint(n).qposadr[0] for n in names])
    va = np.array([M.joint(n).dofadr[0] for n in names])
    lim = np.array([M.jnt_range[M.joint(n).id] for n in names])
    q0 = np.array([home["joint_pos"].get(n, 0.0) for n in names])
    _m.update(M=M, D=mujoco.MjData(M), names=names, kp=np.array(kp), peak=np.array(peak), speed=np.array(speed), qa=qa, va=va, lim=lim, q0=q0, z0=home["root_z"])


def target(k):
    """One keyframe (left hip pitch, knee, ankle, neck pitch, head pitch) offset from home, mirrored to the right leg."""
    t = _m["q0"].copy()
    n = _m["names"]
    for side, sg in (("left", 1), ("right", -1)):
        t[n.index(side + "_hip_pitch")] = sg * k[0]
        t[n.index(side + "_knee")] = sg * k[1]
        t[n.index(side + "_ankle")] = sg * k[2]
    t[n.index("neck_pitch")] = k[3]
    t[n.index("head_pitch")] = k[4]
    return np.clip(t, _m["lim"][:, 0], _m["lim"][:, 1])


def rollout(th, record=False):
    if not _m:
        setup()
    M, D = _m["M"], _m["D"]
    mujoco.mj_resetData(M, D)
    D.qpos[2] = _m["z0"]
    D.qpos[_m["qa"]] = _m["q0"]
    keys = th[:N_KEY * 5].reshape(N_KEY, 5)
    times = np.cumsum(np.clip(th[N_KEY * 5:N_KEY * 6], 0.05, 1.0))
    tg = [target(k) for k in keys]
    i_key, cmd, q_rec, rot, peak_rot = 0, _m["q0"].copy(), [], 0.0, 0.0
    pipe = [cmd.copy() for _ in range(LAG)]
    every = int(round(SEND / DT))
    for s in range(int(SECONDS / DT)):
        t = s * DT
        while i_key < N_KEY - 1 and t > times[i_key]:
            i_key += 1
        pipe.append(tg[i_key])
        cmd = pipe.pop(0)
        q, v = D.qpos[_m["qa"]], D.qvel[_m["va"]]
        tau = _m["kp"] * (cmd - q)
        hi = np.clip(_m["peak"] * (1 - v / _m["speed"]), 0, _m["peak"])    # the DC curve: no torque past no-load speed
        lo = -np.clip(_m["peak"] * (1 + v / _m["speed"]), 0, _m["peak"])
        D.qfrc_applied[:] = 0
        D.qfrc_applied[_m["va"]] = np.clip(tau, lo, hi)
        mujoco.mj_step(M, D)
        if not np.isfinite(D.qpos).all():
            return (-10.0, 0.0, False) if not record else None
        rot += D.qvel[4] * DT                                              # body y: forward roll
        peak_rot = max(peak_rot, rot)
        if record and s % every == 0:
            q_rec.append(np.concatenate([D.qpos[:7], q.copy()]))
    w, x, y = D.qpos[3:6]
    up = 1 - 2 * (x * x + y * y)
    z = D.qpos[2]
    settled = up > np.cos(np.radians(20)) and abs(z - STAND) < 0.15 * STAND
    won = settled and peak_rot > 0.9 * 2 * np.pi
    score = min(peak_rot, 2 * np.pi) + (1.5 * max(up, 0) + 1.5 * float(settled) if peak_rot > 0.75 * 2 * np.pi else 0)
    if record:
        return dict(q=np.array(q_rec), names=np.array(_m["names"]), dt=SEND, won=won, rot=np.degrees(peak_rot), up=up, z=z)
    return score, np.degrees(peak_rot), won


def main():
    out = sys.argv[2]
    # the user's idea as the start: crouch low, drive up and forward with the legs, tuck hard, unfold, stand
    h_ = json.load(open(R + '/home.json'))['joint_pos']
    home = np.array([h_['left_hip_pitch'], h_['left_knee'], h_['left_ankle'], 0.0, 0.0])
    mean = np.concatenate([[-0.9, 1.05, 0.9, 0.0, 0.0], [-0.2, 0.1, -0.6, -1.0, 1.0], [-1.0, 1.05, 1.05, -1.2, 1.0], [-1.0, 1.05, 0.8, -0.3, 0.3], home,
                           [0.5, 0.2, 0.3, 0.5, 1.0]])
    std = np.concatenate([np.full(N_KEY * 5, 0.5), np.full(N_KEY, 0.15)])
    if os.environ.get("INIT"):
        mean = np.load(os.environ["INIT"])["theta"]
        std = std * 0.4
    best, t0 = (-9, None, 0, False), time.time()
    with Pool(int(os.environ.get("WORKERS", 10))) as P:
        print("start score", P.map(rollout, [mean])[0], flush=True)
        for g in range(GENS):
            ths = mean + std * np.random.randn(POP, len(mean))
            ths[0] = mean
            if best[1] is not None:
                ths[1] = best[1]
            rs = P.map(rollout, list(ths))
            sc = np.array([r[0] for r in rs])
            el = np.argsort(-sc)[:POP // 8]
            mean, std = ths[el].mean(0), np.maximum(ths[el].std(0), np.concatenate([np.full(N_KEY * 5, 0.04), np.full(N_KEY, 0.015)]))
            i = int(el[0])
            if sc[i] > best[0]:
                best = (sc[i], ths[i].copy(), rs[i][1], rs[i][2])
            print(f"gen {g} best {sc[i]:.2f} rot {rs[i][1]:.0f} won {rs[i][2]} share won {np.mean([r[2] for r in rs]):.2f} overall rot {best[2]:.0f} won {best[3]} t {time.time() - t0:.0f}s", flush=True)
    rec = rollout(best[1], record=True)
    np.savez(out, **rec, theta=best[1])
    print("BEST rotation", round(rec["rot"]), "deg; won", rec["won"], "final up", round(float(rec["up"]), 2), "z", round(float(rec["z"]), 3))


if __name__ == "__main__":
    main()
