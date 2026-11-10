"""The 50 Hz loop: read servos and IMU, ask the policy, write targets. The same code on the Pi and against the simulator.

Safety, on every tick: targets stay inside each joint's limits, and the loop stops the policy when the body has fallen past
a limit, the battery is low, or a servo stops answering. The servos' torque goes off when the program ends.
"""
import time

import numpy as np

from . import dxl
from .policy import observation

RATE = 50.0
LOW_VOLTS = 6.6        # 3.3 V a cell, for a 2S LiPo
FALL_GRAVITY_Z = -0.3  # gravity's z in the body: -1 upright, 0 on its side


class Stats:
    def __init__(self):
        self.ticks, self.late, self.work = 0, 0, []

    def add(self, work, late):
        self.ticks += 1
        self.late += bool(late)
        self.work.append(work)

    def report(self):
        w = np.array(self.work or [0.0]) * 1000
        return {'ticks': self.ticks, 'late': self.late, 'work_ms_median': round(float(np.median(w)), 2), 'work_ms_p99': round(float(np.percentile(w, 99)), 2), 'budget_ms': 1000 / RATE}


def run(servos, imu, policy, cfg, seconds=None, twist=(0, 0, 0), head=(0, 0, 0, 0), ramp_s=3.0, stop_on_fall=True, log=None):
    """Returns (why it stopped, Stats)."""
    home = np.array([cfg['home'][n] for n in cfg['policy_joints']])
    lo = np.array([cfg['limits'][n][0] for n in cfg['policy_joints']])
    hi = np.array([cfg['limits'][n][1] for n in cfg['policy_joints']])
    servos.setup()
    q, _ = servos.read()
    servos.write(q)                     # hold where it is before the torque comes on, so it doesn't jump
    servos.hold_beak()
    servos.torque(True)
    # slowly to home, then the policy
    t0 = time.perf_counter()
    while (a := (time.perf_counter() - t0) / ramp_s) < 1.0:
        servos.write(q + (home - q) * a)
        time.sleep(1 / RATE)
    servos.write(home)
    last = np.zeros(len(home), np.float32)
    stats, why = Stats(), 'finished'
    period, nxt, t0 = 1 / RATE, time.perf_counter(), time.perf_counter()
    tick = 0
    try:
        while seconds is None or time.perf_counter() - t0 < seconds:
            start = time.perf_counter()
            q, qd = servos.read()
            gyro, gravity = imu.read()
            if stop_on_fall and gravity[2] > FALL_GRAVITY_Z:
                why = 'fell'
                break
            if tick % 50 == 0 and servos.voltage() < LOW_VOLTS:
                why = 'battery low'
                break
            act = np.asarray(policy(observation(gyro, gravity, q - home, qd, last, twist, head)), float)
            servos.write(np.clip(home + act, lo, hi))
            last = act.astype(np.float32)
            if log is not None:
                log.append((time.perf_counter() - t0, q.copy(), gyro.copy(), gravity.copy(), act.copy()))
            work = time.perf_counter() - start
            nxt += period
            wait = nxt - time.perf_counter()
            stats.add(work, wait < 0)
            if wait > 0:
                time.sleep(wait)
            else:
                nxt = time.perf_counter()
            tick += 1
    except dxl.DxlError as e:
        why = f'bus: {e}'
    except KeyboardInterrupt:
        why = 'stopped by hand'
    finally:
        try:
            servos.torque(False)
        except (dxl.DxlError, OSError):
            pass
    return why, stats
