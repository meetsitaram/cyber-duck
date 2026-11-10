"""A simulated duck that looks like the real one to the Pi's code: Dynamixel Protocol 2.0 bytes on one TCP port, the IMU on another, MuJoCo behind them.

  python sim/server.py [--pinned] [--port 9000] [--imu-port 9001]

run.py connects with --port socket://127.0.0.1:9000 --imu 127.0.0.1:9001, and nothing else changes. Servo model as the trainer's: position gain, a torque limit
falling with speed, a 15-30 ms delay. --pinned holds the body in the air, for checking directions and the loop without a policy that balances.
"""
import argparse
import json
import os
import socket
import struct
import threading
import time
from collections import deque

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
import sys  # noqa: E402
sys.path.insert(0, os.path.dirname(HERE))
from duck import dxl  # noqa: E402


class Servo:
    def __init__(self, ident, centre, direction, joint=None):
        self.mem = bytearray(256)
        self.id, self.centre, self.dir, self.joint = ident, centre, direction, joint
        self.put(dxl.PRESENT_VOLTAGE, 78)     # 7.8 V
        self.put(dxl.GOAL_POSITION, centre)
        self.put(dxl.PRESENT_POSITION, centre)

    def put(self, reg, v):
        self.mem[reg[0]:reg[0] + reg[1]] = int(v).to_bytes(reg[1], 'little', signed=v < 0)

    def get(self, reg, signed=False):
        return int.from_bytes(self.mem[reg[0]:reg[0] + reg[1]], 'little', signed=signed)


class Duck:
    def __init__(self, cfg, pinned):
        spec = mujoco.MjSpec.from_file(os.path.join(HERE, 'model', 'milli.xml'))
        w = spec.worldbody
        floor = w.add_geom()
        floor.type = mujoco.mjtGeom.mjGEOM_PLANE
        floor.size = [5, 5, 0.1]
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        self.cfg, self.pinned = cfg, pinned
        self.servos = {}
        k = 2 * np.pi / dxl.TICKS_PER_REV
        self.k = k
        self.joints = {}
        for name, j in cfg['joints'].items():
            jid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.servos[j['id']] = Servo(j['id'], j['centre'], j['direction'], name if jid >= 0 else None)
            if jid >= 0:
                self.joints[j['id']] = (self.m.jnt_qposadr[jid], self.m.jnt_dofadr[jid], self.m.jnt_range[jid])
        self.d.qpos[:3] = [0, 0, cfg['root_z'] + (0.1 if pinned else 0.0)]
        self.d.qpos[3:7] = [1, 0, 0, 0]
        for name, v in cfg['home'].items():
            jid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.d.qpos[self.m.jnt_qposadr[jid]] = v
        self.base = self.d.qpos[:7].copy()
        self.lag = deque(maxlen=max(1, int(0.02 / self.m.opt.timestep)))    # about the trainer's 15-30 ms
        self.lock = threading.Lock()
        self.imu_site = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, 'imu')
        self.stop = False
        self.fallen_at = None
        self.t_real = 0.0
        mujoco.mj_forward(self.m, self.d)
        self.sync_present()

    def sync_present(self):
        for i, (qa, da, _) in self.joints.items():
            s = self.servos[i]
            s.put(dxl.PRESENT_POSITION, int(round(self.d.qpos[qa] * s.dir / self.k + s.centre)))
            s.put(dxl.PRESENT_VELOCITY, int(round(self.d.qvel[da] * s.dir / dxl.VELOCITY_UNIT)))

    def step(self):
        c = self.cfg['servo']
        with self.lock:
            goals = {i: (self.servos[i].get(dxl.GOAL_POSITION) - self.servos[i].centre) * self.k * self.servos[i].dir for i in self.joints}
            on = {i: self.servos[i].get(dxl.TORQUE_ENABLE) for i in self.joints}
            self.lag.append(goals)
            goals = self.lag[0]
            self.d.qfrc_applied[:] = 0
            for i, (qa, da, _) in self.joints.items():
                if not on[i]:
                    continue
                v = self.d.qvel[da]
                tau = c['kp'] * (goals[i] - self.d.qpos[qa])
                # the torque the motor can give, falling linearly to zero at no-load speed
                lim_up = c['peak_nm'] * np.clip(1 - v / c['noload_rads'], 0, 1)
                lim_dn = c['peak_nm'] * np.clip(1 + v / c['noload_rads'], 0, 1)
                self.d.qfrc_applied[da] = np.clip(tau, -lim_dn, lim_up)
            mujoco.mj_step(self.m, self.d)
            if self.pinned or not any(on.values()):     # until the torque comes on, someone is holding it
                self.d.qpos[:7] = self.base
                self.d.qvel[:6] = 0
            self.sync_present()

    def imu(self):
        with self.lock:
            R = self.d.site_xmat[self.imu_site].reshape(3, 3)
            gyro = self.d.sensordata[self.m.sensor_adr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SENSOR, 'imu_ang_vel')]:][:3].copy()
            return np.concatenate([gyro, R.T @ np.array([0, 0, -1.0])])

    def physics(self):
        dt, nxt = self.m.opt.timestep, time.monotonic()
        while not self.stop:
            self.step()
            nxt += dt
            wait = nxt - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            else:
                nxt = time.monotonic()      # running behind real time: slow down, don't jump


def bus_server(duck, sock):
    """Answers instruction packets as the servos would."""
    conn, _ = sock.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    buf = bytearray()

    def status(ident, params=b'', err=0):
        body = dxl.stuff(bytes((dxl.STATUS, err)) + bytes(params))
        head = dxl.HEADER + bytes((ident,)) + struct.pack('<H', len(body) + 2)
        pkt = head + body
        return pkt + struct.pack('<H', dxl.crc16(pkt))

    while not duck.stop:
        try:
            d = conn.recv(4096)
        except OSError:
            break
        if not d:
            break
        buf += d
        while True:
            k = buf.find(dxl.HEADER)
            if k < 0 or len(buf) < k + 7:
                break
            n = struct.unpack('<H', buf[k + 5:k + 7])[0]
            if len(buf) < k + 7 + n:
                break
            pkt = bytes(buf[k:k + 7 + n])
            del buf[:k + 7 + n]
            if struct.unpack('<H', pkt[-2:])[0] != dxl.crc16(pkt[:-2]):
                continue                    # a real servo ignores a bad packet, and the Pi's read times out
            ident, inst, p = pkt[4], pkt[7], dxl.unstuff(pkt[8:-2])
            out = b''
            with duck.lock:
                if inst == dxl.PING and ident in duck.servos:
                    out = status(ident, b'\x60\x00\x01')
                elif inst == dxl.READ and ident in duck.servos:
                    a, s = struct.unpack('<HH', p[:4])
                    out = status(ident, duck.servos[ident].mem[a:a + s])
                elif inst == dxl.WRITE and ident in duck.servos:
                    a = struct.unpack('<H', p[:2])[0]
                    duck.servos[ident].mem[a:a + len(p) - 2] = p[2:]
                    out = status(ident)
                elif inst == dxl.SYNC_READ:
                    a, s = struct.unpack('<HH', p[:4])
                    out = b''.join(status(i, duck.servos[i].mem[a:a + s]) for i in p[4:] if i in duck.servos)
                elif inst == dxl.SYNC_WRITE:
                    a, s = struct.unpack('<HH', p[:4])
                    for k2 in range(4, len(p), s + 1):
                        if p[k2] in duck.servos:
                            duck.servos[p[k2]].mem[a:a + s] = p[k2 + 1:k2 + 1 + s]
            if out:
                conn.sendall(out)
    duck.stop = True


def imu_server(duck, sock):
    conn, _ = sock.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    while not duck.stop:
        try:
            if not conn.recv(1):
                break
            conn.sendall(struct.pack('<6f', *duck.imu()))
        except OSError:
            break


def serve(cfg, pinned=False, port=9000, imu_port=9001):
    duck = Duck(cfg, pinned)
    socks = []
    for p in (port, imu_port):
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('127.0.0.1', p))
        s.listen(1)
        socks.append(s)
    threads = [threading.Thread(target=duck.physics, daemon=True), threading.Thread(target=bus_server, args=(duck, socks[0]), daemon=True), threading.Thread(target=imu_server, args=(duck, socks[1]), daemon=True)]
    for t in threads:
        t.start()
    return duck


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--pinned', action='store_true')
    ap.add_argument('--port', type=int, default=9000)
    ap.add_argument('--imu-port', type=int, default=9001)
    ap.add_argument('--config', default=os.path.join(os.path.dirname(HERE), 'config', 'dynamixel.json'))
    a = ap.parse_args()
    d = serve(json.load(open(a.config, encoding='utf-8')), a.pinned, a.port, a.imu_port)
    print('listening: bus', a.port, 'IMU', a.imu_port, '(Ctrl-C to stop)')
    while not d.stop:
        time.sleep(0.2)
