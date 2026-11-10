"""The duck's servos and IMU, in the policy's units: radians, in the model's joint order.

config (config/dynamixel.json): for each joint its servo id, direction and centre tick. Directions and centres start as
guesses, and `python -m duck.calibrate` measures the centres on the real robot.
"""
import json
import math
import socket
import struct

import numpy as np

from . import dxl


class Servos:
    def __init__(self, port, cfg, baud=1_000_000):
        self.cfg = cfg
        self.bus = dxl.Bus(port, baud=baud)
        self.names = cfg['policy_joints']
        self.ids = [cfg['joints'][n]['id'] for n in self.names]
        self.all_names = self.names + cfg['extra_joints']
        self.all_ids = [cfg['joints'][n]['id'] for n in self.all_names]
        self.sign = np.array([cfg['joints'][n]['direction'] for n in self.names], float)
        self.centre = np.array([cfg['joints'][n]['centre'] for n in self.names], float)
        self.k = 2 * math.pi / dxl.TICKS_PER_REV

    def setup(self):
        """Position mode at the trained gain. Torque must be off to change a mode."""
        for i in self.all_ids:
            if not self.bus.ping(i):
                raise dxl.DxlError(f'no servo {i}')
            self.bus.write(i, dxl.TORQUE_ENABLE, 0)
            self.bus.write(i, dxl.RETURN_DELAY, 0)
            self.bus.write(i, dxl.OPERATING_MODE, dxl.MODE_POSITION)
            self.bus.write(i, dxl.POSITION_P, self.cfg['position_p'])
            self.bus.write(i, dxl.PROFILE_VELOCITY, 0)

    def torque(self, on):
        for i in self.all_ids:
            self.bus.write(i, dxl.TORQUE_ENABLE, 1 if on else 0)

    def read(self):
        """Joint positions (rad) and velocities (rad/s), from one sync read of velocity and position."""
        addr = dxl.PRESENT_VELOCITY[0]
        raw = self.bus.sync_read(self.ids, addr, 8)
        vel, tick = np.zeros(len(self.ids)), np.zeros(len(self.ids))
        for k, i in enumerate(self.ids):
            v, p = struct.unpack('<ii', raw[i])
            vel[k], tick[k] = v, p
        return (tick - self.centre) * self.k * self.sign, vel * dxl.VELOCITY_UNIT * self.sign

    def write(self, q):
        ticks = np.rint(np.asarray(q) * self.sign / self.k + self.centre).astype(int)
        self.bus.sync_write(self.ids, dxl.GOAL_POSITION, [int(t) for t in ticks])

    def hold_beak(self, q=0.0):
        for n in self.cfg['extra_joints']:
            j = self.cfg['joints'][n]
            self.bus.write(j['id'], dxl.GOAL_POSITION, int(round(q * j['direction'] / self.k + j['centre'])))

    def voltage(self):
        """Volts, from the first servo on the bus."""
        return self.bus.read(self.ids[0], dxl.PRESENT_VOLTAGE) * 0.1

    def close(self):
        self.bus.close()


class Bno085:
    """The real IMU over I2C (pins 3 and 5). Not yet tested on a real board. Returns (gyro rad/s, gravity unit vector), both in the robot's frame."""

    def __init__(self, cfg):
        import board
        import busio
        from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
        from adafruit_bno08x.i2c import BNO08X_I2C
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
        # 0x4A on Adafruit's board; the GY-BNO08X's AD0 pad can make it 0x4B. Try both (i2cdetect -y 1 shows which)
        for addr in (0x4A, 0x4B):
            try:
                self.imu = BNO08X_I2C(i2c, address=addr)
                break
            except (ValueError, OSError, RuntimeError):
                if addr == 0x4B:
                    raise
        self.imu.enable_feature(BNO_REPORT_GYROSCOPE)
        self.imu.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
        self.R = np.array(cfg['imu_to_robot'], float)

    def read(self):
        gyro = np.array(self.imu.gyro, float)
        x, y, z, w = self.imu.game_quaternion
        # world down, in the sensor's frame
        g = -np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])
        return self.R @ gyro, self.R @ g

    def close(self):
        pass


class SimImu:
    """The simulated IMU: pi/sim/server.py answers with 6 floats."""

    def __init__(self, addr):
        host, port = addr.rsplit(':', 1)
        self.s = socket.create_connection((host, int(port)), timeout=1.0)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def read(self):
        self.s.sendall(b'r')
        buf = b''
        while len(buf) < 24:
            buf += self.s.recv(24 - len(buf))
        v = np.array(struct.unpack('<6f', buf), float)
        return v[:3], v[3:]

    def close(self):
        self.s.close()


def load_config(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)
