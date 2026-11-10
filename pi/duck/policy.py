"""What the walking policy sees and returns, as the trainer's velocity and stand-up tasks define it.

Input, 61 numbers at 50 Hz, in this order:
  angular velocity 3, gravity 3, joint positions minus home 14, joint velocities 14, last action 14,
  twist command 3 (vx, vy, turn), head command 4 (neck pitch, head pitch, head yaw, head roll), body command 6.
Output: 14 joint targets, as offsets from home, radians. Action scale 1. The beak isn't in it.
The exported ONNX carries the training's normalizer, so nothing here normalizes.
"""
import numpy as np

N_JOINTS = 14
N_OBS = 3 + 3 + N_JOINTS * 3 + 3 + 4 + 6


def observation(gyro, gravity, q_rel, qd, last_action, twist=(0, 0, 0), head=(0, 0, 0, 0), body=(0, 0, 0, 0, 0, 0)):
    return np.concatenate([gyro, gravity, q_rel, qd, last_action, twist, head, body]).astype(np.float32)


class OnnxPolicy:
    def __init__(self, path):
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1                # keep the bus and the IMU's core free on a Pi Zero
        self.sess = ort.InferenceSession(path, opts, providers=['CPUExecutionProvider'])
        self.input = self.sess.get_inputs()[0].name
        n = self.sess.get_inputs()[0].shape[-1]
        if n != N_OBS:
            raise ValueError(f'{path} takes {n} inputs, and this duck gives {N_OBS}. Foot-switch policies take 63: retrain without them')

    def __call__(self, obs):
        return self.sess.run(None, {self.input: obs[None]})[0][0]


class HoldPose:
    """A stand-in until there's a trained policy: all zeros, so the servos hold home. It doesn't balance."""

    def __call__(self, obs):
        return np.zeros(N_JOINTS, np.float32)


class HeadWave:
    """A stand-in that moves: a slow nod and turn on the head, the legs held at home. For checking directions and the loop's timing."""

    def __init__(self, names, rate=50.0):
        self.neck, self.yaw = names.index('neck_pitch'), names.index('head_yaw')
        self.n, self.rate = 0, rate

    def __call__(self, obs):
        t = self.n / self.rate
        self.n += 1
        a = np.zeros(N_JOINTS, np.float32)
        a[self.neck] = -0.2 + 0.2 * np.sin(2 * np.pi * 0.25 * t)
        a[self.yaw] = 0.5 * np.sin(2 * np.pi * 0.2 * t)
        return a
