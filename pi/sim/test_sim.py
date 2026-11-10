"""Does the Pi's code really drive the duck? Runs the real controller, on the real bytes, against the simulator, and checks what happened. No GPU, no training.

  python sim/test_sim.py

1. Protocol: packets, stuffing, CRC.
2. Pinned in the air, `wave`: the loop keeps 50 Hz, and the head follows the commands.
3. On the floor, `hold`: the bus and IMU work with the feet down, and the fall stop fires if it falls.
4. An ONNX file, if onnx and onnxruntime are installed: the policy path, with a stand-in network of the right shape.
"""
import json
import os
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from duck import controller, dxl, hardware, policy  # noqa: E402
import server  # noqa: E402

cfg = json.load(open(os.path.join(os.path.dirname(HERE), 'config', 'dynamixel.json'), encoding='utf-8'))
fails = []


def check(name, ok, detail=''):
    print(('PASS' if ok else 'FAIL'), name, detail)
    if not ok:
        fails.append(name)


# 1. protocol
p = dxl.packet(1, dxl.PING)
check('ping packet matches ROBOTIS\'s worked example', p.hex() == 'fffffd0001030001 19 4e'.replace(' ', ''), p.hex())
raw = bytes((1, 0xFF, 0xFF, 0xFD, 2, 0xFF, 0xFF, 0xFD))
check('byte stuffing round trip', dxl.unstuff(dxl.stuff(raw)) == raw)

port = 9100


def session(pinned, pol, seconds, ramp_s=1.0, **kw):
    global port
    port += 2
    duck = server.serve(cfg, pinned=pinned, port=port, imu_port=port + 1)
    time.sleep(0.2)
    s = hardware.Servos(f'socket://127.0.0.1:{port}', cfg)
    i = hardware.SimImu(f'127.0.0.1:{port + 1}')
    log = []
    why, stats = controller.run(s, i, pol, cfg, seconds=seconds, log=log, ramp_s=ramp_s, **kw)
    s.close()
    i.close()
    duck.stop = True
    return duck, why, stats, log


# 2. pinned, moving the head
d, why, st, log = session(True, policy.HeadWave(cfg['policy_joints']), 6.0, stop_on_fall=False)
r = st.report()
print('   ', json.dumps(r))
check('pinned: ran to the end', why == 'finished', why)
check('pinned: under 5 % of ticks late', r['late'] <= 0.05 * r['ticks'], f"{r['late']} of {r['ticks']}")
names = cfg['policy_joints']
y, n = names.index('head_yaw'), names.index('neck_pitch')
qy = np.array([l[1][y] for l in log])
cmd = np.array([l[4][y] for l in log])
lag = max(range(0, 25), key=lambda k: np.corrcoef(qy[k:], cmd[:len(cmd) - k])[0, 1])
c = np.corrcoef(qy[lag:], cmd[:len(cmd) - lag])[0, 1]
check('pinned: head yaw follows its command', c > 0.98 and np.ptp(qy) > 0.5, f'correlation {c:.3f} at {lag * 20} ms, swing {np.ptp(qy):.2f} rad')
leg = names.index('left_knee')
qk = np.array([l[1][leg] for l in log])
check('pinned: a held leg stays at home', abs(qk.mean() - cfg['home']['left_knee']) < 0.1, f'knee {qk.mean():.3f}, home {cfg["home"]["left_knee"]:.3f}')
g = np.array([l[3] for l in log])
check('pinned: gravity reads straight down', np.allclose(g.mean(0), [0, 0, -1], atol=0.02), np.round(g.mean(0), 3).tolist())

# 3. on the floor, holding: it may fall, as it isn't balancing
d, why, st, log = session(False, policy.HoldPose(), 8.0, ramp_s=0.02)
print('    floor: stopped because', why, 'after', round(log[-1][0], 2) if log else 0, 's')
check('floor: the loop ran, and stopped cleanly', why in ('finished', 'fell') and len(log) > 10, f'{why}, {len(log)} ticks')
check('floor: the body stays above the floor', d.d.qpos[2] > 0.0, f'trunk at {d.d.qpos[2]:.3f} m')

# 4. ONNX
try:
    import onnx
    from onnx import helper, numpy_helper, TensorProto
    import onnxruntime  # noqa: F401
    W = np.zeros((policy.N_OBS, policy.N_JOINTS), np.float32)
    net = helper.make_model(helper.make_graph([helper.make_node('MatMul', ['obs', 'W'], ['act'])], 'standin',
                                              [helper.make_tensor_value_info('obs', TensorProto.FLOAT, [None, policy.N_OBS])],
                                              [helper.make_tensor_value_info('act', TensorProto.FLOAT, [None, policy.N_JOINTS])],
                                              [numpy_helper.from_array(W, 'W')]), opset_imports=[helper.make_opsetid('', 17)])
    net.ir_version = 9
    path = os.path.join(tempfile.mkdtemp(), 'standin.onnx')
    onnx.save(net, path)
    d, why, st, log = session(True, policy.OnnxPolicy(path), 3.0, stop_on_fall=False)
    r = st.report()
    print('   ', json.dumps(r))
    check('onnx: a 61-input policy runs in the loop', why == 'finished' and r['ticks'] > 100, why)
except ImportError as e:
    print('SKIP onnx:', e)

print(f'{len(fails)} failing' + (': ' + ', '.join(fails) if fails else ''))
sys.exit(1 if fails else 0)
