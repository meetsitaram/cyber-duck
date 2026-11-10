"""Run the duck.

On the Pi:         python run.py --policy models/walk.onnx --vx 0.2
Against the sim:   python sim/server.py   (in another terminal), then
                   python run.py --port socket://127.0.0.1:9000 --imu 127.0.0.1:9001 --policy wave

--policy: an ONNX file, or `hold` or `wave`, the two stand-ins that need no training.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from duck import controller, hardware, policy  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--port', default='/dev/serial0', help='the servo bus, or socket://host:port for the simulator')
ap.add_argument('--imu', default='bno085', help='bno085, or host:port for the simulator')
ap.add_argument('--config', default=os.path.join(HERE, 'config', 'dynamixel.json'))
ap.add_argument('--policy', default='hold')
ap.add_argument('--seconds', type=float)
ap.add_argument('--vx', type=float, default=0.0)
ap.add_argument('--vy', type=float, default=0.0)
ap.add_argument('--turn', type=float, default=0.0)
ap.add_argument('--no-fall', action='store_true', help="don't stop when the body tips over: for a robot on a stand")
a = ap.parse_args()

cfg = hardware.load_config(a.config)
servos = hardware.Servos(a.port, cfg)
imu = hardware.Bno085(cfg) if a.imu == 'bno085' else hardware.SimImu(a.imu)
pol = {'hold': policy.HoldPose, 'wave': lambda: policy.HeadWave(cfg['policy_joints'])}.get(a.policy, lambda: policy.OnnxPolicy(a.policy))()
why, stats = controller.run(servos, imu, pol, cfg, seconds=a.seconds, twist=(a.vx, a.vy, a.turn), stop_on_fall=not a.no_fall)
print('stopped:', why)
print(json.dumps(stats.report()))
servos.close()
imu.close()
